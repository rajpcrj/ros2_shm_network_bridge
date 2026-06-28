// shm_to_udp.cpp — stream RAW /dev/shm memory out of a UDP SERVER.
//
// This is an ADAPTER (see docs/06_modular_core.md): it plugs the core
// shm_bridge::Reader into a POSIX UDP server. It does TWO things:
//   1) READER  : attaches to a /dev/shm stream and reads each new frame (0% CPU
//                futex wait) — the RAW bytes exactly as they sit in shared memory.
//   2) UDP SERVER: binds a port, accepts "SUBSCRIBE" datagrams from clients, and
//                streams every subsequent raw frame to all currently-subscribed
//                clients (fragmented to fit the MTU, with a small per-packet header
//                so a client can reassemble + detect loss).
//
// Unlike shm_to_network.cpp (fire-and-forget to ONE fixed host), this is a real
// server: clients come and go, multiple clients can subscribe, and it ships the
// RAW shared-memory bytes (no ROS, no CDR, no transform) — the lowest-overhead way
// to fan a /dev/shm stream onto the network.
//
// Protocol (client -> server), one datagram:
//     "SUB"   -> subscribe this client; server starts streaming frames to it
//     "BYE"   -> unsubscribe
// Server -> client packets reuse the 32-byte wire header from shm_to_network.cpp
// (magic/seq/frag/nfrags/dims/total_len) so the SAME receiver
// (examples/python/network_to_shm.py) works against this server too.
//
// Build (pure library + POSIX, no ROS):
//     g++ -std=c++17 shm_to_udp.cpp -o shm_to_udp -lshm_bridge_cpp -lpthread
// Run (server):
//     ./shm_to_udp --stream rgb --port 6000
// A client subscribes by sending the 3 bytes "SUB" to <server>:6000, then
// receives the raw frames (see examples/python/udp_client.py).
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// 32-byte wire header — IDENTICAL to shm_to_network.cpp so receivers are shared.
#pragma pack(push, 1)
struct WireHdr {
    uint32_t magic;     // 0x53484D31 "SHM1"
    uint32_t seq;
    uint32_t frag;
    uint32_t nfrags;
    uint32_t width;
    uint32_t height;
    uint32_t channels;  // low 8 bits channels, next 8 bits dtype_id
    uint32_t total_len;
};
#pragma pack(pop)
static_assert(sizeof(WireHdr) == 32, "WireHdr must be 32 bytes");

static constexpr uint32_t MAGIC = 0x53484D31u;
static constexpr size_t   MTU_PAYLOAD = 1400;

// key a client by ip:port so we can add/remove from the subscriber set
static std::string key_of(const sockaddr_in& a) {
    char ip[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &a.sin_addr, ip, sizeof(ip));
    return std::string(ip) + ":" + std::to_string(ntohs(a.sin_port));
}

int main(int argc, char** argv) {
    std::string stream = "rgb";
    int port = 6000;
    for (int i = 1; i < argc - 1; ++i) {
        std::string a = argv[i];
        if (a == "--stream") stream = argv[++i];
        else if (a == "--port") port = std::atoi(argv[++i]);
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }
    sockaddr_in me{};
    me.sin_family = AF_INET;
    me.sin_addr.s_addr = htonl(INADDR_ANY);
    me.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(fd, reinterpret_cast<sockaddr*>(&me), sizeof(me)) < 0) {
        perror("bind"); return 1;
    }
    std::printf("shm_to_udp: serving /dev/shm/%s_* on udp://0.0.0.0:%d\n",
                stream.c_str(), port);
    std::printf("  clients: send \"SUB\" to subscribe, \"BYE\" to stop.\n");

    // --- subscriber registry, shared between the control thread and streamer ---
    std::map<std::string, sockaddr_in> subs;
    std::mutex subs_mu;
    std::atomic<bool> running{true};

    // control thread: handle SUB/BYE datagrams from clients.
    std::thread control([&] {
        char buf[64];
        while (running.load()) {
            sockaddr_in from{}; socklen_t fl = sizeof(from);
            ssize_t n = recvfrom(fd, buf, sizeof(buf), 0,
                                 reinterpret_cast<sockaddr*>(&from), &fl);
            if (n <= 0) continue;
            std::string msg(buf, buf + std::min<ssize_t>(n, 3));
            std::lock_guard<std::mutex> lk(subs_mu);
            if (msg == "SUB") {
                subs[key_of(from)] = from;
                std::printf("  + subscriber %s  (total %zu)\n",
                            key_of(from).c_str(), subs.size());
            } else if (msg == "BYE") {
                subs.erase(key_of(from));
                std::printf("  - subscriber %s  (total %zu)\n",
                            key_of(from).c_str(), subs.size());
            }
        }
    });

    // streamer: read raw frames from SHM, fan them to all subscribers.
    shm_bridge::Reader reader(stream);   // throws if the stream isn't up yet
    shm_bridge::Frame f;
    std::vector<uint8_t> pkt(sizeof(WireHdr) + MTU_PAYLOAD);

    while (running.load()) {
        if (!reader.wait_and_read(f, 100ull * 1000 * 1000)) continue;  // 0% CPU wait

        // snapshot the subscriber list (don't hold the lock during sendto loop)
        std::vector<sockaddr_in> targets;
        { std::lock_guard<std::mutex> lk(subs_mu);
          for (auto& kv : subs) targets.push_back(kv.second); }
        if (targets.empty()) continue;

        const size_t total = f.data.size();
        const uint32_t nfrags = static_cast<uint32_t>((total + MTU_PAYLOAD - 1) / MTU_PAYLOAD);
        for (uint32_t fi = 0; fi < nfrags; ++fi) {
            const size_t off = static_cast<size_t>(fi) * MTU_PAYLOAD;
            const size_t len = std::min(MTU_PAYLOAD, total - off);
            WireHdr h{};
            h.magic = MAGIC; h.seq = f.seq; h.frag = fi; h.nfrags = nfrags;
            h.width = f.width; h.height = f.height;
            h.channels = (static_cast<uint32_t>(f.dtype) << 8) | (f.channels & 0xFF);
            h.total_len = static_cast<uint32_t>(total);
            std::memcpy(pkt.data(), &h, sizeof(h));
            // RAW shared-memory bytes copied straight onto the wire — no transform.
            std::memcpy(pkt.data() + sizeof(h), f.data.data() + off, len);
            for (auto& dst : targets)
                sendto(fd, pkt.data(), sizeof(h) + len, 0,
                       reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
        }
    }

    running.store(false);
    if (control.joinable()) control.join();
    close(fd);
    return 0;
}
