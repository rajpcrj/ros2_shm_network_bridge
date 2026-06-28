// shm_to_network.cpp — bridge /dev/shm -> the network, lowest-latency transport.
//
// WHY UDP (and not DDS) for the network hop:
//   For LOCAL (same machine) fan-out the /dev/shm bridge is already the fastest
//   path — nothing beats a shared-memory pointer. For the CROSS-MACHINE hop, the
//   lowest-latency, lowest-overhead option is a plain UDP datagram socket: no
//   discovery handshake, no CDR (de)serialization, no QoS state machine — just
//   the raw frame bytes on the wire. That is the minimal-latency transport for a
//   one-way image/sensor stream. (Trade-off: UDP is best-effort; we add a tiny
//   header + per-frame sequence so the receiver can detect loss. For guaranteed
//   delivery use TCP instead — one #define below.)
//
// It reads frames with shm_bridge::Reader (0% CPU futex wait), prepends a compact
// 32-byte wire header (magic, seq, dims, dtype, payload length), and sends them to
// a destination host:port. A frame larger than one datagram is fragmented across
// UDP packets carrying the same seq + a fragment index, and reassembled by a
// matching receiver (see examples/cpp/network_to_shm.cpp if you add one, or the
// Python receiver in examples/python/).
//
// Run (sender):
//   ros2 run shm_bridge_cpp ex_shm_to_network --ros-args \
//        -p stream:=rgb -p host:=192.168.1.50 -p port:=5005
#include <shm_bridge_cpp/shm_bridge.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// ---- wire header (little-endian, 32 bytes), matches the Python receiver ----
#pragma pack(push, 1)
struct WireHdr {
    uint32_t magic;       // 0x53484D31 = "SHM1"
    uint32_t seq;         // frame sequence
    uint32_t frag;        // fragment index within this frame
    uint32_t nfrags;      // total fragments for this frame
    uint32_t width;
    uint32_t height;
    uint32_t channels;    // low 8 bits channels; next 8 bits dtype_id
    uint32_t total_len;   // total payload bytes for the whole frame
};
#pragma pack(pop)
static_assert(sizeof(WireHdr) == 32, "WireHdr must be 32 bytes");

static constexpr uint32_t MAGIC = 0x53484D31u;
static constexpr size_t   MTU_PAYLOAD = 1400;   // safe UDP payload under typical 1500 MTU

int main(int argc, char** argv) {
    // Minimal arg parsing so this also builds/run outside ROS if you want.
    std::string stream = "rgb", host = "127.0.0.1";
    int port = 5005;
    for (int i = 1; i < argc - 1; ++i) {
        std::string a = argv[i];
        if (a == "--stream") stream = argv[++i];
        else if (a == "--host") host = argv[++i];
        else if (a == "--port") port = std::atoi(argv[++i]);
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);   // SOCK_STREAM for TCP (reliable)
    if (fd < 0) { perror("socket"); return 1; }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host.c_str(), &dst.sin_addr) != 1) {
        std::fprintf(stderr, "bad host %s\n", host.c_str()); return 1;
    }
    std::printf("shm_to_network: /dev/shm/%s_* -> udp://%s:%d\n",
                stream.c_str(), host.c_str(), port);

    shm_bridge::Reader reader(stream);    // throws if the stream isn't up yet
    shm_bridge::Frame f;
    std::vector<uint8_t> pkt(sizeof(WireHdr) + MTU_PAYLOAD);

    while (true) {
        if (!reader.wait_and_read(f, 0)) continue;   // block 0% CPU until new frame

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
            std::memcpy(pkt.data() + sizeof(h), f.data.data() + off, len);
            sendto(fd, pkt.data(), sizeof(h) + len, 0,
                   reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
        }
    }
    close(fd);   // unreachable in this loop; here for completeness
    return 0;
}
