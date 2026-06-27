#include <opencv2/opencv.hpp>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

// ------------------ helpers ------------------

inline bool valid_dims(int w, int h) {
    return (w > 0 && h > 0);
}

uint8_t* map_shm(const char* path, size_t& size) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("open");
        return nullptr;
    }

    size = lseek(fd, 0, SEEK_END);
    if (size == 0) {
        close(fd);
        return nullptr;
    }

    lseek(fd, 0, SEEK_SET);

    void* mem = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    close(fd);

    if (mem == MAP_FAILED) {
        perror("mmap");
        return nullptr;
    }

    return static_cast<uint8_t*>(mem);
}

// Very simple JSON value extractor (safe enough for our fixed format)
bool extract_int(const std::string& text, const std::string& key, int& value) {
    auto pos = text.find(key);
    if (pos == std::string::npos) return false;
    pos = text.find(":", pos);
    if (pos == std::string::npos) return false;
    value = std::stoi(text.substr(pos + 1));
    return true;
}

bool extract_size(const std::string& text, const std::string& key, size_t& value) {
    auto pos = text.find(key);
    if (pos == std::string::npos) return false;
    pos = text.find(":", pos);
    if (pos == std::string::npos) return false;
    value = static_cast<size_t>(std::stoll(text.substr(pos + 1)));
    return true;
}

// ------------------ main ------------------

int main() {
    size_t rgb_size = 0, depth_size = 0;

    uint8_t* rgb_mem   = map_shm("/dev/shm/rgb_frame", rgb_size);
    uint8_t* depth_mem = map_shm("/dev/shm/depth_frame", depth_size);

    if (!rgb_mem || !depth_mem) {
        std::cerr << "Failed to map shared memory buffers\n";
        return 1;
    }

    std::cout << "[SHM VIEWER] Started safely\n";

    while (true) {

        // -------- read metadata safely --------
        std::ifstream rgb_meta("/dev/shm/rgb_meta.json");
        std::ifstream depth_meta("/dev/shm/depth_meta.json");

        if (!rgb_meta || !depth_meta) {
            cv::waitKey(1);
            continue;
        }

        std::stringstream rgb_buf, depth_buf;
        rgb_buf << rgb_meta.rdbuf();
        depth_buf << depth_meta.rdbuf();

        std::string rgb_txt = rgb_buf.str();
        std::string depth_txt = depth_buf.str();

        int rw = 0, rh = 0, dw = 0, dh = 0;
        size_t rs = 0, ds = 0;

        if (!extract_int(rgb_txt, "width", rw) ||
            !extract_int(rgb_txt, "height", rh) ||
            !extract_size(rgb_txt, "data_size", rs) ||
            !extract_int(depth_txt, "width", dw) ||
            !extract_int(depth_txt, "height", dh) ||
            !extract_size(depth_txt, "data_size", ds)) {
            cv::waitKey(1);
            continue;
        }

        if (!valid_dims(rw, rh) || !valid_dims(dw, dh)) {
            cv::waitKey(1);
            continue;
        }

        if (rs > rgb_size || ds > depth_size) {
            cv::waitKey(1);
            continue;
        }

        // -------- create images --------
        cv::Mat rgb(rh, rw, CV_8UC3, rgb_mem);
        cv::Mat depth(dh, dw, CV_8UC3, depth_mem);

        if (rgb.empty() || depth.empty()) {
            cv::waitKey(1);
            continue;
        }

        // -------- resize depth safely --------
        cv::Mat depth_resized;
        cv::resize(
            depth,
            depth_resized,
            cv::Size(rgb.cols, rgb.rows),
            0, 0,
            cv::INTER_NEAREST
        );

        // -------- combine --------
        cv::Mat combined;
        cv::hconcat(rgb, depth_resized, combined);

        // -------- display --------
        cv::imshow("RGB", rgb);
        cv::imshow("Depth Heatmap", depth_resized);
        cv::imshow("Combined", combined);

        // ESC to exit
        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}
