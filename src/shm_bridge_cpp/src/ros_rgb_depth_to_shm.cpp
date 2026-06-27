#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/opencv.hpp>

#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>

using std::placeholders::_1;

// ---------------- CONFIG ----------------
static constexpr int DOWNSAMPLE = 4;

static constexpr size_t MAX_RGB_SIZE   = 1920 * 1080 * 3;
static constexpr size_t MAX_DEPTH_VIZ  = (1920 / DOWNSAMPLE) * (1080 / DOWNSAMPLE) * 3;
static constexpr size_t MAX_DEPTH_RAW  = 1920 * 1080 * sizeof(float);
// ---------------------------------------


// ---------- RealSense D455 intrinsics (RGB 1280x720) ----------
static constexpr float FX = 909.7f;
static constexpr float FY = 909.7f;
static constexpr float CX = 639.5f;
static constexpr float CY = 359.5f;
// ------------------------------------------------------------


// ---------- Linear RGB -> sRGB ----------
static inline cv::Mat linear_to_srgb(const cv::Mat& linear_rgb) {
    cv::Mat f32, srgb;
    linear_rgb.convertTo(f32, CV_32F, 1.0 / 255.0);
    cv::pow(f32, 1.0 / 2.2, srgb);
    srgb.convertTo(srgb, CV_8U, 255.0);
    return srgb;
}


class RGBDepthSHM : public rclcpp::Node {
public:
    RGBDepthSHM() : Node("rgb_depth_to_shm_cpp") {

        // ---------- RGB SHM ----------
        rgb_fd_ = open("/dev/shm/rgb_frame", O_CREAT | O_RDWR, 0666);
        ftruncate(rgb_fd_, MAX_RGB_SIZE);
        rgb_mem_ = static_cast<uint8_t*>(
            mmap(nullptr, MAX_RGB_SIZE, PROT_WRITE, MAP_SHARED, rgb_fd_, 0)
        );

        // ---------- DEPTH VIZ SHM ----------
        depth_viz_fd_ = open("/dev/shm/depth_frame", O_CREAT | O_RDWR, 0666);
        ftruncate(depth_viz_fd_, MAX_DEPTH_VIZ);
        depth_viz_mem_ = static_cast<uint8_t*>(
            mmap(nullptr, MAX_DEPTH_VIZ, PROT_WRITE, MAP_SHARED, depth_viz_fd_, 0)
        );

        // ---------- DEPTH RAW SHM ----------
        depth_raw_fd_ = open("/dev/shm/depth_raw", O_CREAT | O_RDWR, 0666);
        ftruncate(depth_raw_fd_, MAX_DEPTH_RAW);
        depth_raw_mem_ = static_cast<float*>(
            mmap(nullptr, MAX_DEPTH_RAW, PROT_WRITE, MAP_SHARED, depth_raw_fd_, 0)
        );

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgb", 1, std::bind(&RGBDepthSHM::rgb_cb, this, _1));

        depth_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_pcl", 1, std::bind(&RGBDepthSHM::depth_cb, this, _1));

       // RCLCPP_INFO(get_logger(), "RGB + Depth SHM bridge (NO tilt correction)");
    }

private:
    // ================= RGB =================
    void rgb_cb(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto t0 = std::chrono::high_resolution_clock::now();

        // Wrap incoming image (assumed RGB8 linear)
        cv::Mat rgb_linear(
            msg->height, msg->width,
            CV_8UC3,
            const_cast<uint8_t*>(msg->data.data())
        );

        // Linear → sRGB
        cv::Mat rgb_srgb = linear_to_srgb(rgb_linear);

        // RGB → BGR for OpenCV
        cv::Mat bgr;
        cv::cvtColor(rgb_srgb, bgr, cv::COLOR_RGB2BGR);

        size_t size = bgr.total() * 3;
        std::memcpy(rgb_mem_, bgr.data, size);

        std::ofstream meta("/dev/shm/rgb_meta.json");
        meta << "{"
             << "\"width\":" << msg->width << ","
             << "\"height\":" << msg->height << ","
             << "\"encoding\":\"bgr8_srgb\","
             << "\"data_size\":" << size
             << "}";
        meta.close();

        auto t1 = std::chrono::high_resolution_clock::now();
        /*RCLCPP_INFO(
            get_logger(),
            "[RGB] %.2f MB | %.3f ms",
            size / 1e6,
            std::chrono::duration<double, std::milli>(t1 - t0).count()
        );

        */
    }

    // ================= DEPTH =================
    void depth_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto t0 = std::chrono::high_resolution_clock::now();

        int h = msg->height;
        int w = msg->width;

        if (h <= 1) {
            int side = static_cast<int>(
                std::sqrt(msg->data.size() / msg->point_step)
            );
            h = side;
            w = side;
        }

        const uint8_t* ptr = msg->data.data();

        // ---------- RAW DEPTH (float32 meters) ----------
        for (int i = 0; i < h * w; ++i) {
            float z;
            std::memcpy(&z, ptr + i * msg->point_step + 8, sizeof(float));
            depth_raw_mem_[i] = std::isfinite(z) ? z : 0.0f;
        }

        // ---------- RAW META ----------
        std::ofstream raw_meta("/dev/shm/depth_raw_meta.json");
        raw_meta << "{"
                 << "\"width\":" << w << ","
                 << "\"height\":" << h << ","
                 << "\"dtype\":\"float32\","
                 << "\"units\":\"meters\","
                 << "\"data_size\":" << (w * h * sizeof(float)) << ","
                 << "\"intrinsic\":["
                 << "[" << FX << ",0," << CX << "],"
                 << "[0," << FY << "," << CY << "],"
                 << "[0,0,1]"
                 << "]"
                 << "}";
        raw_meta.close();

        // ---------- DEPTH VIZ ----------
        cv::Mat depth(h, w, CV_32F, depth_raw_mem_);

        cv::Mat depth_ds;
        cv::resize(
            depth, depth_ds,
            cv::Size(w / DOWNSAMPLE, h / DOWNSAMPLE),
            0, 0, cv::INTER_NEAREST
        );

        double minv, maxv;
        cv::minMaxLoc(depth_ds, &minv, &maxv);

        cv::Mat norm, heatmap;
        if (maxv > minv) {
            depth_ds.convertTo(
                norm, CV_8U,
                255.0 / (maxv - minv),
                -minv * 255.0 / (maxv - minv)
            );
        } else {
            norm = cv::Mat::zeros(depth_ds.size(), CV_8U);
        }

        cv::applyColorMap(norm, heatmap, cv::COLORMAP_JET);

        size_t viz_size = heatmap.total() * 3;
        std::memcpy(depth_viz_mem_, heatmap.data, viz_size);

        std::ofstream viz_meta("/dev/shm/depth_meta.json");
        viz_meta << "{"
                 << "\"width\":" << heatmap.cols << ","
                 << "\"height\":" << heatmap.rows << ","
                 << "\"encoding\":\"bgr8\","
                 << "\"downsample\":" << DOWNSAMPLE << ","
                 << "\"data_size\":" << viz_size
                 << "}";
        viz_meta.close();

        auto t1 = std::chrono::high_resolution_clock::now();
        /*RCLCPP_INFO(
            get_logger(),
            "[DEPTH] raw %.2f MB + viz %.2f MB | %.3f ms",
            (w * h * sizeof(float)) / 1e6,
            viz_size / 1e6,
            std::chrono::duration<double, std::milli>(t1 - t0).count()
        ); */
    }

    // ---------- SHM ----------
    int rgb_fd_, depth_viz_fd_, depth_raw_fd_;
    uint8_t* rgb_mem_;
    uint8_t* depth_viz_mem_;
    float* depth_raw_mem_;

    // ---------- ROS ----------
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBDepthSHM>());
    rclcpp::shutdown();
    return 0;
}
