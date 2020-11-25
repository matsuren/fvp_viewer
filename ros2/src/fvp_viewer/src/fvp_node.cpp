#include "fvp_viewer/fvp_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <spdlog/spdlog.h>
#include <fvp_core/fvp/fvp_system.hpp>
#include <fvp_core/fvp/config.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using GLuint = unsigned int;

class FVPNode : public rclcpp::Node {
protected:
    // main fvp_system
    std::shared_ptr<fvp::System> system_;
    std::shared_ptr<fvp::Config> cfg_;
    // number of cameras
    const int cam_num_;
    // wall height
    const float wall_height_;

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    std::vector<image_transport::Subscriber> image_subs_;
    std::thread render_thread;

public:
    FVPNode(const int cam_num)
            : Node("fvp_node"), cam_num_(cam_num), wall_height_(3.0) {
        this->declare_parameter<std::string>("config", "../config_FVP_parameters.json");


        // Setup fvp system
        init_fvp_system();
        render_thread = start_rendering();

        // Setup callback
        using std::placeholders::_1;
        using std::placeholders::_2;
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 1000, std::bind(&FVPNode::laser_callback, this, _1));

        image_subs_.reserve(cam_num_);
        for (int i = 0; i < cam_num_; ++i) {
            const std::string topic_name = fmt::format("camera{}/image_raw", i);
            rmw_qos_profile_t custom_qos = rmw_qos_profile_default;  // Could be any of the profiles or completely custom
            auto func = std::bind(&FVPNode::image_callback, this, _1, i);
            auto sub = image_transport::create_subscription(this, topic_name, func, "raw",custom_qos);
            image_subs_.push_back(sub);
        }
    }

    void threadExit(){
        system_->threadExit();
        join();
    }
    void join(){
        render_thread.join();
    }
private:
    void init_fvp_system(){
        std::string cfg_fname;
        this->get_parameter("config", cfg_fname);
        cfg_ = std::make_shared<fvp::Config>(cfg_fname);
        system_ = std::make_shared<fvp::System>(cfg_);
    }

    std::thread start_rendering() {
        return std::thread(&FVPNode::render_worker, this);
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) const {
        RCLCPP_INFO(this->get_logger(), "Receive scan");
        std::vector<LRFPoint> LRF_data;
        LRF_data.clear();
        LRF_data.reserve(2000);
        const int range_size = msg_in->ranges.size();
        for (int i = 0; i < range_size; ++i) {
            float angle = msg_in->angle_min + (i * msg_in->angle_increment);
            float dist = msg_in->ranges[i];
            // Check if distance is valid
            if (dist < msg_in->range_min || dist > msg_in->range_max) continue;
            float x = dist * sin(angle);
            float y = -dist * cos(angle);
            LRF_data.emplace_back(x, y);
        }

        std::vector<float> vertices;
        std::vector<GLuint> elements;

        getLRFGLdata(LRF_data, vertices, elements, wall_height_);

        system_->updateMesh(vertices, elements);
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, const int cam_id) const {
        RCLCPP_INFO(this->get_logger(), "image callback: %d", cam_id);
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // TODO set roi in camera driver
//        cv::Rect roi(224, 224, 1600, 1600);
        cv::Rect roi(0, 0, 1600, 1600);
        // Update texture in fvp_system
        cv::Mat roi_img = cv_ptr->image(roi).clone();
        system_->updateImages(roi_img, cam_id);
    }

    void render_worker() {
        // Initialize GLFW and create window
        system_->initGLFW();
        // Loading sample images
        std::vector<cv::Mat> sample_imgs;
        for (int i = 0; i < cfg_->num_camera(); i++) {
            const std::string fname = cfg_->image_filenames(i);
            spdlog::info("Loading {}", fname);
            cv::Mat img = cv::imread(fname);
            if (img.empty()) {
                spdlog::error("Cannot open image file:{}", fname);
                throw std::runtime_error("No image file");
            }
            sample_imgs.push_back(img);
        }
        system_->initImages(sample_imgs);

        // Loading sample LRF data
        const int tmp_width = 3.0;
        std::vector<float> vertices = {
                0.0f, 2 * tmp_width, 0.0f, -tmp_width, -tmp_width,
                0.0f, tmp_width, -tmp_width, 0.0f,
        };
        std::vector<GLuint> elements = {0, 1, 2};

        system_->initMesh(vertices, elements);

        // Prepare scene
        system_->initScene();
        ///////////////////////////////
        // Enter the main loop
        system_->mainLoop();
        ///////////////////////////////
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Set logger
    // Runtime log levels
    spdlog::set_level(spdlog::level::info);
    //   spdlog::set_level(spdlog::level::trace);

    const int cam_num = 4;
    std::shared_ptr<FVPNode> main_node;
    try {
        main_node = std::make_shared<FVPNode>(cam_num);
        rclcpp::spin(main_node);
        RCLCPP_DEBUG(main_node->get_logger(), "FVPNode main_node->threadExit()");
        main_node->threadExit();
    } catch (const std::exception &e) {
        spdlog::error("Catch exception: {}", e.what());
        // Finish thread
        main_node->threadExit();
        rclcpp::shutdown();
        return -1;
    }
    RCLCPP_INFO(main_node->get_logger(), "FVPNode is closed properly");
    // Exit program
    return 0;
}