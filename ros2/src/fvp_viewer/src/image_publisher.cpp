#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
            : Node("minimal_publisher"), count_(0)
    {
        publisher_ = image_transport::create_publisher(this, "camera0/image_raw");
        timer_ = this->create_wall_timer(
                1000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        sensor_msgs::msg::Image::SharedPtr img;
        std::string fname = "/home/komatsu/work/fromGitHub/fvp_viewer/data/img" + std::to_string(count_%4) + ".jpg";
        RCLCPP_INFO(this->get_logger(), "Publish image:%s", fname.c_str());
        cv::Mat src = cv::imread(fname);
        img = cv_bridge::CvImage(std_msgs::msg::Header(),  sensor_msgs::image_encodings::BGR8, src).toImageMsg();
        img->width = src.cols;
        img->height = src.rows;
        img->step = src.cols * src.channels();
        img->header.frame_id = "camera";
        img->header.stamp = rclcpp::Time();

        publisher_.publish(img);
        count_ ++;
    }
    int count_;
    rclcpp::TimerBase::SharedPtr timer_;
    image_transport::Publisher publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}