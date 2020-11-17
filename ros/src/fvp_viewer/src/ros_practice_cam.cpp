#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "ros/ros.h"

class ImageListenerNode {
  ros::NodeHandle _nh;
  const int _cam_id;
  image_transport::ImageTransport _it;
  image_transport::Subscriber _sub;
  const std::string winname;

 public:
  ImageListenerNode(const int cam_id)
      : _it(_nh),
        _cam_id(cam_id),
        winname("Image window" + std::to_string(_cam_id)) {
    const std::string topic_name =
        "/camera" + std::to_string(_cam_id) + "/image_raw";
    _sub = _it.subscribe(topic_name, 1, &ImageListenerNode::callback, this);
    cv::namedWindow(winname);
  }

  ~ImageListenerNode() { cv::destroyWindow(winname); }

  void callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

    // Update GUI Window
    cv::Mat tmp;
    cv::Rect roi(224, 224, 1600, 1600);
    cv::resize(cv_ptr->image(roi), tmp, cv::Size(), 0.5, 0.5);
    cv::imshow(winname, tmp);
//    cv::imshow(winname, cv_ptr->image);
    cv::waitKey(3);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  std::vector<ImageListenerNode> image_nodes;
  for (int cam_id = 0; cam_id < 1; ++cam_id) {
    image_nodes.emplace_back(cam_id);
  }

  ros::spin();
  return 0;
}