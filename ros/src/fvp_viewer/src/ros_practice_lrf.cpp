#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Point32.h"

using GLuint = unsigned int;
class LaserScanListenerNode {
 protected:
  // Our NodeHandle
  ros::NodeHandle _nh;

  // Components for publishing
  ros::Publisher output_pub_;
  ros::Subscriber _sub;

 public:
  // Constructor
  LaserScanListenerNode() {
    _sub = _nh.subscribe("scan", 1000, &LaserScanListenerNode::callback, this);
//    output_pub_ = _nh.advertise<geometry_msgs::Point32>("output", 1000);
    output_pub_ = _nh.advertise<sensor_msgs::LaserScan>("output", 1000);
  }

  // Callback
  void callback(const sensor_msgs::LaserScan::ConstPtr &msg_in) {
    ROS_INFO("Receive scan angle_increment %f", msg_in->angle_increment);
    ROS_INFO("Receive scan angle_min %f", msg_in->angle_min);
    ROS_INFO("Receive scan angle_max %f", msg_in->angle_max);
    ROS_INFO("Receive scan ranges[0] %f", msg_in->ranges[0]);
    ROS_INFO("Receive scan size %d", int(msg_in->ranges.size()));
    ROS_INFO("Receive scan range_max %f", msg_in->range_max);
    ROS_INFO("Receive scan range_min %f", msg_in->range_min);

    // Publish the output
    sensor_msgs::LaserScan msg_out(*msg_in);
    msg_out.ranges.clear();
    float new_angle;
    for (int i = 0; i < (msg_in->ranges.size()/4); ++i) {
      new_angle = msg_in->angle_min + (i * msg_in->angle_increment);
      msg_out.ranges.push_back(msg_in->ranges[i]);
    }
    msg_out.angle_max = new_angle;
    output_pub_.publish(msg_out);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "fvp_node");
  LaserScanListenerNode scan_node;

  ros::MultiThreadedSpinner spinner(5);  // Use multiple threads
  spinner.spin();  // spin() will not return until the node has been shutdown

  // Exit program
  return 0;
}