#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class OdomRepublisher final : public rclcpp::Node {
public:
  OdomRepublisher() : Node("odom_republisher") {
    input_ = declare_parameter<std::string>("input_topic", "/smartcar/gz/odom");
    output_ = declare_parameter<std::string>("output_topic", "/car/chassis/odom");
    frame_id_ = declare_parameter<std::string>("frame_id", "chassis_odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_, rclcpp::QoS(20).reliable());
    // Accept either Gazebo bridge reliability profile, then expose a stable
    // reliable ROS interface to Cartographer and Nav2.
    sub_ = create_subscription<nav_msgs::msg::Odometry>(input_, rclcpp::QoS(20).best_effort(),
      std::bind(&OdomRepublisher::on_odom, this, _1));
  }
private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    auto out = *msg;
    out.header.frame_id = frame_id_;
    out.child_frame_id = child_frame_id_;
    pub_->publish(out);
  }
  std::string input_, output_, frame_id_, child_frame_id_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<OdomRepublisher>()); rclcpp::shutdown(); return 0;
}
