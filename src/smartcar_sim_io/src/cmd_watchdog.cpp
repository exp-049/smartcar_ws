#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CmdWatchdog final : public rclcpp::Node {
public:
  CmdWatchdog() : Node("cmd_watchdog") {
    input_ = declare_parameter<std::string>("input_topic", "/car/chassis/cmd_vel");
    output_ = declare_parameter<std::string>("output_topic", "/smartcar/gz/cmd_vel");
    timeout_ = declare_parameter<double>("timeout_sec", 0.5);
    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_, rclcpp::QoS(10).reliable());
    sub_ = create_subscription<geometry_msgs::msg::Twist>(input_, rclcpp::QoS(10).reliable(),
      std::bind(&CmdWatchdog::on_cmd, this, _1));
    timer_ = create_wall_timer(20ms, std::bind(&CmdWatchdog::tick, this));
  }
private:
  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg) { last_ = *msg; last_time_ = std::chrono::steady_clock::now(); have_cmd_ = true; }
  void tick() {
    geometry_msgs::msg::Twist out;
    const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_time_).count();
    if (have_cmd_ && age <= timeout_) out = last_;
    pub_->publish(out);
  }
  std::string input_, output_; double timeout_{0.5}; bool have_cmd_{false};
  geometry_msgs::msg::Twist last_{}; std::chrono::steady_clock::time_point last_time_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CmdWatchdog>()); rclcpp::shutdown(); return 0;
}
