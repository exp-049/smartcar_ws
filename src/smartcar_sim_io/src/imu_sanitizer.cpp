#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class ImuSanitizer final : public rclcpp::Node {
public:
  ImuSanitizer() : Node("imu_sanitizer") {
    input_ = declare_parameter<std::string>("input_topic", "/smartcar/gz/imu");
    output_ = declare_parameter<std::string>("output_topic", "/car/imu/data_raw");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    pub_ = create_publisher<sensor_msgs::msg::Imu>(output_, rclcpp::SensorDataQoS());
    sub_ = create_subscription<sensor_msgs::msg::Imu>(input_, rclcpp::SensorDataQoS(),
      std::bind(&ImuSanitizer::on_imu, this, _1));
  }
private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    auto out = *msg;
    out.header.frame_id = frame_id_;
    // Gazebo can provide an orientation quaternion, but an MPU6050 has no
    // absolute yaw reference. Make that absence explicit to Cartographer.
    out.orientation.x = 0.0; out.orientation.y = 0.0;
    out.orientation.z = 0.0; out.orientation.w = 0.0;
    out.orientation_covariance[0] = -1.0;
    if (out.angular_velocity_covariance[0] == 0.0) {
      out.angular_velocity_covariance[0] = 1.0e-4;
      out.angular_velocity_covariance[4] = 1.0e-4;
      out.angular_velocity_covariance[8] = 1.0e-4;
    }
    if (out.linear_acceleration_covariance[0] == 0.0) {
      out.linear_acceleration_covariance[0] = 2.5e-3;
      out.linear_acceleration_covariance[4] = 2.5e-3;
      out.linear_acceleration_covariance[8] = 2.5e-3;
    }
    pub_->publish(out);
  }
  std::string input_, output_, frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<ImuSanitizer>()); rclcpp::shutdown(); return 0;
}
