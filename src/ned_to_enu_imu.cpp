#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & left,
  const geometry_msgs::msg::Quaternion & right)
{
  geometry_msgs::msg::Quaternion result;
  result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
  result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
  result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
  result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
  return result;
}

geometry_msgs::msg::Quaternion normalize(geometry_msgs::msg::Quaternion quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w);
  if (norm <= 0.0) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }

  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

geometry_msgs::msg::Vector3 transformVector(const geometry_msgs::msg::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 converted;
  converted.x = vector.y;
  converted.y = vector.x;
  converted.z = -vector.z;
  return converted;
}

std::array<double, 9> transformCovariance3x3(const std::array<double, 9> & covariance)
{
  return {
    covariance[4], covariance[3], -covariance[5],
    covariance[1], covariance[0], -covariance[2],
    -covariance[7], -covariance[6], covariance[8]};
}

}  // namespace

class NedToEnuImu : public rclcpp::Node
{
public:
  NedToEnuImu()
  : Node("ned_to_enu_imu")
  {
    declare_parameter<std::string>("input_topic", "/cirtesub/sensors/imu");
    declare_parameter<std::string>("output_topic", "/cirtesub/sensors/imu_enu");
    declare_parameter<std::string>("frame_id", "cirtesub/IMU");

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&NedToEnuImu::onImu, this, std::placeholders::_1));

    conversion_quaternion_.x = std::sqrt(0.5);
    conversion_quaternion_.y = std::sqrt(0.5);
    conversion_quaternion_.z = 0.0;
    conversion_quaternion_.w = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "Converting NED IMU %s to ENU %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu converted;
    converted.header = msg->header;
    converted.header.frame_id = frame_id_;
    converted.orientation = normalize(multiply(conversion_quaternion_, msg->orientation));
    converted.orientation_covariance = transformCovariance3x3(msg->orientation_covariance);
    converted.angular_velocity = transformVector(msg->angular_velocity);
    converted.angular_velocity_covariance = transformCovariance3x3(
      msg->angular_velocity_covariance);
    converted.linear_acceleration = transformVector(msg->linear_acceleration);
    converted.linear_acceleration_covariance = transformCovariance3x3(
      msg->linear_acceleration_covariance);
    publisher_->publish(converted);
  }

  std::string frame_id_;
  geometry_msgs::msg::Quaternion conversion_quaternion_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NedToEnuImu>());
  rclcpp::shutdown();
  return 0;
}
