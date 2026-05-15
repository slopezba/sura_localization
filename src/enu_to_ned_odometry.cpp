#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

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

std::array<double, 36> transformPoseCovariance(const std::array<double, 36> & covariance)
{
  constexpr std::array<int, 6> perm{1, 0, 2, 4, 3, 5};
  constexpr std::array<double, 6> sign{1.0, 1.0, -1.0, 1.0, 1.0, -1.0};

  std::array<double, 36> converted{};
  for (size_t row = 0; row < 6; ++row) {
    for (size_t col = 0; col < 6; ++col) {
      converted[row * 6 + col] =
        sign[row] * sign[col] * covariance[perm[row] * 6 + perm[col]];
    }
  }
  return converted;
}

std::array<double, 36> transformTwistAngularCovariance(
  const std::array<double, 36> & covariance)
{
  constexpr std::array<int, 6> perm{0, 1, 2, 4, 3, 5};
  constexpr std::array<double, 6> sign{1.0, 1.0, 1.0, 1.0, 1.0, -1.0};

  std::array<double, 36> converted{};
  for (size_t row = 0; row < 6; ++row) {
    for (size_t col = 0; col < 6; ++col) {
      converted[row * 6 + col] =
        sign[row] * sign[col] * covariance[perm[row] * 6 + perm[col]];
    }
  }
  return converted;
}

}  // namespace

class EnuToNedOdometry : public rclcpp::Node
{
public:
  EnuToNedOdometry()
  : Node("enu_to_ned_odometry")
  {
    declare_parameter<std::string>("input_topic", "/cirtesub/sensors/gps/odometry");
    declare_parameter<std::string>("output_topic", "/cirtesub/sensors/gps/odometry_ned");
    declare_parameter<std::string>("frame_id", "world_ned");
    declare_parameter<std::string>("child_frame_id", "");

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    child_frame_id_ = get_parameter("child_frame_id").as_string();

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&EnuToNedOdometry::onOdometry, this, std::placeholders::_1));

    conversion_quaternion_.x = std::sqrt(0.5);
    conversion_quaternion_.y = std::sqrt(0.5);
    conversion_quaternion_.z = 0.0;
    conversion_quaternion_.w = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "Converting ENU odometry %s to NED %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry converted;
    converted.header = msg->header;
    converted.header.frame_id = frame_id_;
    converted.child_frame_id = child_frame_id_;

    converted.pose.pose.position.x = msg->pose.pose.position.y;
    converted.pose.pose.position.y = msg->pose.pose.position.x;
    converted.pose.pose.position.z = -msg->pose.pose.position.z;
    converted.pose.pose.orientation = normalize(
      multiply(conversion_quaternion_, msg->pose.pose.orientation));
    converted.pose.covariance = transformPoseCovariance(msg->pose.covariance);

    converted.twist = msg->twist;
    converted.twist.twist.angular.x = msg->twist.twist.angular.y;
    converted.twist.twist.angular.y = msg->twist.twist.angular.x;
    converted.twist.twist.angular.z = -msg->twist.twist.angular.z;
    converted.twist.covariance = transformTwistAngularCovariance(msg->twist.covariance);

    publisher_->publish(converted);
  }

  std::string frame_id_;
  std::string child_frame_id_;
  geometry_msgs::msg::Quaternion conversion_quaternion_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnuToNedOdometry>());
  rclcpp::shutdown();
  return 0;
}
