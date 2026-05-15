#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class PressureToPose : public rclcpp::Node
{
public:
  PressureToPose()
  : Node("pressure_to_pose")
  {
    declare_parameter<std::string>("input_topic", "/cirtesub/sensors/pressure");
    declare_parameter<std::string>("output_topic", "/cirtesub/sensors/pressure/pose");
    declare_parameter<std::string>("frame_id", "odom");
    declare_parameter<std::string>("sensor_frame_id", "cirtesub/Pressure");
    declare_parameter<double>("fluid_density", 1000.0);
    declare_parameter<double>("gravity", 9.80665);
    declare_parameter<double>("pressure_offset_pa", 101325.0);
    declare_parameter<bool>("positive_down", false);
    declare_parameter<double>("fallback_z_variance", 0.01);
    declare_parameter<double>("unused_axis_variance", 1000000.0);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    fluid_density_ = get_parameter("fluid_density").as_double();
    gravity_ = get_parameter("gravity").as_double();
    pressure_offset_pa_ = get_parameter("pressure_offset_pa").as_double();
    positive_down_ = get_parameter("positive_down").as_bool();
    fallback_z_variance_ = get_parameter("fallback_z_variance").as_double();
    unused_axis_variance_ = get_parameter("unused_axis_variance").as_double();

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&PressureToPose::onPressure, this, std::placeholders::_1));
  }

private:
  void onPressure(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    if (fluid_density_ <= 0.0 || gravity_ <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Fluid density and gravity must be positive.");
      return;
    }

    const double depth = (msg->fluid_pressure - pressure_offset_pa_) /
      (fluid_density_ * gravity_);
    const double z = positive_down_ ? -depth : depth;

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = msg->header.stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.pose.position.z = z;
    pose.pose.pose.orientation.w = 1.0;

    for (size_t index = 0; index < 6; ++index) {
      pose.pose.covariance[index * 6 + index] = unused_axis_variance_;
    }

    if (msg->variance > 0.0 && std::isfinite(msg->variance)) {
      pose.pose.covariance[14] = msg->variance /
        ((fluid_density_ * gravity_) * (fluid_density_ * gravity_));
    } else {
      pose.pose.covariance[14] = fallback_z_variance_;
    }

    publisher_->publish(pose);
  }

  std::string frame_id_;
  double fluid_density_{1000.0};
  double gravity_{9.80665};
  double pressure_offset_pa_{101325.0};
  bool positive_down_{false};
  double fallback_z_variance_{0.01};
  double unused_axis_variance_{1000000.0};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureToPose>());
  rclcpp::shutdown();
  return 0;
}
