#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure


class PressureToPose(Node):
    def __init__(self):
        super().__init__("pressure_to_pose")

        self.declare_parameter("input_topic", "/cirtesub/sensors/pressure")
        self.declare_parameter("output_topic", "/cirtesub/sensors/pressure/pose")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("sensor_frame_id", "cirtesub/Pressure")
        self.declare_parameter("fluid_density", 1000.0)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("pressure_offset_pa", 101325.0)
        self.declare_parameter("positive_down", False)
        self.declare_parameter("fallback_z_variance", 0.01)
        self.declare_parameter("unused_axis_variance", 1000000.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            output_topic,
            10,
        )
        self.create_subscription(FluidPressure, input_topic, self._pressure_callback, 10)

    def _pressure_callback(self, msg: FluidPressure) -> None:
        density = float(self.get_parameter("fluid_density").value)
        gravity = float(self.get_parameter("gravity").value)
        if density <= 0.0 or gravity <= 0.0:
            self.get_logger().warn("Fluid density and gravity must be positive.")
            return

        pressure_offset_pa = float(self.get_parameter("pressure_offset_pa").value)
        depth = (msg.fluid_pressure - pressure_offset_pa) / (density * gravity)
        z = -depth if bool(self.get_parameter("positive_down").value) else depth

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = str(self.get_parameter("frame_id").value)
        pose.pose.pose.position.z = z
        pose.pose.pose.orientation.w = 1.0

        unused_variance = float(self.get_parameter("unused_axis_variance").value)
        covariance = [0.0] * 36
        for index in range(6):
            covariance[index * 6 + index] = unused_variance

        pressure_variance = msg.variance
        if pressure_variance > 0.0 and math.isfinite(pressure_variance):
            z_variance = pressure_variance / ((density * gravity) ** 2)
        else:
            z_variance = float(self.get_parameter("fallback_z_variance").value)

        covariance[14] = z_variance
        pose.pose.covariance = covariance

        self._publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PressureToPose()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
