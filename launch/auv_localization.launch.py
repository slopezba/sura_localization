import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_share = get_package_share_directory("sura_localization")
    config_file = os.path.join(package_share, "config", "ekf_auv.yaml")
    aruco_map_launch = os.path.join(
        get_package_share_directory("cirtesu_tank_aruco_localization"),
        "launch",
        "aruco_map_localization.launch.py",
    )

    output_odom_topic = LaunchConfiguration("output_odom_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_link_frame = LaunchConfiguration("base_link_frame")
    world_frame = LaunchConfiguration("world_frame")
    publish_tf = LaunchConfiguration("publish_tf")
    datum_latitude = float(LaunchConfiguration("datum_latitude").perform(context))
    datum_longitude = float(LaunchConfiguration("datum_longitude").perform(context))
    datum_heading = float(LaunchConfiguration("datum_heading").perform(context))

    frame_overrides = {
        "map_frame": map_frame,
        "odom_frame": odom_frame,
        "base_link_frame": base_link_frame,
        "world_frame": world_frame,
        "publish_tf": publish_tf,
    }

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_world_enu",
            output="screen",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "3.14159265359",
                "--pitch",
                "0.0",
                "--yaw",
                "1.57079632679",
                "--frame-id",
                "world_ned",
                "--child-frame-id",
                "world_enu",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_ned_to_cirtesu_tank",
            output="screen",
            arguments=[
                "--x",
                "0.0",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--roll",
                "0.0",
                "--pitch",
                "0.0",
                "--yaw",
                "3.1416",
                "--frame-id",
                "world_ned",
                "--child-frame-id",
                "cirtesu_tank",
            ],
        ),
        Node(
            package="sura_localization",
            executable="ned_to_enu_imu",
            name="imu_ned_to_enu",
            output="screen",
            parameters=[
                {
                    "input_topic": "/cirtesub/sensors/imu",
                    "output_topic": "/cirtesub/sensors/imu_enu",
                    "frame_id": "cirtesub/IMU",
                }
            ],
        ),
        Node(
            package="cirtesub_stonefish",
            executable="pressure_to_pose.py",
            name="pressure_to_pose",
            output="screen",
            parameters=[
                {
                    "input_topic": "/cirtesub/sensors/pressure",
                    "output_topic": "/cirtesub/sensors/pressure/pose",
                    "frame_id": "world_enu",
                    "sensor_frame_id": "cirtesub/Pressure",
                    "positive_down": True,
                    "fallback_z_variance": 0.01,
                }
            ],
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(aruco_map_launch)),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            parameters=[
                config_file,
                {
                    "wait_for_datum": True,
                    "datum": [datum_latitude, datum_longitude, datum_heading],
                },
            ],
            remappings=[
                ("gps/fix", "/cirtesub/sensors/gps"),
                ("imu", "/cirtesub/sensors/imu_enu"),
                ("odometry/filtered", output_odom_topic),
                ("odometry/gps", "/cirtesub/sensors/gps/odometry"),
                ("gps/filtered", "/cirtesub/sensors/gps/filtered"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[config_file, frame_overrides],
            remappings=[("odometry/filtered", output_odom_topic)],
        ),
        Node(
            package="sura_localization",
            executable="enu_to_ned_odometry",
            name="gps_enu_to_ned_odometry",
            output="screen",
            parameters=[
                {
                    "input_topic": output_odom_topic,
                    "output_topic": "/cirtesub/localization/odometry",
                    "frame_id": "world_ned",
                    "child_frame_id": "cirtesub/base_link",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_odom_topic",
                default_value="/cirtesub/localization/odometry_enu",
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="world_enu"),
            DeclareLaunchArgument("base_link_frame", default_value="cirtesub/base_link"),
            DeclareLaunchArgument("world_frame", default_value="world_enu"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument("datum_latitude", default_value="39.9944"),
            DeclareLaunchArgument("datum_longitude", default_value="-0.0741"),
            DeclareLaunchArgument("datum_heading", default_value="0.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
