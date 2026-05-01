#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for cow and lady dataset."""

    # Declare launch arguments
    cow_and_lady_bag_arg = DeclareLaunchArgument(
        "cow_and_lady_bag",
        default_value=os.path.expanduser("~/Data/CowAndLady/ros2_bag"),
        description="Path to the cow and lady bag directory",
    )
    gt_point_cloud_file_arg = DeclareLaunchArgument(
        "gt_point_cloud_file",
        default_value=os.path.expanduser("~/Data/CowAndLady/cow_and_lady_gt.ply"),
        description="Path to the ground truth point cloud file",
    )
    rosbag_play_rate_arg = DeclareLaunchArgument(
        "rosbag_play_rate",
        default_value="1.0",
        description="Rate at which to play the rosbag",
    )

    # Static transform publishers
    map_odom_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    map_vicon_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_vicon_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "vicon",
        ],
    )

    kinect_camera_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="kinect_camera_transformer",
        arguments=[
            "--x",
            "0.00114049",
            "--y",
            "0.0450936",
            "--z",
            "0.0430765",
            "--qx",
            "0.0924132",
            "--qy",
            "0.0976455",
            "--qz",
            "0.0702949",
            "--qw",
            "0.988425",
            "--frame-id",
            "kinect",
            "--child-frame-id",
            "camera_rgb_optical_frame",
        ],
    )

    # Transform to TF node
    transform_to_tf_node = Node(
        package="erl_common_ros",
        executable="transform_to_tf_node",
        name="transform_to_tf_node",
        output="screen",
        parameters=[{"transform_topic": "/kinect/vrpn_client/estimated_transform"}],
    )

    # Ground truth point cloud node
    gt_point_cloud_node = Node(
        package="erl_geometry_ros",
        executable="point_cloud_node",
        name="gt_point_cloud_node",
        output="screen",
        parameters=[
            {"frame_id": "map"},
            {"point_cloud_file": LaunchConfiguration("gt_point_cloud_file")},
            {"publish_colors": True},
        ],
    )

    # ROS bag play
    cmd = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("cow_and_lady_bag"),
        "--rate",
        LaunchConfiguration("rosbag_play_rate"),
        "--delay",
        "2",
        "--start-offset",
        "5",
        "--clock",
    ]
    # LTS: humble (2027 EOL), jazzy (2029 EOL)
    # humble does not support --playback-duration
    if os.environ.get("ROS_DISTRO") != "humble":
        cmd.append("--playback-duration")
        cmd.append("128")
    rosbag_play = ExecuteProcess(cmd=cmd, name="rosbag_play", output="screen")

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("erl_geometry_ros"), "rviz2", "cow_and_lady.rviz"])],
    )

    return LaunchDescription(
        [
            cow_and_lady_bag_arg,
            gt_point_cloud_file_arg,
            rosbag_play_rate_arg,
            map_odom_transformer,
            map_vicon_transformer,
            kinect_camera_transformer,
            transform_to_tf_node,
            gt_point_cloud_node,
            rosbag_play,
            rviz_node,
        ]
    )
