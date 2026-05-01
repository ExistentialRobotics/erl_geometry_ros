#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for basic Gazebo room 2D simulation."""

    # Gazebo room 2D node
    gazebo_room_2d_node = Node(
        package="erl_geometry_ros",
        executable="gazebo_room_2d_node",
        name="gazebo_room_2d_node",
        output="screen",
        parameters=[
            {
                "data_folder": PathJoinSubstitution([FindPackageShare("erl_geometry"), "data", "gazebo"]),
                "laser_frame": "front_laser",
                "map_frame": "map",
                "laser_topic.path": "scan",
                "publish_rate": 100.0,
            }
        ],
    )

    # Static transform publisher (map -> odom)
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

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("erl_geometry_ros"), "rviz2", "gazebo_room_2d.rviz"])],
    )

    return LaunchDescription(
        [
            gazebo_room_2d_node,
            map_odom_transformer,
            rviz_node,
        ]
    )
