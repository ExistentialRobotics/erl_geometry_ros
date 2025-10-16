from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="erl_geometry_ros",
                executable="house_expo_2d_node",
                name="house_expo_2d_node",
                output="screen",
                parameters=[
                    {
                        "map_file": PathJoinSubstitution(
                            [
                                FindPackageShare("erl_geometry"),
                                "data",
                                "house_expo_room_1451.json",
                            ]
                        ),
                        "path_file": PathJoinSubstitution(
                            [
                                FindPackageShare("erl_geometry"),
                                "data",
                                "house_expo_room_1451.csv",
                            ]
                        ),
                        "publish_pose": True,
                        "pose_publish_rate": 40.0,
                        "path_feed_rate": 20.0,
                        "init_x": 6.61,
                        "init_y": 6.46,
                        "init_yaw": 0.0,
                        "path_topic": "path",
                        "loaded_path_topic_durability": "transient_local",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("erl_geometry_ros"),
                            "rviz2",
                            "house_expo_2d.rviz",
                        ]
                    ),
                ],
            ),
        ]
    )
