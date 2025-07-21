message(STATUS "ROS2 activated, building ROS2 stuff")

# node: gazebo_room_2d_node
add_executable(gazebo_room_2d_node src/ros2/gazebo_room_2d_node.cpp)
erl_target_dependencies(gazebo_room_2d_node)
erl_collect_targets(EXECUTABLES gazebo_room_2d_node)

# node: point_cloud_node
add_executable(point_cloud_node src/ros2/point_cloud_node.cpp)
erl_target_dependencies(point_cloud_node)
erl_collect_targets(EXECUTABLES point_cloud_node)
