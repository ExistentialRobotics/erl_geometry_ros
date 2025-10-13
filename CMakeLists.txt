cmake_minimum_required(VERSION 3.16)

project(erl_geometry_ros #
        LANGUAGES CXX C #
        VERSION 0.2.0 #
        DESCRIPTION "erl_geometry_ros is a ROS package of erl_geometry.")
message(STATUS "Configuring ${PROJECT_NAME} ${PROJECT_VERSION}")

if (NOT COMMAND erl_project_setup)
    find_package(erl_cmake_tools REQUIRED)
endif ()

erl_project_setup(ERL_PACKAGES erl_geometry)
erl_setup_ros(
    CATKIN_COMPONENTS geometry_msgs nav_msgs sensor_msgs tf2_ros #
    CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs tf2_ros #
    ROS2_COMPONENTS geometry_msgs nav_msgs sensor_msgs tf2_ros)
erl_add_ros_src()
erl_install(EXECUTABLES ${${PROJECT_NAME}_COLLECTED_EXECUTABLES})
erl_mark_project_found()
