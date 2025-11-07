#include "erl_common/angle_utils.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

void
Callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Process the incoming point cloud message
    ROS_INFO("Received point cloud with %d points", msg->width * msg->height);
    // Here you can add code to extract information from the point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    float azimuth_min = M_PI;
    float azimuth_max = -M_PI;
    float elevation_min = M_PI / 2;
    float elevation_max = -M_PI / 2;
    float dist_min = std::numeric_limits<float>::max();
    float dist_max = -std::numeric_limits<float>::min();
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float azimuth, elevation;
        Eigen::Vector3f direction(*iter_x, *iter_y, *iter_z);
        float dist = direction.norm();
        dist_min = std::min(dist_min, dist);
        dist_max = std::max(dist_max, dist);
        direction /= dist;
        erl::common::DirectionToAzimuthElevation<float>(direction, azimuth, elevation);
        azimuth_min = std::min(azimuth_min, azimuth);
        azimuth_max = std::max(azimuth_max, azimuth);
        elevation_min = std::min(elevation_min, elevation);
        elevation_max = std::max(elevation_max, elevation);
    }
    ROS_INFO(
        "Azimuth range: [%f, %f], Elevation range: [%f, %f], Distance range: [%f, %f],Point cloud "
        "width: %d, height: %d",
        azimuth_min,
        azimuth_max,
        elevation_min,
        elevation_max,
        dist_min,
        dist_max,
        msg->width,
        msg->height);
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "get_lidar_info_from_point_cloud");
    ros::NodeHandle nh("~");

    std::string point_cloud_topic = "";
    nh.param("point_cloud_topic", point_cloud_topic, point_cloud_topic);
    if (point_cloud_topic.empty()) {
        ROS_ERROR("Point cloud topic is not specified.");
        return -1;
    }

    ros::Subscriber sub = nh.subscribe(point_cloud_topic, 1, Callback);
    ROS_INFO("Subscribed to point cloud topic: %s", point_cloud_topic.c_str());

    ros::spin();
    return 0;
}
