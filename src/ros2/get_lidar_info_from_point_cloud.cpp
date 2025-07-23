#include "erl_common/angle_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class Node : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

public:
    Node()
        : rclcpp::Node("get_lidar_info_from_point_cloud") {
        // Initialize the node and set up the subscriber
        RCLCPP_INFO(this->get_logger(), "Node initialized.");
        std::string point_cloud_topic =
            this->declare_parameter<std::string>("point_cloud_topic", "");
        if (point_cloud_topic.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Point cloud topic is not specified.");
            return;
        }
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            point_cloud_topic,
            10,
            std::bind(&Node::Callback, this, std::placeholders::_1));
        RCLCPP_INFO(
            this->get_logger(),
            "Subscribed to point cloud topic: %s",
            point_cloud_topic.c_str());
    }

    void
    Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        // Process the incoming point cloud message
        RCLCPP_INFO(
            this->get_logger(),
            "Received point cloud with %d points",
            msg->width * msg->height);
        // Here you can add code to extract information from the point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        float azimuth_min = M_PI;
        float azimuth_max = -M_PI;
        float elevation_min = M_PI / 2;
        float elevation_max = -M_PI / 2;
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float azimuth, elevation;
            Eigen::Vector3f direction(*iter_x, *iter_y, *iter_z);
            erl::common::DirectionToAzimuthElevation<float>(direction, azimuth, elevation);
            azimuth_min = std::min(azimuth_min, azimuth);
            azimuth_max = std::max(azimuth_max, azimuth);
            elevation_min = std::min(elevation_min, elevation);
            elevation_max = std::max(elevation_max, elevation);
        }
        RCLCPP_INFO(
            this->get_logger(),
            "Azimuth range: [%f, %f], Elevation range: [%f, %f], Point cloud width: %d, height: %d",
            azimuth_min,
            azimuth_max,
            elevation_min,
            elevation_max,
            msg->width,
            msg->height);
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Node shutdown.");
    return 0;
}
