#include "erl_common/angle_utils.hpp"
#include "erl_common/ros2_topic_params.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    Ros2TopicParams point_cloud_topic{""};

    ERL_REFLECT_SCHEMA(Options, ERL_REFLECT_MEMBER(Options, point_cloud_topic));

    bool
    PostDeserialization() override {
        if (point_cloud_topic.path.empty()) {
            RCLCPP_ERROR(g_curr_node->get_logger(), "Point cloud topic is not specified.");
            return false;
        }
        return true;
    }
};

class Node : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

public:
    Node()
        : rclcpp::Node("get_lidar_info_from_point_cloud") {
        // Initialize the node and set up the subscriber
        g_curr_node = this;
        auto logger = this->get_logger();
        Options cfg;
        if (!cfg.LoadFromRos2(this, "")) {
            RCLCPP_ERROR(logger, "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded parameters:\n%s", cfg.AsYamlString().c_str());

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg.point_cloud_topic.path,
            cfg.point_cloud_topic.GetQoS(),
            std::bind(&Node::Callback, this, std::placeholders::_1));
        RCLCPP_INFO(
            logger,
            "Subscribed to point cloud topic: %s",
            cfg.point_cloud_topic.path.c_str());
        RCLCPP_INFO(logger, "Node initialized.");
    }

    void
    Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
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
        RCLCPP_INFO(
            this->get_logger(),
            "Azimuth range: [%f, %f], Elevation range: [%f, %f], Distance range: [%f, %f], Point "
            "cloud width: %d, height: %d",
            azimuth_min,
            azimuth_max,
            elevation_min,
            elevation_max,
            dist_min,
            dist_max,
            msg->width,
            msg->height);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Node shutdown.");
    return 0;
}
