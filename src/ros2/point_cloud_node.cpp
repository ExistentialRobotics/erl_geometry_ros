#include "erl_common/ros2_topic_params.hpp"

#include <open3d/io/PointCloudIO.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <filesystem>
#include <memory>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    std::string point_cloud_file = "data/pointcloud.pcd";
    std::string frame_id = "map";
    bool publish_normals = false;
    bool publish_colors = true;
    Ros2TopicParams point_cloud_topic{"point_cloud"};

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, point_cloud_file),
        ERL_REFLECT_MEMBER(Options, frame_id),
        ERL_REFLECT_MEMBER(Options, publish_normals),
        ERL_REFLECT_MEMBER(Options, publish_colors),
        ERL_REFLECT_MEMBER(Options, point_cloud_topic));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();
        if (point_cloud_file.empty()) {
            RCLCPP_ERROR(logger, "Point cloud file path is not specified.");
            return false;
        }
        if (!std::filesystem::exists(point_cloud_file)) {
            RCLCPP_ERROR(logger, "Point cloud file does not exist: %s", point_cloud_file.c_str());
            return false;
        }
        return true;
    }
};

class PointCloudNode : public rclcpp::Node {
    Options m_cfg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr m_point_cloud_msg_;

public:
    PointCloudNode()
        : Node("point_cloud_node") {
        auto logger = this->get_logger();
        RCLCPP_INFO(logger, "Starting PointCloudNode...");

        g_curr_node = this;
        if (!m_cfg_.LoadFromRos2(this, "")) {
            RCLCPP_ERROR(logger, "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded parameters:\n%s", m_cfg_.AsYamlString().c_str());

        // Load point cloud
        open3d::geometry::PointCloud point_cloud;
        if (!open3d::io::ReadPointCloud(m_cfg_.point_cloud_file, point_cloud)) {
            RCLCPP_ERROR(logger, "Failed to read from %s", m_cfg_.point_cloud_file.c_str());
            rclcpp::shutdown();
            return;
        }
        if (point_cloud.IsEmpty()) {
            RCLCPP_ERROR(logger, "Point cloud is empty.");
            rclcpp::shutdown();
            return;
        }

        // Check color availability
        if (m_cfg_.publish_colors && !point_cloud.HasColors()) {
            RCLCPP_WARN(
                logger,
                "Point cloud does not have colors, but publish_colors is set "
                "to true.");
            m_cfg_.publish_colors = false;  // Disable color publishing if not available
        }

        // Convert Open3D PointCloud to ROS2 PointCloud2 message
        auto msg_pcd = std::make_shared<sensor_msgs::msg::PointCloud2>();
        msg_pcd->header.frame_id = m_cfg_.frame_id;
        msg_pcd->header.stamp = this->now();
        msg_pcd->height = 1;                          // unorganized point cloud
        msg_pcd->width = point_cloud.points_.size();  // number of points in the cloud
        msg_pcd->is_bigendian = false;
        msg_pcd->is_dense = false;

        // Setup point cloud fields
        msg_pcd->fields.resize(3);
        msg_pcd->fields[0].name = "x";
        msg_pcd->fields[0].offset = 0;
        msg_pcd->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[0].count = 1;
        msg_pcd->fields[1].name = "y";
        msg_pcd->fields[1].offset = 4;
        msg_pcd->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[1].count = 1;
        msg_pcd->fields[2].name = "z";
        msg_pcd->fields[2].offset = 8;
        msg_pcd->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        msg_pcd->fields[2].count = 1;

        uint32_t offset = 12;
        if (m_cfg_.publish_colors && point_cloud.HasColors()) {
            msg_pcd->fields.resize(4);
            msg_pcd->fields[3].name = "rgb";
            msg_pcd->fields[3].offset = 12;
            msg_pcd->fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            msg_pcd->fields[3].count = 1;
            offset += 4;  // 4 bytes for RGB
        }

        uint32_t normal_offset = offset;  // for normals if needed
        if (m_cfg_.publish_normals) {
            point_cloud.EstimateNormals();
            sensor_msgs::msg::PointField field = msg_pcd->fields.back();
            field.datatype = sensor_msgs::msg::PointField::FLOAT32;
            field.count = 1;

            msg_pcd->fields.resize(msg_pcd->fields.size() + 3);

            field.name = "normal_x";
            field.offset = offset;
            msg_pcd->fields.push_back(field);

            field.name = "normal_y";
            field.offset += 4;
            msg_pcd->fields.push_back(field);

            field.name = "normal_z";
            field.offset += 4;
            msg_pcd->fields.push_back(field);

            offset += 12;
        }

        msg_pcd->point_step = offset;  // total size of one point
        msg_pcd->row_step = msg_pcd->point_step * point_cloud.points_.size();
        msg_pcd->data.resize(msg_pcd->row_step);

        // Fill the PointCloud2 message with point data
        uint8_t *ptr = msg_pcd->data.data();
        for (size_t i = 0; i < point_cloud.points_.size(); ++i) {
            Eigen::Vector3f point = point_cloud.points_[i].cast<float>();
            // x, y, z
            std::memcpy(
                reinterpret_cast<void *>(ptr),
                reinterpret_cast<const void *>(&point[0]),
                12);
            if (m_cfg_.publish_colors && point_cloud.HasColors()) {
                Eigen::Vector3f color = point_cloud.colors_[i].cast<float>();
                ptr[12] = static_cast<uint8_t>(color[2] * 255);
                ptr[13] = static_cast<uint8_t>(color[1] * 255);
                ptr[14] = static_cast<uint8_t>(color[0] * 255);
                ptr[15] = 255;
            }
            if (m_cfg_.publish_normals) {
                Eigen::Vector3f normal = point_cloud.normals_[i].cast<float>();
                std::memcpy(
                    reinterpret_cast<void *>(ptr + normal_offset),
                    reinterpret_cast<const void *>(&normal[0]),
                    12);
            }
            ptr += msg_pcd->point_step;  // move to the next point
        }

        // Create publisher
        m_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            m_cfg_.point_cloud_topic.path,
            m_cfg_.point_cloud_topic.GetQoS());

        // Publish the point cloud once
        m_publisher_->publish(*msg_pcd);

        RCLCPP_INFO(logger, "Published point cloud with %zu points", point_cloud.points_.size());

        // Store the message for republishing if needed
        m_point_cloud_msg_ = msg_pcd;

        // Create a timer to republish periodically (optional)
        m_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PointCloudNode::CallbackTimer, this));
    }

private:
    void
    CallbackTimer() {
        if (m_point_cloud_msg_) {
            m_point_cloud_msg_->header.stamp = this->now();
            m_publisher_->publish(*m_point_cloud_msg_);
        }
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
