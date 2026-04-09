#include "erl_common/ros2_topic_params.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    Ros2TopicParams in_topic{"point_cloud_in"};
    Ros2TopicParams out_topic{"point_cloud_out"};
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    std::string new_frame_id;

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, in_topic),
        ERL_REFLECT_MEMBER(Options, out_topic),
        ERL_REFLECT_MEMBER(Options, transform),
        ERL_REFLECT_MEMBER(Options, new_frame_id));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();
        if (in_topic.path.empty()) {
            RCLCPP_ERROR(logger, "in_topic.path is empty");
            return false;
        }
        if (out_topic.path.empty()) {
            RCLCPP_ERROR(logger, "out_topic.path is empty");
            return false;
        }
        return true;
    }
};

class PointCloudTransformNode : public rclcpp::Node {
    Options m_cfg_;
    Eigen::Matrix4f m_transform_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher_;

public:
    PointCloudTransformNode()
        : Node("point_cloud_transform_node") {
        auto logger = this->get_logger();
        RCLCPP_INFO(logger, "Starting PointCloudTransformNode...");

        g_curr_node = this;
        if (!m_cfg_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(logger, "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(logger, "Loaded parameters:\n%s", m_cfg_.AsYamlString().c_str());

        m_transform_ = m_cfg_.transform.cast<float>();

        m_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            m_cfg_.out_topic.path,
            m_cfg_.out_topic.GetQoS());

        m_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            m_cfg_.in_topic.path,
            m_cfg_.in_topic.GetQoS(),
            std::bind(&PointCloudTransformNode::CallbackPointCloud, this, std::placeholders::_1));
    }

private:
    void
    CallbackPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        auto out_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);

        if (!m_cfg_.new_frame_id.empty()) { out_msg->header.frame_id = m_cfg_.new_frame_id; }

        // Find xyz field offsets
        int x_offset = -1;
        int y_offset = -1;
        int z_offset = -1;
        for (const auto &field: out_msg->fields) {
            if (field.name == "x") { x_offset = static_cast<int>(field.offset); }
            if (field.name == "y") { y_offset = static_cast<int>(field.offset); }
            if (field.name == "z") { z_offset = static_cast<int>(field.offset); }
        }

        if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Point cloud missing x/y/z fields, publishing without transform.");
            m_publisher_->publish(*out_msg);
            return;
        }

        // Transform each point in place
        const uint32_t point_step = out_msg->point_step;
        const uint32_t num_points = out_msg->width * out_msg->height;
        uint8_t *data = out_msg->data.data();

        for (uint32_t i = 0; i < num_points; ++i) {
            uint8_t *ptr = data + i * point_step;
            float x, y, z;
            std::memcpy(&x, ptr + x_offset, sizeof(float));
            std::memcpy(&y, ptr + y_offset, sizeof(float));
            std::memcpy(&z, ptr + z_offset, sizeof(float));

            Eigen::Vector4f pt(x, y, z, 1.0f);
            Eigen::Vector4f pt_transformed = m_transform_ * pt;

            std::memcpy(ptr + x_offset, &pt_transformed[0], sizeof(float));
            std::memcpy(ptr + y_offset, &pt_transformed[1], sizeof(float));
            std::memcpy(ptr + z_offset, &pt_transformed[2], sizeof(float));
        }

        m_publisher_->publish(*out_msg);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudTransformNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
