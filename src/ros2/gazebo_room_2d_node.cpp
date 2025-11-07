#include "erl_common/ros2_topic_params.hpp"
#include "erl_geometry/gazebo_room_2d.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <memory>

using namespace erl::common;
using namespace erl::common::ros_params;
using namespace erl::geometry;

static rclcpp::Node *g_curr_node = nullptr;

struct Options : public Yamlable<Options> {
    std::string data_folder = "data/gazebo";
    std::string laser_frame = "front_laser";
    std::string map_frame = "map";
    Ros2TopicParams laser_topic{"scan"};
    Ros2TopicParams pose_topic{"pose"};
    Ros2TopicParams path_topic{"path"};
    double publish_rate = 10.0;  // Hz

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, data_folder),
        ERL_REFLECT_MEMBER(Options, laser_frame),
        ERL_REFLECT_MEMBER(Options, map_frame),
        ERL_REFLECT_MEMBER(Options, laser_topic),
        ERL_REFLECT_MEMBER(Options, pose_topic),
        ERL_REFLECT_MEMBER(Options, path_topic),
        ERL_REFLECT_MEMBER(Options, publish_rate));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();
        if (data_folder.empty()) {
            RCLCPP_ERROR(logger, "data_folder is empty.");
            return false;
        }
        if (laser_frame.empty()) {
            RCLCPP_ERROR(logger, "laser_frame is empty.");
            return false;
        }
        if (map_frame.empty()) {
            RCLCPP_ERROR(logger, "map_frame is empty.");
            return false;
        }
        if (laser_topic.path.empty()) {
            RCLCPP_ERROR(logger, "laser_topic is empty.");
            return false;
        }
        if (pose_topic.path.empty()) {
            RCLCPP_ERROR(logger, "pose_topic is empty.");
            return false;
        }
        if (path_topic.path.empty()) {
            RCLCPP_ERROR(logger, "path_topic is empty.");
            return false;
        }
        if (publish_rate <= 0.0) {
            RCLCPP_ERROR(logger, "publish_rate must be positive, got %f", publish_rate);
            return false;
        }
        return true;
    }
};

class GazeboRoom2dNode : public rclcpp::Node {

    std::unique_ptr<GazeboRoom2D::TrainDataLoader> m_data_loader_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_pub_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pub_pose_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_path_;
    rclcpp::TimerBase::SharedPtr m_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_broadcaster_;
    long m_frame_index_ = 0;
    sensor_msgs::msg::LaserScan m_msg_scan_;
    geometry_msgs::msg::PoseStamped m_msg_pose_;
    geometry_msgs::msg::TransformStamped m_msg_transform_;
    nav_msgs::msg::Path m_msg_path_;

public:
    GazeboRoom2dNode()
        : Node("gazebo_room_2d_node") {
        RCLCPP_INFO(
            this->get_logger(),
            "Gazebo Room 2D: min=[%f, %f], max=[%f, %f]",
            GazeboRoom2D::kMapMin[0],
            GazeboRoom2D::kMapMin[1],
            GazeboRoom2D::kMapMax[0],
            GazeboRoom2D::kMapMax[1]);

        g_curr_node = this;

        // Load options from parameters
        Options cfg;
        if (!cfg.LoadFromRos2(this, "")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded parameters:\n%s", cfg.AsYamlString().c_str());

        // Load data
        m_data_loader_ = std::make_unique<GazeboRoom2D::TrainDataLoader>(cfg.data_folder);

        // Initialize message headers
        m_msg_scan_.header.frame_id = cfg.laser_frame;      // reference frame for the scan
        m_msg_pose_.header.frame_id = cfg.map_frame;        // reference frame for the pose
        m_msg_transform_.header.frame_id = cfg.map_frame;   // parent frame for the transform
        m_msg_transform_.child_frame_id = cfg.laser_frame;  // child frame for the transform
        m_msg_path_.header.frame_id = cfg.map_frame;        // reference frame for the path
        m_msg_path_.poses.reserve(m_data_loader_->size());

        // Initialize publishers
        m_pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            cfg.laser_topic.path,
            cfg.laser_topic.GetQoS());
        m_pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            cfg.pose_topic.path,
            cfg.pose_topic.GetQoS());
        m_pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
            cfg.path_topic.path,
            cfg.path_topic.GetQoS());

        // Initialize transform broadcaster
        m_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create timer
        auto timer_period = std::chrono::duration<double>(1.0 / cfg.publish_rate);
        m_timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&GazeboRoom2dNode::CallbackTimer, this));

        RCLCPP_INFO(this->get_logger(), "Gazebo Room 2D Node started.");
    }

private:
    void
    CallbackTimer() {
        if (m_frame_index_ >= m_data_loader_->size()) {
            RCLCPP_WARN(this->get_logger(), "No more frames to publish.");
            return;
        }

        auto &frame = (*m_data_loader_)[m_frame_index_];
        auto now = this->now();

        // Set transform
        m_msg_transform_.transform.translation.x = frame.translation[0];
        m_msg_transform_.transform.translation.y = frame.translation[1];
        m_msg_transform_.transform.translation.z = 0.0;
        const double yaw = std::atan2(frame.rotation(1, 0), frame.rotation(0, 0));
        m_msg_transform_.transform.rotation.w = std::cos(yaw / 2.0);
        m_msg_transform_.transform.rotation.x = 0.0;
        m_msg_transform_.transform.rotation.y = 0.0;
        m_msg_transform_.transform.rotation.z = std::sin(yaw / 2.0);
        m_msg_transform_.header.stamp = now;
        // Note: ROS2 doesn't have seq field, it's handled automatically
        m_broadcaster_->sendTransform(m_msg_transform_);

        // Set pose
        m_msg_pose_.header.stamp = now;
        m_msg_pose_.pose.position.x = m_msg_transform_.transform.translation.x;
        m_msg_pose_.pose.position.y = m_msg_transform_.transform.translation.y;
        m_msg_pose_.pose.position.z = 0.0;
        m_msg_pose_.pose.orientation = m_msg_transform_.transform.rotation;
        m_pub_pose_->publish(m_msg_pose_);

        // Set path
        m_msg_path_.header.stamp = now;
        m_msg_path_.poses.emplace_back(m_msg_pose_);
        m_pub_path_->publish(m_msg_path_);

        // Set laser scan
        m_msg_scan_.header.stamp = now;
        m_msg_scan_.angle_min = frame.angles[0];
        m_msg_scan_.angle_max = frame.angles[269];
        m_msg_scan_.angle_increment = frame.angles[1] - frame.angles[0];
        m_msg_scan_.time_increment = 0.0;
        m_msg_scan_.scan_time = 0.0;
        m_msg_scan_.range_min = std::numeric_limits<float>::infinity();
        m_msg_scan_.range_max = -std::numeric_limits<float>::infinity();
        m_msg_scan_.ranges.resize(frame.ranges.size());
        m_msg_scan_.intensities.resize(frame.ranges.size(), 1.0f);

        for (long i = 0; i < frame.ranges.size(); ++i) {
            m_msg_scan_.ranges[i] = frame.ranges[i];
            m_msg_scan_.range_min = std::min(m_msg_scan_.range_min, m_msg_scan_.ranges[i]);
            m_msg_scan_.range_max = std::max(m_msg_scan_.range_max, m_msg_scan_.ranges[i]);
        }
        m_pub_scan_->publish(m_msg_scan_);

        m_frame_index_++;
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboRoom2dNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
