#include "erl_common/yaml.hpp"
#include "erl_geometry/gazebo_room_2d.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace erl::common;
using namespace erl::geometry;

struct Options : public Yamlable<Options> {
    std::string data_folder = "data/gazebo";
    std::string laser_frame = "front_laser";
    std::string map_frame = "map";
    std::string laser_topic = "scan";
    std::string pose_topic = "pose";
    std::string path_topic = "path";
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
        if (data_folder.empty()) {
            ROS_ERROR("data_folder is empty.");
            return false;
        }
        if (map_frame.empty()) {
            ROS_ERROR("map_frame is empty.");
            return false;
        }
        if (laser_frame.empty()) {
            ROS_ERROR("laser_frame is empty.");
            return false;
        }
        if (laser_topic.empty()) {
            ROS_ERROR("laser_topic is empty.");
            return false;
        }
        if (pose_topic.empty()) {
            ROS_ERROR("pose_topic is empty.");
            return false;
        }
        if (path_topic.empty()) {
            ROS_ERROR("path_topic is empty.");
            return false;
        }
        if (publish_rate <= 0.0) {
            ROS_ERROR("publish_rate must be positive, got %f", publish_rate);
            return false;
        }
        return true;
    }
};

class GazeboRoom2dNode {

    std::unique_ptr<GazeboRoom2D::TrainDataLoader> m_data_loader_;
    ros::NodeHandle m_nh_;
    ros::Publisher m_pub_scan_;
    ros::Publisher m_pub_pose_;
    ros::Publisher m_pub_path_;
    ros::Timer m_timer_;
    tf2_ros::TransformBroadcaster m_broadcaster_;
    long m_frame_index_ = 0;
    sensor_msgs::LaserScan m_msg_scan_;
    geometry_msgs::PoseStamped m_msg_pose_;
    geometry_msgs::TransformStamped m_msg_transform_;
    nav_msgs::Path m_msg_path_;

public:
    GazeboRoom2dNode(ros::NodeHandle &nh)
        : m_nh_(nh) {
        ROS_INFO(
            "Gazebo Room 2D: min=[%f, %f], max=[%f, %f]",
            GazeboRoom2D::kMapMin[0],
            GazeboRoom2D::kMapMin[1],
            GazeboRoom2D::kMapMax[0],
            GazeboRoom2D::kMapMax[1]);

        // Load options from parameters
        Options cfg;
        if (!cfg.LoadFromRos1(m_nh_, "")) {
            ROS_ERROR("Failed to load parameters.");
            ros::shutdown();
            return;
        }
        ROS_INFO("Loaded parameters:\n%s", cfg.AsYamlString().c_str());

        m_msg_scan_.header.frame_id = cfg.laser_frame;      // reference frame for the scan
        m_msg_pose_.header.frame_id = cfg.map_frame;        // reference frame for the pose
        m_msg_transform_.header.frame_id = cfg.map_frame;   // parent frame for the transform
        m_msg_transform_.child_frame_id = cfg.laser_frame;  // child frame for the transform
        m_msg_path_.header.frame_id = cfg.map_frame;        // reference frame for the path

        // Load data
        if (!std::filesystem::exists(cfg.data_folder)) {
            ROS_FATAL("Data folder '%s' does not exist.", cfg.data_folder.c_str());
            ros::shutdown();
            return;
        }
        m_data_loader_ = std::make_unique<GazeboRoom2D::TrainDataLoader>(cfg.data_folder);
        m_msg_path_.poses.reserve(m_data_loader_->size());
        // Initialize the node
        m_pub_scan_ = m_nh_.advertise<sensor_msgs::LaserScan>(cfg.laser_topic, 1);
        m_pub_pose_ = m_nh_.advertise<geometry_msgs::PoseStamped>(cfg.pose_topic, 1);
        m_pub_path_ = m_nh_.advertise<nav_msgs::Path>(cfg.path_topic, 1, true);
        m_timer_ = m_nh_.createTimer(
            ros::Duration(1.0 / cfg.publish_rate),
            &GazeboRoom2dNode::CallbackTimer,
            this);
        ROS_INFO("Gazebo Room 2D Node started.");
    }

    void
    CallbackTimer(const ros::TimerEvent &) {
        if (m_frame_index_ >= m_data_loader_->size()) {
            ROS_WARN("%s: No more frames to publish.", m_nh_.getNamespace().c_str());
            return;
        }

        auto &frame = (*m_data_loader_)[m_frame_index_];

        m_msg_transform_.transform.translation.x = frame.translation[0];
        m_msg_transform_.transform.translation.y = frame.translation[1];
        m_msg_transform_.transform.translation.z = 0.0;
        const double yaw = std::atan2(frame.rotation(1, 0), frame.rotation(0, 0));
        m_msg_transform_.transform.rotation.w = std::cos(yaw / 2.0);
        m_msg_transform_.transform.rotation.x = 0.0;
        m_msg_transform_.transform.rotation.y = 0.0;
        m_msg_transform_.transform.rotation.z = std::sin(yaw / 2.0);
        m_msg_transform_.header.stamp = ros::Time::now();
        m_msg_transform_.header.seq = m_frame_index_;
        m_broadcaster_.sendTransform(m_msg_transform_);

        m_msg_pose_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_pose_.header.seq = m_frame_index_;
        m_msg_pose_.pose.position.x = m_msg_transform_.transform.translation.x;
        m_msg_pose_.pose.position.y = m_msg_transform_.transform.translation.y;
        m_msg_pose_.pose.position.z = 0.0;
        m_msg_pose_.pose.orientation = m_msg_transform_.transform.rotation;
        m_pub_pose_.publish(m_msg_pose_);

        m_msg_path_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_path_.header.seq = m_frame_index_;
        m_msg_path_.poses.emplace_back(m_msg_pose_);
        m_pub_path_.publish(m_msg_path_);

        m_msg_scan_.header.stamp = m_msg_transform_.header.stamp;
        m_msg_scan_.header.seq = m_frame_index_++;
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
        m_pub_scan_.publish(m_msg_scan_);
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_room_2d_node");
    ros::NodeHandle nh("~");
    GazeboRoom2dNode node(nh);
    ros::spin();
    return 0;
}
