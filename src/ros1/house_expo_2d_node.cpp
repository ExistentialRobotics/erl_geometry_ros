#include "erl_common/ros2_topic_params.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>

#include <filesystem>
#include <memory>
#include <mutex>

using namespace erl::common;
using namespace erl::common::ros_params;

struct Options : public Yamlable<Options> {
    std::string map_file;
    std::string path_file;
    double wall_thickness = 0.2;
    double lidar_angle_min = -M_PI;
    double lidar_angle_max = M_PI;
    int lidar_num_lines = 360;
    bool publish_pose = false;
    double pose_publish_rate = 20.0;  // Hz
    double path_feed_rate = 2.0;      // Hz
    std::string global_frame = "map";
    std::string lidar_frame = "laser";
    double init_x = 0.0;
    double init_y = 0.0;
    double init_yaw = 0.0;
    std::string pose_topic = "pose";
    std::string path_topic = "path";
    std::string scan_topic = "scan";
    std::string max_observable_area_topic = "max_observable_area";
    std::string loaded_path_topic = "loaded_path";

    ERL_REFLECT_SCHEMA(
        Options,
        ERL_REFLECT_MEMBER(Options, map_file),
        ERL_REFLECT_MEMBER(Options, path_file),
        ERL_REFLECT_MEMBER(Options, wall_thickness),
        ERL_REFLECT_MEMBER(Options, lidar_angle_min),
        ERL_REFLECT_MEMBER(Options, lidar_angle_max),
        ERL_REFLECT_MEMBER(Options, lidar_num_lines),
        ERL_REFLECT_MEMBER(Options, publish_pose),
        ERL_REFLECT_MEMBER(Options, pose_publish_rate),
        ERL_REFLECT_MEMBER(Options, path_feed_rate),
        ERL_REFLECT_MEMBER(Options, global_frame),
        ERL_REFLECT_MEMBER(Options, lidar_frame),
        ERL_REFLECT_MEMBER(Options, init_x),
        ERL_REFLECT_MEMBER(Options, init_y),
        ERL_REFLECT_MEMBER(Options, init_yaw),
        ERL_REFLECT_MEMBER(Options, pose_topic),
        ERL_REFLECT_MEMBER(Options, path_topic),
        ERL_REFLECT_MEMBER(Options, scan_topic),
        ERL_REFLECT_MEMBER(Options, max_observable_area_topic),
        ERL_REFLECT_MEMBER(Options, loaded_path_topic));

    bool
    PostDeserialization() override {
        if (map_file.empty()) {
            ROS_ERROR("Map file is empty!");
            return false;
        }

        if (!std::filesystem::exists(map_file)) {
            ROS_ERROR("Map file does not exist: %s", map_file.c_str());
            return false;
        }

        if (wall_thickness < 0) {
            ROS_ERROR("Wall thickness must be non-negative!");
            return false;
        }
        return true;
    }
};

class HouseExpo2dNode {
    Options m_cfg_;
    ros::NodeHandle m_nh_;

    std::shared_ptr<erl::geometry::HouseExpoMap> m_house_expo_map_ = nullptr;
    std::shared_ptr<erl::geometry::Lidar2D> m_lidar_ = nullptr;
    double m_max_observable_area_ = 100.0;

    ros::Subscriber m_sub_pose_;
    ros::Subscriber m_sub_path_;
    ros::Publisher m_pub_scan_;
    ros::Publisher m_pub_max_observable_area_;
    ros::Publisher m_pub_loaded_path_;

    tf2_ros::TransformBroadcaster m_tf_broadcaster_;
    geometry_msgs::TransformStamped m_cur_pose_;
    ros::Timer m_pose_timer_;
    std::mutex m_pose_mutex_;

    nav_msgs::Path m_path_;
    ros::Timer m_path_timer_;
    std::size_t m_wp_idx_ = 0;

    std_msgs::Float64 m_max_observable_area_msg_;
    geometry_msgs::PoseStamped m_pose_msg_;
    sensor_msgs::LaserScan m_scan_msg_;

public:
    explicit HouseExpo2dNode(ros::NodeHandle &nh)
        : m_nh_(nh) {

        if (!m_cfg_.LoadFromRos1(m_nh_, "")) {
            ROS_ERROR("Failed to load parameters.");
            ros::shutdown();
            return;
        }
        ROS_INFO("Loaded parameters:\n%s", m_cfg_.AsYamlString().c_str());

        m_house_expo_map_ =
            std::make_shared<erl::geometry::HouseExpoMap>(m_cfg_.map_file, m_cfg_.wall_thickness);
        using GridMapInfo = erl::common::GridMapInfo2Dd;
        std::shared_ptr<GridMapInfo> grid_map_info = std::make_shared<GridMapInfo>(
            m_house_expo_map_->GetMapMin(),
            m_house_expo_map_->GetMapMax(),
            Eigen::Vector2d(0.05, 0.05),
            Eigen::Vector2i::Constant(10));
        Eigen::MatrixX8U map_img =
            m_house_expo_map_->GetMeterSpace()->GenerateMapImage(*grid_map_info, true);
        m_max_observable_area_ = map_img.cast<double>().sum() / 255.0 * 0.05 * 0.05;

        auto lidar_setting = std::make_shared<erl::geometry::Lidar2D::Setting>();
        lidar_setting->min_angle = m_cfg_.lidar_angle_min;
        lidar_setting->max_angle = m_cfg_.lidar_angle_max;
        lidar_setting->num_lines = m_cfg_.lidar_num_lines;
        m_lidar_ = std::make_shared<erl::geometry::Lidar2D>(
            lidar_setting,
            m_house_expo_map_->GetMeterSpace());

        m_sub_pose_ = m_nh_.subscribe(m_cfg_.pose_topic, 1, &HouseExpo2dNode::CallbackPose, this);

        m_sub_path_ = m_nh_.subscribe(m_cfg_.path_topic, 1, &HouseExpo2dNode::CallbackPath, this);

        m_pub_scan_ = m_nh_.advertise<sensor_msgs::LaserScan>(m_cfg_.scan_topic, 1);

        m_pub_max_observable_area_ =
            m_nh_.advertise<std_msgs::Float64>(m_cfg_.max_observable_area_topic, 1);
        ROS_INFO("Max observable area: %f m^2", m_max_observable_area_);
        m_max_observable_area_msg_.data = m_max_observable_area_;
        m_pub_max_observable_area_.publish(m_max_observable_area_msg_);

        m_pub_loaded_path_ = m_nh_.advertise<nav_msgs::Path>(m_cfg_.loaded_path_topic, 1, true);

        m_cur_pose_.header.frame_id = m_cfg_.global_frame;
        m_cur_pose_.header.stamp = ros::Time::now();
        m_cur_pose_.child_frame_id = m_cfg_.lidar_frame;
        m_cur_pose_.transform.translation.x = m_cfg_.init_x;
        m_cur_pose_.transform.translation.y = m_cfg_.init_y;
        m_cur_pose_.transform.translation.z = 0.0;
        m_cur_pose_.transform.rotation.x = 0.0;
        m_cur_pose_.transform.rotation.y = 0.0;
        m_cur_pose_.transform.rotation.z = std::sin(m_cfg_.init_yaw / 2);
        m_cur_pose_.transform.rotation.w = std::cos(m_cfg_.init_yaw / 2);

        if (m_cfg_.publish_pose) {
            m_pose_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_cfg_.pose_publish_rate),
                [this](const ros::TimerEvent &) {
                    std::lock_guard<std::mutex> lock(m_pose_mutex_);
                    m_cur_pose_.header.stamp = ros::Time::now();  // update the timestamp
                    m_tf_broadcaster_.sendTransform(m_cur_pose_);

                    Eigen::Matrix2d rotation = Eigen::Quaterniond(
                                                   m_cur_pose_.transform.rotation.w,
                                                   m_cur_pose_.transform.rotation.x,
                                                   m_cur_pose_.transform.rotation.y,
                                                   m_cur_pose_.transform.rotation.z)
                                                   .toRotationMatrix()
                                                   .topLeftCorner<2, 2>();
                    Eigen::Vector2d translation(
                        m_cur_pose_.transform.translation.x,
                        m_cur_pose_.transform.translation.y);
                    PublishScan(ros::Time::now(), rotation, translation);

                    // re-send the transform to ensure the transform is up-to-date
                    m_cur_pose_.header.stamp = ros::Time::now();  // update the timestamp
                    m_tf_broadcaster_.sendTransform(m_cur_pose_);
                });
        }

        if (!m_cfg_.path_file.empty() && std::filesystem::exists(m_cfg_.path_file)) {
            // load the path from file
            Eigen::Matrix3Xd path = erl::common::LoadEigenMatrixFromTextFile<double>(
                m_cfg_.path_file,
                erl::common::EigenTextFormat::kCsvFmt,
                true);
            // convert to nav_msgs::msg::Path
            m_path_.header.frame_id = m_cfg_.global_frame;
            m_path_.header.stamp = ros::Time::now();
            m_path_.poses.clear();
            m_path_.poses.reserve(path.cols());
            for (long i = 0; i < path.cols(); ++i) {
                auto p = path.col(i);
                m_pose_msg_.header.frame_id = m_cfg_.global_frame;
                m_pose_msg_.header.stamp = ros::Time::now();
                m_pose_msg_.pose.position.x = p[0];
                m_pose_msg_.pose.position.y = p[1];
                m_pose_msg_.pose.position.z = 0.0;
                m_pose_msg_.pose.orientation.x = 0.0;
                m_pose_msg_.pose.orientation.y = 0.0;
                m_pose_msg_.pose.orientation.z = std::sin(p[2] / 2);
                m_pose_msg_.pose.orientation.w = std::cos(p[2] / 2);
                m_path_.poses.push_back(m_pose_msg_);
            }
            ROS_INFO(
                "Loaded path from file %s with %lu waypoints.",
                m_cfg_.path_file.c_str(),
                m_path_.poses.size());
            m_pub_loaded_path_.publish(m_path_);
            m_path_timer_ = m_nh_.createTimer(
                ros::Duration(1.0 / m_cfg_.path_feed_rate),
                &HouseExpo2dNode::CallbackTimerPath,
                this);
        }
        ROS_INFO("HouseExpo2d node initialized.");
    }

    void
    CallbackPose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
        Eigen::Matrix2d rotation = q.toRotationMatrix().topLeftCorner<2, 2>();
        Eigen::Vector2d translation(msg->pose.position.x, msg->pose.position.y);

        PublishScan(msg->header.stamp, rotation, translation);
    }

    void
    PublishScan(
        const ros::Time &time,
        const Eigen::Matrix2d &rotation,
        const Eigen::Vector2d &translation) {

        Eigen::VectorXf ranges = m_lidar_->Scan(rotation, translation, true).cast<float>();

        m_scan_msg_.header.frame_id = m_cfg_.lidar_frame;
        m_scan_msg_.header.stamp = time;
        m_scan_msg_.angle_min = m_cfg_.lidar_angle_min;
        m_scan_msg_.angle_max = m_cfg_.lidar_angle_max;
        m_scan_msg_.angle_increment =
            (m_cfg_.lidar_angle_max - m_cfg_.lidar_angle_min) / (m_cfg_.lidar_num_lines - 1);
        m_scan_msg_.time_increment = 0.0;
        m_scan_msg_.scan_time = 0.0;
        m_scan_msg_.range_min = ranges.minCoeff();
        m_scan_msg_.range_max = ranges.maxCoeff();
        m_scan_msg_.ranges.clear();
        m_scan_msg_.ranges.reserve(ranges.size());
        m_scan_msg_.ranges.insert(
            m_scan_msg_.ranges.end(),
            ranges.data(),
            ranges.data() + ranges.size());

        m_pub_scan_.publish(m_scan_msg_);
        m_pub_max_observable_area_.publish(m_max_observable_area_msg_);
    }

    // called when a new path is received
    void
    CallbackPath(const nav_msgs::Path::ConstPtr &msg) {
        if (m_path_timer_ != nullptr) { m_path_timer_.stop(); }

        // the timer is off, it is safe to update the path
        m_path_ = *msg;
        m_wp_idx_ = 0;

        // create a new timer to feed the path
        m_path_timer_ = m_nh_.createTimer(
            ros::Duration(1.0 / m_cfg_.path_feed_rate),
            &HouseExpo2dNode::CallbackTimerPath,
            this);
    }

    void
    CallbackTimerPath(const ros::TimerEvent &) {
        if (m_wp_idx_ >= m_path_.poses.size()) {
            m_path_timer_.stop();
            ROS_INFO("Finished publishing all waypoints.");
            return;
        }
        if (m_cfg_.publish_pose) {
            // set the current pose, let the pose timer publish the transform and scan
            std::lock_guard<std::mutex> lock(m_pose_mutex_);
            auto &pose = m_path_.poses[m_wp_idx_++];
            m_cur_pose_.header.frame_id = m_cfg_.global_frame;
            m_cur_pose_.header.stamp = ros::Time::now();
            m_cur_pose_.child_frame_id = m_cfg_.lidar_frame;
            m_cur_pose_.transform.translation.x = pose.pose.position.x;
            m_cur_pose_.transform.translation.y = pose.pose.position.y;
            m_cur_pose_.transform.translation.z = 0.0;
            m_cur_pose_.transform.rotation = pose.pose.orientation;
            return;  // the pose timer will publish the transform and scan
        }

        auto &pose = m_path_.poses[m_wp_idx_++];

        Eigen::Quaterniond q(
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z);
        Eigen::Matrix2d rotation = q.toRotationMatrix().topLeftCorner<2, 2>();
        Eigen::Vector2d translation(pose.pose.position.x, pose.pose.position.y);

        PublishScan(ros::Time::now(), rotation, translation);
    }
};

int
main(int argc, char **argv) {
    ros::init(argc, argv, "house_expo_2d_node");
    ros::NodeHandle nh("~");
    HouseExpo2dNode node(nh);
    ros::spin();
    return 0;
}
