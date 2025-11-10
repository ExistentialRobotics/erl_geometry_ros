#include "erl_common/ros2_topic_params.hpp"
#include "erl_geometry/house_expo_map.hpp"
#include "erl_geometry/lidar_2d.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <filesystem>
#include <memory>
#include <mutex>

using namespace erl::common;
using namespace erl::common::ros_params;

static rclcpp::Node *g_curr_node = nullptr;

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
    Ros2TopicParams pose_topic{"pose"};
    Ros2TopicParams path_topic{"path"};
    Ros2TopicParams scan_topic{"scan"};
    Ros2TopicParams max_observable_area_topic{"max_observable_area"};
    Ros2TopicParams loaded_path_topic{"loaded_path"};

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
        auto logger = g_curr_node->get_logger();
        if (map_file.empty()) {
            RCLCPP_ERROR(logger, "Map file is empty!");
            return false;
        }

        if (!std::filesystem::exists(map_file)) {
            RCLCPP_ERROR(logger, "Map file does not exist: %s", map_file.c_str());
            return false;
        }

        if (wall_thickness < 0) {
            RCLCPP_ERROR(logger, "Wall thickness must be non-negative!");
            return false;
        }
        return true;
    }
};

class HouseExpo2dNode : public rclcpp::Node {
    Options m_cfg_;

    std::shared_ptr<erl::geometry::HouseExpoMap> m_house_expo_map_ = nullptr;
    std::shared_ptr<erl::geometry::Lidar2D> m_lidar_ = nullptr;
    double m_max_observable_area_ = 100.0;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_pose_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_sub_path_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_pub_scan_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_pub_max_observable_area_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_pub_loaded_path_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_ = nullptr;
    geometry_msgs::msg::TransformStamped m_cur_pose_;
    rclcpp::TimerBase::SharedPtr m_pose_timer_ = nullptr;
    std::mutex m_pose_mutex_;

    nav_msgs::msg::Path m_path_;
    rclcpp::TimerBase::SharedPtr m_path_timer_ = nullptr;
    std::size_t m_wp_idx_ = 0;

public:
    explicit HouseExpo2dNode()
        : Node("house_expo_2d_node"),
          m_tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)) {

        g_curr_node = this;

        if (!m_cfg_.LoadFromRos2(this, "")) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load parameters.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded parameters:\n%s", m_cfg_.AsYamlString().c_str());

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

        m_sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            m_cfg_.pose_topic.path,
            m_cfg_.pose_topic.GetQoS(),
            std::bind(&HouseExpo2dNode::CallbackPose, this, std::placeholders::_1));

        m_sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            m_cfg_.path_topic.path,
            m_cfg_.path_topic.GetQoS(),
            std::bind(&HouseExpo2dNode::CallbackPath, this, std::placeholders::_1));

        m_pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            m_cfg_.scan_topic.path,
            m_cfg_.scan_topic.GetQoS());

        m_pub_max_observable_area_ = this->create_publisher<std_msgs::msg::Float64>(
            m_cfg_.max_observable_area_topic.path,
            m_cfg_.max_observable_area_topic.GetQoS());
        RCLCPP_INFO(this->get_logger(), "Max observable area: %f m^2", m_max_observable_area_);
        m_pub_max_observable_area_->publish(
            std_msgs::msg::Float64().set__data(m_max_observable_area_));

        m_pub_loaded_path_ = this->create_publisher<nav_msgs::msg::Path>(
            m_cfg_.loaded_path_topic.path,
            m_cfg_.loaded_path_topic.GetQoS());

        m_cur_pose_.header.frame_id = m_cfg_.global_frame;
        m_cur_pose_.header.stamp = this->now();
        m_cur_pose_.child_frame_id = m_cfg_.lidar_frame;
        m_cur_pose_.transform.translation.x = m_cfg_.init_x;
        m_cur_pose_.transform.translation.y = m_cfg_.init_y;
        m_cur_pose_.transform.translation.z = 0.0;
        m_cur_pose_.transform.rotation.x = 0.0;
        m_cur_pose_.transform.rotation.y = 0.0;
        m_cur_pose_.transform.rotation.z = std::sin(m_cfg_.init_yaw / 2);
        m_cur_pose_.transform.rotation.w = std::cos(m_cfg_.init_yaw / 2);

        if (m_cfg_.publish_pose) {
            m_pose_timer_ = this->create_timer(
                std::chrono::duration<double>(1.0 / m_cfg_.pose_publish_rate),
                [this]() {
                    std::lock_guard<std::mutex> lock(m_pose_mutex_);
                    m_cur_pose_.header.stamp = this->now();  // update the timestamp
                    m_tf_broadcaster_->sendTransform(m_cur_pose_);

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
                    PublishScan(this->now(), rotation, translation);

                    // re-send the transform to ensure the transform is up-to-date
                    m_cur_pose_.header.stamp = this->now();  // update the timestamp
                    m_tf_broadcaster_->sendTransform(m_cur_pose_);
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
            m_path_.header.stamp = this->now();
            m_path_.poses.clear();
            m_path_.poses.reserve(path.cols());
            for (long i = 0; i < path.cols(); ++i) {
                auto p = path.col(i);
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = m_cfg_.global_frame;
                pose.header.stamp = this->now();
                pose.pose.position.x = p[0];
                pose.pose.position.y = p[1];
                pose.pose.position.z = 0.0;
                pose.pose.orientation.x = 0.0;
                pose.pose.orientation.y = 0.0;
                pose.pose.orientation.z = std::sin(p[2] / 2);
                pose.pose.orientation.w = std::cos(p[2] / 2);
                m_path_.poses.push_back(pose);
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded path from file %s with %lu waypoints.",
                m_cfg_.path_file.c_str(),
                m_path_.poses.size());
            m_pub_loaded_path_->publish(m_path_);
            m_path_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / m_cfg_.path_feed_rate),
                std::bind(&HouseExpo2dNode::CallbackTimerPath, this));
        }
        RCLCPP_INFO(this->get_logger(), "HouseExpo2d node initialized.");
    }

    void
    CallbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
        const rclcpp::Time &time,
        const Eigen::Matrix2d &rotation,
        const Eigen::Vector2d &translation) {

        Eigen::VectorXf ranges = m_lidar_->Scan(rotation, translation, true).cast<float>();

        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.frame_id = m_cfg_.lidar_frame;
        scan_msg.header.stamp = time;
        scan_msg.angle_min = m_cfg_.lidar_angle_min;
        scan_msg.angle_max = m_cfg_.lidar_angle_max;
        scan_msg.angle_increment =
            (m_cfg_.lidar_angle_max - m_cfg_.lidar_angle_min) / (m_cfg_.lidar_num_lines - 1);
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 0.0;
        scan_msg.range_min = ranges.minCoeff();
        scan_msg.range_max = ranges.maxCoeff();
        scan_msg.ranges.clear();
        scan_msg.ranges.reserve(ranges.size());
        scan_msg.ranges.insert(scan_msg.ranges.end(), ranges.data(), ranges.data() + ranges.size());

        m_pub_scan_->publish(scan_msg);
        m_pub_max_observable_area_->publish(
            std_msgs::msg::Float64().set__data(m_max_observable_area_));
    }

    // called when a new path is received
    void
    CallbackPath(const nav_msgs::msg::Path::SharedPtr msg) {
        if (m_path_timer_ != nullptr) {  // stop the previous timer
            m_path_timer_->cancel();
            m_path_timer_ = nullptr;
        }

        // the timer is off, it is safe to update the path
        m_path_ = *msg;
        m_wp_idx_ = 0;

        // create a new timer to feed the path
        m_path_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / m_cfg_.path_feed_rate),
            std::bind(&HouseExpo2dNode::CallbackTimerPath, this));
    }

    void
    CallbackTimerPath() {
        if (m_wp_idx_ >= m_path_.poses.size()) {
            m_path_timer_->cancel();
            m_path_timer_ = nullptr;
            RCLCPP_INFO(this->get_logger(), "Finished publishing all waypoints.");
            return;
        }
        if (m_cfg_.publish_pose) {
            // set the current pose, let the pose timer publish the transform and scan
            std::lock_guard<std::mutex> lock(m_pose_mutex_);
            auto &pose = m_path_.poses[m_wp_idx_++];
            m_cur_pose_.header.frame_id = m_cfg_.global_frame;
            m_cur_pose_.header.stamp = this->now();
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

        PublishScan(this->now(), rotation, translation);
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HouseExpo2dNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
