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

class HouseExpo2dNode : public rclcpp::Node {

    std::string m_map_file_;
    std::string m_path_file_;
    double m_wall_thickness_ = 0.2;
    double m_lidar_angle_min_ = -M_PI;
    double m_lidar_angle_max_ = M_PI;
    int m_lidar_num_lines_ = 360;
    bool m_publish_pose_ = false;
    double m_pose_publish_rate_ = 20.0;  // Hz
    double m_path_feed_rate_ = 2.0;      // Hz
    std::string m_global_frame_ = "map";
    std::string m_lidar_frame_ = "laser";

    double m_init_x_ = 0.0;
    double m_init_y_ = 0.0;
    double m_init_yaw_ = 0.0;

    std::string m_default_qos_reliability_ = "reliable";
    std::string m_default_qos_durability_ = "volatile";

    std::string m_pose_topic_ = "pose";
    std::string m_pose_topic_reliability_ = m_default_qos_reliability_;
    std::string m_pose_topic_durability_ = m_default_qos_durability_;
    std::string m_path_topic_ = "path";
    std::string m_path_topic_reliability_ = m_default_qos_reliability_;
    std::string m_path_topic_durability_ = m_default_qos_durability_;
    std::string m_scan_topic_ = "scan";
    std::string m_scan_topic_reliability_ = m_default_qos_reliability_;
    std::string m_scan_topic_durability_ = m_default_qos_durability_;
    std::string m_max_observable_area_topic_ = "max_observable_area";
    std::string m_max_observable_area_topic_reliability_ = m_default_qos_reliability_;
    std::string m_max_observable_area_topic_durability_ = m_default_qos_durability_;
    std::string m_loaded_path_topic_ = "loaded_path";
    std::string m_loaded_path_topic_reliability_ = m_default_qos_reliability_;
    std::string m_loaded_path_topic_durability_ = m_default_qos_durability_;

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

        this->declare_parameter("map_file", m_map_file_);
        this->declare_parameter("path_file", m_path_file_);
        this->declare_parameter("wall_thickness", m_wall_thickness_);
        this->declare_parameter("lidar_angle_min", m_lidar_angle_min_);
        this->declare_parameter("lidar_angle_max", m_lidar_angle_max_);
        this->declare_parameter("lidar_num_lines", m_lidar_num_lines_);
        this->declare_parameter("publish_pose", m_publish_pose_);
        this->declare_parameter("pose_publish_rate", m_pose_publish_rate_);
        this->declare_parameter("path_feed_rate", m_path_feed_rate_);
        this->declare_parameter("global_frame", m_global_frame_);
        this->declare_parameter("lidar_frame", m_lidar_frame_);

        this->declare_parameter("init_x", m_init_x_);
        this->declare_parameter("init_y", m_init_y_);
        this->declare_parameter("init_yaw", m_init_yaw_);

        this->declare_parameter("default_qos_reliability", m_default_qos_reliability_);
        this->declare_parameter("default_qos_durability", m_default_qos_durability_);
        m_default_qos_reliability_ = this->get_parameter("default_qos_reliability").as_string();
        m_default_qos_durability_ = this->get_parameter("default_qos_durability").as_string();

        this->declare_parameter("pose_topic", m_pose_topic_);
        this->declare_parameter("pose_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("pose_topic_durability", m_default_qos_durability_);

        this->declare_parameter("path_topic", m_path_topic_);
        this->declare_parameter("path_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("path_topic_durability", m_default_qos_durability_);

        this->declare_parameter("scan_topic", m_scan_topic_);
        this->declare_parameter("scan_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("scan_topic_durability", m_default_qos_durability_);

        this->declare_parameter("max_observable_area_topic", m_max_observable_area_topic_);
        this->declare_parameter(
            "max_observable_area_topic_reliability",
            m_default_qos_reliability_);
        this->declare_parameter("max_observable_area_topic_durability", m_default_qos_durability_);

        this->declare_parameter("loaded_path_topic", m_loaded_path_topic_);
        this->declare_parameter("loaded_path_topic_reliability", m_default_qos_reliability_);
        this->declare_parameter("loaded_path_topic_durability", m_default_qos_durability_);

        m_map_file_ = this->get_parameter("map_file").as_string();
        m_path_file_ = this->get_parameter("path_file").as_string();
        m_wall_thickness_ = this->get_parameter("wall_thickness").as_double();
        m_lidar_angle_min_ = this->get_parameter("lidar_angle_min").as_double();
        m_lidar_angle_max_ = this->get_parameter("lidar_angle_max").as_double();
        m_lidar_num_lines_ = this->get_parameter("lidar_num_lines").as_int();
        m_publish_pose_ = this->get_parameter("publish_pose").as_bool();
        m_pose_publish_rate_ = this->get_parameter("pose_publish_rate").as_double();
        m_path_feed_rate_ = this->get_parameter("path_feed_rate").as_double();
        m_global_frame_ = this->get_parameter("global_frame").as_string();
        m_lidar_frame_ = this->get_parameter("lidar_frame").as_string();

        m_init_x_ = this->get_parameter("init_x").as_double();
        m_init_y_ = this->get_parameter("init_y").as_double();
        m_init_yaw_ = this->get_parameter("init_yaw").as_double();

        m_pose_topic_ = this->get_parameter("pose_topic").as_string();
        m_pose_topic_reliability_ = this->get_parameter("pose_topic_reliability").as_string();
        m_pose_topic_durability_ = this->get_parameter("pose_topic_durability").as_string();

        m_path_topic_ = this->get_parameter("path_topic").as_string();
        m_path_topic_reliability_ = this->get_parameter("path_topic_reliability").as_string();
        m_path_topic_durability_ = this->get_parameter("path_topic_durability").as_string();

        m_scan_topic_ = this->get_parameter("scan_topic").as_string();
        m_scan_topic_reliability_ = this->get_parameter("scan_topic_reliability").as_string();
        m_scan_topic_durability_ = this->get_parameter("scan_topic_durability").as_string();

        m_max_observable_area_topic_ = this->get_parameter("max_observable_area_topic").as_string();
        m_max_observable_area_topic_reliability_ =
            this->get_parameter("max_observable_area_topic_reliability").as_string();
        m_max_observable_area_topic_durability_ =
            this->get_parameter("max_observable_area_topic_durability").as_string();

        m_loaded_path_topic_ = this->get_parameter("loaded_path_topic").as_string();
        m_loaded_path_topic_reliability_ =
            this->get_parameter("loaded_path_topic_reliability").as_string();
        m_loaded_path_topic_durability_ =
            this->get_parameter("loaded_path_topic_durability").as_string();

        RCLCPP_INFO(
            this->get_logger(),
            "Parameters:\n"
            "map_file: %s\n"
            "wall_thickness: %f\n"
            "lidar_angle_min: %f\n"
            "lidar_angle_max: %f\n"
            "lidar_num_lines: %d\n"
            "publish_pose: %s\n"
            "pose_publish_rate: %f\n"
            "path_feed_rate: %f\n"
            "global_frame: %s\n"
            "lidar_frame: %s\n"
            "init_x: %f\n"
            "init_y: %f\n"
            "init_yaw: %f\n"
            "default_qos_reliability: %s\n"
            "default_qos_durability: %s\n"
            "pose_topic: %s\n"
            "pose_topic_reliability: %s\n"
            "pose_topic_durability: %s\n"
            "path_topic: %s\n"
            "path_topic_reliability: %s\n"
            "path_topic_durability: %s\n"
            "scan_topic: %s\n"
            "scan_topic_reliability: %s\n"
            "scan_topic_durability: %s\n"
            "max_observable_area_topic: %s\n"
            "max_observable_area_topic_reliability: %s\n"
            "max_observable_area_topic_durability: %s",
            m_map_file_.c_str(),
            m_wall_thickness_,
            m_lidar_angle_min_,
            m_lidar_angle_max_,
            m_lidar_num_lines_,
            m_publish_pose_ ? "true" : "false",
            m_pose_publish_rate_,
            m_path_feed_rate_,
            m_global_frame_.c_str(),
            m_lidar_frame_.c_str(),
            m_init_x_,
            m_init_y_,
            m_init_yaw_,
            m_default_qos_reliability_.c_str(),
            m_default_qos_durability_.c_str(),
            m_pose_topic_.c_str(),
            m_pose_topic_reliability_.c_str(),
            m_pose_topic_durability_.c_str(),
            m_path_topic_.c_str(),
            m_path_topic_reliability_.c_str(),
            m_path_topic_durability_.c_str(),
            m_scan_topic_.c_str(),
            m_scan_topic_reliability_.c_str(),
            m_scan_topic_durability_.c_str(),
            m_max_observable_area_topic_.c_str(),
            m_max_observable_area_topic_reliability_.c_str(),
            m_max_observable_area_topic_durability_.c_str());

        if (m_map_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Map file is empty!");
            throw std::runtime_error("Map file is empty!");
        }

        if (!std::filesystem::exists(m_map_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Map file does not exist: %s", m_map_file_.c_str());
            throw std::runtime_error("Map file does not exist: " + m_map_file_);
        }

        if (m_wall_thickness_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Wall thickness must be non-negative!");
            throw std::runtime_error("Wall thickness must be non-negative!");
        }

        m_house_expo_map_ =
            std::make_shared<erl::geometry::HouseExpoMap>(m_map_file_, m_wall_thickness_);
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
        lidar_setting->min_angle = m_lidar_angle_min_;
        lidar_setting->max_angle = m_lidar_angle_max_;
        lidar_setting->num_lines = m_lidar_num_lines_;
        m_lidar_ = std::make_shared<erl::geometry::Lidar2D>(
            lidar_setting,
            m_house_expo_map_->GetMeterSpace());

        m_sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            m_pose_topic_,
            GetQoS(m_pose_topic_reliability_, m_pose_topic_durability_),
            std::bind(&HouseExpo2dNode::CallbackPose, this, std::placeholders::_1));

        m_sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
            m_path_topic_,
            GetQoS(m_path_topic_reliability_, m_path_topic_durability_),
            std::bind(&HouseExpo2dNode::CallbackPath, this, std::placeholders::_1));

        m_pub_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            m_scan_topic_,
            GetQoS(m_scan_topic_reliability_, m_scan_topic_durability_));

        m_pub_max_observable_area_ = this->create_publisher<std_msgs::msg::Float64>(
            m_max_observable_area_topic_,
            GetQoS(
                m_max_observable_area_topic_reliability_,
                m_max_observable_area_topic_durability_));
        RCLCPP_INFO(this->get_logger(), "Max observable area: %f m^2", m_max_observable_area_);
        m_pub_max_observable_area_->publish(
            std_msgs::msg::Float64().set__data(m_max_observable_area_));

        m_pub_loaded_path_ = this->create_publisher<nav_msgs::msg::Path>(
            m_loaded_path_topic_,
            GetQoS(m_loaded_path_topic_reliability_, m_loaded_path_topic_durability_));

        m_cur_pose_.header.frame_id = m_global_frame_;
        m_cur_pose_.header.stamp = this->now();
        m_cur_pose_.child_frame_id = m_lidar_frame_;
        m_cur_pose_.transform.translation.x = m_init_x_;
        m_cur_pose_.transform.translation.y = m_init_y_;
        m_cur_pose_.transform.translation.z = 0.0;
        m_cur_pose_.transform.rotation.x = 0.0;
        m_cur_pose_.transform.rotation.y = 0.0;
        m_cur_pose_.transform.rotation.z = std::sin(m_init_yaw_ / 2);
        m_cur_pose_.transform.rotation.w = std::cos(m_init_yaw_ / 2);

        if (m_publish_pose_) {
            m_pose_timer_ = this->create_timer(
                std::chrono::duration<double>(1.0 / m_pose_publish_rate_),
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

        if (!m_path_file_.empty() && std::filesystem::exists(m_path_file_)) {
            // load the path from file
            Eigen::Matrix3Xd path = erl::common::LoadEigenMatrixFromTextFile<double>(
                m_path_file_,
                erl::common::EigenTextFormat::kCsvFmt,
                true);
            // convert to nav_msgs::msg::Path
            m_path_.header.frame_id = m_global_frame_;
            m_path_.header.stamp = this->now();
            m_path_.poses.clear();
            m_path_.poses.reserve(path.cols());
            for (long i = 0; i < path.cols(); ++i) {
                auto p = path.col(i);
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = m_global_frame_;
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
                m_path_file_.c_str(),
                m_path_.poses.size());
            m_pub_loaded_path_->publish(m_path_);
            m_path_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / m_path_feed_rate_),
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
        scan_msg.header.frame_id = m_lidar_frame_;
        scan_msg.header.stamp = time;
        scan_msg.angle_min = m_lidar_angle_min_;
        scan_msg.angle_max = m_lidar_angle_max_;
        scan_msg.angle_increment =
            (m_lidar_angle_max_ - m_lidar_angle_min_) / (m_lidar_num_lines_ - 1);
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
            std::chrono::duration<double>(1.0 / m_path_feed_rate_),
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
        if (m_publish_pose_) {
            // set the current pose, let the pose timer publish the transform and scan
            std::lock_guard<std::mutex> lock(m_pose_mutex_);
            auto &pose = m_path_.poses[m_wp_idx_++];
            m_cur_pose_.header.frame_id = m_global_frame_;
            m_cur_pose_.header.stamp = this->now();
            m_cur_pose_.child_frame_id = m_lidar_frame_;
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

    static rclcpp::QoS
    GetQoS(const std::string &reliability, const std::string &durability) {
        rclcpp::QoS qos(rclcpp::KeepLast(5));
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        } else {
            throw std::runtime_error("Invalid reliability: " + reliability);
        }
        if (durability == "volatile") {
            qos.durability_volatile();
        } else if (durability == "transient_local") {
            qos.transient_local();
        } else {
            throw std::runtime_error("Invalid durability: " + durability);
        }
        return qos;
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
