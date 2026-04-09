#include "erl_common/yaml.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

using namespace erl::common;

struct Options : public Yamlable<Options> {
    std::string in_topic = "point_cloud_in";
    std::string out_topic = "point_cloud_out";
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
        if (in_topic.empty()) {
            ROS_ERROR("in_topic is empty");
            return false;
        }
        if (out_topic.empty()) {
            ROS_ERROR("out_topic is empty");
            return false;
        }
        return true;
    }
};

static Options g_cfg;
static Eigen::Matrix4f g_transform;
static ros::Publisher g_publisher;

void
CallbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    sensor_msgs::PointCloud2 out_msg = *msg;

    if (!g_cfg.new_frame_id.empty()) { out_msg.header.frame_id = g_cfg.new_frame_id; }

    // Find xyz field offsets
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    for (const auto &field: out_msg.fields) {
        if (field.name == "x") { x_offset = static_cast<int>(field.offset); }
        if (field.name == "y") { y_offset = static_cast<int>(field.offset); }
        if (field.name == "z") { z_offset = static_cast<int>(field.offset); }
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
        ROS_WARN_THROTTLE(5, "Point cloud missing x/y/z fields, publishing without transform.");
        g_publisher.publish(out_msg);
        return;
    }

    // Transform each point in place
    const uint32_t point_step = out_msg.point_step;
    const uint32_t num_points = out_msg.width * out_msg.height;
    uint8_t *data = out_msg.data.data();

    for (uint32_t i = 0; i < num_points; ++i) {
        uint8_t *ptr = data + i * point_step;
        float x, y, z;
        std::memcpy(&x, ptr + x_offset, sizeof(float));
        std::memcpy(&y, ptr + y_offset, sizeof(float));
        std::memcpy(&z, ptr + z_offset, sizeof(float));

        Eigen::Vector4f pt(x, y, z, 1.0f);
        Eigen::Vector4f pt_transformed = g_transform * pt;

        std::memcpy(ptr + x_offset, &pt_transformed[0], sizeof(float));
        std::memcpy(ptr + y_offset, &pt_transformed[1], sizeof(float));
        std::memcpy(ptr + z_offset, &pt_transformed[2], sizeof(float));
    }

    g_publisher.publish(out_msg);
}

int
main(int argc, char **argv) {
    ros::init(argc, argv, "point_cloud_transform_node");
    ros::NodeHandle nh("~");

    if (!g_cfg.LoadFromRos1(nh, "")) {
        ROS_ERROR("Failed to load parameters.");
        return -1;
    }
    ROS_INFO("Loaded parameters:\n%s", g_cfg.AsYamlString().c_str());

    g_transform = g_cfg.transform.cast<float>();

    g_publisher = nh.advertise<sensor_msgs::PointCloud2>(g_cfg.out_topic, 10);
    ros::Subscriber subscriber =
        nh.subscribe<sensor_msgs::PointCloud2>(g_cfg.in_topic, 10, CallbackPointCloud);

    ros::spin();
    return 0;
}
