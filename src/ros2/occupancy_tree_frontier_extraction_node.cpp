#include "erl_common/ros2_topic_params.hpp"
#include "erl_common/yaml.hpp"
#include "erl_geometry/aabb.hpp"
#include "erl_geometry/abstract_occupancy_octree.hpp"
#include "erl_geometry/abstract_occupancy_quadtree.hpp"
#include "erl_geometry/occupancy_octree_base.hpp"
#include "erl_geometry/occupancy_quadtree_base.hpp"
#include "erl_geometry_msgs/msg/frontier.hpp"
#include "erl_geometry_msgs/msg/frontier_array.hpp"
#include "erl_geometry_msgs/msg/occupancy_tree_msg.hpp"
#include "erl_geometry_msgs/ros2/occupancy_tree_msg.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace erl::common;
using namespace erl::common::ros_params;
using namespace erl::geometry;

static rclcpp::Node *g_curr_node = nullptr;

struct FrontierExtractionNodeConfig : public Yamlable<FrontierExtractionNodeConfig> {
    Ros2TopicParams tree_topic{"surface_mapping_tree"};
    Ros2TopicParams frontier_topic{"frontiers"};
    bool use_aabb = false;
    Aabb3Dd aabb = {};
    bool use_slice = false;
    std::vector<double> z_slices = {0.0};

    ERL_REFLECT_SCHEMA(
        FrontierExtractionNodeConfig,
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, tree_topic),
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, frontier_topic),
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, use_aabb),
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, aabb),
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, use_slice),
        ERL_REFLECT_MEMBER(FrontierExtractionNodeConfig, z_slices));

    bool
    PostDeserialization() override {
        auto logger = g_curr_node->get_logger();
        if (tree_topic.path.empty()) {
            RCLCPP_WARN(logger, "tree_topic.path is empty");
            return false;
        }
        if (frontier_topic.path.empty()) {
            RCLCPP_WARN(logger, "frontier_topic.path is empty");
            return false;
        }
        if (use_slice && z_slices.empty()) {
            RCLCPP_WARN(logger, "use_slice is true but z_slices is empty");
            return false;
        }
        return true;
    }
};

class OccupancyTreeFrontierExtractionNode : public rclcpp::Node {
    FrontierExtractionNodeConfig m_config_;
    rclcpp::Subscription<erl_geometry_msgs::msg::OccupancyTreeMsg>::SharedPtr m_tree_sub_;
    rclcpp::Publisher<erl_geometry_msgs::msg::FrontierArray>::SharedPtr m_frontier_pub_;

public:
    OccupancyTreeFrontierExtractionNode()
        : Node("occupancy_tree_frontier_extraction_node") {

        g_curr_node = this;

        if (!m_config_.LoadFromRos2(this, "")) {
            RCLCPP_FATAL(this->get_logger(), "Failed to load parameters");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded parameters:\n%s", m_config_.AsYamlString().c_str());

        m_frontier_pub_ = this->create_publisher<erl_geometry_msgs::msg::FrontierArray>(
            m_config_.frontier_topic.path,
            m_config_.frontier_topic.GetQoS());

        m_tree_sub_ = this->create_subscription<erl_geometry_msgs::msg::OccupancyTreeMsg>(
            m_config_.tree_topic.path,
            m_config_.tree_topic.GetQoS(),
            std::bind(
                &OccupancyTreeFrontierExtractionNode::CallbackTree,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribed to tree topic: %s, publishing frontiers on: %s",
            m_config_.tree_topic.path.c_str(),
            m_config_.frontier_topic.path.c_str());
    }

private:
    void
    CallbackTree(const erl_geometry_msgs::msg::OccupancyTreeMsg::ConstSharedPtr &msg) {
        if (m_frontier_pub_->get_subscription_count() == 0) { return; }

        if (msg->dim == 2) {
            if (msg->is_double) {
                HandleQuadtree<double>(msg);
            } else {
                HandleQuadtree<float>(msg);
            }
        } else if (msg->dim == 3) {
            if (msg->is_double) {
                HandleOctree<double>(msg);
            } else {
                HandleOctree<float>(msg);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported tree dimension: %u", msg->dim);
        }
    }

    // -- Quadtree (2D) --

    template<typename Dtype>
    void
    HandleQuadtree(const erl_geometry_msgs::msg::OccupancyTreeMsg::ConstSharedPtr &msg) {
        auto tree_setting = std::make_shared<OccupancyQuadtreeBaseSetting>();
        auto abstract_tree = AbstractQuadtree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyQuadtree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to create quadtree: %s",
                msg->tree_type.c_str());
            return;
        }
        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            RCLCPP_WARN(this->get_logger(), "Failed to deserialize quadtree");
            return;
        }

        // downcast to access ExtractFrontiers
        using QuadtreeBase =
            OccupancyQuadtreeBase<Dtype, OccupancyQuadtreeNode, OccupancyQuadtreeBaseSetting>;
        auto concrete = std::dynamic_pointer_cast<QuadtreeBase>(tree);
        if (concrete == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Quadtree type does not support frontier extraction");
            return;
        }

        const double scale = msg->scale;
        std::vector<typename QuadtreeBase::Frontier> frontiers;
        if (m_config_.use_aabb) {
            auto aabb = m_config_.aabb.template Cast<Dtype>();
            // the tree is scaled by msg->scale, but the aabb is in the original size.
            frontiers = concrete->ExtractFrontiers(
                static_cast<Dtype>(aabb.min()[0] * scale),
                static_cast<Dtype>(aabb.min()[1] * scale),
                static_cast<Dtype>(aabb.max()[0] * scale),
                static_cast<Dtype>(aabb.max()[1] * scale));
        } else {
            frontiers = concrete->ExtractFrontiers();
        }

        PublishPolylineFrontiers(frontiers, scale, msg->header);
    }

    // -- Octree (3D) --

    template<typename Dtype>
    void
    HandleOctree(const erl_geometry_msgs::msg::OccupancyTreeMsg::ConstSharedPtr &msg) {
        auto tree_setting = std::make_shared<OccupancyOctreeBaseSetting>();
        auto abstract_tree = AbstractOctree<Dtype>::CreateTree(msg->tree_type, tree_setting);
        auto tree = std::dynamic_pointer_cast<AbstractOccupancyOctree<Dtype>>(abstract_tree);
        if (tree == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Failed to create octree: %s", msg->tree_type.c_str());
            return;
        }
        if (!LoadFromOccupancyTreeMsg<Dtype>(*msg, tree)) {
            RCLCPP_WARN(this->get_logger(), "Failed to deserialize octree");
            return;
        }

        using OctreeBase =
            OccupancyOctreeBase<Dtype, OccupancyOctreeNode, OccupancyOctreeBaseSetting>;
        auto concrete = std::dynamic_pointer_cast<OctreeBase>(tree);
        if (concrete == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Octree type does not support frontier extraction");
            return;
        }

        const double scale = msg->scale;

        if (m_config_.use_slice) {
            // Extract 2D slice frontiers at each z height, merge into one message
            std::vector<typename OctreeBase::SliceFrontier> all_frontiers;
            for (double z: m_config_.z_slices) {
                auto slice_frontiers =
                    concrete->ExtractSliceFrontiers(static_cast<Dtype>(z * scale));
                all_frontiers.insert(
                    all_frontiers.end(),
                    std::make_move_iterator(slice_frontiers.begin()),
                    std::make_move_iterator(slice_frontiers.end()));
            }
            PublishPolylineFrontiers(all_frontiers, scale, msg->header);
        } else {
            // Full 3D frontier extraction
            std::vector<typename OctreeBase::Frontier> frontiers;
            if (m_config_.use_aabb) {
                auto aabb = m_config_.aabb.template Cast<Dtype>();
                frontiers = concrete->ExtractFrontiers(
                    static_cast<Dtype>(aabb.min()[0] * scale),
                    static_cast<Dtype>(aabb.min()[1] * scale),
                    static_cast<Dtype>(aabb.min()[2] * scale),
                    static_cast<Dtype>(aabb.max()[0] * scale),
                    static_cast<Dtype>(aabb.max()[1] * scale),
                    static_cast<Dtype>(aabb.max()[2] * scale));
            } else {
                frontiers = concrete->ExtractFrontiers();
            }
            PublishMeshFrontiers<Dtype>(frontiers, scale, msg->header);
        }
    }

    // -- Score computation --

    template<typename Dtype>
    static double
    PolylineLength(const Eigen::Matrix2X<Dtype> &polyline, double scale) {
        double length = 0.0;
        for (long i = 1; i < polyline.cols(); ++i) {
            length += (polyline.col(i) - polyline.col(i - 1)).norm();
        }
        return length / scale;  // convert back to original scale for message
    }

    template<typename Dtype>
    static double
    MeshArea(
        const std::vector<Eigen::Vector3<Dtype>> &vertices,
        const std::vector<Eigen::Vector3i> &faces,
        double scale) {
        double area = 0.0;
        const double scale2 = scale * scale;
        for (const auto &face: faces) {
            auto v0 = vertices[face[0]].template cast<double>();
            auto v1 = vertices[face[1]].template cast<double>();
            auto v2 = vertices[face[2]].template cast<double>();
            area += 0.5 * (v1 - v0).cross(v2 - v0).norm() / scale2;
        }
        return area;
    }

    // -- Publishing --

    template<typename Dtype>
    void
    PublishPolylineFrontiers(
        const std::vector<Eigen::Matrix2X<Dtype>> &frontiers,
        double scale,
        const std_msgs::msg::Header &header) {

        erl_geometry_msgs::msg::FrontierArray msg;
        msg.header = header;
        msg.dim = 2;
        msg.resolution = scale;
        msg.frontiers.reserve(frontiers.size());

        uint32_t id = 0;
        for (const auto &polyline: frontiers) {
            erl_geometry_msgs::msg::Frontier f;
            f.id = id++;
            f.score = PolylineLength(polyline, scale);
            f.vertices.resize(polyline.cols());
            for (long i = 0; i < polyline.cols(); ++i) {
                // convert back to original scale for message
                f.vertices[i].x = static_cast<double>(polyline(0, i)) / scale;
                f.vertices[i].y = static_cast<double>(polyline(1, i)) / scale;
                f.vertices[i].z = 0.0;
            }
            // indices: pairs forming line segments
            f.indices.reserve((polyline.cols() - 1) * 2);
            for (long i = 0; i + 1 < polyline.cols(); ++i) {
                f.indices.push_back(static_cast<uint32_t>(i));
                f.indices.push_back(static_cast<uint32_t>(i + 1));
            }
            msg.frontiers.push_back(std::move(f));
        }
        m_frontier_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published %zu 2D frontiers", frontiers.size());
    }

    template<typename Dtype>
    void
    PublishMeshFrontiers(
        const std::vector<
            typename OccupancyOctreeBase<Dtype, OccupancyOctreeNode, OccupancyOctreeBaseSetting>::
                Frontier> &frontiers,
        double scale,
        const std_msgs::msg::Header &header) {

        erl_geometry_msgs::msg::FrontierArray msg;
        msg.header = header;
        msg.dim = 3;
        msg.resolution = scale;
        msg.frontiers.reserve(frontiers.size());

        uint32_t id = 0;
        for (const auto &mesh: frontiers) {
            erl_geometry_msgs::msg::Frontier f;
            f.id = id++;
            f.score = MeshArea(mesh.vertices, mesh.faces, scale);
            f.vertices.resize(mesh.vertices.size());
            for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
                // convert back to original scale for message
                f.vertices[i].x = static_cast<double>(mesh.vertices[i][0]) / scale;
                f.vertices[i].y = static_cast<double>(mesh.vertices[i][1]) / scale;
                f.vertices[i].z = static_cast<double>(mesh.vertices[i][2]) / scale;
            }
            // indices: triples forming triangles
            f.indices.reserve(mesh.faces.size() * 3);
            for (const auto &face: mesh.faces) {
                f.indices.push_back(static_cast<uint32_t>(face[0]));
                f.indices.push_back(static_cast<uint32_t>(face[1]));
                f.indices.push_back(static_cast<uint32_t>(face[2]));
            }
            msg.frontiers.push_back(std::move(f));
        }
        m_frontier_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published %zu 3D frontiers", frontiers.size());
    }
};

int
main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OccupancyTreeFrontierExtractionNode>());
    rclcpp::shutdown();
    return 0;
}
