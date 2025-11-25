#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <swarm_msgs/msg/global_extrinsic_status.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>

using std::placeholders::_1;

struct Extrinsic {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    bool valid = false;
};

struct LocalMap {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    rclcpp::Time stamp;
    bool has_data = false;
};

class GlobalMapNode : public rclcpp::Node {
public:
    GlobalMapNode()
        : Node("global_map_node")
    {
        root_id_ = this->declare_parameter<int>("root_id", 1);
        uav_ids_ = this->declare_parameter<std::vector<int64_t>>("uav_ids", std::vector<int64_t>{1, 2, 3, 4});

        root_frame_id_ = "quad" + std::to_string(root_id_) + "/world";

        // init maps & subscribers for each quadX/downsampled_map
        for (int64_t id64 : uav_ids_) {
            int id = static_cast<int>(id64);

            LocalMap lm;
            lm.cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            local_maps_[id] = lm;

            std::string topic =
                "/quad" + std::to_string(id) + "/downsampled_map";

            auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic,
                rclcpp::SensorDataQoS(),
                [this, id](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
                    this->localMapCallback(id, msg);
                });
            map_subs_.push_back(sub);
        }

        extrinsic_sub_ = this->create_subscription<swarm_msgs::msg::GlobalExtrinsicStatus>(
            "/global_extrinsic_to_teammate",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&GlobalMapNode::extrinsicCallback, this, _1));

        global_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_downsampled_map", rclcpp::SystemDefaultsQoS());

        // publish at low rate to avoid heavy load
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GlobalMapNode::publishGlobalMap, this));

        // root's own transform = identity
        Extrinsic self;
        self.R.setIdentity();
        self.t.setZero();
        self.valid = true;
        extrinsics_[root_id_] = self;

        RCLCPP_INFO(this->get_logger(),
                    "GlobalMapNode started. root_id=%d, frame=%s",
                    root_id_, root_frame_id_.c_str());
    }

private:
    int root_id_;
    std::vector<int64_t> uav_ids_;
    std::string root_frame_id_;

    std::unordered_map<int, LocalMap> local_maps_;
    std::unordered_map<int, Extrinsic> extrinsics_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> map_subs_;
    rclcpp::Subscription<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr extrinsic_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    float intensityForId(int id) const {
        // simple debug scheme: quad1=0, quad2=100, quad3=200, quad4=300
        return 100.0f * static_cast<float>(id - 1);
    }

    void localMapCallback(int id, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
        auto it = local_maps_.find(id);
        if (it == local_maps_.end())
            return;

        pcl::PointCloud<pcl::PointXYZI> tmp;
        pcl::fromROSMsg(*msg, tmp);
        *(it->second.cloud) = std::move(tmp);
        it->second.stamp = msg->header.stamp;
        it->second.has_data = true;
    }

    void extrinsicCallback(const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr msg) {
        int src = msg->drone_id;  // this drone's id

        // For now, only use extrinsics from the chosen root
        if (src != root_id_)
            return;

        for (const auto &ex : msg->extrinsic) {
            int tid = ex.teammate_id;
            if (tid == root_id_)
                continue;

            Extrinsic T;
            T.R = eulerDegToRot(ex.rot_deg);
            T.t = Eigen::Vector3d(ex.trans[0], ex.trans[1], ex.trans[2]);
            T.valid = true;
            extrinsics_[tid] = T;
        }
    }

    Eigen::Matrix3d eulerDegToRot(const std::array<float, 3> &deg) {
        double roll  = deg[0] * M_PI / 180.0;
        double pitch = deg[1] * M_PI / 180.0;
        double yaw   = deg[2] * M_PI / 180.0;

        Eigen::AngleAxisd Rx(roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(yaw,   Eigen::Vector3d::UnitZ());

        // Explicitly convert to matrices before multiplication
        Eigen::Matrix3d R =
            Rz.toRotationMatrix() *
            Ry.toRotationMatrix() *
            Rx.toRotationMatrix();

        return R;
    }

    void publishGlobalMap() {
        if (global_pub_->get_subscription_count() == 0)
            return;

        auto merged = pcl::PointCloud<pcl::PointXYZI>();
        merged.reserve(100000); // simple guess to avoid frequent realloc

        for (int64_t id64 : uav_ids_) {
            int id = static_cast<int>(id64);

            auto it_map = local_maps_.find(id);
            auto it_T   = extrinsics_.find(id);
            if (it_map == local_maps_.end() || it_T == extrinsics_.end())
                continue;
            if (!it_map->second.has_data || !it_T->second.valid)
                continue;

            const auto &cloud = *(it_map->second.cloud);
            const auto &R = it_T->second.R;
            const auto &t = it_T->second.t;
            float base_int = intensityForId(id);

            for (const auto &p : cloud.points) {
                Eigen::Vector3d v(p.x, p.y, p.z);
                Eigen::Vector3d vg = R * v + t;

                pcl::PointXYZI q;
                q.x = static_cast<float>(vg.x());
                q.y = static_cast<float>(vg.y());
                q.z = static_cast<float>(vg.z());
                q.intensity = base_int; // encode origin robot in intensity
                merged.push_back(q);
            }
        }

        if (merged.empty())
            return;

        sensor_msgs::msg::PointCloud2 out;
        pcl::toROSMsg(merged, out);
        out.header.stamp = this->now();
        out.header.frame_id = root_frame_id_;
        global_pub_->publish(out);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalMapNode>());
    rclcpp::shutdown();
    return 0;
}
