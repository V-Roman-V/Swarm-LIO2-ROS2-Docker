#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <swarm_msgs/msg/global_extrinsic_status.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>

using std::placeholders::_1;

struct Extrinsic {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    bool valid = false;
};

class RobotExtrinsicTfPublisher : public rclcpp::Node {
public:
    RobotExtrinsicTfPublisher() : Node("robot_extrinsic_tf_publisher") {
        root_id_ = this->declare_parameter<int>("root_id", 1);
        robot_ids_ = this->declare_parameter<std::vector<int64_t>>(
            "robot_ids", std::vector<int64_t>{1, 2, 3, 4});

        robot_prefix_ = this->declare_parameter<std::string>("robot_prefix", "bot");
        origin_frame_id_ = normalizeFrame(
            this->declare_parameter<std::string>("origin_frame_id", "map_origin"));
        robot_frame_suffix_ = normalizeFrame(
            this->declare_parameter<std::string>("robot_frame_suffix", "world"));

        accept_only_from_origin_ =
            this->declare_parameter<bool>("accept_only_from_origin", true);

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Root extrinsic = identity
        Extrinsic self;
        self.R.setIdentity();
        self.t.setZero();
        self.valid = true;
        extrinsics_[root_id_] = self;

        extrinsic_sub_ =
            this->create_subscription<swarm_msgs::msg::GlobalExtrinsicStatus>(
                "/global_extrinsic_to_teammate", rclcpp::SystemDefaultsQoS(),
                std::bind(&RobotExtrinsicTfPublisher::extrinsicCallback, this, _1));

        timer_ = rclcpp::create_timer(
            this->get_node_base_interface(),
            this->get_node_timers_interface(),
            this->get_clock(),
            std::chrono::milliseconds(100),
            std::bind(&RobotExtrinsicTfPublisher::publishAll, this));

        RCLCPP_INFO(this->get_logger(),
                    "RobotExtrinsicTfPublisher started. root_id=%d, origin_frame=%s",
                    root_id_, origin_frame_id_.c_str());
    }

private:
    int root_id_;
    std::vector<int64_t> robot_ids_;
    std::string robot_prefix_;
    std::string robot_frame_suffix_;
    std::string origin_frame_id_;
    bool accept_only_from_origin_;

    std::unordered_map<int, Extrinsic> extrinsics_;

    rclcpp::Subscription<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr extrinsic_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    static std::string normalizeFrame(std::string s) {
        while (!s.empty() && s.front() == '/') s.erase(s.begin());
        return s;
    }

    std::string robotName(int id) const { return robot_prefix_ + std::to_string(id); }

    std::string childFrame(int id) const {
        // Example: "botX/world"
        return normalizeFrame(robotName(id) + "/" + robot_frame_suffix_);
    }

    Eigen::Matrix3d eulerDegToRot(const std::array<float, 3> &deg) const {
        constexpr double kDegToRad = M_PI / 180.0;
        const double roll  = static_cast<double>(deg[0]) * kDegToRad;
        const double pitch = static_cast<double>(deg[1]) * kDegToRad;
        const double yaw   = static_cast<double>(deg[2]) * kDegToRad;

        Eigen::AngleAxisd Rx(roll,  Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd Rz(yaw,   Eigen::Vector3d::UnitZ());
        return (Rz.toRotationMatrix() * Ry.toRotationMatrix() * Rx.toRotationMatrix());
    }

    void extrinsicCallback(const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr msg) {
        const int src = msg->drone_id;
        if (accept_only_from_origin_ && src != root_id_) return;

        for (const auto &ex : msg->extrinsic) {
            const int tid = ex.teammate_id;

            Extrinsic T;
            T.R = eulerDegToRot(ex.rot_deg);
            T.t = Eigen::Vector3d(ex.trans[0], ex.trans[1], ex.trans[2]);
            T.valid = true;
            extrinsics_[tid] = T;
        }

        // Optionally push out immediately as well (timer will also republish)
        publishAll();
    }

    void publishAll() {
        const rclcpp::Time stamp = this->now();

        std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
        tf_msgs.reserve(robot_ids_.size());

        for (auto id64 : robot_ids_) {
            const int id = static_cast<int>(id64);

            auto itT = extrinsics_.find(id);
            if (itT == extrinsics_.end() || !itT->second.valid) continue;

            const auto &R = itT->second.R;
            const auto &t = itT->second.t;

            Eigen::Quaterniond q(R);
            q.normalize();

            // TF: origin_frame -> botX/world
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = stamp;
            tf.header.frame_id = origin_frame_id_;  // map_origin
            tf.child_frame_id = childFrame(id);     // botX/world
            tf.transform.translation.x = t.x();
            tf.transform.translation.y = t.y();
            tf.transform.translation.z = t.z();
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_msgs.push_back(tf);
        }

        if (!tf_msgs.empty()) tf_broadcaster_->sendTransform(tf_msgs);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotExtrinsicTfPublisher>());
    rclcpp::shutdown();
    return 0;
}
