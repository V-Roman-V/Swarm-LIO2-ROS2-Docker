#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <swarm_msgs/msg/global_extrinsic_status.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

Eigen::Matrix3d rotZ(const double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

void projectExtrinsicSE2(Eigen::Matrix3d &R, Eigen::Vector3d &t) {
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    R = rotZ(yaw);
    t.z() = 0.0;
}

Eigen::Vector3d rotMToEuler(const Eigen::Matrix3d &rot) {
    const double sy = std::sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
    const bool singular = sy < 1e-6;
    double x, y, z;

    if (!singular) {
        x = std::atan2(rot(2, 1), rot(2, 2));
        y = std::atan2(-rot(2, 0), sy);
        z = std::atan2(rot(1, 0), rot(0, 0));
    } else {
        x = std::atan2(-rot(1, 2), rot(1, 1));
        y = std::atan2(-rot(2, 0), sy);
        z = 0.0;
    }
    return Eigen::Vector3d(x, y, z);
}

struct PoseState {
    Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d t{Eigen::Vector3d::Zero()};
    rclcpp::Time stamp;
    std::string frame_id;
    bool ready{false};
};

}  // namespace

class GroundtruthExtrinsicInitializer : public rclcpp::Node {
public:
    GroundtruthExtrinsicInitializer() : Node("gt_extrinsic_initializer") {
        bypass_initialization_ = declare_parameter<bool>("bypass_initialization", false);
        const auto robot_ids_param = declare_parameter<std::vector<int64_t>>("robot_ids", {1, 2});
        robot_ids_.reserve(robot_ids_param.size());
        for (const auto id64 : robot_ids_param) {
            robot_ids_.push_back(static_cast<int>(id64));
        }
        robot_prefix_ = declare_parameter<std::string>("robot_prefix", "bot");
        odom_suffix_ = declare_parameter<std::string>("odom_suffix", "/gt/odom");
        publish_topic_ = declare_parameter<std::string>("publish_topic", "/global_extrinsic_from_teammate");
        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 5.0);
        frame_id_override_ = declare_parameter<std::string>("frame_id_override", "");
        publish_once_ = declare_parameter<bool>("publish_once", true);
        min_ready_duration_sec_ = declare_parameter<double>("min_ready_duration_sec", 10.0);
        stamp_tolerance_sec_ = declare_parameter<double>("stamp_tolerance_sec", 0.05);

        extrinsic_pub_ = create_publisher<swarm_msgs::msg::GlobalExtrinsicStatus>(
            publish_topic_, rclcpp::SystemDefaultsQoS());

        const double period = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.5;
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(period)),
            std::bind(&GroundtruthExtrinsicInitializer::publishExtrinsics, this));

        for (const int id : robot_ids_) {
            const std::string topic = makeTopicName(id);
            RCLCPP_INFO(get_logger(), "Listening for ground-truth odometry: id=%d topic=%s", id, topic.c_str());
            auto sub = create_subscription<nav_msgs::msg::Odometry>(
                topic, rclcpp::SystemDefaultsQoS(),
                [this, id](const nav_msgs::msg::Odometry::SharedPtr msg) { handleOdom(id, *msg); });
            odom_subs_.push_back(sub);
        }

        if (!bypass_initialization_) {
            RCLCPP_WARN(get_logger(), "bypass_initialization=false; gt_extrinsic_initializer will stay idle.");
        }
    }

private:
    void handleOdom(const int id, const nav_msgs::msg::Odometry &odom) {
        PoseState &state = poses_[id];
        state.t = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                             odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        if (q.norm() > 1e-6) {
            q.normalize();
            state.R = q.toRotationMatrix();
        } else {
            state.R.setIdentity();
        }
        state.stamp = odom.header.stamp;
        state.frame_id = odom.header.frame_id;
        state.ready = true;
    }

    std::string makeTopicName(const int id) const {
        std::ostringstream oss;
        oss << "/" << robot_prefix_ << id << odom_suffix_;
        return oss.str();
    }

    bool allReady() const {
        std::size_t ready_count = 0;
        for (const auto id : robot_ids_) {
            auto it = poses_.find(id);
            if (it != poses_.end() && it->second.ready) {
                ready_count++;
            }
        }
        return ready_count >= 2;
    }

    bool allReadyAll() const {
        for (const auto id : robot_ids_) {
            auto it = poses_.find(id);
            if (!(it != poses_.end() && it->second.ready)) {
                return false;
            }
        }
        return !robot_ids_.empty();
    }

    bool timestampsClose() const {
        rclcpp::Time min_stamp(0, 0, get_clock()->get_clock_type());
        rclcpp::Time max_stamp(0, 0, get_clock()->get_clock_type());
        bool init = false;
        for (const auto &kv : poses_) {
            if (!kv.second.ready) continue;
            if (!init) {
                min_stamp = max_stamp = kv.second.stamp;
                init = true;
            } else {
                if (kv.second.stamp < min_stamp) min_stamp = kv.second.stamp;
                if (kv.second.stamp > max_stamp) max_stamp = kv.second.stamp;
            }
        }
        if (!init) return false;
        return (max_stamp - min_stamp).seconds() <= stamp_tolerance_sec_;
    }

    void publishExtrinsics() {
        if (!bypass_initialization_) return;
        if (robot_ids_.size() < 2) return;

        const bool ready_now = allReadyAll();
        const rclcpp::Time now_time = now();
        if (publish_once_) {
            if (ready_now) {
                if (first_all_ready_time_.nanoseconds() == 0) {
                    first_all_ready_time_ = now_time;
                }
                const double ready_duration = (now_time - first_all_ready_time_).seconds();
                if (ready_duration < min_ready_duration_sec_) {
                    return;
                }
            } else {
                // not all ready, reset the timer
                first_all_ready_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
                return;
            }
        } else {
            if (!allReady()) return;
        }

        // avoid mixing stale odom stamps when computing GT extrinsics
        if (!timestampsClose()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "GT extrinsic wait: odom stamps not synchronized (tolerance %.3fs)",
                                 stamp_tolerance_sec_);
            return;
        }

        const rclcpp::Time stamp_now = now_time;
        bool published_any = false;
        for (const auto src_id : robot_ids_) {
            auto src_it = poses_.find(src_id);
            if (src_it == poses_.end() || !src_it->second.ready) {
                continue;
            }

            swarm_msgs::msg::GlobalExtrinsicStatus status_msg;
            status_msg.header.stamp = stamp_now;
            status_msg.header.frame_id =
                frame_id_override_.empty()
                    ? (src_it->second.frame_id.empty() ? std::string("map_origin") : src_it->second.frame_id)
                    : frame_id_override_;
            status_msg.drone_id = static_cast<uint8_t>(src_id);
            status_msg.world_to_gravity_deg = {0.0f, 0.0f, 0.0f};

            for (const auto dst_id : robot_ids_) {
                if (dst_id == src_id) continue;
                auto dst_it = poses_.find(dst_id);
                if (dst_it == poses_.end() || !dst_it->second.ready) {
                    continue;
                }

                // Use yaw-only SE2 relative pose to avoid roll/pitch leakage into translation
                const double yaw_src = std::atan2(src_it->second.R(1, 0), src_it->second.R(0, 0));
                const double yaw_dst = std::atan2(dst_it->second.R(1, 0), dst_it->second.R(0, 0));
                const double yaw_rel = std::atan2(std::sin(yaw_dst - yaw_src), std::cos(yaw_dst - yaw_src));
                Eigen::Matrix3d R_rel = rotZ(yaw_rel);
                Eigen::Vector3d t_rel = rotZ(-yaw_src) * (dst_it->second.t - src_it->second.t);
                t_rel.z() = 0.0;

                const Eigen::Vector3d euler_deg = rotMToEuler(R_rel) * 57.29577951308232;

                swarm_msgs::msg::GlobalExtrinsic extrinsic_msg;
                extrinsic_msg.teammate_id = static_cast<uint8_t>(dst_id);
                extrinsic_msg.rot_deg = {static_cast<float>(euler_deg.x()),
                                         static_cast<float>(euler_deg.y()),
                                         static_cast<float>(euler_deg.z())};
                extrinsic_msg.trans = {static_cast<float>(t_rel.x()),
                                       static_cast<float>(t_rel.y()),
                                       static_cast<float>(t_rel.z())};
                extrinsic_msg.world_to_gravity_deg = {0.0f, 0.0f, 0.0f};
                status_msg.extrinsic.push_back(extrinsic_msg);
            }

            if (!status_msg.extrinsic.empty()) {
                extrinsic_pub_->publish(status_msg);
                published_sources_.insert(src_id);
                published_any = true;
            }
        }

        if (publish_once_ && published_any) {
            published_once_ = true;
            RCLCPP_INFO(get_logger(), "Published ground-truth extrinsics once for all robots; stopping.");
            timer_->cancel();
        }
    }

    bool bypass_initialization_;
    bool publish_once_;
    double min_ready_duration_sec_;
    double stamp_tolerance_sec_;
    bool published_once_{false};
    rclcpp::Time first_all_ready_time_{0, 0, RCL_ROS_TIME};
    std::vector<int> robot_ids_;
    std::string robot_prefix_;
    std::string odom_suffix_;
    std::string publish_topic_;
    std::string frame_id_override_;
    double publish_rate_hz_;

    std::unordered_map<int, PoseState> poses_;
    std::unordered_set<int> published_sources_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subs_;
    rclcpp::Publisher<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr extrinsic_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundtruthExtrinsicInitializer>());
    rclcpp::shutdown();
    return 0;
}
