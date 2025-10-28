#include <rclcpp/rclcpp.hpp>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "fmt/color.h"
#include "udp_bridge/protocol.h"
#include "algorithm"
#include <string>
#include <map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "swarm_msgs/msg/team_status.hpp"
#include "swarm_msgs/msg/teammate_info.hpp"
#include "swarm_msgs/msg/quad_state_pub.hpp"
#include "swarm_msgs/msg/observe_teammate.hpp"
#include "swarm_msgs/msg/global_extrinsic_status.hpp"
#include "swarm_msgs/msg/global_extrinsic.hpp"
#include "swarm_msgs/msg/spatial_temporal_offset.hpp"
#include "swarm_msgs/msg/spatial_temporal_offset_status.hpp"
#include "swarm_msgs/msg/connected_teammate_list.hpp"
#include <ifaddrs.h>
#include "Teammate.hpp"
#include <errno.h>
#include <ctime>
#include <csignal>
#include <boost/filesystem.hpp>
#include <unordered_map>
#include "so3_math.h"
#include <Eigen/Eigen>
#include  "sensor_msgs/msg/battery_state.hpp"
#include "udp_bridge/scope_timer.hpp"
#include <numeric>
#include <std_msgs/msg/int8.hpp>

#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "log/"+ name))

//#define BACKWARD_HAS_DW 1
//#include "backward.hpp"
//namespace backward {
//    backward::SignalHandling sh;
//}

using namespace Eigen;
using namespace std;
using namespace fmt;
#define UDP_PORT 8821
#define BUFFER_SIZE 10086
#define OBSERVE_MSG_TYPE 0x04u

using namespace udp_bridge;
typedef unordered_map<int, Teammate> id_teammate_map;
typedef id_teammate_map::value_type position;
std::atomic_bool exit_process = false;
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "log/"+ name))


void SigHandle(int sig) {
    RCLCPP_WARN(rclcpp::get_logger("swarm_lio2"), "Exit the process, catch sig %d", sig);
    exit_process = true;
}


class UdpBridge : public rclcpp::Node {
private:
    int udp_server_fd_;
    int udp_send_ip_fd_ptr_;
    char udp_recv_buf_[BUFFER_SIZE];
    string local_ip;
    int local_id;
    int drone_state = -1;
    id_teammate_map teammates;
    string broadcast_ip, offset_path;
    ofstream save_offset;

    sockaddr_in addr_udp_send_ip_;
    boost::thread *udp_callback_thread_;
    rclcpp::Publisher<swarm_msgs::msg::TeamStatus>::SharedPtr team_status_pub_;
    rclcpp::TimerBase::SharedPtr sync_timer_, broadcast_timer_, debug_timer_, drone_state_timer_;
    rclcpp::Publisher<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_pub_;
    rclcpp::Publisher<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SpatialTemporalOffsetStatus>::SharedPtr ST_OffsetStatus_pub_;
    rclcpp::Subscription<swarm_msgs::msg::QuadStatePub>::SharedPtr QuadState_sub_;
    rclcpp::Subscription<swarm_msgs::msg::GlobalExtrinsicStatus>::SharedPtr GlobalExtrinsic_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr Battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr DroneState_sub_;
    rclcpp::Subscription<swarm_msgs::msg::ConnectedTeammateList>::SharedPtr TeammateListTraj_sub_;
    rclcpp::Time udp_start_time_;
    fstream log_writer_;
    ofstream fout_delay;
    vector<int> teammate_id_by_traj_matching;
    mutex traj_buffer_mtx;


///////////////////// TEMPLETE FOR A NEW MSG //////////////////////////////
private:
    struct IpIdData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        IpIdMsgCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT; // The system time when the msg is received.
    } ip_id_data_;

    void IpIdMsgCallback_() {
        if (!ip_id_data_.rcv_new_msg) {
            return;
        }
        ip_id_data_.update_lock_.lock();
        ip_id_data_.rcv_new_msg = false;
        ip_id_data_.processing_msg = ip_id_data_.latest_msg;
        ip_id_data_.update_lock_.unlock();

        //Record IP
        uint8_t *data = ip_id_data_.processing_msg.data.local_ip;
        string rcv_ip;
        CharIp2StringIp(data, rcv_ip);

        //Record ID
        int rcv_id = ip_id_data_.processing_msg.data.local_id;
        if (rcv_id == local_id) {
            return;
        }
        auto iter = teammates.find(rcv_id);
        double rcv_udp_start_time = ip_id_data_.processing_msg.data.udp_start_time.sec + ip_id_data_.processing_msg.data.udp_start_time.nsec * 1e-9;
        //检查收到的id是否已经在队友列表中，如果没有，即发现新队友，创建新的Teammate对象
        if(iter != teammates.end()) {
            if(abs(iter->second.udp_start_time_ - rcv_udp_start_time) > 1.0){
                teammates.erase(iter);
                Teammate drone(rcv_ip, rcv_id, ip_id_data_.rcv_WT.seconds(), rcv_udp_start_time);
                drone.udp_send_fd_ptr_ = InitUdpUnicast(drone, UDP_PORT);
                teammates.insert(position(rcv_id, drone));
                print(fg(color::lime_green), " -- [Re-synchronize with Teammate]: UAV{}, {}\n", rcv_id, rcv_ip);
            }else
                iter->second.last_rcv_time_ = ip_id_data_.rcv_WT.seconds();
        }
        else{
            Teammate drone(rcv_ip, rcv_id, ip_id_data_.rcv_WT.seconds(), rcv_udp_start_time);
            drone.udp_send_fd_ptr_ = InitUdpUnicast(drone, UDP_PORT);
            teammates.insert(position(rcv_id, drone));
            print(fg(color::lime_green), " -- [Found New Teammate]: UAV{}, {}\n", rcv_id, rcv_ip);
        }
    }


///////////////////// TIME SYNC MSG //////////////////////////////
private:
    struct TimeSyncData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        TimeSyncMsgCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT; // The system time when the msg is received.
    } time_sync_data_;

    void TimeSyncCallback_() {
        if (!time_sync_data_.rcv_new_msg) {
            return;
        }
        time_sync_data_.update_lock_.lock();
        time_sync_data_.rcv_new_msg = false;
        time_sync_data_.processing_msg = time_sync_data_.latest_msg;
        time_sync_data_.update_lock_.unlock();
        //////////////WRITE THE PROCESS CODE BELOW/////////////////
        // 收到时间同步消息，首先判断client的ID是不是自己
        int rcv_client_id = time_sync_data_.processing_msg.data.client_id;
        int rcv_server_id = time_sync_data_.processing_msg.data.server_id;

        //查看消息的client ID，如果是自己发的消息：
        if (rcv_client_id == local_id) {
            //判断对server的同步是否完成，如果完成就continue；没完成则计算时差和延迟
            auto iter = teammates.find(rcv_server_id);
            auto &drone = iter->second;
            if (drone.sync_done_) {
                return;
            }
            //计算公式中的t1,t2,t3,t4
            rclcpp::Time t4 = now();
            rclcpp::Time t1, t2, t3;
            t1 = rclcpp::Time((int64_t)time_sync_data_.processing_msg.data.t1.sec * 1000000000ll + time_sync_data_.processing_msg.data.t1.nsec);
            t2 = rclcpp::Time((int64_t)time_sync_data_.processing_msg.data.t2.sec * 1000000000ll + time_sync_data_.processing_msg.data.t2.nsec);
            t3 = rclcpp::Time((int64_t)time_sync_data_.processing_msg.data.t3.sec * 1000000000ll + time_sync_data_.processing_msg.data.t3.nsec);
            double dt1 = (t2 - t1).seconds();
            double dt2 = (t4 - t3).seconds();

            double offset_t = 0.5 * (dt1) - 0.5 * (dt2);
            double dt = (dt1) + (dt2);

            drone.offset_ts_.push_back(offset_t);
            drone.delay_ts_.push_back(dt);

            if (drone.offset_ts_.size() >= 30) {
                drone.sync_done_ = true;
                drone.offset_time_ = accumulate(drone.offset_ts_.begin(), drone.offset_ts_.end(), 0.0) / drone.offset_ts_.size();
                double delay_ = accumulate(drone.delay_ts_.begin(), drone.delay_ts_.end(), 0.0) / drone.delay_ts_.size();
                print("Update offset with UAV{0}\n\tdelay = {1:.6} ms, offset time = {2:.6} ms\n",
                      drone.id_, delay_ * 1000,
                      drone.offset_time_ * 1000);
            }
            return;
        }

        //如果client ID不是自己的，检查client ID是否已经在自己的队友列表中，如果在，则将数据发回去
        auto iter = teammates.find(rcv_client_id);
        if (iter != teammates.end()) {
            auto &drone = iter->second;
            //如果队友ID已经存放在我的队友列表中，则需要记录 t2 t3 返回给指定ID
            time_sync_data_.processing_msg.data.t2.nsec = (uint32_t)(time_sync_data_.rcv_WT.nanoseconds() % 1000000000ll);
            time_sync_data_.processing_msg.data.t2.sec = (int32_t)(time_sync_data_.rcv_WT.nanoseconds() / 1000000000ll);
            rclcpp::Time t3 = now();
            time_sync_data_.processing_msg.data.t3.sec = (int32_t)(t3.nanoseconds() / 1000000000ll);
            time_sync_data_.processing_msg.data.t3.nsec = (uint32_t)(t3.nanoseconds() % 1000000000ll);
            int len = sizeof(time_sync_data_.processing_msg.binary) + 2 * sizeof(uint32_t);
            std::vector<char> send_buf(len * 5);
            EncodeMsgToBuffer(MESSAGE_TYPE::TIME_SYNC, time_sync_data_.processing_msg, send_buf.data());
            if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                       sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(get_logger(), "UDP SEND BACK ERROR!!!");
            }
        }
    }

///////////////////// QUADSTATE MSG //////////////////////////////
    struct QuadStateData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        QuadStateCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT; // The system time when the msg is received.
    } quad_state_data_;

    void QuadStateCallback_() {
        if (!quad_state_data_.rcv_new_msg) {
            return;
        }
        quad_state_data_.update_lock_.lock();

        quad_state_data_.processing_msg = quad_state_data_.latest_msg;
        quad_state_data_.update_lock_.unlock();
        //////////////WRITE THE PROCESS CODE BELOW/////////////////
        swarm_msgs::msg::QuadStatePub quadstate_msg;
        quadstate_msg.teammate.clear();
        quadstate_msg.drone_id = quad_state_data_.processing_msg.data.drone_id;
        int rcv_id = quad_state_data_.processing_msg.data.drone_id;

        static int cnt[MAX_UAV_NUM] = {0};
        cnt[rcv_id]++;

        auto iter = teammates.find(rcv_id);
        if (iter != teammates.end() && iter->second.sync_done_) {
            double dt_ns = iter->second.offset_time_ * 1e9; //nanoseconds
            if (cnt[rcv_id] % 30 == 0)
                cout << "Fuse msg from UAV" << int(rcv_id) << ", offset time: " << dt_ns / 1e6 << " ms" << endl;
            {
                int64_t ns = (int64_t)quad_state_data_.processing_msg.data.header.sec * 1000000000ll +
                             (int64_t)quad_state_data_.processing_msg.data.header.nsec - (int64_t)dt_ns;
                quadstate_msg.header.stamp.sec = (int32_t)(ns / 1000000000ll);
                quadstate_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000ll);
            }
            double delay_time = (now().seconds() - (quadstate_msg.header.stamp.sec + quadstate_msg.header.stamp.nanosec * 1e-9)) * 1000.0;
            fout_delay << "teammate_id: " << int(rcv_id) << ", delay: " << delay_time << " ms" << endl;
            quadstate_msg.header.frame_id = "world";
            quadstate_msg.child_frame_id = "odom";

            quadstate_msg.pose.pose.orientation.w = quad_state_data_.processing_msg.data.quat_w;
            quadstate_msg.pose.pose.orientation.x = quad_state_data_.processing_msg.data.quat_x;
            quadstate_msg.pose.pose.orientation.y = quad_state_data_.processing_msg.data.quat_y;
            quadstate_msg.pose.pose.orientation.z = quad_state_data_.processing_msg.data.quat_z;
            quadstate_msg.pose.pose.position.x = quad_state_data_.processing_msg.data.pos[0];
            quadstate_msg.pose.pose.position.y = quad_state_data_.processing_msg.data.pos[1];
            quadstate_msg.pose.pose.position.z = quad_state_data_.processing_msg.data.pos[2];

            for (int i = 0; i < 3; i++) {
                quadstate_msg.vel[i] = quad_state_data_.processing_msg.data.vel[i];
                quadstate_msg.gyr[i] = quad_state_data_.processing_msg.data.gyr[i];
                quadstate_msg.world_to_gravity_deg[i] = quad_state_data_.processing_msg.data.world_to_gravity_deg[i];
            }
            for (int i = 0; i < 12; ++i) {
                quadstate_msg.pose_cov[i] = quad_state_data_.processing_msg.data.pose_cov[i];
            }
            quadstate_msg.swarmlio_start_time = quad_state_data_.processing_msg.data.swarmlio_start_time;
            quadstate_msg.degenerated = quad_state_data_.processing_msg.data.degenerated;

            for (int i = 0; i < MAX_UAV_NUM; i++) {
                if (quad_state_data_.processing_msg.data.teammate[i].is_observe) {
                    swarm_msgs::msg::ObserveTeammate obs_teammate;
                    obs_teammate.is_observe = true;
                    //drone id of observed teammate
                    obs_teammate.teammate_id = quad_state_data_.processing_msg.data.teammate[i].teammate_id;
                    //observed teammate's position and timestamp
                    obs_teammate.observed_pos[0] = quad_state_data_.processing_msg.data.teammate[i].observed_pos[0];
                    obs_teammate.observed_pos[1] = quad_state_data_.processing_msg.data.teammate[i].observed_pos[1];
                    obs_teammate.observed_pos[2] = quad_state_data_.processing_msg.data.teammate[i].observed_pos[2];
                    quadstate_msg.teammate.push_back(obs_teammate);
                }
            }
            QuadState_pub_->publish(quadstate_msg);
        }
        quad_state_data_.rcv_new_msg = false;
    }

///////////////////// Global Extrinsic MSG //////////////////////////////
    struct GlobalExtrinsicStatusData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        GlobalExtrinsicStatusCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT; // The system time when the msg is received.
    } global_extrinsic_data_;

    void GlobalExtrinsicCallback_() {
        if (!global_extrinsic_data_.rcv_new_msg) {
            return;
        }
        global_extrinsic_data_.update_lock_.lock();
        global_extrinsic_data_.processing_msg = global_extrinsic_data_.latest_msg;
        global_extrinsic_data_.update_lock_.unlock();
        //////////////WRITE THE PROCESS CODE BELOW/////////////////
        swarm_msgs::msg::GlobalExtrinsicStatus global_extrinsic_status_msg;
        global_extrinsic_status_msg.extrinsic.clear();
        global_extrinsic_status_msg.drone_id = global_extrinsic_data_.processing_msg.data.drone_id;
        int rcv_id = global_extrinsic_status_msg.drone_id;
        auto iter = teammates.find(rcv_id);
        if (iter != teammates.end() && iter->second.sync_done_) {
            double dt_ns = iter->second.offset_time_ * 1e9; //nanoseconds
            {
                int64_t ns = (int64_t)global_extrinsic_data_.processing_msg.data.header.sec * 1000000000ll +
                             (int64_t)global_extrinsic_data_.processing_msg.data.header.nsec - (int64_t)dt_ns;
                global_extrinsic_status_msg.header.stamp.sec = (int32_t)(ns / 1000000000ll);
                global_extrinsic_status_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000ll);
            }
            global_extrinsic_status_msg.header.frame_id = "world";
            for (int i = 0; i < MAX_UAV_NUM; i++) {
                swarm_msgs::msg::GlobalExtrinsic global_extrinsic;
                global_extrinsic.teammate_id = global_extrinsic_data_.processing_msg.data.extrinsic[i].teammate_id;
                if (int(global_extrinsic.teammate_id) > MAX_UAV_NUM)
                    continue;

                for (int j = 0; j < 3; ++j) {
                    global_extrinsic.rot_deg[j] = global_extrinsic_data_.processing_msg.data.extrinsic[i].rot_deg[j];
                    global_extrinsic.trans[j] = global_extrinsic_data_.processing_msg.data.extrinsic[i].trans[j];
                    global_extrinsic.world_to_gravity_deg[j] = global_extrinsic_data_.processing_msg.data.extrinsic[i].world_to_gravity_deg[j];
                }
                global_extrinsic_status_msg.extrinsic.push_back(global_extrinsic);
            }
            GlobalExtrinsic_pub_->publish(global_extrinsic_status_msg);
        }

        global_extrinsic_data_.rcv_new_msg = false;
    }

    struct GCSCmdMsgData {
        rclcpp::TimerBase::SharedPtr process_timer;
        bool rcv_new_msg{false};
        mutex update_lock_;
        GCSCmdMsgCvt latest_msg, processing_msg;
        rclcpp::Time rcv_WT; // The system time when the msg is received.
    } GCSCmdMsg_data_;
    void GCSCmdMsgCallback(){
        if (!GCSCmdMsg_data_.rcv_new_msg) {
            return;
        }
        GCSCmdMsg_data_.update_lock_.lock();
        GCSCmdMsg_data_.processing_msg = GCSCmdMsg_data_.latest_msg;
        GCSCmdMsg_data_.update_lock_.unlock();
        int rcv_cmd = GCSCmdMsg_data_.latest_msg.data.GCS_CmdSelection;
        // CharIp2StringIp(data, rcv_cmd);
        cout << "The latest GCSCmdMsg is : "<<rcv_cmd<<endl;
        // system("cd ~");
        switch (rcv_cmd)
        {
            case 1:
                cout << "Launch roscore && Livox && MAVROS" << endl;
                system("cd ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts && bash driver.sh");
                break;
            case 2:
                cout << "Launch Swarm LIO && Planner && Recorder" << endl;
                system("cd ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts && bash swarm.sh");
                break;
            case 3:
                cout << "Kill All Nodes" << endl;
                system("cd ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts && bash kill_all.sh");
                break;
            case 4:
                cout << "Reboot Computer" << endl;
                system("cd ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts && bash reboot.sh");
                break;
            case 5:
                cout << "Poweroff Computer" << endl;
                system("cd ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts && bash poweroff.sh");
                break;
            case 6:
                cout << "Update Code" << endl;
                system("bash ~/Workspace/swarm_lio_ws/src/multi_uav_lio/scripts/local_git/pull_code.sh");
                break;
            default:
                cout << "Invalid command received" << endl;
                break;
        }
        GCSCmdMsg_data_.rcv_new_msg = false;
    }
/////////////////////// DEBUG MSG //////////////////////////////
//private:
//    struct DebugData {
//        ros::Timer process_timer;
//        bool rcv_new_msg{false};
//        mutex update_lock_;
//        DebugMsgCvt latest_msg, processing_msg;
//        ros::Time rcv_WT; // The system time when the msg is received.
//    } debug_data_;
//
//    void DebugCallback(const ros::TimerEvent &e) {
//        if (!debug_data_.rcv_new_msg) {
//            return;
//        }
//        debug_data_.update_lock_.lock();
//        debug_data_.processing_msg = debug_data_.latest_msg;
//        debug_data_.update_lock_.unlock();
//        //////////////WRITE THE PROCESS CODE BELOW/////////////////
//        cout << debug_data_.processing_msg.data.seq << endl;
//        auto iter = teammates.find(14);
//        if (iter != teammates.end() && iter->second.sync_done_) {
//            double dt = double(debug_data_.processing_msg.data.header.sec * 1e9 +
//                               debug_data_.processing_msg.data.header.nsec - ros::Time::now().toNSec() -
//                               iter->second.offset_time_ * 1e9) / 1e9;
//            cout << "Dt: " << dt << endl;
//            log_writer_ << debug_data_.processing_msg.data.seq << " " << dt << endl;
//        }
//        ros::Duration(0.1).sleep();
//        debug_data_.rcv_new_msg = false;
//    }

    void initDataCallback() {
        using namespace std::chrono_literals;
        ip_id_data_.process_timer = this->create_wall_timer(1ms, std::bind(&UdpBridge::IpIdMsgCallback_, this));
        time_sync_data_.process_timer = this->create_wall_timer(1ms, std::bind(&UdpBridge::TimeSyncCallback_, this));
        quad_state_data_.process_timer = this->create_wall_timer(1ms, std::bind(&UdpBridge::QuadStateCallback_, this));
        global_extrinsic_data_.process_timer = this->create_wall_timer(1ms, std::bind(&UdpBridge::GlobalExtrinsicCallback_, this));
        GCSCmdMsg_data_.process_timer = this->create_wall_timer(1ms, std::bind(&UdpBridge::GCSCmdMsgCallback, this));
        // debug timer omitted
    }

public:

    UdpBridge() : rclcpp::Node("udp_soft_time_sync") {
        RCLCPP_INFO(this->get_logger(), "UdpBridge ctor: start");
        //Acquire LOCAL IP
        char ip[16];
        memset(ip, 0, sizeof(ip));
        if (get_local_ip(ip) != 0) {
            RCLCPP_WARN(this->get_logger(), "get_local_ip failed, defaulting to 127.0.0.1");
            strncpy(ip, "127.0.0.1", sizeof(ip)-1);
        }
        local_ip = ip;
        RCLCPP_INFO(this->get_logger(), "Local IP: %s", local_ip.c_str());

        //Set DRONE ID
        uint8_t ip_c[4] = {0};
        StringIp2CharIp(local_ip, ip_c);
        local_id = ip_c[3] - 100;

        //Set Broadcast IP
        {
            uint8_t bcast[4] = {ip_c[0], ip_c[1], ip_c[2], 255};
            CharIp2StringIp(bcast, broadcast_ip);
        }

        print(fg(color::lime_green), " -- [BROAD IP]: {}\n", broadcast_ip);
        print(fg(color::lime_green), " -- [LOCAL IP]: {}\n", local_ip);
        print(fg(color::lime_green), " -- [DRONE ID]: {}\n", local_id);
        RCLCPP_INFO(this->get_logger(), "Broadcast IP: %s, Drone ID: %d", broadcast_ip.c_str(), local_id);

        // write to log and shutdown
        try {
            auto share_dir = ament_index_cpp::get_package_share_directory("udp_bridge");
            offset_path = share_dir + "/config";
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "get_package_share_directory failed: %s. Fallback to ROOT_DIR/config", e.what());
            offset_path = string(ROOT_DIR) + string("config");
        }
        RCLCPP_INFO(this->get_logger(), "Creating config/log dir: %s", offset_path.c_str());
        boost::filesystem::create_directories(offset_path);
        offset_path += "/teammate_" + GetSystemTime() + ".txt";
        save_offset.open(offset_path, ios::out);
        RCLCPP_INFO(this->get_logger(), "Offset file: %s", offset_path.c_str());

        teammate_id_by_traj_matching.clear();

        //Init fd for sending IP and drone ID
        udp_send_ip_fd_ptr_ = InitUdpBoardcast(UDP_PORT);
        RCLCPP_INFO(this->get_logger(), "InitUdpBoardcast done, fd=%d", udp_send_ip_fd_ptr_);
        udp_callback_thread_ = new boost::thread(boost::bind(&UdpBridge::UdpCallback, this));
        RCLCPP_INFO(this->get_logger(), "Spawned UdpCallback thread");
        using namespace std::chrono_literals;
        broadcast_timer_ = this->create_wall_timer(1000ms, std::bind(&UdpBridge::BroadcastCallback, this));
        sync_timer_ = this->create_wall_timer(100ms, std::bind(&UdpBridge::SyncRequestCallback, this));
        drone_state_timer_ = this->create_wall_timer(200ms, std::bind(&UdpBridge::DroneStateTimerCallback, this));
        RCLCPP_INFO(this->get_logger(), "Timers created");

        team_status_pub_ = this->create_publisher<swarm_msgs::msg::TeamStatus>("/team_status", 10);
        QuadState_pub_ = this->create_publisher<swarm_msgs::msg::QuadStatePub>("/quadstate_from_teammate", 10);
        GlobalExtrinsic_pub_ = this->create_publisher<swarm_msgs::msg::GlobalExtrinsicStatus>("/global_extrinsic_from_teammate", 10);
        ST_OffsetStatus_pub_ = this->create_publisher<swarm_msgs::msg::SpatialTemporalOffsetStatus>("/spatial_temporal_offset", 10);
        QuadState_sub_ = this->create_subscription<swarm_msgs::msg::QuadStatePub>(
            "/quadstate_to_teammate", 10, std::bind(&UdpBridge::QuadStateCallback, this, std::placeholders::_1));
        GlobalExtrinsic_sub_ = this->create_subscription<swarm_msgs::msg::GlobalExtrinsicStatus>(
            "/global_extrinsic_to_teammate", 10, std::bind(&UdpBridge::GlobalExtrinsicCallback, this, std::placeholders::_1));
        TeammateListTraj_sub_ = this->create_subscription<swarm_msgs::msg::ConnectedTeammateList>(
            "/teammate_id_with_traj_matching", 10, std::bind(&UdpBridge::TeammateListTrajCallback, this, std::placeholders::_1));
        Battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/mavros/battery", 10, std::bind(&UdpBridge::BatteryStatusCallback, this, std::placeholders::_1));
        DroneState_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/mpc/drone_state", 10, std::bind(&UdpBridge::DroneStateCallback, this, std::placeholders::_1));
        log_writer_.open(DEBUG_FILE_DIR("udp_debug.txt"), ios::out);
        fout_delay.open(DEBUG_FILE_DIR("udp_delay.txt"), ios::out);
        initDataCallback();
        udp_start_time_ = now();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(this->get_logger(), "UdpBridge ctor: done");
    }

    ~UdpBridge() {
        close(udp_server_fd_);
        log_writer_.close();
        fout_delay.close();
    }

    void BroadcastCallback() {
        //Write team status into .txt
        if (exit_process) {
            //Write down the time offset

//                   |             |               |
//         local_id  | teammate_id | offset_time(s)| teammate_ip
//                   |             |               |

            for (auto it = teammates.begin(); it != teammates.end(); it++) {
                auto &drone = it->second;
                if (!drone.write_done_) {
                    if (drone.sync_done_) {
                        save_offset << local_id << " " << drone.id_ << " " << drone.offset_time_ << " " << drone.ip_ << endl;
                        drone.write_done_ = true;
                    }
                }
            }
            save_offset.close();
            rclcpp::shutdown();
        }
        //Broadcast local Ip and Id
        SendLocalIp();

        //Publish team bimap
        swarm_msgs::msg::TeamStatus team_msg;
        team_msg.my_drone_id = local_id;
        for (auto it = teammates.begin(); it != teammates.end(); it++) {
            swarm_msgs::msg::TeammateInfo teammate_info_msg;
            teammate_info_msg.is_connect = false;
            auto &drone = it->second;
            double cur_time = now().seconds();
            if (drone.is_connect(cur_time))
                teammate_info_msg.is_connect = true;
            teammate_info_msg.id = drone.id_;

            //boost array to unsigned char
            uint8_t *ip_c = new uint8_t[4];
            StringIp2CharIp(drone.ip_, ip_c);
            for (int i = 0; i < 4; i++) {
                teammate_info_msg.ip[i] = ip_c[i];
            }
            team_msg.teammate_info.push_back(teammate_info_msg);
        }
        team_status_pub_->publish(team_msg);
    }

    void SyncRequestCallback() {
        //map的遍历
        for (auto it = teammates.begin(); it != teammates.end(); it++) {
            auto &drone = it->second;
            if (!drone.sync_done_) {
                if (!drone.offset_ts_.empty())
                    print(" -- [Sync to] UAV{} {}/30.\n", drone.id_, drone.offset_ts_.size());
                CallSyncRequest(drone);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void UdpCallback() {
        int valread;
        struct sockaddr_in addr_client;
        socklen_t addr_len = sizeof(addr_client);
        RCLCPP_INFO(this->get_logger(), "UdpCallback thread: starting");

        // Connect
        if (BindToUdpPort(UDP_PORT, udp_server_fd_) < 0) {
            RCLCPP_ERROR(get_logger(), "[bridge_node]Socket receiver creation error!");
            exit(EXIT_FAILURE);
        }
        RCLCPP_INFO(this->get_logger(), "UDP bind OK on port %d, server_fd=%d", UDP_PORT, udp_server_fd_);

        while (true) {
            addr_len = sizeof(addr_client);
            if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUFFER_SIZE, 0, (struct sockaddr *) &addr_client,
                                    (socklen_t *) &addr_len)) < 0) {
                perror("recvfrom() < 0, error:");
                exit(EXIT_FAILURE);
            }
            // Minimal heartbeat log every N packets
            static int recv_cnt = 0;
            if ((++recv_cnt % 1000) == 0) {
                RCLCPP_DEBUG(this->get_logger(), "recvfrom ok, bytes=%d", valread);
            }
            rclcpp::Time t2 = now();
            // 收到所有的消息，如果分离出来不是用于时间同步的，就直接continue掉
            char *ptr = udp_recv_buf_;
            switch (*((MESSAGE_TYPE *) ptr)) {
                //如果收到的消息是发来的ip和id
                case MESSAGE_TYPE::IP_ID: {
                    IpIdMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    ip_id_data_.update_lock_.lock();
                    ip_id_data_.latest_msg = rcv_msg;
                    ip_id_data_.rcv_WT = t2;
                    ip_id_data_.rcv_new_msg = true;
                    ip_id_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::TIME_SYNC: {
                    // 收到时间同步消息，首先判断client的ID是不是自己
                    TimeSyncMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    time_sync_data_.update_lock_.lock();
                    time_sync_data_.latest_msg = rcv_msg;
                    time_sync_data_.rcv_WT = t2;
                    time_sync_data_.rcv_new_msg = true;
                    time_sync_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::QUAD_STATE: {
                    QuadStateCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    quad_state_data_.update_lock_.lock();
                    quad_state_data_.latest_msg = rcv_msg;
                    quad_state_data_.rcv_WT = t2;
                    quad_state_data_.rcv_new_msg = true;
                    quad_state_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::GLOBAL_EXTRINSIC: {
                    GlobalExtrinsicStatusCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    global_extrinsic_data_.update_lock_.lock();
                    global_extrinsic_data_.latest_msg = rcv_msg;
                    global_extrinsic_data_.rcv_WT = t2;
                    global_extrinsic_data_.rcv_new_msg = true;
                    global_extrinsic_data_.update_lock_.unlock();
                    break;
                }
                case MESSAGE_TYPE::GCS_CMD: {
                    cout<<"Received GCS Msg"<<endl;
                    GCSCmdMsgCvt rcv_msg;
                    DecodeMsgFromBuffer(rcv_msg);
                    GCSCmdMsg_data_.update_lock_.lock();
                    GCSCmdMsg_data_.latest_msg = rcv_msg;
                    GCSCmdMsg_data_.rcv_WT = t2;
                    GCSCmdMsg_data_.rcv_new_msg = true;
                    GCSCmdMsg_data_.update_lock_.unlock();
                    break;
                }
//                case MESSAGE_TYPE::DEBUG: {
//                    DebugMsgCvt rcv_msg;
//                    DecodeMsgFromBuffer(rcv_msg);
//                    debug_data_.update_lock_.lock();
//                    debug_data_.latest_msg = rcv_msg;
//                    debug_data_.rcv_WT = t2;
//                    debug_data_.rcv_new_msg = true;
//                    debug_data_.update_lock_.unlock();
//                    break;
//                }
                default:
                    break;
            }
        }
    }

    void DebugProcessCallback() {
        for (auto it = teammates.begin(); it != teammates.end(); it++) {
            auto &drone = it->second;
            if (drone.sync_done_) {
                DebugMsgCvt cvt;
                static int seq = 0;
                cvt.data.seq = seq;
                seq++;
                for (int i = 0; i < 100; i++) {
                    cvt.data.payload[i] = i;
                }
                rclcpp::Time t1 = now();
                int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
                std::vector<char> send_buf(len * 5);
                cvt.data.header.sec = (int32_t)(t1.nanoseconds() / 1000000000ll);
                cvt.data.header.nsec = (uint32_t)(t1.nanoseconds() % 1000000000ll);
                EncodeMsgToBuffer(MESSAGE_TYPE::DEBUG, cvt, send_buf.data());
                if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &(drone.addr_udp_send_),
                           sizeof(drone.addr_udp_send_)) <= 0) {
                    RCLCPP_ERROR(get_logger(), "UDP SEND ERROR!!!");
                    printf("errno is: %d\n", errno);
                }
            }
        }
    }

private:
    void CallSyncRequest(const Teammate &drone) {
        TimeSyncMsgCvt cvt;
        cvt.data.client_id = local_id;
        cvt.data.server_id = drone.id_;
        rclcpp::Time t1 = now();
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        cvt.data.t1.sec = (int32_t)(t1.nanoseconds() / 1000000000ll);
        cvt.data.t1.nsec = (uint32_t)(t1.nanoseconds() % 1000000000ll);
        EncodeMsgToBuffer(MESSAGE_TYPE::TIME_SYNC, cvt, send_buf.data());
        if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &(drone.addr_udp_send_),
                   sizeof(drone.addr_udp_send_)) <= 0) {
            RCLCPP_ERROR(get_logger(), "UDP SEND ERROR!!!");
            printf("errno is: %d\n", errno);
        }
//        print("Call a sync request to Drone {}\n", drone.id_);
    }

    void SendLocalIp() {
        IpIdMsgCvt cvt;
        StringIp2CharIp(local_ip, cvt.data.local_ip);
        cvt.data.local_id = local_id;
        cvt.data.udp_start_time.sec = (int32_t)(udp_start_time_.nanoseconds() / 1000000000ll);
        cvt.data.udp_start_time.nsec = (uint32_t)(udp_start_time_.nanoseconds() % 1000000000ll);
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        EncodeMsgToBuffer(MESSAGE_TYPE::IP_ID, cvt, send_buf.data());
        if (sendto(udp_send_ip_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &addr_udp_send_ip_,
                   sizeof(addr_udp_send_ip_)) <= 0) {
            RCLCPP_ERROR(get_logger(), "UDP BROADCAST ERROR !!!");
        }
//     print("Broadcast local Ip and Id.\n");
    }

    /*
     * In this function, we will encode a ros_msg type and its type id to a
     * serialzed msg(uint32_t), and the msg type id should be in the first
     * bite.
     * */
    template<typename union_msg>
    int EncodeMsgToBuffer(const MESSAGE_TYPE msg_type_id, union_msg &msg, char *send_buf_) {
        uint32_t msg_size = sizeof(msg.binary);
        int len = msg_size + 2 * sizeof(uint32_t);
        auto ptr = (uint8_t *) (send_buf_);
        *((MESSAGE_TYPE *) ptr) = msg_type_id;
        ptr += sizeof(MESSAGE_TYPE);
        *((uint32_t *) ptr) = msg_size;
        ptr += sizeof(uint32_t);
        memcpy(ptr, msg.binary, msg_size);
        return len;
    }

    /*
        * In this function, we will decode a ros_msg
        * */
    template<typename union_msg>
    int DecodeMsgFromBuffer(union_msg &msg) {
        auto ptr = (uint8_t *) (udp_recv_buf_ + sizeof(uint32_t));
        uint32_t msg_size = *((uint32_t *) ptr);
        ptr += sizeof(uint32_t);
        memcpy(msg.binary, ptr, msg_size);
        return msg_size + sizeof(uint32_t) * 2;
    }

    int InitUdpUnicast(Teammate &drone, const int &port) {
        string ip_s = drone.ip_;
        const char *ip = ip_s.c_str();
        int fd;

        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            RCLCPP_ERROR(get_logger(), "[udo_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }


        drone.addr_udp_send_.sin_family = AF_INET;
        drone.addr_udp_send_.sin_port = htons(port);

        if (inet_pton(AF_INET, ip, &drone.addr_udp_send_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }
        return fd;
    }

    int InitUdpBoardcast(const int port) {
        int fd;

        if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0) {
            RCLCPP_ERROR(get_logger(), "[udp_bridge] Socket sender creation error!");
            exit(EXIT_FAILURE);
        }

        //将udpfd的属性设置为广播
        int so_broadcast = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0) {
            cout << "Error in setting Broadcast option";
            exit(EXIT_FAILURE);
        }

        addr_udp_send_ip_.sin_family = AF_INET;
        addr_udp_send_ip_.sin_port = htons(port);

        if (inet_pton(AF_INET, broadcast_ip.c_str(), &addr_udp_send_ip_.sin_addr) <= 0) {
            printf("\nInvalid address/ Address not supported \n");
            return -1;
        }

        return fd;
    }

    int BindToUdpPort(const int port, int &server_fd) {
        struct sockaddr_in address;
        int opt = 1;

        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                       &opt, sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        // Forcefully attaching socket to the port
        if (bind(server_fd, (struct sockaddr *) &address,
                 sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        return server_fd;
    }

    void TeammateListTrajCallback(const swarm_msgs::msg::ConnectedTeammateList::SharedPtr msg){
        if(teammate_id_by_traj_matching.size() != msg->connected_teammate_id.size()){
            traj_buffer_mtx.lock();
            teammate_id_by_traj_matching.clear();
            for(size_t i = 0; i < msg->connected_teammate_id.size(); i++){
                teammate_id_by_traj_matching.push_back(msg->connected_teammate_id[i]);
            }
            traj_buffer_mtx.unlock();
        }
    }


    void GlobalExtrinsicCallback(const swarm_msgs::msg::GlobalExtrinsicStatus::SharedPtr msg) {
        GlobalExtrinsicStatusCvt cvt;
        cvt.data.header.sec = msg->header.stamp.sec;
        cvt.data.header.nsec = msg->header.stamp.nanosec;
        cvt.data.drone_id = msg->drone_id;
        //Reset global extrinsic
        for (size_t i = 0; i < teammate_id_by_traj_matching.size(); i++) {
            cvt.data.extrinsic[i].teammate_id = 255;
        }
        //Global Extrinsic
        int id_counter = 0;
        for (size_t i = 0; i < msg->extrinsic.size(); i++) {
            traj_buffer_mtx.lock();
            auto iter = find(teammate_id_by_traj_matching.begin(), teammate_id_by_traj_matching.end(), msg->extrinsic[i].teammate_id);
            if(iter == teammate_id_by_traj_matching.end()){
                continue;
            }
            traj_buffer_mtx.unlock();
            cvt.data.extrinsic[id_counter].teammate_id = msg->extrinsic[i].teammate_id;
            for (int j = 0; j < 3; ++j) {
                cvt.data.extrinsic[id_counter].rot_deg[j] = msg->extrinsic[i].rot_deg[j];
                cvt.data.extrinsic[id_counter].trans[j] = msg->extrinsic[i].trans[j];
                cvt.data.extrinsic[id_counter].world_to_gravity_deg[j] = msg->extrinsic[i].world_to_gravity_deg[j];
            }
            id_counter++;
        }
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        EncodeMsgToBuffer(MESSAGE_TYPE::GLOBAL_EXTRINSIC, cvt, send_buf.data());
        static int cnt[MAX_UAV_NUM] = {0};
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            cnt[drone.id_]++;
            if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0,
                       (struct sockaddr *) &drone.addr_udp_send_, sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(get_logger(), "Global Extrinsic SEND ERROR !!!");
            } else if (cnt[drone.id_] % 10 == 0)
                cout << "UAV" << int(msg->drone_id) << " Send Global Extrinsic to UAV" << drone.id_ << endl;
        }


        //Publish Spatial Temporal Offset Status
        swarm_msgs::msg::SpatialTemporalOffsetStatus st_offset_status;
        st_offset_status.st_offset.clear();
        st_offset_status.header.stamp = msg->header.stamp;
        st_offset_status.drone_id = msg->drone_id;
        Vector3d world_to_gravity_rad(msg->world_to_gravity_deg[0],
                                      msg->world_to_gravity_deg[1],
                                      msg->world_to_gravity_deg[2]);
        world_to_gravity_rad /= 57.3;
        Matrix4d world_to_gravity_i = Matrix4d::Identity();
        world_to_gravity_i.block<3, 3>(0, 0) = EulerToRotM(world_to_gravity_rad);

        for (size_t i = 0; i < msg->extrinsic.size(); i++) {
            swarm_msgs::msg::SpatialTemporalOffset st_offset;
            st_offset.teammate_id = msg->extrinsic[i].teammate_id;

            //Global extrinsic --> Gravity frame extrinsic
            Vector3d world_to_gravity_teammate_rad(msg->extrinsic[i].world_to_gravity_deg[0],
                                                   msg->extrinsic[i].world_to_gravity_deg[1],
                                                   msg->extrinsic[i].world_to_gravity_deg[2]);
            world_to_gravity_teammate_rad /= 57.3;
            Matrix4d gravity_to_world_j = Matrix4d::Identity();
            gravity_to_world_j.block<3, 3>(0, 0) = EulerToRotM(world_to_gravity_teammate_rad).transpose();

            Matrix4d world_j_to_world_i = Matrix4d::Identity();
            Vector3d world_j_to_world_i_rad(msg->extrinsic[i].rot_deg[0],
                                            msg->extrinsic[i].rot_deg[1],
                                            msg->extrinsic[i].rot_deg[2]);
            world_j_to_world_i_rad /= 57.3;
            world_j_to_world_i.block<3, 3>(0, 0) = EulerToRotM(world_j_to_world_i_rad);
            world_j_to_world_i.block<3, 1>(0, 3) = Vector3d(msg->extrinsic[i].trans[0],
                                                            msg->extrinsic[i].trans[1],
                                                            msg->extrinsic[i].trans[2]);
            Matrix4d gravity_j_to_gravity_i = world_to_gravity_i * world_j_to_world_i * gravity_to_world_j;
            Matrix3d gravity_j_to_gravity_i_rot = gravity_j_to_gravity_i.block<3, 3>(0, 0);
            Vector3d gravity_j_to_gravity_i_trans = gravity_j_to_gravity_i.block<3, 1>(0, 3);
            Vector3d gravity_j_to_gravity_i_deg = RotMtoEuler(gravity_j_to_gravity_i_rot) * 57.3;

            for (int j = 0; j < 3; ++j) {
                st_offset.rot_deg[j] = gravity_j_to_gravity_i_deg(j);
                st_offset.trans[j] = gravity_j_to_gravity_i_trans(j);
            }

            Quaterniond q(gravity_j_to_gravity_i_rot);
            st_offset.rot_quaternion[0] = q.w();
            st_offset.rot_quaternion[1] = q.x();
            st_offset.rot_quaternion[2] = q.y();
            st_offset.rot_quaternion[3] = q.z();

            auto iter = teammates.find(int(st_offset.teammate_id));
            if (iter != teammates.end()) {
                st_offset.time_offset = iter->second.offset_time_;
                st_offset_status.st_offset.push_back(st_offset);
            }
        }
        ST_OffsetStatus_pub_->publish(st_offset_status);
    }

    void QuadStateCallback(const swarm_msgs::msg::QuadStatePub::SharedPtr msg) {
        QuadStateCvt cvt;
        cvt.data.header.sec = msg->header.stamp.sec;
        cvt.data.header.nsec = msg->header.stamp.nanosec;
        cvt.data.drone_id = msg->drone_id;
        cvt.data.quat_w = msg->pose.pose.orientation.w;
        cvt.data.quat_x = msg->pose.pose.orientation.x;
        cvt.data.quat_y = msg->pose.pose.orientation.y;
        cvt.data.quat_z = msg->pose.pose.orientation.z;
        cvt.data.pos[0] = msg->pose.pose.position.x;
        cvt.data.pos[1] = msg->pose.pose.position.y;
        cvt.data.pos[2] = msg->pose.pose.position.z;
        for (int i = 0; i < 3; i++) {
            cvt.data.gyr[i] = msg->gyr[i];
            cvt.data.vel[i] = msg->vel[i];
            cvt.data.world_to_gravity_deg[i] = msg->world_to_gravity_deg[i];
        }
        for (int i = 0; i < 12; ++i) {
            cvt.data.pose_cov[i] = msg->pose_cov[i];
        }
        cvt.data.degenerated = msg->degenerated;
        cvt.data.swarmlio_start_time = msg->swarmlio_start_time;
        //Teammates
        //Reset is_observe
        for (int i = 0; i < MAX_UAV_NUM; i++) {
            cvt.data.teammate[i].is_observe = false;
        }

        for (size_t i = 0; i < msg->teammate.size(); i++) {
            cvt.data.teammate[i].is_observe = msg->teammate[i].is_observe;
            cvt.data.teammate[i].teammate_id = msg->teammate[i].teammate_id;
            for (int j = 0; j < 3; j++) {
                cvt.data.teammate[i].observed_pos[j] = msg->teammate[i].observed_pos[j];
            }
        }
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        EncodeMsgToBuffer(MESSAGE_TYPE::QUAD_STATE, cvt, send_buf.data());
        static int cnt[MAX_UAV_NUM] = {0};
        //逐个单播
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            cnt[drone.id_]++;
            if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                       sizeof(drone.addr_udp_send_)) <= 0) {
                RCLCPP_ERROR(get_logger(), "QUADSTATE SEND ERROR !!!");
            } else if (cnt[drone.id_] % 50 == 0)
                cout << "UAV" << int(msg->drone_id) << ", sent quadstate to UAV" << drone.id_ << endl;
        }
    }

    void BatteryStatusCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg){
        if(msg->voltage <= 20.4 || msg->voltage >= 25.2)
            return;
        float remaining = (msg->voltage - 20.4) / (25.2 - 20.4) * 100;
        BatteryStatusMsgCvt cvt;
        cvt.data.remaining = int(remaining);
        cvt.data.drone_id = local_id;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        EncodeMsgToBuffer(MESSAGE_TYPE::BATTERY_STATUS, cvt, send_buf.data());
        static int cnt= 0;
        //发给地面站
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            //only send to Ground Station
            if(drone.id_ == 0){
                cnt++;
                if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                           sizeof(drone.addr_udp_send_)) <= 0) {
                    RCLCPP_ERROR(get_logger(), "QUADSTATE SEND ERROR !!!");
                } else if (cnt % 20 == 0)
                    cout << "UAV" << int(local_id) << ", sent battery info to Ground Station." << endl;
                break;
            }
        }
    }

    void DroneStateTimerCallback(){
        Drone_StateMsgCvt cvt;
        cvt.data.drone_id = local_id;
        cvt.data.drone_state = drone_state;
        int len = sizeof(cvt.binary) + 2 * sizeof(uint32_t);
        std::vector<char> send_buf(len * 5);
        EncodeMsgToBuffer(MESSAGE_TYPE::DRONE_STATE, cvt, send_buf.data());

        //发给地面站
        static int cnt = 0;
        for (auto iter = teammates.begin(); iter != teammates.end(); iter++) {
            auto &drone = iter->second;
            //only send to Ground Station
            if(drone.id_ == 0){
                if (sendto(drone.udp_send_fd_ptr_, send_buf.data(), len, 0, (struct sockaddr *) &drone.addr_udp_send_,
                           sizeof(drone.addr_udp_send_)) <= 0) {
                    RCLCPP_ERROR(get_logger(), "QUADSTATE SEND ERROR !!!");
                } else if(cnt % 5 == 0){
                    cout << "UAV" << int(local_id) << ", sent drone state to Ground Station." << endl;
                    cnt++;
                }
                break;
            }
        }
    }

    void DroneStateCallback(const std_msgs::msg::Int8::SharedPtr msg){
        drone_state = msg->data;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    auto node = std::make_shared<UdpBridge>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
