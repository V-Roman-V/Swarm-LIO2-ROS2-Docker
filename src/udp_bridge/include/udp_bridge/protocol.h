#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "cstdio"
#include "math.h"
#include "string.h"
#include <ifaddrs.h>

#define MAX_DRONE_ID (31) //集群中最大的飞机编号

#define MAX_UAV_NUM (MAX_DRONE_ID + 1) //集群中可能的最大飞机数量
namespace udp_bridge {

    // The maximum safe UDP payload is 508 bytes.
    // This is a packet size of 576 (the "minimum maximum reassembly buffer size"),
    // minus the maximum 60-byte IP header and the 8-byte UDP header.
    // where
    // char	1 byte
    // int	2 or 4 bytes
    // float	4 bytes
    // double	8 bytes

    enum MESSAGE_TYPE {
        TIME_SYNC = 100,
        IP_ID, //101
        IMU,
        SIM_ODOM,
        QUAD_STATE,
        BAG_TIME,
        GLOBAL_EXTRINSIC,
        TRIGGER_AUTO,
        GCS_CMD,
        BATTERY_STATUS,
        DRONE_STATE,
        DEBUG
    };

    struct Header {
        int32_t sec;
        int32_t nsec;
    };
///================USER DEFINED DATA STRUCTURE======================================
    struct Drone_StateMsg{
        uint8_t drone_id;
        int8_t drone_state;
    };
    union Drone_StateMsgCvt{
        Drone_StateMsg data;
        uint8_t binary[sizeof(Drone_StateMsg)];
    };
    struct BatteryStatusMsg{
        uint8_t drone_id;
        uint8_t remaining; //0 to 100
    };
    union BatteryStatusMsgCvt{
        BatteryStatusMsg data;
        uint8_t binary[sizeof(BatteryStatusMsg)];
    };
    struct GCSCmdMsg{
        uint8_t GCS_CmdSelection;
    };
    union GCSCmdMsgCvt{
        GCSCmdMsg data;
        uint8_t binary[sizeof(GCSCmdMsg)];
    };
    struct TimeSyncMsg {
        Header t1, t2, t3, t4;
        uint8_t server_id; //从机
        uint8_t client_id; //主机
        uint8_t sender_id; //发送者
    };
    union TimeSyncMsgCvt {
        TimeSyncMsg data;
        uint8_t binary[sizeof(TimeSyncMsg)];
    };
//Send my Ip and drone id
    struct IpIdMsg {
        uint8_t local_ip[4]; //四个uint_t存储ip: xx.xx.xx.xx
        uint8_t local_id;
        Header udp_start_time;
    };
    union IpIdMsgCvt {
        IpIdMsg data;
        uint8_t binary[sizeof(IpIdMsg)];
    };

    struct DebugMsg {
        int seq;
        float payload[100];
        Header header;
    };
    union DebugMsgCvt {
        DebugMsg data;
        uint8_t binary[sizeof(DebugMsg)];
    };

//Send my Bag Start Time
    struct BagTimeMsg {
        Header stamp;
    };
    union BagTimeCvt {
        BagTimeMsg data;
        uint8_t binary[sizeof(BagTimeMsg)];
    };

//Send Trigger
    struct TriggerMsg {
        int32_t trigger2auto;
    };
    union TriggerCvt {
        TriggerMsg data;
        uint8_t binary[sizeof(TriggerMsg)];
    };
///================USER DEFINED DATA STRUCTURE======================================
/// 1) define the typename and struct
    struct QuadStateTeammate {
        uint8_t teammate_id;
        bool is_observe;
        float observed_pos[3];
    };
    struct QuadState {
        Header header;
        uint8_t drone_id;
        float pos[3];
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
        float gyr[3];
        float vel[3];
        float world_to_gravity_deg[3];
        float swarmlio_start_time;
        bool degenerated;
        float pose_cov[12];
        QuadStateTeammate teammate[MAX_UAV_NUM];
    };
    union QuadStateCvt {
        QuadState data;
        uint8_t binary[sizeof(QuadState)];
    };
///================USER DEFINED DATA STRUCTURE======================================
    struct GlobalExtrinsic {
        uint8_t teammate_id;
        float rot_deg[3];
        float trans[3];
        float world_to_gravity_deg[3];
    };
    struct GlobalExtrinsicStatus {
        Header header;
        uint8_t drone_id;
        GlobalExtrinsic extrinsic[MAX_UAV_NUM];
    };
    union GlobalExtrinsicStatusCvt {
        GlobalExtrinsicStatus data;
        uint8_t binary[sizeof(GlobalExtrinsicStatus)];
    };

    struct SimOdom {
        Header header;
        uint8_t drone_id;
        uint8_t PC_id;
        float pos[3];
        float quat_w;
        float quat_x;
        float quat_y;
        float quat_z;
    };
    union SimOdomCvt {
        SimOdom data;
        uint8_t binary[sizeof(SimOdom)];
    };


    static void StringIp2CharIp(std::string &str_ip, uint8_t *ch_ip) {
        std::stringstream s(str_ip);
        int data[4];
        char ch; //to temporarily store the '.'
        s >> data[0] >> ch >> data[1] >> ch >> data[2] >> ch >> data[3];
        for (int i = 0; i < 4; i++) {
            ch_ip[i] = data[i];
        }
    }

    static void CharIp2StringIp(uint8_t *ch_ip, std::string &str_ip) {
        str_ip.clear();
        str_ip += std::to_string(ch_ip[0]);
        str_ip.push_back('.');
        str_ip += std::to_string(ch_ip[1]);
        str_ip.push_back('.');
        str_ip += std::to_string(ch_ip[2]);
        str_ip.push_back('.');
        str_ip += std::to_string(ch_ip[3]);
    }


    template<typename T>
    static int sgn(T val) {
        return (T(0) < val) - (val < T(0));
    }


    static int get_local_ip(char *ip) {
        struct ifaddrs *ifAddrStruct = nullptr;
        if (getifaddrs(&ifAddrStruct) != 0 || ifAddrStruct == nullptr) {
            return -1;
        }
        int ret = -1;
        for (struct ifaddrs *ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next) {
            if (ifa->ifa_addr == nullptr) continue;
            if (ifa->ifa_addr->sa_family != AF_INET) continue;
            void *tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
            char buf[INET_ADDRSTRLEN] = {0};
            if (inet_ntop(AF_INET, tmpAddrPtr, buf, INET_ADDRSTRLEN) == nullptr) continue;
            // Skip loopback
            if (strncmp(buf, "127.", 4) == 0) continue;
            // Copy chosen IP
            strncpy(ip, buf, INET_ADDRSTRLEN);
            ret = 0;
            break;
        }
        freeifaddrs(ifAddrStruct);
        return ret;
    }

    std::string GetSystemTime() {
        time_t now = time(NULL);
        tm *t = localtime(&now);
        // 将信息输出到字符串流
        std::stringstream ss;
        ss << t->tm_mon + 1 << "_" <<
           t->tm_mday << "_" << t->tm_hour << "_" << t->tm_min << "_" << t->tm_sec;
        return ss.str();
    }


}

#endif