#ifndef __ZCM_DATA_H__
#define __ZCM_DATA_H__

#include <zcm/zcm-cpp.hpp>
#include <mutex>
#include <chrono>
#include <string>
#include <iostream>
#include <atomic>
#include "config.hpp"
#include "state_estimator_zcmt.hpp"
// #include "rc_command_zcmt.hpp"
#include "upper_joint_control_zcmt.hpp"
#include "upper_joint_state_zcmt.hpp"
#include "system_init_zcmt.hpp"
#include "chassis_control_zcmt.hpp"
#include "chassis_state_zcmt.hpp"
#include "ChassisState.hpp"
#include "waist_control_zcmt.hpp"
#include "waist_state_zcmt.hpp"
#include "gripper_control_zcmt.hpp"
#include "gripper_state_zcmt.hpp"
#include "gripper_control_mode_zcmt.hpp"
#include "head_control_zcmt.hpp"
#include "head_state_zcmt.hpp"
#include "fixed_rod_zcmt.hpp"
#include "xbox_map_zcmt.hpp"
#include "robot_info_zcmt.hpp"
#include "error_log_zcmt.hpp"
#include "control_mode_zcmt.hpp"
#include "power_state_zcmt.hpp"
#include "communication_board_zcmt.hpp"
#include "arm_actions_zcmt.hpp"


class ZCM_Data
{
public:
    // 共享内存模式构造函数
    ZCM_Data();
    // UDP多播模式构造函数
    ZCM_Data(const std::string& udp_url);
    // 通用构造函数
    ZCM_Data(const std::string& internal_url, const std::string& udp_url);


    template <typename T>
    void publish(const std::string& channel, const T& msg) {
        _udpZCM.publish(channel, &msg);

        #ifdef LOG_PUBLISH
        std::cout << "Published " << channel << std::endl;
        #endif
    }


    ~ZCM_Data();
    
    // 枚举类

    // ZCM 实例
    zcm::ZCM _internalZCM;
    zcm::ZCM _udpZCM;

    // 数据成员 
    system_init_zcmt system_init_cmd{};                     // 系统初始化控制指令
    system_init_zcmt system_init_state{};                   // 系统初始化反馈指令
    system_init_zcmt  reset_signal{};                       // 重置位置控制指令
    system_init_zcmt  reset_state{};                        // 重置位置状态反馈   


    chassis_control_zcmt chassis_control{};                 // 底盘控制
    ChassisState chassis_state{};                     // 底盘状态

    waist_control_zcmt waist_control{};                     // 腰部控制
    waist_state_zcmt waist_state{};                         // 腰部反馈

    head_control_zcmt head_control{};                       // 头部控制
    head_state_zcmt head_state{};                           // 头部状态

    upper_joint_control_zcmt upper_joint_control{};         // 上肢关节控制
    upper_joint_state_zcmt upper_joint_state{};             // 上肢关节状态
    arm_actions_zcmt    left_arm_target{};                  // 左臂目标末端位姿
    arm_actions_zcmt    right_arm_target{};                 // 右臂目标末端位姿
    arm_actions_zcmt    left_arm_state{};                   // 左臂实际末端位姿
    arm_actions_zcmt    right_arm_state{};                  // 右臂实际末端位姿

    gripper_control_zcmt gripper_control{};                 // 夹爪控制
    gripper_state_zcmt gripper_state{};                     // 夹爪状态
    gripper_control_mode_zcmt gripper_control_mode{};       // 夹爪控制模式
    gripper_control_mode_zcmt gripper_control_mode_state{}; // 夹爪控制模式状态

    fixed_rod_zcmt fixed_rod_control{};                     // 固定杆控制
    fixed_rod_zcmt fixed_rod_state{};                       // 固定杆状态

    xbox_map_zcmt xbox_map_control{};                       // Xbox手柄控制
    xbox_map_zcmt xbox_map_state{};                         // Xbox手柄状态

    state_estimator_zcmt imu_state{};                       // IMU状态

    robot_info_zcmt robot_info{};                           // 机器人基本信息
    error_log_zcmt error_log{};                             // 错误日志
    control_mode_zcmt control_mode{};                       // 控制模式
    control_mode_zcmt control_mode_state{};                 // 控制模式状态

    power_state_zcmt power_state{}; //电池状态
    communication_board_zcmt communication_board_state{}; // 通信板状态
    // 机器人状态

    std::atomic<bool> is_info_received{false}; // 系统是否初始化
    std::atomic<bool> if_control_ok{false};//是否可以控制

    int64_t last_heartbeat_received_;

    friend class H1Robot; 
   

private:
    // 初始化函数
    void initialize();
    void subscribeChannels();
    
    // ZCM 回调函数
    // ===== 状态反馈 =====
    void System_init_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const system_init_zcmt* msg);
    void reset_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const system_init_zcmt* msg);
    void Chassis_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const ChassisState* msg);
    void Waist_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                          const std::string& chan, 
                          const waist_state_zcmt* msg);
    void Head_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                         const std::string& chan, 
                         const head_state_zcmt* msg);
    void upper_joint_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const upper_joint_state_zcmt* msg);
    void left_arm_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                             const std::string& chan, 
                             const arm_actions_zcmt* msg);
    void right_arm_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                              const std::string& chan, 
                              const arm_actions_zcmt* msg);
    void Gripper_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                        const std::string& chan, 
                        const gripper_state_zcmt* msg);
    void Gripper_control_mode_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan,
                            const gripper_control_mode_zcmt* msg);
    void Fixed_rod_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                        const std::string& chan, 
                        const fixed_rod_zcmt* msg);
    void xbox_map_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const xbox_map_zcmt* msg);
    void IMU_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const state_estimator_zcmt* msg);
    void robot_info_ZCM(const zcm::ReceiveBuffer* rbuf, 
                         const std::string& chan,
                            const robot_info_zcmt* msg);    
    void error_log_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const error_log_zcmt* msg);
    void control_mode_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const control_mode_zcmt* msg);
    void power_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const power_state_zcmt* msg);
    void communication_board_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                                         const std::string& chan,
                                         const communication_board_zcmt* msg);
};

#endif