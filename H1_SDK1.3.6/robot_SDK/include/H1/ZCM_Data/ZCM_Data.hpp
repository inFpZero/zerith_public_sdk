#ifndef __ZCM_DATA_H__
#define __ZCM_DATA_H__

#include <zcm/zcm-cpp.hpp>
#include <mutex>
#include <chrono>
#include <string>
#include <iostream>
#include <atomic>
#include "config.hpp"
#include "ImuData.hpp"
#include "UpperJointControl.hpp"
#include "UpperJointState.hpp"
#include "SystemInit.hpp"
#include "ChassisControl.hpp"
#include "ChassisState.hpp"
#include "WaistControl.hpp"
#include "WaistState.hpp"
#include "GripperControl.hpp"
#include "GripperState.hpp"
#include "GripperControlMode.hpp"
#include "HeadControl.hpp"
#include "HeadState.hpp"
#include "FixedRod.hpp"
#include "XboxMap.hpp"
#include "RobotInfo.hpp"
#include "ControlMode.hpp"
#include "PowerState.hpp"
#include "CommunicationBoardState.hpp"
#include "ArmEndPose.hpp"
#include "ForceSensorControl.hpp"
#include "ForceSensorState.hpp"
#include "HandControl.hpp"
#include "HandState.hpp"
#include "HighLevelControl.hpp"
#include "HighLevelState.hpp"

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
    SystemInit system_init_cmd{};                     // 系统初始化控制指令
    SystemInit system_init_state{};                   // 系统初始化反馈指令
    SystemInit  reset_signal{};                       // 重置位置控制指令
    SystemInit  reset_state{};                        // 重置位置状态反馈   


    ChassisControl chassis_control{};                 // 底盘控制
    ChassisState chassis_state{};                     // 底盘状态

    WaistControl waist_control{};                     // 腰部控制
    WaistState waist_state{};                         // 腰部反馈

    HeadControl head_control{};                       // 头部控制
    HeadState head_state{};                           // 头部状态

    UpperJointControl upper_joint_control{};         // 上肢关节控制
    UpperJointState upper_joint_state{};             // 上肢关节状态
    ArmEndPose    left_arm_target{};                  // 左臂目标末端位姿
    ArmEndPose    right_arm_target{};                 // 右臂目标末端位姿
    ArmEndPose    left_arm_state{};                   // 左臂实际末端位姿
    ArmEndPose    right_arm_state{};                  // 右臂实际末端位姿

    GripperControl gripper_control{};                 // 夹爪控制
    GripperState gripper_state{};                     // 夹爪状态
    GripperControlMode gripper_control_mode{};       // 夹爪控制模式
    GripperControlMode gripper_control_mode_state{}; // 夹爪控制模式状态

    FixedRod fixed_rod_control{};                     // 固定杆控制
    FixedRod fixed_rod_state{};                       // 固定杆状态

    XboxMap xbox_map_control{};                       // Xbox手柄控制
    XboxMap xbox_map_state{};                         // Xbox手柄状态

    ImuData imu_state{};                       // IMU状态

    ForceSensorControl force_sensor_control{};   // 力传感器控制
    ForceSensorState force_sensor_state{};       // 力传感器状态

    HandControl left_hand_control{};                     // 手部控制
    HandControl right_hand_control{};                     // 手部控制
    HandState left_hand_state{};                         // 手部状态
    HandState right_hand_state{};                         // 手部状态

    HighLevelControl high_level_control{};         // 高级控制指令
    HighLevelState high_level_state{};             // 高级状态反馈

    RobotInfo robot_info{};                           // 机器人基本信息
    ControlMode control_mode{};                       // 控制模式
    ControlMode control_mode_state{};                 // 控制模式状态

    PowerState power_state{}; //电池状态
    CommunicationBoardState communication_board_state{}; // 通信板状态
    // 机器人状态

    std::atomic<bool> is_info_received{false}; // 系统是否初始化
    std::atomic<bool> if_control_ok{false};//是否可以控制

    // int64_t last_heartbeat_received_;
    std::atomic<int64_t> last_heartbeat_received_{0};  // 原: int64_t

    friend class H1Robot; 
   

private:
    // 初始化函数
    void initialize();
    void subscribeChannels();
    
    // 刷新最近一次任意消息的接收时间（steady clock）
    void touchHeartbeatRx();

    // ZCM 回调函数
    // ===== 状态反馈 =====
    void System_init_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const SystemInit* msg);
    void reset_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const SystemInit* msg);
    void Chassis_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const ChassisState* msg);
    void Waist_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                          const std::string& chan, 
                          const WaistState* msg);
    void Head_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                         const std::string& chan, 
                         const HeadState* msg);
    void upper_joint_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const UpperJointState* msg);
    void left_arm_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                             const std::string& chan, 
                             const ArmEndPose* msg);
    void right_arm_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                              const std::string& chan, 
                              const ArmEndPose* msg);
    void Gripper_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                        const std::string& chan, 
                        const GripperState* msg);
    void Gripper_control_mode_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan,
                            const GripperControlMode* msg);
    void Fixed_rod_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                        const std::string& chan, 
                        const FixedRod* msg);
    void xbox_map_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const XboxMap* msg);
    void IMU_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const ImuData* msg);
    void Force_sensor_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const ForceSensorState* msg);
    void Hand_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const HandState* msg);
    void HighLevel_state_ZCM(const zcm::ReceiveBuffer* rbuf, 
                            const std::string& chan, 
                            const HighLevelState* msg);
    void robot_info_ZCM(const zcm::ReceiveBuffer* rbuf, 
                         const std::string& chan,
                            const RobotInfo* msg);    
    void control_mode_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const ControlMode* msg);
    void power_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const PowerState* msg);
    void communication_board_state_ZCM(const zcm::ReceiveBuffer* rbuf,
                                         const std::string& chan,
                                         const CommunicationBoardState* msg);
};

#endif