#pragma once
#include <thread>
#include <mutex>
#include <zcm/zcm-cpp.hpp>
#include "Robot.hpp"
#include "ZCM_Data.hpp"
#include "GrpcServiceClient.hpp"
#include "config.hpp"
#include "state_estimator_zcmt.hpp"
#include "upper_joint_control_zcmt.hpp"
#include "upper_joint_state_zcmt.hpp"
#include "system_init_zcmt.hpp"
#include "chassis_control_zcmt.hpp"
#include "chassis_state_zcmt.hpp"
#include "waist_control_zcmt.hpp"
#include "waist_state_zcmt.hpp"
#include "gripper_control_zcmt.hpp"
#include "gripper_state_zcmt.hpp"
#include "head_control_zcmt.hpp"
#include "head_state_zcmt.hpp"
#include "fixed_rod_zcmt.hpp"
// #include "MotorStateMonitor.hpp"



class H1Robot : public RobotBase {
public:
    // 机器人主机内通信
    explicit H1Robot();
    
    // 局域网跨设备通信
    explicit H1Robot(const std::string& udp_url);
    

    ~H1Robot();


    // ===== 模式管理 =====
    // 连接机器人   
    // 返回值：true - 成功，false - 失败
    bool robot_connect();
    // 切换控制模式
    // 返回值：true - 成功，false - 失败
    bool switchControlMode(MotorControlMode new_mode);
    
    // 初始化/反初始化，返回是否成功，失败报错，阻塞到收到状态为止，阻塞期间不能进行其他操作
    //初始化和反初始化互斥
    // 初始化机器人，运动接口
    // 返回值：true - 成功，false - 失败
    bool robot_init() ;
    // 反初始化机器人，运动接口
    // 返回值：true - 成功，false - 失败
    bool robot_deinit() ;
  
    // ===== LOW-LEVEL 接口 =====

    // 直接控制电机
    //参数说明：motor_id - 电机索引,0-22
    //          control - 电机控制参数,如下
    bool setMotorControl_low(EtherCAT_Motor_Index motor_id, const Motor_Control& control);

    // 设置底盘
    // 参数说明：chassis_id - 底盘电机索引, 左轮:0, 右轮:1
    //          control - 电机控制参数,仅支持控制速度
    bool setChassis_low(EtherCAT_Motor_Index chassis_id, const Motor_Control& control);
    // 设置腰部
    // 参数说明：waist_id - 腰部电机索引, 升降2，俯仰:3, 旋转:4
    //          control - 电机控制参数，升降仅支持控制位置，俯仰和旋转支持控制速度，位置和力矩
    bool setWaist_low(EtherCAT_Motor_Index waist_id, const Motor_Control& control);
    // 设置头部
    // 参数说明：head_id - 头部电机索引, 旋转:5, 俯仰:6
    //          control - 电机控制参数，支持控制位置、速度和力矩
    bool setHead_low(EtherCAT_Motor_Index head_id, const Motor_Control& control);
    // 设置手臂
    // 参数说明：arm_id 关节索引, 7-13 - 左臂 肩部->手腕，15-21 右臂 肩部->手腕
    //          control - 电机控制参数，支持控制位置、速度和力矩
    bool setArm_low(EtherCAT_Motor_Index arm_id, const Motor_Control& control);
    // 设置夹爪
    // 夹爪实际为手部的最后一个电机
    // 参数说明：gripper_id - 夹爪电机索引,14 - 左手，22 - 右手
    //          control - 电机控制参数，支持控制位置、速度和力矩
    //          is_hold_torque - 是否保持力矩不变，默认true
    bool setGripper_low(EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque = true);
    // 设置固定杆,0-关闭，1-打开
    bool setFixedRod_low(const int& is_open);

    // ===== 状态监控 =====
    
    // 获取机器人基本信息
    // 返回值：true - 成功，false - 失败
    //可以在机器人未初始化时调用，以及用于判断SDK是否连接上了机器人
    bool getRobotInfo(robot_info_zcmt& state);

    // 获取电机状态
    // 参数说明：motor_id - 电机索引, state - 电机状态结构
    // state.Position_Actual: 当前电机位置
    // state.Speed_Actual: 当前电机速度
    // state.Torque_Actual: 当前电机力矩
    // state.Error_flag: 错误标志位，0表示正常，其他表示电机异常
    // 返回值：true - 成功，false - 失败
    bool getMotorState(EtherCAT_Motor_Index motor_id, Motor_Information& state);
    // 获取底盘状态
    bool getChassisState(EtherCAT_Motor_Index chassis_id, Motor_Information& state);
    // 获取腰部状态
    bool getWaistState(EtherCAT_Motor_Index waist_id, Motor_Information& state);
    // 获取头部状态
    bool getHeadState(EtherCAT_Motor_Index head_id, Motor_Information& state);
    // Motor_Information
    bool getArmState(EtherCAT_Motor_Index joint_id, Motor_Information& state);
    // 获取夹爪状态
    bool getGripperState(EtherCAT_Motor_Index gripper_id, Motor_Information& state);
    // 获取夹爪控制模式，0为保持力矩模式，1为MIT自由控制模式
    bool getGripperControlMode(int& control_mode);
    // 获取固定杆状态
    bool getFixedRodState(int& is_open);

    //获取手柄状态
    //各按键参数
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    bool getJoystickState(xbox_map_zcmt& state);
    //获取IMU状态
    //state.rpy  : 当前IMU的滚转、俯仰、偏航角
    //state.omega: 当前IMU的角速度
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    // 返回值：true - 成功，false - 失败
    bool getIMU_State(state_estimator_zcmt& state);
    //获取电池状态,
    //state.soc：电量，0-100
    //state.avg_time_to_empty：预估剩余时间，单位分，值为65535时表示正在充电
    //state.avg_time_to_full：预估充满时间，单位分，值为65535时表示正在放电
    //以上两个值都为65535时表示为已充满电但仍插上了电源适配器
    // 返回值：true - 成功，false - 失败
    bool getPowerState(power_state_zcmt& state);

    //获取电源充电和放电状态
    bool getPowerChargeState(power_charge_state & state);
    
    // 获取当前模式
    MotorControlMode getCurrentMode();
    // 获取当前初始化状态
    // 返回值：-1 - 未初始化，0 - 初始化中，1 - 已初始化, 2 - 反初始化中，3 - 已反初始化
    InitState getInitState();
    
    bool isRobotConnected(); // 检查机器人是否连接成功

        // 声明友元类
    friend class MotorStateMonitor;

private:

    // ZCM数据处理类实例
    ZCM_Data zcm_data_;

    bool is_robot_connected = false; // 是否连接到机器人

    //直接用ZCM_Data::system_init_zcmt::InitState代替
    // //0表示未初始化，1表示初始化中，2表示已初始化,3表示反初始化中，4表示已反初始化
    MotorControlMode current_mode_{UNINITIALIZED};
    // 互斥锁保护共享数据   
     mutable std::mutex state_mutex_; 

    // 心跳相关成员
    std::atomic<bool> heartbeat_running_{false};
    std::thread heartbeat_thread_;
    std::atomic<bool> connection_timeout_{false};
    
    // 心跳配置
    static constexpr int HEARTBEAT_INTERVAL_MS = 100;  // 发送间隔100ms
    static constexpr int HEARTBEAT_TIMEOUT_MS = 3000;   // 超时时间3s

    //ip
    std::string robot_ip_;
    std::unique_ptr<GrpcServiceClient> grpc_client_; // gRPC客户端成员
    // 心跳相关私有函数
    void heartbeatThreadFunc();
    void checkHeartbeatTimeout();
    bool sendHeartbeat();
    void startHeartbeat();
    void stopHeartbeat();
    void handleConnectionTimeout();
};