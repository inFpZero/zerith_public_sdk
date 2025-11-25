#pragma once
#include <thread>
#include <mutex>
#include <zcm/zcm-cpp.hpp>
#include "Robot.hpp"
#include "ZCM_Data.hpp"
#include "GrpcServiceClient.hpp"
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
#include "HeadControl.hpp"
#include "HeadState.hpp"
#include "ArmEndPose.hpp"
#include "FixedRod.hpp"
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
    //          control - 电机控制参数
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

    // ===== HIGH-LEVEL 接口 =====
                       
    // 设置底盘线速度和角速度
    //   speed_X: 前进, 后退,
    //   speed_y: 左转, 右转,
    // 返回值：true - 成功，false - 失败
    bool setChassis_high(const float& speed_x , const float& speed_y);

    // 设置腰部姿态
    bool setWaist_high(const ArmEndPose& action);

    // 设置头部姿态,实际与低层控制相同
    bool setHead_high(EtherCAT_Motor_Index head_id, const Motor_Control& control);

    // 设置手臂末端姿态
    bool setArm_high(ArmAction arm, const ArmEndPose& action);

    // 设置夹爪末端姿态，实际与低层控制相同
    bool setGripper_high(EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque = true);

    // 设置固定杆,0-关闭，1-打开
    bool setFixedRod_high(const int& is_open);

    // 新增：将传统的 ArmPose(x,y,z,roll,pitch,yaw) 转换为 ArmEndPose(Position[3], Rotation[4]{x,y,z,w})
    // 四元数顺序 (x, y, z, w)
    ArmEndPose armPoseToArmEndPose(const ArmPose& pose);

    // ===== 状态监控 =====
    
    // 获取机器人基本信息
    // 返回值：true - 成功，false - 失败
    //可以在机器人未初始化时调用，以及用于判断SDK是否连接上了机器人
    bool getRobotInfo(RobotInfo& state);

    // 获取电机状态
    // 参数说明：motor_id - 电机索引, state - 电机状态结构
    // state.Position_Actual: 当前电机位置
    // state.Speed_Actual: 当前电机速度
    // state.Torque_Actual: 当前电机力矩
    // state.KP: 当前电机KP
    // state.KD: 当前电机KD
    // state.Error_flag: 错误标志位，0表示正常，其他表示电机异常
    // 返回值：true - 成功，false - 失败
    bool getMotorState(EtherCAT_Motor_Index motor_id, Motor_Information& state);
    // 获取底盘状态
    bool getChassisState(EtherCAT_Motor_Index chassis_id, Motor_Information& state);
    // 获取底盘速度状态，包括实际电机速度和解算后的速度
    // speed_actual: 实际电机速度，0-左轮，1-右轮
    // speed_algo: 解算后的速度，0-线速度，1-角速度
    bool getChassisSpeedState(std::array<float,2> &speed_actual, std::array<float,2> &speed_algo);
    // 获取腰部状态
    bool getWaistState(EtherCAT_Motor_Index waist_id, Motor_Information& state);
    // 获取头部状态
    bool getHeadState(EtherCAT_Motor_Index head_id, Motor_Information& state);
    // Motor_Information
    bool getArmState(EtherCAT_Motor_Index joint_id, Motor_Information& state);
    // 获取high_level或摇操模式下的手臂末端实际位姿
    bool getArmTargetState(ArmAction arm, ArmEndPose& state);
    // 获取夹爪状态
    bool getGripperState(EtherCAT_Motor_Index gripper_id, Motor_Information& state);
    // 获取夹爪控制模式，0为保持力矩模式，1为MIT自由控制模式
    bool getGripperControlMode(int& control_mode);
    // 获取固定杆状态
    bool getFixedRodState(int& is_open);

    //获取手柄状态
    //各按键参数
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    bool getJoystickState(XboxMap& state);
    //获取IMU状态
    //state.rpy  : 当前IMU的滚转、俯仰、偏航角
    //state.omega: 当前IMU的角速度
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    // 返回值：true - 成功，false - 失败
    bool getIMU_State(ImuData& state);

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

    //直接用ZCM_Data::SystemInit::InitState代替
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