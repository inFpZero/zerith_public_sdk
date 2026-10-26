#pragma once
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "ClientTypes.hpp"
#include "robot.pb.h"
#include "config.hpp"

using robot_client::ArmEndPose;
using robot_client::CommunicationBoardState;
using robot_client::HighLevelState;
using robot_client::ImuData;
using robot_client::PowerState;
using robot_client::RobotInfo;
using robot_client::SystemInitStatus;
using robot_client::ControlModeStatus;
using robot_client::XboxMap;

class H1Robot {
public:
    // 获取进程内唯一实例；首次调用传入的 IP 决定通信目标。
    static H1Robot& getInstance(
        const std::string& robot_ip = "127.0.0.1");

    virtual ~H1Robot() = default;

    H1Robot(const H1Robot&) = delete;
    H1Robot& operator=(const H1Robot&) = delete;
    H1Robot(H1Robot&&) = delete;
    H1Robot& operator=(H1Robot&&) = delete;


    // ===== 模式管理 =====
    // 连接机器人   
    // 返回值：true - 成功，false - 失败
    virtual bool robot_connect() = 0;
    // 显式申请控制权限。连接成功只提供状态订阅能力，不自动获得控制权。
    virtual bool getControlPermission() = 0;
    // 主动释放控制权限；不会改变机器人当前控制模式或初始化状态。
    virtual bool releaseControlPermission() = 0;
    // 返回当前 SDK 实例记录的控制权限状态。
    virtual bool hasControlPermission() const = 0;

    // 切换控制模式
    // 返回值：true - 成功，false - 失败
    virtual bool switchControlMode(MotorControlMode new_mode) = 0;

    // 初始化/反初始化，返回是否成功，失败报错，阻塞到收到状态为止，阻塞期间不能进行其他操作
    //初始化和反初始化互斥
    // 初始化机器人，运动接口
    // 返回值：true - 成功，false - 失败
    virtual bool robot_init()  = 0;
    // 反初始化机器人，运动接口
    // 返回值：true - 成功，false - 失败
    virtual bool robot_deinit()  = 0;
  
    // ===== LOW-LEVEL 接口 =====

    // 直接控制电机
    //参数说明：motor_id - 电机索引,0-22
    //          control - 电机控制参数
    virtual bool setMotorControl_low(EtherCAT_Motor_Index motor_id, const Motor_Control& control) = 0;

    // 设置底盘
    // 参数说明：chassis_id - 底盘电机索引, 左轮:0, 右轮:1
    //          control - 电机控制参数,仅支持控制速度
    virtual bool setChassis_low(EtherCAT_Motor_Index chassis_id, const Motor_Control& control) = 0;
    // 设置腰部
    // 参数说明：waist_id - 腰部电机索引, 升降2，俯仰:3, 旋转:4
    //          control - 电机控制参数，升降仅支持控制位置，俯仰和旋转支持控制速度，位置和力矩
    virtual bool setWaist_low(EtherCAT_Motor_Index waist_id, const Motor_Control& control) = 0;
    // 设置头部
    // 参数说明：head_id - 头部电机索引, 旋转:5, 俯仰:6
    //          control - 电机控制参数，支持控制位置、速度和力矩
    virtual bool setHead_low(EtherCAT_Motor_Index head_id, const Motor_Control& control) = 0;
    // 设置手臂
    // 参数说明：arm_id 关节索引, 7-13 - 左臂 肩部->手腕，15-21 右臂 肩部->手腕
    //          control - 电机控制参数，支持控制位置、速度和力矩
    virtual bool setArm_low(EtherCAT_Motor_Index arm_id, const Motor_Control& control) = 0;
    // 设置夹爪
    // 夹爪实际为手部的最后一个电机
    // 参数说明：gripper_id - 夹爪电机索引,14 - 左手，22 - 右手
    //          control - 电机控制参数，支持控制位置、速度和力矩
    //          is_hold_torque - 是否保持力矩不变，默认true
    virtual bool setGripper_low(EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque = true) = 0;


    // 设置固定杆,0-关闭，1-打开
    virtual bool setFixedRod_low(const int& is_open) = 0;

    // ===== HIGH-LEVEL 接口 =====
                       
    // 设置底盘线速度和角速度
    //   speed_X: 前进, 后退,
    //   speed_y: 左转, 右转,
    // 返回值：true - 成功，false - 失败
    virtual bool setChassis_high(const float& speed_x , const float& speed_y) = 0;

    // 设置腰部姿态
    virtual bool setWaist_high(const ArmEndPose& action) = 0;

    // 设置头部姿态,实际与低层控制相同
    virtual bool setHead_high(EtherCAT_Motor_Index head_id, const Motor_Control& control) = 0;

    // 设置手臂末端姿态
    virtual bool setArm_high(ArmAction arm, const ArmEndPose& action) = 0;


    // // 高阶控制命令类型
    // struct HighLevelControl
    // {
    //     // 控制类型：0=连续控制, 1=Move模式，
    //     int32_t control_type;

    //     // 控制部位：0=左手, 1=右手, 2=腰部
    //     int32_t control_part;

    //     // 腰部三个电机位置
    //     float waist_position[3];
        
    //     // 右手位姿：位置(3) + 四元数(4) = 7个float
    //     // 格式：[x, y, z, qx, qy, qz, qw]
    //     float hand_pose[7];
        
    //     // 末端执行器速度 (m/s)，可选，默认值 0.0 表示使用默认速度
    //     float eef_velocity;
        
    //     // 末端执行器加速度 (m/s^2)，可选，默认值 0.0 表示使用默认加速度
    //     float eef_acceleration;
        
    //     // 是否阻塞执行，true=阻塞，false=非阻塞
    //     boolean block;
        
    //     // 运动持续时间 (秒)，可选，默认值 0.0 表示自动计算
    //     float duration;
    // }

    // 设置手臂末端一键运动,控制速度或时间
    // eef_velocity: 末端执行器速度 (m/s)，0表示使用默认速度
    // eef_acceleration: 末端执行器加速度 (m/s^2)，0表示使用默认加速度
    // duration: 运动持续时间 (秒)，0表示自动计算
    //bool block: 是否阻塞后续动作执行，true=阻塞，false=非阻塞，默认false
    virtual bool setArmMove_high(ArmAction arm, const ArmEndPose& action, float eef_velocity = 0.0f, float eef_acceleration = 0.0f, float duration = 0.0f, bool block = false) = 0;

    // 设置夹爪末端姿态，实际与低层控制相同
    virtual bool setGripper_high(EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque = true) = 0;

    //设置灵巧手控制

    // 设置固定杆,0-关闭，1-打开
    virtual bool setFixedRod_high(const int& is_open) = 0;

    // 新增：将传统的 ArmPose(x,y,z,roll,pitch,yaw) 转换为 ArmEndPose(Position[3], Rotation[4]{x,y,z,w})
    // 四元数顺序 (x, y, z, w)
    virtual ArmEndPose armPoseToArmEndPose(const ArmPose& pose) = 0;

    // ===== 不区分控制模式的接口 =====
    // 清零力传感器数据,0左传感器，1右传感器
    

    // ===== 数据采集接口（gRPC）=====
    // 创建采集会话并发送首条 json_config。当前只实现 V2。
    virtual bool CreateCollectSession(const std::string& config) = 0;
    virtual bool Start() = 0;
    virtual bool NextStep() = 0;
    virtual bool Pause() = 0;
    virtual bool Resume() = 0;
    virtual bool Stop() = 0;
    virtual void Close() = 0;

    // ===== 状态监控 =====
        
    // 获取机器人基本信息
    // 返回值：true - 成功，false - 失败
    //可以在机器人未初始化时调用，以及用于判断SDK是否连接上了机器人
    virtual bool getRobotInfo(RobotInfo& state) = 0;


    // 获取电机状态
    // 参数说明：motor_id - 电机索引, state - 电机状态结构
    // state.Position_Actual: 当前电机位置
    // state.Speed_Actual: 当前电机速度
    // state.Torque_Actual: 当前电机力矩
    // state.KP: 当前电机KP
    // state.KD: 当前电机KD
    // state.Error_flag: 错误标志位，0表示正常，其他表示电机异常
    // 返回值：true - 成功，false - 失败
    virtual bool getMotorState(EtherCAT_Motor_Index motor_id, Motor_Information& state) = 0;
    // 获取底盘状态
    virtual bool getChassisState(EtherCAT_Motor_Index chassis_id, Motor_Information& state) = 0;
    // 获取底盘速度状态，包括实际电机速度和解算后的速度
    // speed_actual: 实际电机速度，0-左轮，1-右轮
    // speed_algo: 解算后的速度，0-线速度，1-角速度
    virtual bool getChassisSpeedState(std::array<float,2> &speed_actual, std::array<float,2> &speed_algo) = 0;
    // 获取腰部状态
    virtual bool getWaistState(EtherCAT_Motor_Index waist_id, Motor_Information& state) = 0;
    // 获取头部状态
    virtual bool getHeadState(EtherCAT_Motor_Index head_id, Motor_Information& state) = 0;
    // Motor_Information
    virtual bool getArmState(EtherCAT_Motor_Index joint_id, Motor_Information& state) = 0;
    // 获取手臂和头部末端实际和相对位姿
    //相对电机零位的坐标系（relative）
    virtual bool getHandRelative(ArmAction arm, ArmEndPose& state) = 0;
    virtual bool getHeadRelative(ArmEndPose& state) = 0;
    
    virtual bool getHandCameraRelative(ArmAction arm, ArmEndPose& state) = 0;
    virtual bool getHeadCameraRelative(ArmEndPose& state) = 0;
    // 获取夹爪状态
    virtual bool getGripperState(EtherCAT_Motor_Index gripper_id, Motor_Information& state) = 0;
    // 获取夹爪控制模式，0为保持力矩模式，1为MIT自由控制模式
    virtual bool getGripperControlMode(int& control_mode) = 0;
    // 获取固定杆状态
    virtual bool getFixedRodState(int& is_open) = 0;



    // // HighLevel 控制状态反馈
    // struct HighLevelState
    // {
    //     // 状态码：
    //     // 0 = 未激活/不可用
    //     // 1 = 激活中（插值到零位）
    //     // 2 = 空闲（等待命令）
    //     // 3 = 执行中（MoveL/MoveC）
    //     // 4 = 执行完成
    //     // 5 = 错误/异常
    //     int16_t state;
        
    //     // 额外信息（可选，预留）
    //     // 例如：当前执行进度百分比（0-100）
    //     int16_t progress;
    // }
    //具体参数含义如上述
    // 获取高级控制状态反馈
    virtual bool getHighLevelState(HighLevelState& state) = 0;

    //获取手柄状态
    //各按键参数
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    virtual bool getJoystickState(XboxMap& state) = 0;
    //获取IMU状态
    //state.rpy  : 当前IMU的滚转、俯仰、偏航角
    //state.omega: 当前IMU的角速度
    //error_flag: 错误标志位，0表示正常，其他表示未连接设备
    // 返回值：true - 成功，false - 失败
    virtual bool getIMU_State(ImuData& state) = 0;


    //获取电源充电和放电状态
    virtual bool getPowerChargeState(power_charge_state & state) = 0;
    
    // 获取最近一次收到的实际控制模式
    virtual MotorControlMode getCurrentMode() = 0;
    // 获取最近一次收到的完整控制模式状态
    virtual ControlModeStatus getControlModeStatus() = 0;
    
    // 0: UNKNOWN, 1: DDS_COMMAND, 2: REMOTE_CONTROLLER,
    // 3: LOW_LEVEL_PROTECTION.
    // 获取最近一次收到的当前初始化状态
    virtual InitState getInitState() = 0;
    // 获取最近一次收到的完整系统初始化状态
    virtual SystemInitStatus getSystemInitStatus() = 0;
    
    // 获取最近一次收到的通信板状态；未连接或尚未收到有效 DDS 数据时返回 false。
    virtual bool getCommunicationBoardState(CommunicationBoardState& state) = 0;

    virtual bool isRobotConnected() = 0; // 检查机器人是否连接成功

    // ===== 相机接口 =====
    // DDS 相机接口仅支持 SDK 与机器人同机运行；异机调用 startCameraDds() 会抛出异常。
    // WebRTC 相机接口同机、异机均可使用。
    virtual void startCameraDds() = 0;
    virtual void stopCameraDds() = 0;

    virtual void startCameraWebRTC() = 0;
    virtual void stopCameraWebRTC() = 0;

    // 返回最近一帧及其单调时钟时间戳；尚未收到数据时返回 std::nullopt。
    virtual std::optional<std::pair<cv::Mat, double>>
    getLatestCameraFrame(const std::string& camera_name) = 0;
    virtual std::optional<std::pair<cv::Mat, double>>
    getLatestCameraDepth(const std::string& camera_name) = 0;

    // camera_names 为空时查询全部相机。
    virtual robot::RecorderStateResponse getCameraState(
        const std::vector<std::string>& camera_names = {},
        float timeout_sec = 5.0f) = 0;

protected:
    H1Robot() = default;
};
