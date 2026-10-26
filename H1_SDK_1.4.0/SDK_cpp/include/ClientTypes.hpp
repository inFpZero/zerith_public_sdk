#pragma once

#include <cstdint>
#include <string>

// SDK 客户端进程内消息类型（与 DDS IDL 字段对应，无 ZCM 依赖）。
namespace robot_client {

struct SystemInitRequestData {
  int32_t system_init = 0;
  uint64_t timestamp_ns = 0;
};

struct SystemInitStatus {
  int8_t last_state = 0;
  int8_t current_state = 0;
  int8_t change_reason = 0;
  int32_t error_code = 0;
  uint64_t state_changed_at_ns = 0;
  uint64_t timestamp_ns = 0;
};

struct ChassisControl {
  float speed[2]{};
  int8_t control_mode = 0;
  uint64_t timestamp_ns = 0;
};

struct ChassisState {
  float speed_actual[2]{};
  float speed_algo[2]{};
  int32_t error_flag[2]{};
  uint64_t timestamp_ns = 0;
};

struct WaistControl {
  float position[3]{};
  float speed[3]{};
  float torque[3]{};
  uint64_t timestamp_ns = 0;
};

struct WaistState {
  float position_actual[3]{};
  float speed_actual[3]{};
  float torque_actual[3]{};
  int32_t error_flag[3]{};
  uint64_t timestamp_ns = 0;
};

struct HeadControl {
  float position[2]{};
  float speed[2]{};
  float torque[2]{};
  float KP[2]{};
  float KD[2]{};
  bool enable_kp_kd = false;
  uint64_t timestamp_ns = 0;
};

struct HeadState {
  float position_actual[2]{};
  float speed_actual[2]{};
  float torque_actual[2]{};
  int32_t error_flag[2]{};
  float KP[2]{};
  float KD[2]{};
  uint64_t timestamp_ns = 0;
};

struct UpperJointControl {
  float position[14]{};
  float speed[14]{};
  float torque[14]{};
  float KP[14]{};
  float KD[14]{};
  bool enable_kp_kd = false;
  uint64_t timestamp_ns = 0;
};

struct UpperJointState {
  float position_actual[14]{};
  float speed_actual[14]{};
  float torque_actual[14]{};
  float KP[14]{};
  float KD[14]{};
  int32_t error_flag[14]{};
  uint64_t timestamp_ns = 0;
};

struct GripperControl {
  float position[2]{};
  float speed[2]{};
  float torque[2]{};
  float KP[2]{};
  float KD[2]{};
  bool enable_kp_kd = false;
  uint64_t timestamp_ns = 0;
};

struct GripperState {
  float position_actual[2]{};
  float speed_actual[2]{};
  float torque_actual[2]{};
  float KP[2]{};
  float KD[2]{};
  int32_t error_flag[2]{};
  uint64_t timestamp_ns = 0;
};

struct GripperControlMode {
  int8_t control_mode = 0;
  uint64_t timestamp_ns = 0;
};

struct FixedRod {
  int8_t is_open = 0;
  uint64_t timestamp_ns = 0;
};

struct ArmEndPose {
  float PositionData[3]{};
  float RotationData[4]{};
  uint64_t timestamp_ns = 0;
};

struct HighLevelControl {
  int32_t control_type = 0;
  int32_t control_part = 0;
  float waist_position[3]{};
  float hand_pose[7]{};
  float eef_velocity = 0.f;
  float eef_acceleration = 0.f;
  int8_t block = 0;
  float duration = 0.f;
  uint64_t timestamp_ns = 0;
};

struct HighLevelState {
  int16_t state = 0;
  int16_t progress = 0;
  uint64_t timestamp_ns = 0;
};

struct ControlModeRequestData {
  int8_t control_mode = 0;
  uint64_t timestamp_ns = 0;
};

struct ControlModeStatus {
  int8_t last_state = 0;
  int8_t current_state = 0;
  uint64_t state_changed_at_ns = 0;
  uint64_t timestamp_ns = 0;
};

struct CommunicationBoardState {
  int8_t error_flag[6]{};
  uint64_t timestamp_ns = 0;
};

struct ImuData {
  float rpy[3]{};
  float omega[3]{};
  float accel[3]{};
  float quat[4]{};
  int32_t error_flag = 1;
  uint64_t timestamp_ns = 0;
};

struct PowerState {
  int32_t soc = 0;
  int32_t avg_time_to_empty = 0;
  int32_t avg_time_to_full = 0;
  int32_t temperatures[5]{};
  int16_t power_flag = 0;
  uint64_t timestamp_ns = 0;
};

struct XboxMap {
  bool a = false, b = false, x = false, y = false;
  bool rightShoulder = false, leftShoulder = false;
  bool leftStick = false, rightStick = false;
  bool start = false, back = false;
  bool dpadUp = false, dpadDown = false, dpadLeft = false, dpadRight = false;
  float leftX = 0.f, leftY = 0.f, rightX = 0.f, rightY = 0.f;
  float leftTrigger = 0.f, rightTrigger = 0.f;
  int8_t error_flag = 1;
  uint64_t timestamp_ns = 0;
};

struct RobotInfo {
  std::string robot_name;
  std::string robot_type;
  std::string firmware_version;
  std::string hardware_version;
  std::string software_version;
  std::string manufacturer;
  uint64_t timestamp_ns = 0;
};

}  // namespace robot_client
