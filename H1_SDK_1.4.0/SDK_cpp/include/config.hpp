#pragma once

#include <cstdint>
#include <string>

//========= DDS（与 SDKService 对齐）==========
namespace sdk_dds {
constexpr int kDdsDomainClient = 1;
constexpr int kDdsDomainCamera = 0;

// connection_probe Service 的 DDS liveliness 配置。
constexpr int kServerOfflineTimeoutMs = 3000;
constexpr int kLivelinessAnnouncementMs = 1000;
}  // namespace sdk_dds

//========= 机器人配置 ==========

enum RobotModelHW { HW_UNKNOWN = 0, HW_PRO = 1, HW_MAX = 2 };

struct HardwareVersionInfo {
  RobotModelHW model = HW_UNKNOWN;
  int major = 0;
  int minor = 0;
  int patch = 0;
  std::string raw = "未知";
};

enum InitCommand { Wait_Init = 0, Init, Deinit, ReInit };
enum InitState {
  Uninit = 0,
  Initializing,
  Init_Complete,
  Deinitializing,
  Deinit_Complete,
  Error_State
};

enum ArmAction { LEFT_ARM, RIGHT_ARM };

struct ArmPose {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
};

enum MotorControlMode {
  UNINITIALIZED,
  LOW_LEVEL,
  HIGH_LEVEL,
  GRAVITY_COMPENSATION_LEVEL
};

struct power_charge_state {
  enum ChargeStatus { UNKNOWN = 0, CHARGING = 1, DISCHARGING = 2, FULL = 3 } status = UNKNOWN;
  uint32_t temperature = 0;
  uint16_t soc = 0;
};

typedef enum {
  MOTOR_WHEEL_LEFT = 0,
  MOTOR_WHEEL_RIGHT = 1,
  MOTOR_LIFT = 2,
  MOTOR_WAIST_DOWN = 3,
  MOTOR_WAIST_UP = 4,
  MOTOR_HEAD_DOWN = 5,
  MOTOR_HEAD_UP = 6,
  MOTOR_LEFT_ARM_1 = 7,
  MOTOR_LEFT_ARM_2 = 8,
  MOTOR_LEFT_ARM_3 = 9,
  MOTOR_LEFT_ARM_4 = 10,
  MOTOR_LEFT_ARM_5 = 11,
  MOTOR_LEFT_ARM_6 = 12,
  MOTOR_LEFT_ARM_7 = 13,
  MOTOR_LEFT_ARM_8 = 14,
  MOTOR_RIGHT_ARM_1 = 15,
  MOTOR_RIGHT_ARM_2 = 16,
  MOTOR_RIGHT_ARM_3 = 17,
  MOTOR_RIGHT_ARM_4 = 18,
  MOTOR_RIGHT_ARM_5 = 19,
  MOTOR_RIGHT_ARM_6 = 20,
  MOTOR_RIGHT_ARM_7 = 21,
  MOTOR_RIGHT_ARM_8 = 22,
  MOTOR_COUNT
} EtherCAT_Motor_Index;

typedef enum {
  MOTOR_ERROR_NONE = 0,
  MOTOR_ERROR_DISCONNECTED = (1 << 0),
  MOTOR_ERROR_OVERVOLTAGE = (1 << 1),
  MOTOR_ERROR_UNDERVOLTAGE = (1 << 2),
  MOTOR_ERROR_OVERHEAT = (1 << 3),
  MOTOR_ERROR_BLOCKED = (1 << 4),
  MOTOR_ERROR_OVERCURRENT = (1 << 5),
  MOTOR_ERROR_COMM_LOSS = (1 << 6),
  MOTOR_ERROR_OVERLOAD = (1 << 7),
  MOTOR_ERROR_Battery_LOW = (1 << 8),
  MOTOR_ERROR_OVERSPEED = (1 << 9),
  MOTOR_ERROR_ENCODER_FAULT = (1 << 10),
  MOTOR_ERROR_BRAKE_OVERVOLTAGE = (1 << 11),
  MOTOR_ERROR_DRIVER_FAULT = (1 << 12),
  MOTOR_ERROR_COIL_OVERTEMP = (1 << 13),
  MOTOR_ERROR_MOS_OVERTEMP = (1 << 14),
  MOTOR_ERROR = (1 << 15)
} Motor_Error_State;

typedef enum { MIX_CONTROL = 0, POS_CONTROL, SPD_CONTROL, TOR_CONTROL } Control_Mode;

typedef struct {
  float Position_Actual;
  float Speed_Actual;
  float Torque_Actual;
  float KP_Actual;
  float KD_Actual;
  uint16_t Error_flag;
} __attribute__((packed)) Motor_Information;

typedef struct {
  float Position = 0;
  float Speed = 0;
  float Torque = 0;
  float KP = -1;
  float KD = -1;
} __attribute__((packed)) Motor_Control;

typedef struct {
  uint8_t manufacturer : 4;
  uint8_t model : 4;
} __attribute__((packed)) Motor_TypeDef;

typedef struct {
  Motor_TypeDef Motor_Type;
  uint16_t Motor_ID;
  uint8_t Init_flag;
  uint16_t Control_Mode;
} __attribute__((packed)) Motor_Config;

typedef struct {
  uint8_t slave;
  uint8_t passage;
  float redirect;
  Motor_Config config;
  Motor_Control control;
  Motor_Information information;
} __attribute__((packed)) EtherCAT_Motor_TypeDef;

extern EtherCAT_Motor_TypeDef EtherCAT_Motor[];
