#pragma once

//========= ZCM =========

enum RobotModelHW { HW_UNKNOWN=0, HW_PRO=1, HW_MAX=2 };

struct HardwareVersionInfo {
    RobotModelHW model = HW_UNKNOWN;
    int major = 0;
    int minor = 0;
    int patch = 0;
    std::string raw = "未知";
};

// 初始化命令
enum InitCommand { Wait_Init = 0, Init, Deinit };
// 初始化反馈状态
enum    InitState { Uninit = 0, Initializing, Init_Complete, Deinitializing, Deinit_Complete };

enum ArmAction {
    LEFT_ARM,
    RIGHT_ARM
};

struct ArmPose{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

//high-level or low-level控制模式
enum MotorControlMode {
    UNINITIALIZED,  // 未初始化，即默认的遥操作模式
    LOW_LEVEL,      // 底层直接控制
    HIGH_LEVEL,      // 高层位姿控制
    GRAVITY_COMPENSATION_LEVEL      // 附带重力补偿的底层控制
};

// 电源充电/放电状态结构体
struct power_charge_state {
    enum ChargeStatus {
        UNKNOWN = 0,
        CHARGING = 1,
        DISCHARGING = 2,
        FULL = 3
    } status = UNKNOWN;
    uint32_t temperature = 0;    // 电池温度
    uint16_t soc = 0;            // 电池电量百分比
};

// 电机枚举定义
typedef enum
{
    // 轮毂电机
    MOTOR_WHEEL_LEFT  = 0,   // 轮毂电机（左，ID2）
    MOTOR_WHEEL_RIGHT = 1,   // 轮毂电机（右，ID1）

    // 升降电机
    MOTOR_LIFT        = 2,   // 升降电机（ID0）

    // 腰部电机
    MOTOR_WAIST_DOWN  = 3,   // 腰部电机（下，ID3） 俯仰
    MOTOR_WAIST_UP    = 4,   // 腰部电机（上，ID4） 旋转

    // 头部电机
    MOTOR_HEAD_DOWN   = 5,   // 头部电机（下，ID5） 旋转
    MOTOR_HEAD_UP     = 6,   // 头部电机（上，ID6） 俯仰

    // 左臂电机（上->下:7~14）
    MOTOR_LEFT_ARM_1  = 7,   // 左臂电机1（ID7）
    MOTOR_LEFT_ARM_2  = 8,   // 左臂电机2（ID8）
    MOTOR_LEFT_ARM_3  = 9,   // 左臂电机3（ID9）
    MOTOR_LEFT_ARM_4  = 10,  // 左臂电机4（ID10）
    MOTOR_LEFT_ARM_5  = 11,  // 左臂电机5（ID11）
    MOTOR_LEFT_ARM_6  = 12,  // 左臂电机6（ID12）
    MOTOR_LEFT_ARM_7  = 13,  // 左臂电机7（ID13）
    MOTOR_LEFT_ARM_8  = 14,  // 左臂电机8（ID14，左臂夹爪

    // 右臂电机（上->下:15~22）
    MOTOR_RIGHT_ARM_1 = 15,  // 右臂电机1（ID15）
    MOTOR_RIGHT_ARM_2 = 16,  // 右臂电机2（ID16）
    MOTOR_RIGHT_ARM_3 = 17,  // 右臂电机3（ID17）
    MOTOR_RIGHT_ARM_4 = 18,  // 右臂电机4（ID18）
    MOTOR_RIGHT_ARM_5 = 19,  // 右臂电机5（ID19）
    MOTOR_RIGHT_ARM_6 = 20,  // 右臂电机6（ID20）
    MOTOR_RIGHT_ARM_7 = 21,  // 右臂电机7（ID21）
    MOTOR_RIGHT_ARM_8 = 22,  // 右臂电机8（ID22，右臂夹爪

    MOTOR_COUNT              // 电机总数
} EtherCAT_Motor_Index;

// 电机报错汇总
typedef enum {
    MOTOR_ERROR_NONE              = 0,         // 无错误 → 0x0000
    MOTOR_ERROR_DISCONNECTED      = (1 << 0),  // 电机断联 → 0x0001
    MOTOR_ERROR_OVERVOLTAGE       = (1 << 1),  // 过压 → 0x0002
    MOTOR_ERROR_UNDERVOLTAGE      = (1 << 2),  // 欠压 → 0x0004
    MOTOR_ERROR_OVERHEAT          = (1 << 3),  // 电机过热 → 0x0008
    MOTOR_ERROR_BLOCKED           = (1 << 4),  // 电机堵转 → 0x0010
    MOTOR_ERROR_OVERCURRENT       = (1 << 5),  // 过流 → 0x0020
    MOTOR_ERROR_COMM_LOSS         = (1 << 6),  // 通信丢失 → 0x0040
    MOTOR_ERROR_OVERLOAD          = (1 << 7),  // 过载 → 0x0080
    MOTOR_ERROR_Battery_LOW       = (1 << 8),  // 电池电压低 → 0x0100
    MOTOR_ERROR_OVERSPEED         = (1 << 9),  // 电机超速 → 0x0200
    MOTOR_ERROR_ENCODER_FAULT     = (1 << 10), // 电机编码器错误 → 0x0400
    MOTOR_ERROR_BRAKE_OVERVOLTAGE = (1 << 11), // 刹车电压过高 → 0x0800
    MOTOR_ERROR_DRIVER_FAULT      = (1 << 12), // DRV驱动错误 → 0x1000
    MOTOR_ERROR_COIL_OVERTEMP     = (1 << 13), // 线圈过温 → 0x2000
    MOTOR_ERROR_MOS_OVERTEMP      = (1 << 14), // MOS过温 → 0x4000
    MOTOR_ERROR                   = (1 << 15)  // 其他错误 → 0x8000
} Motor_Error_State;

typedef struct{
    float Position_Actual;
    float Speed_Actual;
    float Torque_Actual;
    float KP_Actual;
    float KD_Actual;
    uint16_t Error_flag;
} __attribute__((packed)) Motor_Information;

typedef struct{
    float Position = 0;
    float Speed = 0;
    float Torque = 0;
    float KP = -1; // -1表示未设置
    float KD = -1; // -1表示未设置
} __attribute__((packed)) Motor_Control;
