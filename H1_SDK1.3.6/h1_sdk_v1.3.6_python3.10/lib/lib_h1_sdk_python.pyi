from typing import Tuple, List, Any, Optional, Union, overload

# --- 枚举定义 (Derived from config.hpp & enums_bind.cpp) ---

class InitCommand:
    """初始化命令枚举"""
    Wait_Init: int  # 等待初始化
    Init: int       # 执行初始化
    Deinit: int     # 执行反初始化

class InitState:
    """机器人初始化状态反馈"""
    Uninit: int          # 未初始化
    Initializing: int    # 初始化中
    Init_Complete: int   # 初始化完成
    Deinitializing: int  # 反初始化中
    Deinit_Complete: int # 反初始化完成

class ArmAction:
    """手臂选择枚举"""
    LEFT_ARM: int
    RIGHT_ARM: int

class MotorControlMode:
    """控制模式枚举"""
    UNINITIALIZED: int               # 未初始化（默认遥控模式）
    LOW_LEVEL: int                   # 底层直接控制（控制电机参数）
    HIGH_LEVEL: int                  # 高层位姿控制（笛卡尔空间控制）
    GRAVITY_COMPENSATION_LEVEL: int  # 带重力补偿的底层控制

class EtherCAT_Motor_Index:
    """电机索引枚举 (0-22)"""
    MOTOR_WHEEL_LEFT: int   # 轮毂电机（左，ID2）
    MOTOR_WHEEL_RIGHT: int  # 轮毂电机（右，ID1）
    MOTOR_LIFT: int         # 升降电机（ID0）
    MOTOR_WAIST_DOWN: int   # 腰部下电机（俯仰）
    MOTOR_WAIST_UP: int     # 腰部上电机（旋转）
    MOTOR_HEAD_DOWN: int    # 头部下电机（旋转）
    MOTOR_HEAD_UP: int      # 头部上电机（俯仰）
    MOTOR_LEFT_ARM_1: int   # 左臂电机 1-8 (ID7-ID14, 8为夹爪)
    MOTOR_LEFT_ARM_2: int   
    MOTOR_LEFT_ARM_3: int   
    MOTOR_LEFT_ARM_4: int   
    MOTOR_LEFT_ARM_5: int   
    MOTOR_LEFT_ARM_6: int   
    MOTOR_LEFT_ARM_7: int   
    MOTOR_LEFT_ARM_8: int   
    MOTOR_RIGHT_ARM_1: int   # 右臂电机 1-8 (ID15-ID22, 8为夹爪)
    MOTOR_RIGHT_ARM_2: int   
    MOTOR_RIGHT_ARM_3: int   
    MOTOR_RIGHT_ARM_4: int   
    MOTOR_RIGHT_ARM_5: int   
    MOTOR_RIGHT_ARM_6: int   
    MOTOR_RIGHT_ARM_7: int   
    MOTOR_RIGHT_ARM_8: int   


# --- 数据结构定义 ---

class Motor_Information:
    """电机状态反馈信息"""
    Position_Actual: float  # 当前位置
    Speed_Actual: float     # 当前速度
    Torque_Actual: float    # 当前力矩
    KP_Actual: float        # 当前 KP
    KD_Actual: float        # 当前 KD
    Error_flag: int         # 错误标志位，0 表示正常
    def __init__(self) -> None: ...

class Motor_Control:
    """电机控制指令参数"""
    Position: float  # 目标位置
    Speed: float     # 目标速度
    Torque: float    # 目标力矩
    KP: float        # 比例增益
    KD: float        # 微分增益
    def __init__(self) -> None: ...

class ArmPose:
    """旧版欧拉角位姿结构"""
    x: float; y: float; z: float
    roll: float; pitch: float; yaw: float
    def __init__(self) -> None: ...

class ArmEndPose:
    """新版末端位姿结构 (位置 + 四元数)"""
    position: List[float] # [x, y, z]
    rotation: List[float] # [qx, qy, qz, qw]
    def __init__(self) -> None: ...

class HandControl:
    """灵巧手控制结构"""
    hand_mode: int             # 控制模式：1-位置速度模式, 2-角度速度模式
    hand_control: List[float]  # 控制目标数组（长度 6）
    hand_speed: List[float]    # 速度数组（长度 6）
    hand_type: int             # 手部类型
    def __init__(self) -> None: ...

class HandState:
    """灵巧手状态反馈"""
    hand_position: List[float]
    hand_angle: List[float]
    hand_current: List[float]
    hand_status: List[float]
    hand_type: int
    error_flag: int
    def __init__(self) -> None: ...

class ForceSensorState:
    """六维力传感器状态"""
    Fx: List[float]; Fy: List[float]; Fz: List[float] # 分别为左右侧传感器的力
    Tx: List[float]; Ty: List[float]; Tz: List[float] # 分别为左右侧传感器的力矩
    Error_flag: List[int] # 传感器错误代码
    def __init__(self) -> None: ...

class RobotInfo:
    """机器人固件与硬件版本信息"""
    robot_name: str
    robot_type: str
    firmware_version: str
    hardware_version: str
    software_version: str
    manufacturer: str
    def __init__(self) -> None: ...

class power_charge_state:
    """电池与电源状态"""
    status: int      # 充电状态
    temperature: int # 电池温度
    soc: int         # 剩余电量百分比
    def __init__(self) -> None: ...

class ImuData:
    """IMU 姿态与角速度数据"""
    rpy: List[float]   # 滚转、俯仰、偏航角
    omega: List[float] # 角速度
    error_flag: int
    def __init__(self) -> None: ...

class XboxMap:
    """手柄按键与摇杆映射状态"""
    a: int; b: int; x: int; y: int
    leftShoulder: int; rightShoulder: int
    leftStick: int; rightStick: int
    start: int; back: int
    dpadUp: int; dpadDown: int; dpadLeft: int; dpadRight: int
    leftX: float; leftY: float; rightX: float; rightY: float
    leftTrigger: float; rightTrigger: float
    error_flag: int # 0 表示连接正常
    def __init__(self) -> None: ...

class HighLevelControl:
    """高阶笛卡尔空间运动控制指令"""
    control_type: int            # 0=连续控制, 1=Move模式
    control_part: int            # 0=左手, 1=右手, 2=腰部
    control_coordinate: int      # 0=相对初始位姿, 2=相对原点
    waist_position: List[float]  # 腰部电机位置 (3个)
    hand_pose: List[float]       # 手部 [x,y,z,qx,qy,qz,qw]
    middle_position: List[float] # 圆弧插值中间点 (仅手臂)
    eef_velocity: float          # 末端速度 (m/s)
    eef_acceleration: float      # 末端加速度 (m/s^2)
    block: bool                  # 是否阻塞执行
    duration: float              # 运动持续时间 (秒)
    def __init__(self) -> None: ...

class HighLevelState:
    """高阶控制状态反馈"""
    state: int    # 状态码: 2=空闲, 3=执行中, 4=执行完成
    progress: int # 执行进度百分比 (0-100)
    def __init__(self) -> None: ...

# --- 主机器人导航类 ---

class H1Robot:
    """H1 机器人 SDK 主类，提供运动控制与状态获取接口"""

    @overload
    def __init__(self) -> None: 
        """初始化 H1Robot 实例（机器人主机内通信）"""
        ...
    @overload
    def __init__(self, udp_url: str) -> None: 
        """初始化 H1Robot 实例（局域网跨设备通信）"""
        ...

    def robot_connect(self) -> bool: 
        """连接机器人主机，返回值表示是否成功"""
        ...

    def switchControlMode(self, new_mode: int) -> bool: 
        """切换机器人控制模式（LOW_LEVEL / HIGH_LEVEL）"""
        ...

    def robot_init(self) -> bool: 
        """初始化机器人运动接口，初始化与反初始化互斥"""
        ...

    def robot_deinit(self) -> bool: 
        """反初始化机器人运动接口，停止所有运动"""
        ...

    # --- LOW-LEVEL 控制接口 (底层直接控制电机) ---

    def setMotorControl_low(self, motor_id: int, control: Motor_Control) -> bool: 
        """直接控制单个电机（索引 0-22）"""
        ...

    def setChassis_low(self, chassis_id: int, control: Motor_Control) -> bool: 
        """底层控制底盘电机（0-左轮，1-右轮），仅支持速度控制"""
        ...

    def setWaist_low(self, waist_id: int, control: Motor_Control) -> bool: 
        """底层控制腰部（2-升降，3-俯仰，4-旋转），升降仅支持位置控制"""
        ...

    def setHead_low(self, head_id: int, control: Motor_Control) -> bool: 
        """底层控制头部，支持位置、速度和力矩控制"""
        ...

    def setArm_low(self, arm_id: int, control: Motor_Control) -> bool: 
        """底层控制手臂关节（7-13左臂，15-21右臂）"""
        ...

    def setGripper_low(self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True) -> bool: 
        """底层控制夹爪，is_hold_torque 表示是否保持力矩不变"""
        ...

    def setHand_low(self, arm: int, control: HandControl) -> bool: 
        """底层控制灵巧手"""
        ...

    def setFixedRod_low(self, is_open: int) -> bool: 
        """底层控制推杆：0-关闭，1-打开"""
        ...

    def resetForceSensorData(self, sensor_id: int) -> bool: 
        """清零力传感器数据：0-左传感器，1-右传感器"""
        ...

    # --- HIGH-LEVEL 控制接口 (高层笛卡尔/解算控制) ---

    def setChassis_high(self, speed_x: float, speed_y: float) -> bool: 
        """高层设置底盘速度：speed_x (前后)，speed_y (转向)"""
        ...

    def setWaist_high(self, action: ArmEndPose) -> bool: 
        """高层设置腰部姿态"""
        ...

    def setHead_high(self, head_id: int, control: Motor_Control) -> bool: 
        """高层控制头部，逻辑同低层"""
        ...

    def setArm_high(self, arm: int, action: ArmEndPose) -> bool: 
        """高层设置手臂末端姿态（基于 ArmEndPose 结构）"""
        ...

    def setGripper_high(self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True) -> bool: 
        """高层控制夹爪，逻辑同低层"""
        ...

    def setHand_high(self, arm: int, control: HandControl) -> bool: 
        """高层控制灵巧手"""
        ...

    def setFixedRod_high(self, is_open: int) -> bool: 
        """高层控制推杆"""
        ...

    def armPoseToArmEndPose(self, pose: ArmPose) -> ArmEndPose: 
        """辅助接口：将欧拉角姿态 (ArmPose) 转换为四元数位姿 (ArmEndPose)"""
        ...

    def setHighLevelControl(self, control: HighLevelControl) -> bool: 
        """设置高级笛卡尔空间运动指令（包含 Move 模式和阻塞选项）"""
        ...

    # --- 状态获取接口 (Python 返回 Tuple[bool, Struct]，第一个参数 ok 表示获取是否成功) ---

    def getRobotInfo(self) -> Tuple[bool, RobotInfo]: 
        """获取机器人基本信息（SDK 连接状态判定）"""
        ...

    def getMotorState(self, motor_id: int) -> Tuple[bool, Motor_Information]: 
        """获取指定电机的状态（包含位置、速度、力矩及错误标志）"""
        ...

    def getChassisState(self, chassis_id: int) -> Tuple[bool, Motor_Information]: 
        """获取底盘电机状态"""
        ...

    def getChassisSpeedState(self) -> Tuple[bool, List[float], List[float]]:
        """
        获取底盘速度：
        返回: (ok, [左轮速度, 右轮速度], [线速度, 角速度])
        """
        ...

    def getWaistState(self, waist_id: int) -> Tuple[bool, Motor_Information]: 
        """获取腰部电机状态"""
        ...

    def getHeadState(self, head_id: int) -> Tuple[bool, Motor_Information]: 
        """获取头部电机状态"""
        ...

    def getArmState(self, joint_id: int) -> Tuple[bool, Motor_Information]: 
        """获取手臂指定关节状态"""
        ...

    def getArmTargetState(self, arm: int) -> Tuple[bool, ArmEndPose]: 
        """获取高层模式下手部末端的实际位姿"""
        ...

    def getGripperState(self, gripper_id: int) -> Tuple[bool, Motor_Information]: 
        """获取夹爪电机状态"""
        ...

    def getGripperControlMode(self) -> Tuple[bool, int]: 
        """获取夹爪控制模式：0-保持力矩模式, 1-MIT 自由控制模式"""
        ...

    def getFixedRodState(self) -> Tuple[bool, int]: 
        """获取推杆状态"""
        ...

    def getJoystickState(self) -> Tuple[bool, XboxMap]: 
        """获取手柄状态数据"""
        ...

    def getIMU_State(self) -> Tuple[bool, ImuData]: 
        """获取 IMU 姿态与加速度数据"""
        ...

    def getHandState(self, arm: int) -> Tuple[bool, HandState]: 
        """获取灵巧手状态反馈"""
        ...

    def getHighLevelState(self) -> Tuple[bool, HighLevelState]: 
        """获取高阶控制的执行进度与状态码"""
        ...

    def getForceSensorState(self) -> Tuple[bool, ForceSensorState]: 
        """获取六维力传感器实时数据"""
        ...

    def getPowerChargeState(self) -> Tuple[bool, power_charge_state]: 
        """获取电源电量与充电状态"""
        ...

    def getCurrentMode(self) -> int: 
        """获取机器人当前所处的控制模式枚举"""
        ...

    def getInitState(self) -> int: 
        """获取初始化状态反馈"""
        ...

    def isRobotConnected(self) -> bool: 
        """检查 SDK 与机器人的通信是否已建立"""
        ...