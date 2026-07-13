"""
H1 Robot SDK Python 绑定类型桩 (robot_SDK 1.3.9)

导入:
    from lib.lib_h1_sdk_python import H1Robot, MotorControlMode, ArmAction

典型流程:
    robot = H1Robot()
    robot.robot_connect()                              # 1. 连接
    robot.switchControlMode(MotorControlMode.HIGH_LEVEL)  # 2. 切模式
    robot.robot_init()                               # 3. 初始化 (阻塞)
    ok, info = robot.getRobotInfo()                  # 4. 读状态 / 发控制
    robot.robot_deinit()                             # 5. 反初始化

约定:
    - get* 状态接口统一返回 Tuple[bool, ...]，首元素 ok 表示 C++ 侧是否成功
    - arm / new_mode / motor_id 等枚举参数在 Python 侧为 int，请用本模块常量
    - ArmEndPose.position / rotation 等为读写 list property，赋值时长度必须匹配

1.3.9 变更 (相对 1.3.7):
    - 新增: setArmMove_high, getHandRelative, getHeadRelative,
            getHandCameraRelative, getHeadCameraRelative
    - 移除: setHighLevelControl, getArmTargetState
    - ImuData 新增 accel, quat 字段
"""

from typing import Tuple, List, overload

# ---------------------------------------------------------------------------
# 枚举 (config.hpp / enums_bind.cpp)
# ---------------------------------------------------------------------------

class InitCommand:
    """系统初始化命令 (ZCM SystemInit.system_init)"""
    Wait_Init: int   # 等待
    Init: int        # 初始化
    Deinit: int      # 反初始化
    ReInit: int      # 重新初始化

class InitState:
    """系统初始化反馈状态 (getInitState 返回值)"""
    Uninit: int              # 0 - 未初始化
    Initializing: int        # 1 - 初始化中
    Init_Complete: int       # 2 - 已初始化
    Deinitializing: int      # 3 - 反初始化中
    Deinit_Complete: int     # 4 - 已反初始化
    Error_State: int         # 5 - 错误

class ArmAction:
    """左右臂选择 (setArm_high / getHandRelative 等)"""
    LEFT_ARM: int
    RIGHT_ARM: int

class MotorControlMode:
    """控制模式 (switchControlMode / getCurrentMode)"""
    UNINITIALIZED: int               # 未初始化 / 默认遥操作
    LOW_LEVEL: int                   # 底层直接电机控制
    HIGH_LEVEL: int                  # 高层位姿控制
    GRAVITY_COMPENSATION_LEVEL: int  # 带重力补偿的底层控制

class EtherCAT_Motor_Index:
    """EtherCAT 电机索引 (0~22)"""
    MOTOR_WHEEL_LEFT: int    # 0  左轮毂
    MOTOR_WHEEL_RIGHT: int   # 1  右轮毂
    MOTOR_LIFT: int          # 2  升降
    MOTOR_WAIST_DOWN: int    # 3  腰部俯仰
    MOTOR_WAIST_UP: int      # 4  腰部旋转
    MOTOR_HEAD_DOWN: int     # 5  头部旋转
    MOTOR_HEAD_UP: int       # 6  头部俯仰
    MOTOR_LEFT_ARM_1: int    # 7  左臂关节 1 (肩)
    MOTOR_LEFT_ARM_2: int    # 8
    MOTOR_LEFT_ARM_3: int    # 9
    MOTOR_LEFT_ARM_4: int    # 10
    MOTOR_LEFT_ARM_5: int    # 11
    MOTOR_LEFT_ARM_6: int    # 12
    MOTOR_LEFT_ARM_7: int    # 13  左腕
    MOTOR_LEFT_ARM_8: int    # 14  左夹爪
    MOTOR_RIGHT_ARM_1: int   # 15  右臂关节 1 (肩)
    MOTOR_RIGHT_ARM_2: int   # 16
    MOTOR_RIGHT_ARM_3: int   # 17
    MOTOR_RIGHT_ARM_4: int   # 18
    MOTOR_RIGHT_ARM_5: int   # 19
    MOTOR_RIGHT_ARM_6: int   # 20
    MOTOR_RIGHT_ARM_7: int   # 21  右腕
    MOTOR_RIGHT_ARM_8: int   # 22  右夹爪

# ---------------------------------------------------------------------------
# 数据结构 (ZCM_Data / config.hpp)
# ---------------------------------------------------------------------------

class RobotBase:
    """H1Robot 基类 (无额外方法)"""
    ...

class Motor_Information:
    """电机实际状态反馈"""
    Position_Actual: float  # 当前位置
    Speed_Actual: float     # 当前速度
    Torque_Actual: float    # 当前力矩
    KP_Actual: float
    KD_Actual: float
    Error_flag: int         # 0=正常，非 0=异常 (位标志)
    def __init__(self) -> None: ...

class Motor_Control:
    """电机控制指令"""
    Position: float  # 目标位置
    Speed: float     # 目标速度
    Torque: float    # 目标力矩
    KP: float        # -1 表示未设置
    KD: float        # -1 表示未设置
    def __init__(self) -> None: ...

class ArmPose:
    """欧拉角位姿，可经 armPoseToArmEndPose 转为 ArmEndPose"""
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    def __init__(self) -> None: ...

class ArmEndPose:
    """末端位姿：位置 + 四元数"""
    position: List[float]  # 读写 property，长度 3: [x, y, z]
    rotation: List[float]  # 读写 property，长度 4: [qx, qy, qz, qw]
    def __init__(self) -> None: ...
    def __repr__(self) -> str: ...

class HandControl:
    """灵巧手控制"""
    hand_mode: int
    hand_control: List[float]  # property，长度 6
    hand_speed: List[float]    # property，长度 6
    hand_type: int
    def __init__(self) -> None: ...

class HandState:
    """灵巧手状态"""
    hand_position: List[float]  # property，长度 6
    hand_angle: List[float]
    hand_current: List[float]
    hand_status: List[float]
    hand_type: int
    error_flag: int
    def __init__(self) -> None: ...

class ForceSensorState:
    """六维力传感器 (索引 0=左, 1=右)"""
    Fx: List[float]  # property，长度 2
    Fy: List[float]
    Fz: List[float]
    Tx: List[float]
    Ty: List[float]
    Tz: List[float]
    Error_flag: List[int]
    def __init__(self) -> None: ...

class RobotInfo:
    """机器人基本信息 (getRobotInfo)；未 init 也可读，用于确认连接"""
    robot_name: str
    robot_type: str
    firmware_version: str
    hardware_version: str
    software_version: str
    manufacturer: str
    def __init__(self) -> None: ...

class power_charge_state:
    """电池充放电状态 (getPowerChargeState)"""
    status: int        # 0=UNKNOWN, 1=CHARGING, 2=DISCHARGING, 3=FULL
    temperature: int   # 电池温度
    soc: int           # 电量 0~100
    def __init__(self) -> None: ...

class ImuData:
    """IMU 数据 (getIMU_State)"""
    rpy: List[float]    # property，[roll, pitch, yaw] rad
    omega: List[float]  # property，角速度 rad/s
    accel: List[float]  # property，线加速度 m/s^2
    quat: List[float]   # property，[w, x, y, z]
    error_flag: int     # 0=正常，非 0=异常
    def __init__(self) -> None: ...

class XboxMap:
    """手柄状态 (getJoystickState)；按键 0/1，摇杆/扳机为 float"""
    a: int
    b: int
    x: int
    y: int
    leftShoulder: int
    rightShoulder: int
    leftStick: int
    rightStick: int
    start: int
    back: int
    dpadUp: int
    dpadDown: int
    dpadLeft: int
    dpadRight: int
    leftX: float
    leftY: float
    rightX: float
    rightY: float
    leftTrigger: float
    rightTrigger: float
    error_flag: int     # 0=正常，非 0=未连接
    def __init__(self) -> None: ...

class HighLevelControl:
    """
    ZCM 高阶控制消息 (仅数据结构，H1Robot 无对应 set 接口)

    control_type: 0=连续, 1=Move
    control_part: 0=左手, 1=右手, 2=腰部
    """
    control_type: int
    control_part: int
    waist_position: List[float]  # property，长度 3
    hand_pose: List[float]       # property，长度 7: [x,y,z,qx,qy,qz,qw]
    eef_velocity: float
    eef_acceleration: float
    block: bool
    duration: float
    def __init__(self) -> None: ...

class HighLevelState:
    """
    高层控制状态 (getHighLevelState)

    state: 0=未激活, 1=激活中, 2=空闲, 3=执行中, 4=完成, 5=错误
    progress: 0~100
    """
    state: int
    progress: int
    def __init__(self) -> None: ...

class CommunicationBoardState:
    """通信板状态 (ZCM，当前 H1Robot 无 get 接口)"""
    error_flag: List[float]  # property
    def __init__(self) -> None: ...

class ControlMode:
    """控制模式 ZCM 消息 (ZCM，当前 H1Robot 无 get 接口)"""
    control_mode: int
    def __init__(self) -> None: ...

class SystemInit:
    """系统初始化 ZCM 消息 (ZCM，当前 H1Robot 无 get 接口)"""
    system_init: int  # InitCommand 枚举值
    def __init__(self) -> None: ...

class PowerState:
    """电源 ZCM 消息 (ZCM；H1Robot 用 getPowerChargeState 代替 getPowerState)"""
    soc: int
    avg_time_to_empty: int
    avg_time_to_full: int
    power_flag: int
    def __init__(self) -> None: ...

# ---------------------------------------------------------------------------
# H1Robot 接口
# ---------------------------------------------------------------------------

class H1Robot(RobotBase):
    """
    H1 机器人 SDK 主类。

    接口分组:
        连接/模式  robot_connect, switchControlMode, robot_init, robot_deinit
        低层控制   set*_low, resetForceSensorData
        高层控制   set*_high, setArmMove_high, armPoseToArmEndPose
        状态读取   get*, getCurrentMode, getInitState, isRobotConnected
    """

    # --- 构造 ---

    @overload
    def __init__(self) -> None:
        """本机 ZCM 通信 (运行在机器人主机上时使用)"""
        ...
    @overload
    def __init__(self, udp_url: str) -> None:
        """
        局域网 UDP 通信。

        Args:
            udp_url: 如 "udp://239.0.0.1:1234"
        """
        ...

    # --- 连接与模式 ---

    def robot_connect(self) -> bool:
        """
        连接机器人 (gRPC 获取 IP + 建立 ZCM)。

        Returns:
            True=成功, False=失败
        """
        ...
    def switchControlMode(self, new_mode: int) -> bool:
        """
        切换控制模式。

        Args:
            new_mode: MotorControlMode 枚举值

        Returns:
            True=切换成功
        """
        ...
    def robot_init(self) -> bool:
        """
        初始化运动接口。阻塞直到收到初始化状态反馈；期间勿并发调用其他运动接口。

        Returns:
            True=成功
        """
        ...
    def robot_deinit(self) -> bool:
        """
        反初始化运动接口。与 robot_init 互斥，同样阻塞。

        Returns:
            True=成功
        """
        ...

    # --- LOW-LEVEL 控制 ---
    # 适用模式: LOW_LEVEL 或 GRAVITY_COMPENSATION_LEVEL

    def setMotorControl_low(self, motor_id: int, control: Motor_Control) -> bool:
        """
        直接控制单个电机。

        Args:
            motor_id: EtherCAT_Motor_Index, 0~22
            control:  Motor_Control
        """
        ...
    def setChassis_low(self, chassis_id: int, control: Motor_Control) -> bool:
        """
        底盘轮速控制。

        Args:
            chassis_id: MOTOR_WHEEL_LEFT(0) 或 MOTOR_WHEEL_RIGHT(1)
            control:    仅 Speed 有效
        """
        ...
    def setWaist_low(self, waist_id: int, control: Motor_Control) -> bool:
        """
        腰部电机控制。

        Args:
            waist_id: MOTOR_LIFT(2) 仅 Position;
                      MOTOR_WAIST_DOWN(3)/MOTOR_WAIST_UP(4) 支持 Position/Speed/Torque
        """
        ...
    def setHead_low(self, head_id: int, control: Motor_Control) -> bool:
        """
        头部电机控制。

        Args:
            head_id: MOTOR_HEAD_DOWN(5) 旋转, MOTOR_HEAD_UP(6) 俯仰
            control: Position / Speed / Torque
        """
        ...
    def setArm_low(self, arm_id: int, control: Motor_Control) -> bool:
        """
        手臂关节控制。

        Args:
            arm_id: 7~13 左臂肩→腕, 15~21 右臂肩→腕 (不含夹爪 14/22)
        """
        ...
    def setGripper_low(
        self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True
    ) -> bool:
        """
        夹爪控制。

        Args:
            gripper_id: MOTOR_LEFT_ARM_8(14) 或 MOTOR_RIGHT_ARM_8(22)
            is_hold_torque: True=保持力矩模式
        """
        ...
    def setHand_low(self, arm: int, control: HandControl) -> bool:
        """
        灵巧手低层控制。

        Args:
            arm: ArmAction.LEFT_ARM 或 RIGHT_ARM
        """
        ...
    def setFixedRod_low(self, is_open: int) -> bool:
        """
        推杆/固定杆。

        Args:
            is_open: 0=关闭, 1=打开
        """
        ...
    def resetForceSensorData(self, sensor_id: int) -> bool:
        """
        力传感器清零 (任意控制模式下可用)。

        Args:
            sensor_id: 0=左传感器, 1=右传感器
        """
        ...

    # --- HIGH-LEVEL 控制 ---
    # 适用模式: HIGH_LEVEL

    def setChassis_high(self, speed_x: float, speed_y: float) -> bool:
        """
        底盘速度控制。

        Args:
            speed_x: 线速度 m/s (前进为正)
            speed_y: 角速度 rad/s (左转为正)
        """
        ...
    def setWaist_high(self, action: ArmEndPose) -> bool:
        """
        腰部姿态 (末端位姿形式)。

        Args:
            action: ArmEndPose
        """
        ...
    def setHead_high(self, head_id: int, control: Motor_Control) -> bool:
        """
        头部姿态。语义同 setHead_low。

        Args:
            head_id: MOTOR_HEAD_DOWN(5) 或 MOTOR_HEAD_UP(6)
        """
        ...
    def setArm_high(self, arm: int, action: ArmEndPose) -> bool:
        """
        手臂末端连续位姿控制 (实时跟随)。

        Args:
            arm: ArmAction
            action: 目标 ArmEndPose
        """
        ...
    def setArmMove_high(
        self,
        arm: int,
        action: ArmEndPose,
        eef_velocity: float = 0.0,
        eef_acceleration: float = 0.0,
        duration: float = 0.0,
        block: bool = False,
    ) -> bool:
        """
        手臂末端 Move 运动 (替代 1.3.7 的 setHighLevelControl Move 模式)。

        Args:
            arm: ArmAction
            action: 目标 ArmEndPose
            eef_velocity: 末端速度 m/s，0=默认
            eef_acceleration: 末端加速度 m/s²，0=默认
            duration: 运动时间 s，0=自动计算
            block: True=阻塞后续动作队列
        """
        ...
    def setGripper_high(
        self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True
    ) -> bool:
        """夹爪高层控制，语义同 setGripper_low。"""
        ...
    def setHand_high(self, arm: int, control: HandControl) -> bool:
        """灵巧手高层控制。Args: arm=ArmAction"""
        ...
    def setFixedRod_high(self, is_open: int) -> bool:
        """推杆/固定杆。Args: is_open 0=关, 1=开"""
        ...
    def armPoseToArmEndPose(self, pose: ArmPose) -> ArmEndPose:
        """
        欧拉角 → 四元数末端位姿。

        Returns:
            ArmEndPose，rotation 顺序 [qx, qy, qz, qw]
        """
        ...

    # --- 状态读取 ---
    # 均返回 (ok, ...)；ok=False 时后续字段可能无效

    def getRobotInfo(self) -> Tuple[bool, RobotInfo]:
        """机器人基本信息。未 init 也可调用。"""
        ...
    def getMotorState(self, motor_id: int) -> Tuple[bool, Motor_Information]:
        """
        指定电机状态。

        Args:
            motor_id: EtherCAT_Motor_Index 0~22
        """
        ...
    def getChassisState(self, chassis_id: int) -> Tuple[bool, Motor_Information]:
        """底盘电机状态。chassis_id: 0=左轮, 1=右轮"""
        ...
    def getChassisSpeedState(self) -> Tuple[bool, List[float], List[float]]:
        """
        底盘速度。

        Returns:
            (ok, speed_actual, speed_algo)
            speed_actual[0/1]: 左/右轮实际转速
            speed_algo[0/1]: 线速度 / 角速度
        """
        ...
    def getWaistState(self, waist_id: int) -> Tuple[bool, Motor_Information]:
        """腰部电机状态。waist_id: 2/3/4"""
        ...
    def getHeadState(self, head_id: int) -> Tuple[bool, Motor_Information]:
        """头部电机状态。head_id: 5/6"""
        ...
    def getArmState(self, joint_id: int) -> Tuple[bool, Motor_Information]:
        """手臂关节状态。joint_id: 7~13 或 15~21"""
        ...
    def getHandRelative(self, arm: int) -> Tuple[bool, ArmEndPose]:
        """
        手臂末端相对电机零位位姿 (替代 1.3.7 getArmTargetState)。

        Args:
            arm: ArmAction
        """
        ...
    def getHeadRelative(self) -> Tuple[bool, ArmEndPose]:
        """头部相对零位位姿。"""
        ...
    def getHandCameraRelative(self, arm: int) -> Tuple[bool, ArmEndPose]:
        """手相机相对位姿。Args: arm=ArmAction"""
        ...
    def getHeadCameraRelative(self) -> Tuple[bool, ArmEndPose]:
        """头相机相对位姿。"""
        ...
    def getGripperState(self, gripper_id: int) -> Tuple[bool, Motor_Information]:
        """夹爪状态。gripper_id: 14 或 22"""
        ...
    def getGripperControlMode(self) -> Tuple[bool, int]:
        """
        夹爪控制模式。

        Returns:
            (ok, mode)  mode: 0=保持力矩, 1=MIT 自由控制
        """
        ...
    def getFixedRodState(self) -> Tuple[bool, int]:
        """
        推杆状态。

        Returns:
            (ok, is_open)  is_open: 0=关, 1=开
        """
        ...
    def getJoystickState(self) -> Tuple[bool, XboxMap]:
        """手柄按键与摇杆状态。"""
        ...
    def getIMU_State(self) -> Tuple[bool, ImuData]:
        """IMU 姿态、角速度、加速度、四元数。"""
        ...
    def getHandState(self, arm: int) -> Tuple[bool, HandState]:
        """灵巧手状态。Args: arm=ArmAction"""
        ...
    def getHighLevelState(self) -> Tuple[bool, HighLevelState]:
        """高层控制器状态 (state/progress)。"""
        ...
    def getForceSensorState(self) -> Tuple[bool, ForceSensorState]:
        """左右六维力传感器状态。"""
        ...
    def getPowerChargeState(self) -> Tuple[bool, power_charge_state]:
        """电池充放电状态 (status/temperature/soc)。"""
        ...

    # --- 模式/连接查询 ---

    def getCurrentMode(self) -> int:
        """当前 MotorControlMode 枚举值。"""
        ...
    def getInitState(self) -> int:
        """当前 InitState 枚举值。"""
        ...
    def isRobotConnected(self) -> bool:
        """SDK 与机器人通信是否正常 (含心跳)。"""
        ...
