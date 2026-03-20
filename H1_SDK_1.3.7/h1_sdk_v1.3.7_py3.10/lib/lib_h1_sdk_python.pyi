from typing import Tuple, List, Any, Optional, Union, overload

# --- 枚举定义 (Derived from config.hpp & enums_bind.cpp) ---

class InitCommand:
    Wait_Init: int
    Init: int
    Deinit: int

class InitState:
    Uninit: int
    Initializing: int
    Init_Complete: int
    Deinitializing: int
    Deinit_Complete: int

class ArmAction:
    LEFT_ARM: int
    RIGHT_ARM: int

class MotorControlMode:
    UNINITIALIZED: int
    LOW_LEVEL: int
    HIGH_LEVEL: int
    GRAVITY_COMPENSATION_LEVEL: int

class EtherCAT_Motor_Index:
    MOTOR_WHEEL_LEFT: int
    MOTOR_WHEEL_RIGHT: int
    MOTOR_LIFT: int
    MOTOR_WAIST_DOWN: int
    MOTOR_WAIST_UP: int
    MOTOR_HEAD_DOWN: int
    MOTOR_HEAD_UP: int
    MOTOR_LEFT_ARM_1: int
    MOTOR_LEFT_ARM_2: int
    MOTOR_LEFT_ARM_3: int
    MOTOR_LEFT_ARM_4: int
    MOTOR_LEFT_ARM_5: int
    MOTOR_LEFT_ARM_6: int
    MOTOR_LEFT_ARM_7: int
    MOTOR_LEFT_ARM_8: int
    MOTOR_RIGHT_ARM_1: int
    MOTOR_RIGHT_ARM_2: int
    MOTOR_RIGHT_ARM_3: int
    MOTOR_RIGHT_ARM_4: int
    MOTOR_RIGHT_ARM_5: int
    MOTOR_RIGHT_ARM_6: int
    MOTOR_RIGHT_ARM_7: int
    MOTOR_RIGHT_ARM_8: int

# --- 数据结构定义 (Derived from ZCM_Data & config.hpp) ---

class Motor_Information:
    Position_Actual: float
    Speed_Actual: float
    Torque_Actual: float
    KP_Actual: float
    KD_Actual: float
    Error_flag: int
    def __init__(self) -> None: ...

class Motor_Control:
    Position: float
    Speed: float
    Torque: float
    KP: float
    KD: float
    def __init__(self) -> None: ...

class ArmPose:
    """旧版欧拉角位姿结构"""
    x: float; y: float; z: float
    roll: float; pitch: float; yaw: float
    def __init__(self) -> None: ...

class ArmEndPose:
    """新版末端位姿结构：位置(3) + 四元数(4, [qx, qy, qz, qw])"""
    position: List[float]
    rotation: List[float]
    def __init__(self) -> None: ...

class HandControl:
    hand_mode: int
    hand_control: List[float] # 长度 6
    hand_speed: List[float]   # 长度 6
    hand_type: int
    def __init__(self) -> None: ...

class HandState:
    hand_position: List[float]
    hand_angle: List[float]
    hand_current: List[float]
    hand_status: List[float]
    hand_type: int
    error_flag: int
    def __init__(self) -> None: ...

class ForceSensorState:
    Fx: List[float]; Fy: List[float]; Fz: List[float] # 长度 2
    Tx: List[float]; Ty: List[float]; Tz: List[float]
    Error_flag: List[int]
    def __init__(self) -> None: ...

class RobotInfo:
    robot_name: str
    robot_type: str
    firmware_version: str
    hardware_version: str
    software_version: str
    manufacturer: str
    def __init__(self) -> None: ...

class power_charge_state:
    status: int
    temperature: int
    soc: int
    def __init__(self) -> None: ...

class ImuData:
    rpy: List[float]
    omega: List[float]
    error_flag: int
    def __init__(self) -> None: ...

class XboxMap:
    a: int; b: int; x: int; y: int
    leftShoulder: int; rightShoulder: int
    leftStick: int; rightStick: int
    start: int; back: int
    dpadUp: int; dpadDown: int; dpadLeft: int; dpadRight: int
    leftX: float; leftY: float; rightX: float; rightY: float
    leftTrigger: float; rightTrigger: float
    error_flag: int
    def __init__(self) -> None: ...

class HighLevelControl:
    control_type: int
    control_part: int
    waist_position: List[float]  # 长度 3
    hand_pose: List[float]       # 长度 7: [x,y,z,qx,qy,qz,qw]
    eef_velocity: float
    eef_acceleration: float
    block: bool
    duration: float
    def __init__(self) -> None: ...

class HighLevelState:
    state: int
    progress: int
    def __init__(self) -> None: ...

# --- 主机器人导航类 ---

class H1Robot:
    @overload
    def __init__(self) -> None: ...
    @overload
    def __init__(self, udp_url: str) -> None: ...

    def robot_connect(self) -> bool: ...
    def switchControlMode(self, new_mode: int) -> bool: ...
    def robot_init(self) -> bool: ...
    def robot_deinit(self) -> bool: ...

    # --- LOW-LEVEL 控制 ---
    def setMotorControl_low(self, motor_id: int, control: Motor_Control) -> bool: ...
    def setChassis_low(self, chassis_id: int, control: Motor_Control) -> bool: ...
    def setWaist_low(self, waist_id: int, control: Motor_Control) -> bool: ...
    def setHead_low(self, head_id: int, control: Motor_Control) -> bool: ...
    def setArm_low(self, arm_id: int, control: Motor_Control) -> bool: ...
    def setGripper_low(self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True) -> bool: ...
    def setHand_low(self, arm: int, control: HandControl) -> bool: ...
    def setFixedRod_low(self, is_open: int) -> bool: ...
    def resetForceSensorData(self, sensor_id: int) -> bool: ...

    # --- HIGH-LEVEL 控制 ---
    def setChassis_high(self, speed_x: float, speed_y: float) -> bool: ...
    def setWaist_high(self, action: ArmEndPose) -> bool: ...
    def setHead_high(self, head_id: int, control: Motor_Control) -> bool: ...
    def setArm_high(self, arm: int, action: ArmEndPose) -> bool: ...
    def setArmMove_high(
        self,
        arm: int,
        action: ArmEndPose,
        eef_velocity: float = 0.0,
        eef_acceleration: float = 0.0,
        duration: float = 0.0,
        block: bool = False) -> bool:
        """
        高层手臂一键运动接口。
        参数:
            arm: ArmAction.LEFT_ARM 或 ArmAction.RIGHT_ARM
            action: ArmEndPose 末端目标位姿
            eef_velocity: 末端速度 (m/s)，0表示默认
            eef_acceleration: 末端加速度 (m/s^2)，0表示默认
            duration: 运动持续时间 (秒)，0表示自动计算
            block: 是否阻塞执行，默认False
        返回:
            bool: 操作是否成功
        """
    def setGripper_high(self, gripper_id: int, control: Motor_Control, is_hold_torque: bool = True) -> bool: ...
    def setHand_high(self, arm: int, control: HandControl) -> bool: ...
    def setFixedRod_high(self, is_open: int) -> bool: ...
    def armPoseToArmEndPose(self, pose: ArmPose) -> ArmEndPose: ...

    # --- 状态获取 (Python 返回 Tuple[bool, Struct]) ---
    def getRobotInfo(self) -> Tuple[bool, RobotInfo]: ...
    def getMotorState(self, motor_id: int) -> Tuple[bool, Motor_Information]: ...
    def getChassisState(self, chassis_id: int) -> Tuple[bool, Motor_Information]: ...
    def getChassisSpeedState(self) -> Tuple[bool, List[float], List[float]]:
        """返回: (ok, speed_actual[2], speed_algo[2])"""
        ...
    def getWaistState(self, waist_id: int) -> Tuple[bool, Motor_Information]: ...
    def getHeadState(self, head_id: int) -> Tuple[bool, Motor_Information]: ...
    def getArmState(self, joint_id: int) -> Tuple[bool, Motor_Information]: ...
    def getGripperState(self, gripper_id: int) -> Tuple[bool, Motor_Information]: ...
    def getGripperControlMode(self) -> Tuple[bool, int]: ...
    def getFixedRodState(self) -> Tuple[bool, int]: ...
    def getJoystickState(self) -> Tuple[bool, XboxMap]: ...
    def getIMU_State(self) -> Tuple[bool, ImuData]: ...
    def getHandState(self, arm: int) -> Tuple[bool, HandState]: ...
    def getHighLevelState(self) -> Tuple[bool, HighLevelState]: ...
    def getForceSensorState(self) -> Tuple[bool, ForceSensorState]: ...
    def getPowerChargeState(self) -> Tuple[bool, power_charge_state]: ...

    # --- 新增：手臂/头部/相机末端相对电机零点位姿 ---
    def getHandActual(self, arm: int) -> Tuple[bool, ArmEndPose]: ...
    def getHeadActual(self) -> Tuple[bool, ArmEndPose]: ...
    def getHandRelative(self, arm: int) -> Tuple[bool, ArmEndPose]: ...
    def getHeadRelative(self) -> Tuple[bool, ArmEndPose]: ...
    def getHeadCameraActual(self) -> Tuple[bool, ArmEndPose]: ...
    def getHandCameraRelative(self, arm: int) -> Tuple[bool, ArmEndPose]: ...
    def getHeadCameraRelative(self) -> Tuple[bool, ArmEndPose]: ...

    def getCurrentMode(self) -> int: ...
    def getInitState(self) -> int: ...
    def isRobotConnected(self) -> bool: ...