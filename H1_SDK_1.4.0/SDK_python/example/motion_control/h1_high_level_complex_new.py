"""Python counterpart of SDKClient h1_high_level_complex_new.cpp."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import connect_and_init, h1, parser, robot_for


rate = 500
time_sleep = 1.0 / rate
advance, retreat, stop = 0.2, -0.2, 0.0
left, right = 0.1, -0.1
lift_up, lift_down = 0.3, 0.0
waist_pitch_buffer, waist_yaw_buffer = 0.0005, 0.0002
x, y, z = 0.0001, 0.0001, 0.0001
roll, pitch, yaw = 0.001, 0.001, 0.0001
gripper_open, gripper_close = 0.0, 1.45
head_pitch_buffer, head_yaw_buffer = 0.0002, 0.001


def check(ok, operation):
    if not ok:
        raise RuntimeError(f"{operation} failed")


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    value = h1.ArmPose()
    value.x, value.y, value.z = x, y, z
    value.roll, value.pitch, value.yaw = roll, pitch, yaw
    return value


def motor_control(position=0.0, speed=0.0, torque=0.0):
    value = h1.Motor_Control()
    value.Position, value.Speed, value.Torque = position, speed, torque
    return value


def _send_waist(robot, action):
    check(robot.setWaist_high(robot.armPoseToArmEndPose(action)), "setWaist_high")


def ChassisTest_advance(robot):
    for speed_x, speed_y, seconds in (
        (advance, stop, 3.0), (stop, stop, 2.0),
        (retreat, stop, 3.0), (stop, stop, 2.0),
    ):
        check(robot.setChassis_high(speed_x, speed_y), "setChassis_high")
        time.sleep(seconds)


def ChassisTest_turn(robot):
    for speed_x, speed_y, seconds in (
        (stop, left, 3.0), (stop, stop, 2.0),
        (stop, right, 3.0), (stop, stop, 2.0),
    ):
        check(robot.setChassis_high(speed_x, speed_y), "setChassis_high")
        time.sleep(seconds)


def WaistTest(robot):
    action = pose()
    for field, count, increment, dwell in (
        ("z", 2000, 0.00005, 2.0),
        ("pitch", 1000, 0.0015, 2.0),
        ("yaw", 500, 0.001, 1.5),
        ("yaw", 1000, -0.001, 1.5),
        ("yaw", 500, 0.001, 1.5),
        ("pitch", 1000, -0.0015, 1.5),
        ("z", 2000, -0.00005, 5.0),
    ):
        for _ in range(count):
            setattr(action, field, getattr(action, field) + increment)
            _send_waist(robot, action)
            time.sleep(time_sleep)
        time.sleep(dwell)
    time.sleep(2.0)


def WaistTest_rise(robot):
    _send_waist(robot, pose(z=lift_up))
    time.sleep(10.0)


def WaistTest_descend(robot):
    _send_waist(robot, pose(z=lift_down))
    time.sleep(10.0)


def WaistTest_uplift(robot):
    action = pose()
    for _ in range(500):
        action.yaw += waist_yaw_buffer
        _send_waist(robot, action)
        time.sleep(time_sleep)
    time.sleep(1.5)


def WaistTest_leftspin(robot):
    action = pose()
    for count, increment in ((500, waist_yaw_buffer), (1000, -waist_yaw_buffer), (500, waist_yaw_buffer)):
        for _ in range(count):
            action.yaw += increment
            _send_waist(robot, action)
            time.sleep(time_sleep)
        time.sleep(1.5)
    time.sleep(2.0)


def HeadTest(robot):
    for motor, segments in (
        (h1.EtherCAT_Motor_Index.MOTOR_HEAD_UP,
         ((1000, head_pitch_buffer), (2000, -head_pitch_buffer), (1000, head_pitch_buffer))),
        (h1.EtherCAT_Motor_Index.MOTOR_HEAD_DOWN,
         ((1000, head_yaw_buffer), (2000, -head_yaw_buffer), (1000, head_yaw_buffer))),
    ):
        control = motor_control()
        for count, increment in segments:
            for _ in range(count):
                control.Position += increment
                check(robot.setHead_high(motor, control), "setHead_high")
                time.sleep(time_sleep)
            time.sleep(1.0)
    time.sleep(1.0)


def ArmTest(robot):
    left_pose, right_pose = pose(), pose()
    segments = (
        ("z", z, z), ("x", x, x), ("y", y, -y),
        ("roll", roll, -roll), ("roll", -roll, roll),
        ("pitch", pitch, pitch), ("pitch", -pitch, -pitch),
        ("yaw", yaw, -yaw), ("yaw", -yaw, yaw),
        ("y", -y, y), ("x", -x, -x), ("z", -z, -z),
    )
    for field, left_increment, right_increment in segments:
        for _ in range(1000):
            setattr(left_pose, field, getattr(left_pose, field) + left_increment)
            setattr(right_pose, field, getattr(right_pose, field) + right_increment)
            check(robot.setArm_high(h1.ArmAction.LEFT_ARM, robot.armPoseToArmEndPose(left_pose)), "setArm_high(left)")
            check(robot.setArm_high(h1.ArmAction.RIGHT_ARM, robot.armPoseToArmEndPose(right_pose)), "setArm_high(right)")
            time.sleep(time_sleep)
    time.sleep(3.0)


def GripperTest(robot):
    for position in (gripper_open, gripper_close):
        control = motor_control(position)
        check(robot.setGripper_high(h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8, control), "setGripper_high(left)")
        check(robot.setGripper_high(h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8, control), "setGripper_high(right)")
        time.sleep(3.0)


def HighLevelControlTest_arm(robot):
    for arm_pose in (pose(0.1, -0.1, 0.2, 0.0, 0.0, 0.0),):
        target = robot.armPoseToArmEndPose(arm_pose)
        check(robot.setArmMove_high(h1.ArmAction.RIGHT_ARM, target, 0.0, 0.0, 10.0), "setArmMove_high")
        time.sleep(20.0)


def HighLevelControlTest_arm1(robot):
    target = robot.armPoseToArmEndPose(pose(0.2, -0.2, 0.0, 0.0, 0.0, 0.0))
    check(robot.setArmMove_high(h1.ArmAction.LEFT_ARM, target), "setArmMove_high")
    time.sleep(5.0)


TESTS = {
    "chassis-advance": ChassisTest_advance,
    "chassis-turn": ChassisTest_turn,
    "waist": WaistTest,
    "waist-rise": WaistTest_rise,
    "waist-descend": WaistTest_descend,
    "waist-uplift": WaistTest_uplift,
    "waist-leftspin": WaistTest_leftspin,
    "head": HeadTest,
    "arm": ArmTest,
    "gripper": GripperTest,
    "arm-move": HighLevelControlTest_arm,
    "arm-move-left": HighLevelControlTest_arm1,
}


def main():
    args_parser = parser("H1 SDK C++ 对齐版高层控制示例")
    args_parser.add_argument("--test", choices=(*TESTS, "all"), default="arm")
    args = args_parser.parse_args()
    selected = tuple(TESTS) if args.test == "all" else (args.test,)
    robot = robot_for(args.robot_ip)
    initialized = False
    try:
        connect_and_init(robot, h1.MotorControlMode.HIGH_LEVEL)
        initialized = True
        time.sleep(5.0)
        for name in selected:
            TESTS[name](robot)
        time.sleep(3.0)
        time.sleep(5.0)
    finally:
        if initialized and not robot.robot_deinit():
            print("警告：robot_deinit failed", file=sys.stderr)


if __name__ == "__main__":
    main()
