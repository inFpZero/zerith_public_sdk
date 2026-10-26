"""Python counterpart of SDKClient h1_low_level_motor_limit.cpp."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import connect_and_init, h1, parser, robot_for


rate = 500
time_sleep = 1.0 / rate
action_time = 3000
action_frequency = action_time * rate // 1000
motor_limit_max = [0, 0, 0.8, 1.3, 0.7, 1.5, 0.25, 1.5, 2.0, 2.9, 1.5, 2.9, 1.0, 1.0, 1.5, 1.5, 0.3, 2.9, 1.5, 2.9, 1.0, 1.0, 1.5]
motor_limit_min = [0, 0, 0, 0, -0.7, -1.5, -0.17, -2.7, -0.3, -2.9, -1.3, -2.9, -1.0, -1.0, 0, -2.7, -2.0, -2.9, -1.3, -2.9, -1.0, -1.0, 0]


def _index(motor_id):
    return int(motor_id.value if hasattr(motor_id, "value") else motor_id)


def _command(position=0.0, speed=0.0, torque=0.0):
    control = h1.Motor_Control()
    control.Position, control.Speed, control.Torque = position, speed, torque
    return control


def _send(robot, motor_id, control):
    if not robot.setMotorControl_low(motor_id, control):
        raise RuntimeError("setMotorControl_low failed")


def _move(robot, motor_id, point_a, point_b):
    control = _command(point_a)
    increment = (point_b - point_a) / action_frequency
    for _ in range(action_frequency):
        control.Position += increment
        _send(robot, motor_id, control)
        time.sleep(time_sleep)
    time.sleep(2.0)


def MoveZeroToMax(robot, motor_id):
    _move(robot, motor_id, 0.0, motor_limit_max[_index(motor_id)])


def MoveMaxToZero(robot, motor_id):
    _move(robot, motor_id, motor_limit_max[_index(motor_id)], 0.0)


def MoveZeroToMin(robot, motor_id):
    _move(robot, motor_id, 0.0, motor_limit_min[_index(motor_id)])


def MoveMinToZero(robot, motor_id):
    _move(robot, motor_id, motor_limit_min[_index(motor_id)], 0.0)


def MovePointAToB(robot, motor_id, point_a, point_b):
    _move(robot, motor_id, point_a, point_b)


def ChassisTest(robot):
    left = h1.EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT
    right = h1.EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT
    sequence = (
        ((left, right), 1.0, 3.0), ((left, right), 0.0, 2.0),
        ((left, right), -1.0, 3.0), ((left, right), 0.0, 2.0),
        ((left,), 1.0, 3.0), ((left,), 0.0, 2.0),
        ((right,), 1.0, 3.0), ((right,), 0.0, 2.0),
    )
    for motors, speed, seconds in sequence:
        control = _command(speed=speed)
        for motor_id in motors:
            _send(robot, motor_id, control)
        time.sleep(seconds)


def WaistTest_lift_up(robot):
    motor = h1.EtherCAT_Motor_Index.MOTOR_LIFT
    _send(robot, motor, _command(motor_limit_max[_index(motor)]))
    time.sleep(8.0)
    _send(robot, motor, _command(motor_limit_max[_index(motor)] - 0.2))
    time.sleep(2.0)


def WaistTest_lift_down(robot):
    motor = h1.EtherCAT_Motor_Index.MOTOR_LIFT
    _send(robot, motor, _command(motor_limit_min[_index(motor)]))
    time.sleep(2.0)


def WaistTest_bend(robot):
    MoveZeroToMax(robot, h1.EtherCAT_Motor_Index.MOTOR_WAIST_DOWN)


def WaistTest_raiseup(robot):
    MoveMaxToZero(robot, h1.EtherCAT_Motor_Index.MOTOR_WAIST_DOWN)


def WaistTest_turn(robot):
    motor = h1.EtherCAT_Motor_Index.MOTOR_WAIST_UP
    for callback in (MoveZeroToMax, MoveMaxToZero, MoveZeroToMin, MoveMinToZero):
        callback(robot, motor)


def HeadTest(robot):
    for motor in (h1.EtherCAT_Motor_Index.MOTOR_HEAD_UP, h1.EtherCAT_Motor_Index.MOTOR_HEAD_DOWN):
        for callback in (MoveZeroToMax, MoveMaxToZero, MoveZeroToMin, MoveMinToZero):
            callback(robot, motor)


def _four_way(robot, motor):
    for callback in (MoveZeroToMax, MoveMaxToZero, MoveZeroToMin, MoveMinToZero):
        callback(robot, motor)


def ArmTest_all_arm1(robot):
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_1)
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_1)


def ArmTest_left_arm2(robot):
    MoveZeroToMax(robot, h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_2)


def ArmTest_right_arm2(robot):
    MoveZeroToMin(robot, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_2)


def ArmTest_left_arm2_restore(robot):
    motor = h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_2
    MoveMaxToZero(robot, motor)
    MoveZeroToMin(robot, motor)
    MoveMinToZero(robot, motor)


def ArmTest_right_arm2_restore(robot):
    motor = h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_2
    MoveMinToZero(robot, motor)
    MoveZeroToMax(robot, motor)
    MoveMaxToZero(robot, motor)


def ArmTest_left_arm3(robot):
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_3)


def ArmTest_right_arm3(robot):
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_3)


def ArmTest_left_arm4(robot):
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_4)


def ArmTest_right_arm4(robot):
    _four_way(robot, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_4)


def _paired_four_way(robot, joint, max_zero_order=("left", "right")):
    motors = {
        "left": getattr(h1.EtherCAT_Motor_Index, f"MOTOR_LEFT_ARM_{joint}"),
        "right": getattr(h1.EtherCAT_Motor_Index, f"MOTOR_RIGHT_ARM_{joint}"),
    }
    for callback, order in (
        (MoveZeroToMax, ("left", "right")),
        (MoveMaxToZero, max_zero_order),
        (MoveZeroToMin, ("left", "right")),
        (MoveMinToZero, ("left", "right")),
    ):
        for side in order:
            callback(robot, motors[side])


def ArmTest_all_arm5(robot):
    _paired_four_way(robot, 5)


def ArmTest_all_arm6(robot):
    _paired_four_way(robot, 6)


def ArmTest_all_arm7(robot):
    _paired_four_way(robot, 7, ("right", "left"))


def GripperTest(robot):
    motors = (h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8)
    for position in (0.0, 1.5):
        control = _command(position)
        for motor in motors:
            _send(robot, motor, control)
        time.sleep(3.0)


TESTS = {
    "chassis": ChassisTest,
    "waist-lift-up": WaistTest_lift_up,
    "waist-bend": WaistTest_bend,
    "waist-turn": WaistTest_turn,
    "waist-raiseup": WaistTest_raiseup,
    "left-arm2": ArmTest_left_arm2,
    "right-arm2": ArmTest_right_arm2,
    "arm1": ArmTest_all_arm1,
    "left-arm3": ArmTest_left_arm3,
    "right-arm3": ArmTest_right_arm3,
    "left-arm4": ArmTest_left_arm4,
    "right-arm4": ArmTest_right_arm4,
    "arm5": ArmTest_all_arm5,
    "arm6": ArmTest_all_arm6,
    "arm7": ArmTest_all_arm7,
    "left-arm2-restore": ArmTest_left_arm2_restore,
    "right-arm2-restore": ArmTest_right_arm2_restore,
    "head": HeadTest,
    "gripper": GripperTest,
    "waist-lift-down": WaistTest_lift_down,
}


def main():
    args_parser = parser("H1 SDK C++ 对齐版低层电机极限位置测试")
    args_parser.add_argument("--test", choices=(*TESTS, "all"), default="all")
    args = args_parser.parse_args()
    selected = tuple(TESTS) if args.test == "all" else (args.test,)
    robot = robot_for(args.robot_ip)
    initialized = False
    try:
        connect_and_init(robot, h1.MotorControlMode.LOW_LEVEL)
        initialized = True
        time.sleep(5.0)
        for name in selected:
            TESTS[name](robot)
    finally:
        if initialized and not robot.robot_deinit():
            print("警告：robot_deinit failed", file=sys.stderr)


if __name__ == "__main__":
    main()
