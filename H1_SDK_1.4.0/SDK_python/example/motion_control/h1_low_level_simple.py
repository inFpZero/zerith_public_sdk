"""Python counterpart of SDKClient h1_low_level_simple.cpp."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import connect_and_init, h1, parser, robot_for


rate = 500
time_sleep = 1.0 / rate


def check(ok, operation):
    if not ok:
        raise RuntimeError(f"{operation} failed")


def command(position=0.0, speed=0.0, torque=0.0):
    value = h1.Motor_Control()
    value.Position, value.Speed, value.Torque = position, speed, torque
    return value


def send(robot, motor, control):
    check(robot.setMotorControl_low(motor, control), "setMotorControl_low")


def read_state(robot, motor):
    result = robot.getMotorState(motor)
    if isinstance(result, tuple) and not result[0]:
        raise RuntimeError("getMotorState failed")
    return result


def ChassisTest(robot):
    left = h1.EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT
    right = h1.EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT
    for speed, seconds, read in ((1.0, 3.0, True), (0.0, 2.0, False), (-1.0, 3.0, True), (0.0, 2.0, False)):
        control = command(speed=speed)
        send(robot, left, control)
        send(robot, right, control)
        time.sleep(seconds)
        if read:
            read_state(robot, left)
            read_state(robot, right)


def WaistTest(robot):
    lift = h1.EtherCAT_Motor_Index.MOTOR_LIFT
    control = command(position=0.5)
    send(robot, lift, control)
    time.sleep(3.0)
    control.Position = 0.0
    send(robot, lift, control)
    time.sleep(3.0)

    waist_down = h1.EtherCAT_Motor_Index.MOTOR_WAIST_DOWN
    control = command()
    for count, increment in ((1000, 0.0002), (1000, -0.0005)):
        for _ in range(count):
            control.Position += increment
            send(robot, waist_down, control)
            time.sleep(time_sleep)
        time.sleep(1.5)
        read_state(robot, waist_down)

    waist_up = h1.EtherCAT_Motor_Index.MOTOR_WAIST_UP
    control = command()
    for count, increment in ((500, 0.0005), (1000, -0.0005), (500, 0.0005)):
        for _ in range(count):
            control.Position += increment
            send(robot, waist_up, control)
            time.sleep(time_sleep)
        time.sleep(1.5)
        read_state(robot, waist_up)
    time.sleep(2.0)


def HeadTest(robot):
    head_up = h1.EtherCAT_Motor_Index.MOTOR_HEAD_UP
    control = command()
    for count, increment in ((1000, 0.0002), (1800, -0.0002), (1000, 0.0002)):
        for _ in range(count):
            control.Position += increment
            send(robot, head_up, control)
            time.sleep(time_sleep)
        time.sleep(1.0)
        read_state(robot, head_up)

    head_down = h1.EtherCAT_Motor_Index.MOTOR_HEAD_DOWN
    control = command()
    for count, increment in ((1000, 0.001), (2000, -0.001), (1000, 0.001)):
        for _ in range(count):
            control.Position += increment
            send(robot, head_down, control)
            time.sleep(time_sleep)
        time.sleep(1.0)
        read_state(robot, head_down)
    time.sleep(1.0)


def ArmTest(robot):
    motors = (h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_7, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_7)
    control = command()
    for count, increment in ((500, -0.001), (1000, 0.001), (500, -0.001)):
        for _ in range(count):
            control.Position += increment
            for motor in motors:
                send(robot, motor, control)
            time.sleep(time_sleep)
        for motor in motors:
            read_state(robot, motor)


def GripperTest(robot):
    motors = (h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8, h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8)
    opened, closed = command(position=0.0), command(position=1.0)
    for _ in range(5):
        for control in (opened, closed):
            for motor in motors:
                send(robot, motor, control)
            time.sleep(1.0)


TESTS = {
    "chassis": ChassisTest,
    "waist": WaistTest,
    "head": HeadTest,
    "arm": ArmTest,
    "gripper": GripperTest,
}


def main():
    args_parser = parser("H1 SDK C++ 对齐版低层控制示例")
    args_parser.add_argument("--test", choices=(*TESTS, "all"), default="all")
    args = args_parser.parse_args()
    selected = tuple(TESTS) if args.test == "all" else (args.test,)
    robot = robot_for(args.robot_ip)
    initialized = False
    try:
        connect_and_init(robot, h1.MotorControlMode.LOW_LEVEL)
        initialized = True
        time.sleep(3.0)
        for name in selected:
            TESTS[name](robot)
        time.sleep(1.0)
    finally:
        if initialized and not robot.robot_deinit():
            print("警告：robot_deinit failed", file=sys.stderr)


if __name__ == "__main__":
    main()
