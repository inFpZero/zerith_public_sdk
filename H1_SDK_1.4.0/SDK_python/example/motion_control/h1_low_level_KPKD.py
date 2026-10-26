"""Python counterpart of SDKClient h1_low_level_KPKD.cpp."""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import connect_and_init, h1, parser, robot_for


motor_kp = [0, 0, 0, 500, 500, 20, 20, 250, 250, 250, 250, 30, 40, 40, 10, 250, 250, 250, 250, 30, 40, 40, 10]
motor_kd = [0, 0, 0, 5, 5, 1, 1, 5, 5, 5, 5, 1, 1, 1, 1, 5, 5, 5, 5, 1, 1, 1, 1]


def _index(motor_id):
    return int(motor_id.value if hasattr(motor_id, "value") else motor_id)


def _command(position, kp, kd):
    control = h1.Motor_Control()
    control.Position = position
    control.Speed = 0.0
    control.Torque = 0.0
    control.KP = kp
    control.KD = kd
    return control


def _send(robot, motor_id, control):
    if not robot.setMotorControl_low(motor_id, control):
        raise RuntimeError("setMotorControl_low failed")


def _read(robot, motor_id):
    result = robot.getMotorState(motor_id)
    if isinstance(result, tuple) and not result[0]:
        raise RuntimeError("getMotorState failed")


def TestMotorKP(robot, motor_id, init_pos, steps=10, kp_min_coeff=0.8, kp_max_coeff=1.2):
    index = _index(motor_id)
    if motor_kp[index] == 0:
        return
    default = float(motor_kp[index])
    maximum, minimum = default * kp_max_coeff, default * kp_min_coeff
    control = _command(init_pos, default, motor_kd[index])
    paths = ((default, maximum), (maximum, default), (default, minimum), (minimum, default))
    for start, end in paths:
        for step in range(steps + 1):
            control.KP = start + (end - start) * step / steps
            _send(robot, motor_id, control)
            time.sleep(0.3)
        time.sleep(2.0)
    _read(robot, motor_id)


def TestMotorKD(robot, motor_id, init_pos, steps=10, kd_min_coeff=0.8, kd_max_coeff=1.2):
    index = _index(motor_id)
    if motor_kd[index] == 0:
        return
    default = float(motor_kd[index])
    maximum, minimum = default * kd_max_coeff, default * kd_min_coeff
    control = _command(init_pos, motor_kp[index], default)
    paths = ((default, maximum), (maximum, default), (default, minimum), (minimum, default))
    for start, end in paths:
        for step in range(steps + 1):
            control.KD = start + (end - start) * step / steps
            _send(robot, motor_id, control)
            time.sleep(0.3)
        time.sleep(2.0)
    _read(robot, motor_id)


HEAD_IDS = (
    h1.EtherCAT_Motor_Index.MOTOR_HEAD_DOWN,
    h1.EtherCAT_Motor_Index.MOTOR_HEAD_UP,
)
ARM_IDS = tuple(
    getattr(h1.EtherCAT_Motor_Index, f"MOTOR_{side}_ARM_{joint}")
    for side in ("LEFT", "RIGHT") for joint in range(1, 8)
)
GRIPPER_IDS = (
    h1.EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8,
    h1.EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8,
)


def _group(robot, ids, callback, steps, minimum):
    for motor_id in ids:
        callback(robot, motor_id, 0.0, steps, minimum, 1.2)
        time.sleep(3.0)


def TestHeadKP(robot, steps=10):
    _group(robot, HEAD_IDS, TestMotorKP, steps, 0.0)


def TestHeadKD(robot, steps=10):
    _group(robot, HEAD_IDS, TestMotorKD, steps, 0.0)


def TestArmKP(robot, steps=10):
    _group(robot, ARM_IDS, TestMotorKP, steps, 0.3)


def TestArmKD(robot, steps=10):
    _group(robot, ARM_IDS, TestMotorKD, steps, 0.3)


def TestGripperKP(robot, steps=10):
    _group(robot, GRIPPER_IDS, TestMotorKP, steps, 0.0)


def TestGripperKD(robot, steps=10):
    _group(robot, GRIPPER_IDS, TestMotorKD, steps, 0.0)


TESTS = {
    "head-kp": TestHeadKP,
    "head-kd": TestHeadKD,
    "arm-kp": TestArmKP,
    "arm-kd": TestArmKD,
    "gripper-kp": TestGripperKP,
    "gripper-kd": TestGripperKD,
}


def main():
    args_parser = parser("H1 SDK C++ 对齐版电机 KP/KD 测试")
    args_parser.add_argument("--test", choices=(*TESTS, "all"), default="head-kp")
    args_parser.add_argument("--steps", type=int, default=10)
    args = args_parser.parse_args()
    if args.steps <= 0:
        args_parser.error("--steps 必须大于 0")
    selected = tuple(TESTS) if args.test == "all" else (args.test,)
    robot = robot_for(args.robot_ip)
    initialized = False
    try:
        connect_and_init(robot, h1.MotorControlMode.LOW_LEVEL)
        initialized = True
        time.sleep(3.0)
        for name in selected:
            TESTS[name](robot, args.steps)
    finally:
        if initialized and not robot.robot_deinit():
            print("警告：robot_deinit failed", file=sys.stderr)


if __name__ == "__main__":
    main()
