"""只读机器人状态监控示例"""

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import h1, parser, robot_for


def to_plain(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if isinstance(value, dict):
        return {str(key): to_plain(item) for key, item in value.items()}
    if hasattr(value, "name"):
        return value.name

    result = {}
    for name in dir(value):
        if name.startswith("_"):
            continue
        try:
            item = getattr(value, name)
        except Exception:
            continue
        if not callable(item):
            result[name] = to_plain(item)
    return result or str(value)


def query(result: dict, key: str, callback) -> None:
    try:
        response = callback()
        if isinstance(response, tuple) and response and isinstance(response[0], bool):
            if response[0]:
                values = response[1:]
                result[key] = to_plain(values[0] if len(values) == 1 else values)
            else:
                result[key] = {"available": False}
        else:
            result[key] = to_plain(response)
    except Exception as error:
        result[key] = {"error": str(error)}


def motor_members():
    return {
        name: value
        for name, value in h1.EtherCAT_Motor_Index.__members__.items()
        if name != "MOTOR_COUNT"
    }


def selected_motors(specification: str):
    members = motor_members()
    if specification == "all":
        return members
    names = [name.strip() for name in specification.split(",") if name.strip()]
    unknown = [name for name in names if name not in members]
    if unknown:
        raise ValueError("未知电机: " + ", ".join(unknown))
    return {name: members[name] for name in names}


def snapshot(robot, motors) -> dict:
    result = {
        "timestamp": time.time(),
        "connected": robot.isRobotConnected(),
        "system_init": to_plain(robot.getInitState()),
        "system_init_status": to_plain(robot.getSystemInitStatus()),
        "control_mode": to_plain(robot.getCurrentMode()),
        "control_mode_status": to_plain(robot.getControlModeStatus()),
    }
    query(result, "robot_info", robot.getRobotInfo)
    query(result, "power", robot.getPowerChargeState)
    query(result, "imu", robot.getIMU_State)
    query(result, "joystick", robot.getJoystickState)
    query(result, "high_level", robot.getHighLevelState)
    query(result, "chassis_speed", robot.getChassisSpeedState)
    query(result, "gripper_mode", robot.getGripperControlMode)
    query(result, "fixed_rod", robot.getFixedRodState)
    query(result, "head_relative", robot.getHeadRelative)
    query(result, "head_camera_relative", robot.getHeadCameraRelative)

    arms = {}
    for name, side in (
        ("LEFT_ARM", h1.ArmAction.LEFT_ARM),
        ("RIGHT_ARM", h1.ArmAction.RIGHT_ARM),
    ):
        query(arms, f"{name}.hand_relative", lambda side=side: robot.getHandRelative(side))
        query(
            arms,
            f"{name}.hand_camera_relative",
            lambda side=side: robot.getHandCameraRelative(side),
        )
    result["arms"] = arms

    motor_states = {}
    for name, motor in motors.items():
        query(motor_states, name, lambda motor=motor: robot.getMotorState(motor))
    result["motors"] = motor_states
    return result


def main() -> None:
    args_parser = parser("DDS 版 H1 SDK 只读状态监控")
    args_parser.add_argument("--period", type=float, default=1.0, help="查询周期（秒）")
    args_parser.add_argument("--count", type=int, default=0, help="输出次数，0 表示持续运行")
    args_parser.add_argument(
        "--motors",
        default="all",
        help="all 或逗号分隔的 EtherCAT_Motor_Index 成员名",
    )
    args = args_parser.parse_args()
    if args.period <= 0 or args.count < 0:
        args_parser.error("--period 必须大于 0，--count 不能小于 0")
    try:
        motors = selected_motors(args.motors)
    except ValueError as error:
        args_parser.error(str(error))

    robot = robot_for(args.robot_ip)
    if not robot.robot_connect():
        raise RuntimeError("robot_connect failed")

    emitted = 0
    try:
        while args.count == 0 or emitted < args.count:
            print(json.dumps(snapshot(robot, motors), ensure_ascii=False, indent=2))
            emitted += 1
            if args.count == 0 or emitted < args.count:
                time.sleep(args.period)
    except KeyboardInterrupt:
        print("状态监控已停止", file=sys.stderr)


if __name__ == "__main__":
    main()
