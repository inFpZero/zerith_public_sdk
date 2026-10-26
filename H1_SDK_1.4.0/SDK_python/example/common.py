import argparse
import sys
from pathlib import Path


LIB_DIR = Path(__file__).resolve().parents[1] / "lib"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

import lib_h1_sdk_python as h1


def parser(description: str) -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=description)
    result.add_argument(
        "--ip", dest="robot_ip", default="127.0.0.1", help="机器人 IP"
    )
    return result


def robot_for(ip: str):
    return h1.H1Robot.get_instance(ip)


def connect_and_init(robot, mode) -> None:
    if not robot.robot_connect():
        raise RuntimeError("robot_connect failed")
    if not robot.getControlPermission():
        raise RuntimeError("getControlPermission failed")
    if not robot.switchControlMode(mode):
        raise RuntimeError("switchControlMode failed")
    if not robot.robot_init():
        raise RuntimeError("robot_init failed")


def require_api(robot, *names: str) -> None:
    missing = [name for name in names if not hasattr(robot, name)]
    if missing:
        raise RuntimeError(
            "当前 H1Robot 公共接口未提供: " + ", ".join(missing)
        )
