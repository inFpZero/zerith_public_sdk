import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


DEFAULT_CONFIG = """{
    "root_dir":"/data/zerith_data",
    "task_id":990001,
    "task_name":"sdk_demo",
    "subtask_num":2,
    "max_episode_time":60,
    "frequency":30,
    "min_free_disk_gb":0
}"""


def run_collection(robot, config=DEFAULT_CONFIG):
    if not robot.robot_connect():
        raise RuntimeError("robot_connect failed")
    if not robot.CreateCollectSession(config):
        raise RuntimeError("CreateCollectSession failed")
    try:
        if not robot.Start():
            raise RuntimeError("Start failed")

        print("Collecting subtask 1 for 5 seconds...")
        time.sleep(5)
        if not robot.NextStep():
            raise RuntimeError("NextStep (subtask 1) failed")

        time.sleep(1)
        if not robot.Pause():
            raise RuntimeError("Pause failed")

        print("Collection paused for 3 seconds...")
        time.sleep(3)
        if not robot.Resume():
            raise RuntimeError("Resume failed")

        time.sleep(1)
        print("Collecting subtask 2 for 5 seconds...")
        time.sleep(5)
        if not robot.NextStep():
            raise RuntimeError("NextStep (subtask 2) failed")

        print("All subtasks completed; waiting for DataEngine to finalize "
              "the episode...")
        time.sleep(10)
    finally:
        robot.Close()

    print("Collection flow finished. Confirm EPISODE_END contains "
          "is_success=true in the log above.")


def main():
    args_parser = parser("gRPC 数据采集控制示例")
    args_parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = args_parser.parse_args()
    run_collection(robot_for(args.robot_ip), args.config)


if __name__ == "__main__":
    main()
