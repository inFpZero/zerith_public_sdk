import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


args = parser("查询全部相机配置与内参").parse_args()
robot = robot_for(args.robot_ip)

try:
    robot.startCameraWebRTC()
    state = robot.getCameraState([], 5.0)
    print(json.dumps(state, ensure_ascii=False, indent=2))
finally:
    robot.stopCameraWebRTC()
