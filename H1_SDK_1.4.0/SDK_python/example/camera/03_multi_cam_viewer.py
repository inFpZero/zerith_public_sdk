import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


args = parser("自动发现并显示多路彩色相机").parse_args()
robot = robot_for(args.robot_ip)

try:
    robot.startCameraWebRTC()
    state = robot.getCameraState([], 5.0)
    camera_names = [item["camera_name"] for item in state["camera_configs"]]
    print("发现相机:", camera_names)
    while True:
        for name in camera_names:
            sample = robot.getLatestCameraFrame(name)
            if sample is not None:
                cv2.imshow("Stream: " + name, sample[0])
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    robot.stopCameraWebRTC()
    cv2.destroyAllWindows()
