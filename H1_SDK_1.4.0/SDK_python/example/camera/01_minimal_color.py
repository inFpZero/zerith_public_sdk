import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


args_parser = parser("最简彩色图获取")
args_parser.add_argument("--camera", default="rs/cam_left_wrist")
args = args_parser.parse_args()
robot = robot_for(args.robot_ip)

try:
    robot.startCameraWebRTC()
    while True:
        sample = robot.getLatestCameraFrame(args.camera)
        if sample is not None:
            frame, _timestamp = sample
            cv2.imshow("Color Viewer", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    robot.stopCameraWebRTC()
    cv2.destroyAllWindows()
