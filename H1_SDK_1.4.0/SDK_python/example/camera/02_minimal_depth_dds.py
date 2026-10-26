import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


args_parser = parser("深度图读取与伪彩色显示")
args_parser.add_argument("--camera", default="rs/cam_high")
args = args_parser.parse_args()
robot = robot_for(args.robot_ip)

try:
    robot.startCameraDds()
    while True:
        sample = robot.getLatestCameraDepth(args.camera)
        if sample is not None:
            depth, _timestamp = sample
            depth_8bit = cv2.convertScaleAbs(depth, alpha=0.03)
            cv2.imshow("Depth Viewer", cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    robot.stopCameraDds()
    cv2.destroyAllWindows()
