import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import parser, robot_for


class FpsTracker:
    def __init__(self):
        self.start = None
        self.last = -1.0
        self.frames = 0
        self.fps = 0.0

    def update(self, timestamp: float) -> float:
        if timestamp <= self.last:
            return self.fps
        if self.start is None:
            self.start = timestamp
        self.frames += 1
        self.last = timestamp
        elapsed = timestamp - self.start
        if elapsed >= 1.0:
            self.fps = self.frames / elapsed
            self.start, self.frames = timestamp, 0
        return self.fps


def placeholder(text: str):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(image, text, (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
    return image


def hud(image, name: str, kind: str, timestamp: float, fps: float):
    output = image.copy()
    for line, y in ((f"CAM: {name} ({kind})", 30), (f"FPS: {fps:.1f}", 55),
                    (f"TS: {timestamp:.3f}", 80)):
        cv2.putText(output, line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
    return output


args = parser("相机 SDK 综合演示").parse_args()
robot = robot_for(args.robot_ip)
trackers = {}

try:
    robot.startCameraWebRTC()
    configs = robot.getCameraState([], 5.0)["camera_configs"]
    rgb = [c["camera_name"] for c in configs if any(s["type"] == "color" for s in c["streams"])]
    depth = [c["camera_name"] for c in configs if any(s["type"] == "depth" for s in c["streams"])]
    print("RGB:", rgb, "Depth:", depth)
    while True:
        updated = False
        for kind, names in (("RGB", rgb), ("Depth", depth)):
            for name in names:
                key = kind + name
                trackers.setdefault(key, FpsTracker())
                sample = (robot.getLatestCameraFrame(name) if kind == "RGB"
                          else robot.getLatestCameraDepth(name))
                if sample is None:
                    image = placeholder("Waiting: " + name)
                else:
                    image, timestamp = sample
                    if kind == "Depth":
                        image = cv2.applyColorMap(cv2.convertScaleAbs(image, alpha=0.03), cv2.COLORMAP_JET)
                    image = hud(image, name, kind, timestamp, trackers[key].update(timestamp))
                cv2.imshow(kind + " - " + name, image)
                updated = True
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        if not updated:
            time.sleep(0.01)
finally:
    robot.stopCameraWebRTC()
    cv2.destroyAllWindows()
