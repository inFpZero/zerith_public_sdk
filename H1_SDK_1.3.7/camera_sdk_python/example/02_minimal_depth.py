"""
@file 02_minimal_depth.py
@brief 【示例 02】深度图原始数据读取与可视化
* [功能介绍]
1. 展示如何获取 16位 (uint16) 原始深度像素数据。
2. 演示如何读取特定像素点（如中心点）的物理深度值（单位：mm）。
3. 展示将原始深度数据映射为伪彩色图（Color Map）以进行预览。
* [使用说明]
1. 修改 GRPC_TARGET 为服务端 IP，确保相机支持深度流。
2. 原始深度图直接显示通常全黑，必须经过缩放转换。
* [关键技术]
- 伪彩色应用：使用 cv2.COLORMAP_JET 增强深度层次感。
"""

import os
import sys
import cv2
import numpy as np

# 路径兼容：将 src 目录加入搜索路径以导入 camera_client
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

def main():
    GRPC_TARGET = "localhost:50051"
    CAMERA_NAME = "rs/cam_left_wrist"

    client = CameraClient(grpc_target=GRPC_TARGET)
    client.start()

    print(f"[*] 深度流取流中: {CAMERA_NAME}，按 'q' 键退出...")

    try:
        while True:
            # 获取数据 (numpy 数组格式)
            data = client.get_latest_depth(CAMERA_NAME)
            
            if data:
                depth_raw, timestamp = data
                
                # 可视化处理
                # 将 16-bit 线性缩放到 8-bit (0.03 为经验值，可根据实际量程调整)
                depth_viz = (depth_raw * 0.03).clip(0, 255).astype(np.uint8)
                # 应用色彩映射
                depth_color = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)

                cv2.imshow("Depth Viewer", depth_color)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        client.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()