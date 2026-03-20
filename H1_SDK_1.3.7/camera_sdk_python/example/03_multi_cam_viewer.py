"""
@file 03_multi_cam_viewer.py
@brief 【示例 03】多相机自动发现与流显示
* [功能介绍]
1. 调用 get_state 接口动态查询当前服务端所有在线相机。
2. 自动为每个相机开启对应的预览窗口。
* [使用说明]
1. 仅需配置 GRPC_TARGET。
2. 适合用于验证系统内所有相机是否工作正常。
"""

import os
import sys
import cv2

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

def main():
    GRPC_TARGET = "localhost:50051"
    client = CameraClient(grpc_target=GRPC_TARGET)
    client.start()

    # 1. 动态发现相机
    try:
        state = client.get_state()
        # 提取所有相机的名称
        camera_names = [config.camera_name for config in state.camera_configs]
        print(f"[+] 发现相机列表: {camera_names}")
    except Exception as e:
        print(f"[-] 获取状态失败: {e}")
        client.stop()
        return

    try:
        while True:
            for name in camera_names:
                # 获取每一路的彩色帧
                frame_data = client.get_latest_frame(name)
                if frame_data:
                    img, _ = frame_data
                    cv2.imshow(f"Camera: {name}", img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        client.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()