"""
@file 01_minimal_color.py
@brief 【示例 01】最简彩色图获取
* [功能介绍]
1. 展示如何初始化 CameraClient 并建立连接。
2. 演示非阻塞式获取最新彩色图像 (RGB) 并使用 OpenCV 显示。
* [使用说明]
1. 修改 GRPC_TARGET 为服务端真实的 IP 地址。
2. 修改 CAMERA_NAME 为服务端对应的相机 ID。
3. 运行脚本后，按 'q' 键退出预览。
"""

import os
import sys
import cv2

# --- 路径兼容处理：确保能找到上一级目录中的 camera_client.py ---
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

def main():
    # ---------------------------------------------------------
    # 【用户配置区】请根据实际环境修改以下参数
    # ---------------------------------------------------------
    GRPC_TARGET = "localhost:50051"  # 服务端 IP:端口
    CAMERA_NAME = "rs/cam_left_wrist"         # 彩色相机 ID
    # ---------------------------------------------------------

    # 1. 初始化客户端
    client = CameraClient(grpc_target=GRPC_TARGET)
    
    try:
        # 2. 启动客户端 (执行 gRPC 握手与 WebRTC 建立)
        print(f"[*] 正在连接服务 {GRPC_TARGET} ...")
        client.start()
        print(f"[+] 连接成功。正在获取相机 [{CAMERA_NAME}] 的画面...")

        while True:
            # 3. 获取最新彩色帧 (返回: (numpy_bgr_array, timestamp) 或 None)
            data = client.get_latest_frame(CAMERA_NAME)
            
            if data:
                img, ts = data
                # 显示图像
                cv2.imshow("Color Viewer", img)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\n[+] 用户请求退出。")
                break

    except Exception as e:
        print(f"\n[!] 运行出错: {e}")
    finally:
        # 4. 停止客户端，释放网络资源
        print("[*] 正在释放资源并断开连接...")
        client.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()