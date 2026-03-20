"""
@file 04_get_full_state.py
@brief 【示例 04】获取服务端全量元数据与相机内参
* [功能介绍]
展示如何解析复杂的嵌套 Protobuf 结构，获取相机 ID、分辨率、FPS 以及光学内参（Intrinsics）。
* [使用说明]
运行后程序会打印出结构化的相机配置树，随后自动退出。
"""

import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

def main():
    GRPC_TARGET = "localhost:50051"
    client = CameraClient(grpc_target=GRPC_TARGET)
    
    try:
        client.start()
        state = client.get_state()
        
        print(f"\n{'='*60}")
        print(f"服务端相机状态报告 (Total: {len(state.camera_configs)})")
        print(f"{'='*60}")

        for cam in state.camera_configs:
            print(f"\n[相机 ID]: {cam.camera_name}")
            for i, stream in enumerate(cam.streams):
                print(f"  └─ 流 [{i}]: 类型: {stream.type:8} | 分辨率: {stream.width}x{stream.height} | FPS: {stream.fps}")
                
                # 检查是否有内参数据
                if stream.HasField("intrinsics"):
                    intr = stream.intrinsics
                    print(f"     [光学内参] 模型: {intr.model}")
                    print(f"     [光学内参] fx: {intr.fx:.2f}, fy: {intr.fy:.2f}")
                    print(f"     [光学内参] 主点: ({intr.ppx:.2f}, {intr.ppy:.2f})")
                    print(f"     [光学内参] 畸变系数: {list(intr.coeffs)}")
        print(f"\n{'='*60}")
    except Exception as e:
        print(f"[ERROR] 获取元数据失败: {e}")
    finally:
        client.stop()

if __name__ == "__main__":
    main()