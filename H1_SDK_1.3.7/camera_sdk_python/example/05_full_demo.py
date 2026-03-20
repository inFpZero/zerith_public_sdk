#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Robot Camera Client Demo
========================
本程序演示如何使用 Python 客户端连接机器人服务，并实时显示视频流。

主要功能：
1. 连接 gRPC 服务端。
2. 自动发现所有可用相机（包括 RGB 和 深度相机）。
3. 实时拉取视频流并解码。
4. 在画面上叠加 真实帧率(Source FPS) 和 时间戳。
5. 将 16位 深度数据可视化为伪彩色热力图。
"""
import os
import sys
import cv2
import numpy as np
import time
from typing import Dict, List

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from camera_client import CameraClient

# ==========================================
#              用户配置区域
# ==========================================
class Config:
    # 服务端地址 (请根据实际情况修改 IP)
    GRPC_TARGET = "localhost:50051"
    
    # 深度图可视化参数
    # 深度图原始数据单位通常是毫米(mm)。
    # 缩放系数 0.03 意味着: 像素值 255 对应深度约 8.5米 (255 / 0.03 = 8500mm)
    # 如果画面太暗，可以调大此数值；如果太亮（一片红），请调小。
    DEPTH_SCALE_FACTOR = 0.03
    
    # 界面显示设置
    FONT_SCALE = 0.6            # 字体大小
    TEXT_COLOR = (0, 255, 0)    # 字体颜色 (B, G, R) -> 绿色
    TEXT_THICKNESS = 1          # 字体粗细
    
    # 窗口刷新逻辑
    WAIT_KEY_DELAY = 1          # cv2.waitKey 的等待时间(ms)


# ==========================================
#              工具类与辅助函数
# ==========================================

class SourceFpsTracker:
    """
    [工具类] 源端帧率追踪器
    ---------------------
    用途：计算相机实际传输的帧率，而不是本地循环的刷新率。
    原理：仅当接收到的图像 '时间戳' 发生变化时才计数。
    """
    def __init__(self, stat_interval=1.0):
        self.last_ts = -1.0
        self.window_start_ts = -1.0
        self.frames_in_window = 0
        self.current_fps = 0.0
        self.stat_interval = stat_interval # 统计周期(秒)

    def update(self, timestamp: float) -> float:
        # 如果时间戳没变（或者是旧帧），说明没有新数据，直接返回旧FPS
        if timestamp <= self.last_ts:
            return self.current_fps

        # 初始化第一个时间窗口
        if self.window_start_ts < 0:
            self.window_start_ts = timestamp
            self.last_ts = timestamp
            return 0.0

        # 累加帧数
        self.frames_in_window += 1
        self.last_ts = timestamp
        
        # 检查是否达到统计周期
        duration = timestamp - self.window_start_ts
        if duration >= self.stat_interval:
            # 计算 FPS
            self.current_fps = self.frames_in_window / duration
            # 重置窗口
            self.window_start_ts = timestamp
            self.frames_in_window = 0
            
        return self.current_fps

def visualize_depth(depth_raw: np.ndarray) -> np.ndarray:
    """
    将 16位(uint16) 深度数据转换为 8位(uint8) 彩色热力图供人眼观察。
    """
    # 1. 线性缩放: uint16 -> uint8
    depth_8bit = cv2.convertScaleAbs(depth_raw, alpha=Config.DEPTH_SCALE_FACTOR)
    
    # 2. 伪彩色映射: 这里的 COLORMAP_JET 会让近处偏蓝/青，远处偏红
    depth_color = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
    return depth_color

def draw_hud(image: np.ndarray, text_lines: List[str]) -> np.ndarray:
    """
    在图像左上角绘制 HUD (Head-Up Display) 信息。
    包含黑色描边，确保在任何背景下文字都清晰可见。
    """
    x, y = 10, 30
    line_height = 25
    
    for line in text_lines:
        # 绘制黑色描边 (背景)
        cv2.putText(image, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 
                    Config.FONT_SCALE, (0, 0, 0), Config.TEXT_THICKNESS + 2)
        # 绘制彩色文字 (前景)
        cv2.putText(image, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 
                    Config.FONT_SCALE, Config.TEXT_COLOR, Config.TEXT_THICKNESS)
        y += line_height
    return image

def create_placeholder_image(text: str) -> np.ndarray:
    """创建一个黑色背景图，用于无信号时占位"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, text, (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 
                1.0, (200, 200, 200), 2)
    return img


# ==========================================
#                 主程序逻辑
# ==========================================

def main():
    # 1. 初始化客户端
    # -------------------------------------------------
    client = CameraClient(grpc_target=Config.GRPC_TARGET)
    print(f"[*] 正在连接 gRPC 服务: {Config.GRPC_TARGET} ...")
    
    try:
        client.start()
        print("[+] 连接建立成功，开始握手...")
        
        # 获取服务端状态，用于发现有哪些相机可用
        state = client.get_state(timeout=5.0)
    except Exception as e:
        print(f"[!] 连接或初始化失败: {e}")
        print("    请检查: 1. 服务端是否运行  2. IP端口配置  3. 网络防火墙")
        return

    # 2. 解析相机列表
    # -------------------------------------------------
    rgb_cameras = []
    depth_cameras = []
    
    print("\n--- 发现设备 ---")
    for cam_config in state.camera_configs:
        name = cam_config.camera_name
        
        is_depth = False
        is_color = False
        stream_info = []
        for s in cam_config.streams:
            stream_info.append(f"{s.type}")
            if s.type == "depth":
                is_depth = True
            elif s.type == "color":
                is_color = True
                
        if is_depth:
            depth_cameras.append(name)
        if is_color:
            rgb_cameras.append(name)
            
        print(f"• 相机: {name:<15} 流: {stream_info}")
    print("----------------\n")

    if not rgb_cameras:
        print("[!] 未找到可用相机，程序退出。")
        client.stop()
        return

    # 3. 准备 FPS 追踪器
    # -------------------------------------------------
    # 为每一个相机创建独立的 FPS 计数器
    trackers: Dict[str, SourceFpsTracker] = {}
    for name in rgb_cameras:
        trackers[f"rgb_{name}"] = SourceFpsTracker()
    for name in depth_cameras:
        trackers[f"depth_{name}"] = SourceFpsTracker()

    print(">> 视频流接收中... 按 'q' 键或 'Ctrl+C' 退出。")

    # 4. 主循环 (渲染与显示)
    # -------------------------------------------------
    try:
        while True:
            # 标记本轮循环是否有窗口更新，用于优化 CPU 占用
            any_window_updated = False

            # --- 处理所有 RGB 相机 ---
            for name in rgb_cameras:
                # 获取最新一帧 (Image, Timestamp)
                frame_data = client.get_latest_frame(name)
                
                if frame_data:
                    image, timestamp = frame_data
                    
                    # 计算源端 FPS
                    fps = trackers[f"rgb_{name}"].update(timestamp)
                    
                    # 绘制 OSD 信息
                    info_lines = [
                        f"CAM: {name} (RGB)",
                        f"FPS: {fps:.1f}",
                        f"TS : {timestamp:.3f}"
                    ]
                    image_display = draw_hud(image, info_lines)
                else:
                    # 如果还没有收到数据，显示等待画面
                    image_display = create_placeholder_image(f"Waiting: {name}")

                cv2.imshow(f"RGB - {name}", image_display)
                any_window_updated = True

            # --- 处理所有 Depth 相机 ---
            for name in depth_cameras:
                depth_data = client.get_latest_depth(name)
                
                if depth_data:
                    depth_raw, timestamp = depth_data
                    
                    # 计算 FPS
                    fps = trackers[f"depth_{name}"].update(timestamp)
                    
                    # 可视化处理 (关键步骤)
                    depth_display = visualize_depth(depth_raw)
                    
                    info_lines = [
                        f"CAM: {name} (Depth)",
                        f"FPS: {fps:.1f}"
                    ]
                    depth_display = draw_hud(depth_display, info_lines)
                    
                    cv2.imshow(f"Depth - {name}", depth_display)
                    any_window_updated = True

            # --- 键盘控制 ---
            key = cv2.waitKey(Config.WAIT_KEY_DELAY) & 0xFF
            if key == ord('q'):
                print("\n[+] 用户请求退出...")
                break
            
            # 如果没有创建任何窗口（比如刚启动），稍微休眠防止空转
            if not any_window_updated:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[+] 检测到键盘中断...")
    finally:
        # 5. 资源清理
        # -------------------------------------------------
        print("[*] 正在断开连接与释放资源...")
        client.stop()
        cv2.destroyAllWindows()
        print("[*] 程序已结束。")

if __name__ == "__main__":
    main()