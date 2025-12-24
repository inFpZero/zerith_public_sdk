#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
demo.py - Multi-Camera Display (RGB Only)
"""

import time
import logging
import signal
import threading
from typing import Dict, Tuple, Optional, List
import cv2
import numpy as np
import os
import sys


base = os.path.dirname(__file__)
_libdir = os.path.join(base, "lib")
# ensure package parent dir is importable and lib is a fallback
if base not in sys.path:
    sys.path.insert(0, base)
if _libdir not in sys.path:
    sys.path.insert(0, _libdir)

# try imports in package form first, then fallback to direct names
try:
    from lib.unified_client import UnifiedReceiverClient
    import lib.robot_pb2 as pb
except Exception:
    try:
        from lib.camera_client import UnifiedReceiverClient
        import lib.robot_pb2 as pb
    except Exception:
        # final fallback: allow direct import if lib is on sys.path
        from camera_client import UnifiedReceiverClient
        import robot_pb2 as pb


logging.basicConfig(level=logging.INFO, format="[demo] %(asctime)s %(levelname)s: %(message)s")

# GRPC_TARGET = "localhost:50051"
GRPC_TARGET = "192.168.2.85:50051"



def list_cameras(cli: UnifiedReceiverClient) -> Tuple[List[str], List[str]]:
    """获取所有摄像头并分类返回"""
    st = cli.get_state()
    rs_names, v4l2_names = [], []
    for a in st.actuals:
        which = a.WhichOneof("detail")
        if which == "rs":
            rs_names.append(a.rs.camera_name)
        elif which == "v4l2":
            v4l2_names.append(a.v4l2.camera_name)
    logging.info("RealSense cameras: %s", rs_names)
    logging.info("V4L2 cameras    : %s", v4l2_names)
    return rs_names, v4l2_names


def show_all_cameras_loop(cli: UnifiedReceiverClient, camera_names: List[str], stop_event: threading.Event):
    """
    在单个线程中显示所有摄像头画面（仅RGB）
    """
    window_name = "All Cameras"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1200, 800)
    
    # 存储每个摄像头的状态
    camera_status = {}
    for name in camera_names:
        camera_status[name] = {
            'last_ts': None,
            'fps': 0.0,
            'frame_count': 0,
            'start_time': time.time(),
            'last_update_time': time.time()
        }
    
    try:
        while not stop_event.is_set():
            frames = []
            display_texts = []
            
            for cam_name in camera_names:
                item = cli.get_latest_frame(cam_name)
                status = camera_status[cam_name]
                
                if item is not None:
                    bgr, ts = item
                    status['frame_count'] += 1
                    current_time = time.time()
                    
                    # 计算实时FPS（基于时间间隔）
                    if status['last_ts'] is not None:
                        dt = ts - status['last_ts']
                        if dt > 0:
                            fps = 1.0 / dt
                            # 平滑处理FPS
                            status['fps'] = 0.8 * status['fps'] + 0.2 * fps
                    
                    status['last_ts'] = ts
                    
                    # 计算平均FPS（基于帧计数）
                    elapsed = current_time - status['start_time']
                    avg_fps = status['frame_count'] / elapsed if elapsed > 1 else 0
                    
                    # 每5秒重置一次平均FPS计数，避免长期运行后不准确
                    if current_time - status['start_time'] > 5:
                        status['frame_count'] = 0
                        status['start_time'] = current_time
                    
                    # 调整图像大小以适应显示
                    h, w = bgr.shape[:2]
                    if w > 640 or h > 480:
                        scale = min(640/w, 480/h)
                        new_w, new_h = int(w*scale), int(h*scale)
                        bgr = cv2.resize(bgr, (new_w, new_h))
                    
                    # 添加信息文本
                    info1 = f"{cam_name}"
                    info2 = f"FPS: {status['fps']:4.1f}"
                    info3 = f"Size: {w}x{h}"
                    
                    cv2.putText(bgr, info1, (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(bgr, info2, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    cv2.putText(bgr, info3, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    
                    frames.append(bgr)
                    display_texts.append(f"{cam_name}: {status['fps']:4.1f} FPS")
                else:
                    # 创建等待画面
                    placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                    cv2.putText(placeholder, f"{cam_name}", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
                    cv2.putText(placeholder, "No signal", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    frames.append(placeholder)
                    display_texts.append(f"{cam_name}: No signal")
            
            # 创建多画面拼接
            if frames:
                if len(frames) == 1:
                    # 单个摄像头
                    composite = frames[0]
                elif len(frames) == 2:
                    # 两个摄像头水平排列
                    composite = np.hstack(frames)
                elif len(frames) <= 4:
                    # 2x2 网格
                    rows = []
                    for i in range(0, len(frames), 2):
                        row_frames = frames[i:i+2]
                        # 确保所有行中的图像高度一致
                        max_h = max(f.shape[0] for f in row_frames)
                        padded_frames = []
                        for f in row_frames:
                            if f.shape[0] < max_h:
                                pad_bottom = max_h - f.shape[0]
                                f = cv2.copyMakeBorder(f, 0, pad_bottom, 0, 0, cv2.BORDER_CONSTANT, value=0)
                            padded_frames.append(f)
                        rows.append(np.hstack(padded_frames))
                    
                    # 确保所有行宽度一致
                    max_w = max(r.shape[1] for r in rows)
                    padded_rows = []
                    for r in rows:
                        if r.shape[1] < max_w:
                            pad_right = max_w - r.shape[1]
                            r = cv2.copyMakeBorder(r, 0, 0, 0, pad_right, cv2.BORDER_CONSTANT, value=0)
                        padded_rows.append(r)
                    
                    composite = np.vstack(padded_rows)
                else:
                    # 超过4个摄像头，只显示前4个
                    composite = np.vstack([
                        np.hstack(frames[0:2]),
                        np.hstack(frames[2:4])
                    ])
                
                cv2.imshow(window_name, composite)
            
            # 处理按键事件
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                stop_event.set()
                break
            
            # 稍微降低循环频率，减少CPU占用
            time.sleep(0.01)
            
    except Exception as e:
        logging.error(f"显示循环错误: {e}")
    finally:
        try:
            cv2.destroyWindow(window_name)
        except Exception:
            pass
        logging.info("显示线程结束")


def sync_reinit_all_cameras(cli: UnifiedReceiverClient, rs_cameras: List[str], v4l2_cameras: List[str], stop_event: threading.Event):
    """
    同步重配置所有摄像头（仅RGB）
    """
    # RealSense 配置序列 - 仅颜色流
    rs_configs = [
        # 阶段0: 低分辨率
        [{
            "camera_name": cam, 
            "streams": [
                {"type": "color", "width": 640, "height": 480, "fps": 30, "fmt": "bgr8"}
            ]
        } for cam in rs_cameras],
        # 阶段1: 中等分辨率
        [{
            "camera_name": cam, 
            "streams": [
                {"type": "color", "width": 1280, "height": 720, "fps": 30, "fmt": "bgr8"}
            ]
        } for cam in rs_cameras],
        # 阶段2: 高分辨率
        [{
            "camera_name": cam, 
            "streams": [
                {"type": "color", "width": 1920, "height": 1080, "fps": 15, "fmt": "bgr8"}
            ]
        } for cam in rs_cameras]
    ]
    
    # V4L2 配置序列
    v4l2_configs = [
        [{"camera_name": cam, "width": 640, "height": 480, "fps": 30, "fourcc": "MJPG"} for cam in v4l2_cameras],
        [{"camera_name": cam, "width": 1280, "height": 720, "fps": 30, "fourcc": "MJPG"} for cam in v4l2_cameras],
        [{"camera_name": cam, "width": 1920, "height": 1080, "fps": 15, "fourcc": "MJPG"} for cam in v4l2_cameras]
    ]
    
    # 确保配置序列长度一致
    max_configs = max(len(rs_configs), len(v4l2_configs))
    
    for stage in range(max_configs):
        if stop_event.is_set():
            break
            
        logging.info(f"=== 配置阶段 {stage+1}/{max_configs} ===")
        
        # 应用 RealSense 配置
        if stage < len(rs_configs) and rs_cameras:
            rs_config = rs_configs[stage]
            logging.info(f"[RS] 应用配置到 {len(rs_cameras)} 个摄像头")
            
            if stage == 0:
                # 第一次只重配置不等待
                reply = cli.reinit_rs(rs_config)
                logging.info(f"[RS] reinit返回: ok={reply.ok} msg={reply.message}")
            else:
                # 后续配置使用等待模式
                reply, matched = cli.reinit_rs_and_wait(rs_config, wait_timeout=6.0, poll_interval=0.2)
                logging.info(f"[RS] reinit_and_wait返回: ok={reply.ok} matched={matched} msg={reply.message}")
        
        # 应用 V4L2 配置
        if stage < len(v4l2_configs) and v4l2_cameras:
            v4l2_config = v4l2_configs[stage]
            logging.info(f"[V4L2] 应用配置到 {len(v4l2_cameras)} 个摄像头")
            
            if stage == 0:
                # 第一次只重配置不等待
                reply = cli.reinit_v4l2(v4l2_config)
                logging.info(f"[V4L2] reinit返回: ok={reply.ok} msg={reply.message}")
            else:
                # 后续配置使用等待模式
                reply, matched = cli.reinit_v4l2_and_wait(v4l2_config, wait_timeout=6.0, poll_interval=0.2)
                logging.info(f"[V4L2] reinit_and_wait返回: ok={reply.ok} matched={matched} msg={reply.message}")
        
        # 如果不是最后一个阶段，等待一段时间
        if stage < max_configs - 1:
            logging.info(f"等待10秒后切换到下一个配置阶段...")
            for j in range(10):
                if stop_event.is_set():
                    break
                time.sleep(1)


def main():
    cli = UnifiedReceiverClient(grpc_target=GRPC_TARGET)
    stop_event = threading.Event()
    display_thread = None

    # 信号处理器：Ctrl-C / SIGTERM 触发安全退出
    def _on_signal(sig, frame):
        logging.info("收到信号 %s，准备安全退出...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        # 建立 WebRTC 连接
        logging.info("正在连接服务端...")
        cli.start()
        logging.info("连接成功")

        # 列出所有相机
        rs_cameras, v4l2_cameras = list_cameras(cli)
        all_cameras = rs_cameras + v4l2_cameras
        
        if not all_cameras:
            logging.error("没有发现任何摄像头，退出程序")
            return

        logging.info("发现 %d 个摄像头: %s", len(all_cameras), all_cameras)
        logging.info("启动所有摄像头显示...")

        # 启动单个显示线程处理所有摄像头
        display_thread = threading.Thread(
            target=show_all_cameras_loop, 
            args=(cli, all_cameras, stop_event), 
            name="display-all-cameras",
            daemon=True
        )
        display_thread.start()
        logging.info("显示线程已启动")

        # 等待一下让窗口初始化完成
        time.sleep(2)

        # 同步重配置所有摄像头
        sync_reinit_all_cameras(cli, rs_cameras, v4l2_cameras, stop_event)

        logging.info("重配置演示完成，按'q'关闭窗口或Ctrl+C退出程序")

        # 主线程等待退出信号或显示线程结束
        while not stop_event.is_set() and display_thread.is_alive():
            time.sleep(0.5)

    except Exception as e:
        logging.error("程序运行出错: %s", e)
    finally:
        # 清理资源
        logging.info("正在清理资源...")
        stop_event.set()
        
        # 等待显示线程结束
        if display_thread and display_thread.is_alive():
            display_thread.join(timeout=2.0)
            if display_thread.is_alive():
                logging.warning("显示线程未正常结束")
        
        # 关闭所有OpenCV窗口
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
            
        # 停止客户端
        try:
            cli.stop()
        except Exception:
            pass
            
        logging.info("程序退出完成")


if __name__ == "__main__":
    main()