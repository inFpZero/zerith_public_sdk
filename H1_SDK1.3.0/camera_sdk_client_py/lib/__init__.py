"""
Unified WebRTC Client SDK
加密版本 - 请勿修改
"""

import sys
import os

# 添加当前目录到 Python 路径
sys.path.insert(0, os.path.dirname(__file__))

# 导出主要类和函数
try:
    from .camera_client import UnifiedReceiverClient
    from . import robot_pb2
    from . import robot_pb2_grpc
    
    # 将加密模块注册到 sys.modules，以便其他模块可以导入
    sys.modules['robot_pb2'] = robot_pb2
    sys.modules['robot_pb2_grpc'] = robot_pb2_grpc
    
except ImportError as e:
    print(f"导入加密模块失败: {e}")
    print("请确保已正确安装所有依赖项")

__version__ = "1.0.0"
