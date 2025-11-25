"""
Audio SDK 加密库
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from . import robot_pb2
from . import robot_pb2_grpc
sys.modules['robot_pb2'] = robot_pb2
sys.modules['robot_pb2_grpc'] = robot_pb2_grpc
