# zerith_public_sdk
## Zerith SDK public repository for external development
```
H1_SDK
├── audio_sdk_py      #语音sdk python版本
│   ├── asr_test.py   #示例
│   └── lib
│       ├── __init__.py
│       ├── robot_pb2_grpc.so
│       └── robot_pb2.so
├── camera_sdk_client_cpp    #相机sdk c++版本
│   ├── CMakeLists.txt
│   ├── demo.cpp       #示例
│   ├── include
│   │   ├── camera_client.h
│   │   ├── robot.grpc.pb.h
│   │   └── robot.pb.h
│   └── lib
│       └── libcamera_client.a
├── camera_sdk_client_py     #相机sdk python版本
│   ├── demo.py            #示例
│   └── lib
│       ├── camera_client.so
│       ├── __init__.py
│       ├── robot_pb2_grpc.so
│       └── robot_pb2.so
├── h1_sdk_v1.x.x_python3.xx  #运动控制SDK python版本,默认使用python3.10
│   ├── example
│   │   ├── h1_high_level.py
│   │   ├── h1_low_level.py
│   │   └── state_monitor.py
│   └── lib
│       └── lib_h1_sdk_python.so
├── robot_SDK             #运动控制SDK cpp版本
│   ├── CMakeLists.txt
│   ├── example           #示例
│   │   ├── h1_high_level_complex_new.cpp
│   │   ├── h1_low_level_KPKD.cpp
│   │   ├── h1_low_level_motor_limit.cpp
│   │   └── h1_low_level_simple.cpp
│   ├── include
│   │   └── H1
│   │       ├── config.hpp
│   │       ├── GrpcServiceClient.hpp
│   │       ├── H1_Robot.hpp
│   │       ├── ip_service.grpc.pb.h
│   │       ├── ip_service.pb.h
│   │       ├── MotorStateMonitor.hpp
│   │       ├── Robot.hpp
│   │       └── ZCM_Data
│   │           ├── ArmEndPose.hpp
│   │           ├── ChassisControl.hpp
│   │           ├── ChassisParam.hpp
│   │           ├── ChassisState.hpp
│   │           ├── CommunicationBoardState.hpp
│   │           ├── ControlMode.hpp
│   │           ├── FixedRod.hpp
│   │           ├── GripperControl.hpp
│   │           ├── GripperControlMode.hpp
│   │           ├── GripperState.hpp
│   │           ├── HeadControl.hpp
│   │           ├── HeadState.hpp
│   │           ├── ImuData.hpp
│   │           ├── PowerState.hpp
│   │           ├── RobotInfo.hpp
│   │           ├── SystemInit.hpp
│   │           ├── SystemSpace.hpp
│   │           ├── UpperJointControl.hpp
│   │           ├── UpperJointState.hpp
│   │           ├── WaistControl.hpp
│   │           ├── WaistState.hpp
│   │           ├── XboxMap.hpp
│   │           └── ZCM_Data.hpp
│   └── lib
│   │   └── H1
│   │       └── librobot_core.a
│   └── VERSION
└── VERSION      #软件版本
```
