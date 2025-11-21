# zerith_public_sdk
## Zerith SDK public repository for external development
```
H1_SDK
├── audio_sdk_py      #语音sdk
│   ├── asr_test.py   #示例
│   └── lib
│       ├── __init__.py
│       ├── robot_pb2_grpc.so
│       └── robot_pb2.so
├── camera_sdk_client_cpp    #相机sdk c++版本
│   ├── include
│   │   ├── camera_client.h
│   │   ├── robot.grpc.pb.h
│   │   └── robot.pb.h
│   └── lib
├── camera_sdk_client_py     #相机sdk python版本
│   ├── CMakeLists.txt
│   ├── demo.cpp               #示例
│   ├── include
│   │   ├── camera_client.h
│   │   ├── robot.grpc.pb.h
│   │   └── robot.pb.h
│   └── lib
│       └── libcamera_client.a
├── robot_SDK             #运动控制SDK
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
│       └── H1
│           └── librobot_core.a
└── VERSION      #软件版本
```
