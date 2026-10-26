## 目录结构
H1_SDK
│
├── SDKClient                         # C++ SDK 源码及用户接口
│   ├── CMakeLists.txt                # C++ SDK 构建入口
│   ├── README.md                     # C++ SDK 使用说明
│   │
│   ├── robot                         # 用户可使用的公共接口
│   │   ├── H1_Robot.hpp             # H1Robot 单例及全部 SDK 接口
│   │   ├── ClientTypes.hpp           # 状态、数据采集等客户端数据结构
│   │   ├── config.hpp                # 电机、控制模式、位姿等结构和枚举
│   │   ├── MotorStateMonitor.hpp     # 电机状态监控辅助类
│   │   ├── log
│   │   │   ├── log.hpp               # C++ 日志接口
│   │   │   ├── log_stream.hpp        # 流式日志接口
│   │   │   └── log_c.h               # C 日志接口
│   │   └── proto
│   │       └── camera_service.proto  # 相机状态与 WebRTC 信令协议
│   │
│   └── examples                      # C++ 示例
│       ├── motion_control
│       │   ├── h1_high_level_complex_new.cpp
│       │   ├── h1_low_level_simple.cpp
│       │   ├── h1_low_level_KPKD.cpp
│       │   ├── h1_low_level_motor_limit.cpp
│       │   └── h1_data_collection.cpp
│       │
│       └── camera
│           ├── 01_minimal_color.cpp
│           ├── 02_minimal_depth.cpp
│           ├── 03_multi_cam_viewer.cpp
│           ├── 04_get_full_state.cpp
│           └── 05_full_demo.cpp
│
└── H1_SDK_python                    # Python 3.10 SDK
    ├── CMakeLists.txt                # Python Bind 构建入口
    ├── README.md
    ├── requirements.txt
    │
    ├── lib                          # Python 用户导入目录
    │   ├── lib_h1_sdk_python.cpython-310-x86_64-linux-gnu.so
    │   ├── lib_h1_sdk_python.pyi
    │   ├── logger.py
    │   └── logger.pyi
    │
    ├── example                      # Python 示例
    │   ├── common.py                # IP 参数与 H1Robot 单例创建
    │   │
    │   ├── motion_control
    │   │   ├── h1_high_level.py
    │   │   ├── h1_low_level.py
    │   │   ├── h1_low_level_KPKD.py
    │   │   ├── h1_low_level_motor_limit.py
    │   │   ├── state_monitor.py
    │   │   └── h1_data_collection.py
    │   │
    │   └── camera
    │       ├── 01_minimal_color.py
    │       ├── 02_minimal_depth.py
    │       ├── 03_multi_cam_viewer.py
    │       ├── 04_get_full_state.py
    │       └── 05_full_demo.py
    │
    └── robot_sdk                    # Python Bind 使用的预编译 C++ SDK
        ├── CMakeLists.txt
        ├── README.md
        ├── VERSION
        │
        ├── include                  # C++ 公共头文件副本
        │   ├── H1_Robot.hpp
        │   ├── ClientTypes.hpp
        │   ├── config.hpp
        │   ├── MotorStateMonitor.hpp
        │   ├── camera_service.pb.h
        │   ├── log.hpp
        │   ├── log_stream.hpp
        │   └── log_c.h
        │
        ├── lib                      # SDK 和 DDS/WebRTC 运行库
        │   ├── librobot_core.a
        │   ├── libh1_dds_interfaces.a
        │   ├── libnexus_bus.so
        │   ├── libfastdds.so
        │   ├── libfastcdr.so
        │   └── libdatachannel.so
        │
        ├── 3rd_party
        │   ├── grpc_static_lib
        │   ├── spdlog_static_lib
        │   └── libdatachannel
        │
        └── examples                 # C++ 发布示例副本
            ├── motion_control
            └── camera


# ZERITH H1 SDK — Ubuntu 20.04 x86_64

本发布包可在解压目录内重新编译全部公开 C++ 示例和 CPython 3.10 Python
Binding。SDK 核心 `SDK_cpp/lib/librobot_core.so` 为闭源预编译库，不在用户侧重新
编译。

相机用例与 SDK 核心统一使用包内动态 OpenCV 4.6，不再静态链接第二份
OpenCV/IPP。Python 示例的 `--ip` 参数已统一映射为 `args.robot_ip`。

## 系统要求

- Ubuntu 20.04 x86_64
- GCC/G++（支持 C++17）
- CMake 3.16 或更高版本
- CPython 3.10 及其开发头文件

H1 SDK、Fast DDS、OpenCV、Protobuf、spdlog、pybind11、FFmpeg、OpenSSL、NumPy
和 cv2 所需文件均在包内。不需要其他 SDK/DDS 源码目录，不需要安装对应开发库，
不需要联网执行 pip，也不需要配置 `LD_LIBRARY_PATH`。

## 重新编译

分别编译 C++ 示例和 Python Binding：

```bash
cmake -S SDK_cpp -B SDK_cpp/build
cmake --build SDK_cpp/build --parallel 2

cmake -S SDK_python -B SDK_python/build \
  -DPython3_EXECUTABLE=/usr/bin/python3.10
cmake --build SDK_python/build --parallel 2
```

## 运行相机与 Python 示例

C++ 相机示例在已登录桌面的终端运行：

```bash
export ROBOT_LOG_ROOT=/tmp/robot-logs
./SDK_cpp/build/bin/camera_03_multi_cam_viewer --ip 10.10.202.189
```

图形窗口需要有效的 `DISPLAY` 和桌面访问权限。仅通过 SSH 登录不代表能够
打开机器人桌面；若提示 `Can't initialize GTK backend`，应检查显示会话。
本包的 GTK/GLib 运行库面向 Ubuntu 20.04；

使用包内启动脚本运行 Python 示例，无需手动设置 `PYTHONPATH`；脚本会自动加载包内的
Python Binding、NumPy 和 cv2：

```bash
./run_python.sh SDK_python/example/motion_control/state_monitor.py \
  --ip 192.168.1.2
./run_python.sh SDK_python/example/motion_control/h1_data_collection.py \
  --ip 192.168.1.2
```

### Python 解释器选择与 sudo

无需事先激活 Conda，也不要求环境必须叫 `zerith`。脚本按以下顺序选择解释器，
每个候选都必须是 **CPython 3.10**：

1. 显式指定的 `PYTHON3`（手动覆盖）；指定无效时直接报错，不自动换环境。
2. 用户的 `zerith` 环境；不存在或版本不兼容时继续查找。
3. 当前 `PATH` 中的 `python3.10`、`python3`、`python`。
4. 用户其他名称的 Conda 环境，以及 Conda 安装目录中的基础解释器。

脚本检查可见的 `CONDA_PREFIX`、用户的 `~/.conda/environments.txt`，以及用户家目录下
`miniconda3`、`anaconda3`、`miniforge3`、`mambaforge` 和 `.conda` 的环境目录。
使用 `sudo` 时按 `SUDO_USER` 查找原登录用户的家目录，不依赖 root 的 Conda 环境，
也不保证沿用 sudo 前激活的环境。自定义安装位置未被登记时可显式设置 `PYTHON3`。
未找到兼容解释器时会报错；脚本不会自动安装 Python 或依赖。

在机器人本体上，若 SDK 普通用户与 root 服务之间出现 DDS 共享内存
`Permission denied`，可使用 sudo。先只检查解释器和连接，不执行运动用例：

```bash
sudo ./run_python.sh -c 'import sys; print(sys.executable)'
sudo ./run_python.sh -c 'import lib_h1_sdk_python as h1; robot = h1.H1Robot.get_instance("127.0.0.1"); print(robot.robot_connect())'
```

运行示例时，将 `-c '...'` 替换为示例文件路径及参数即可；无需额外传入 Conda 路径。
运动控制示例可能申请控制权、初始化并驱动电机，运行前必须确认现场安全。
sudo 只是共享内存权限问题的临时处理方式；本次启动脚本调整不修改 SDK 二进制，
也不修复已观察到的退出阶段 `std::system_error: Invalid argument`。

如需手动指定解释器（路径按实际安装位置修改）：

```bash
PYTHON3=/path/to/python3.10 ./run_python.sh -c 'import sys; print(sys.executable)'
sudo env PYTHON3=/path/to/python3.10 ./run_python.sh -c 'import sys; print(sys.executable)'
```

Python 启动脚本默认日志目录为 `/tmp/h1-sdk-logs-<运行用户UID>`，例如 root 使用
`/tmp/h1-sdk-logs-0`，普通用户 UID 为 1000 时使用 `/tmp/h1-sdk-logs-1000`。
显式设置的 `ROBOT_LOG_ROOT` 会保留，目录应对运行用户可写；不建议 root 和普通用户
共用同一日志目录。直接启动 C++ 示例时仍需按上文自行配置日志目录。

数据采集示例执行完整非交互流程：创建会话、开始、进入下一步、暂停、恢复、再次进入
下一步并结束会话。命令被 DataEngine 接受说明通信接口已走通；最终 episode 是否成功
还取决于机器人相机、电机和存储状态。
