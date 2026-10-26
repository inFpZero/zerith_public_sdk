# Python examples

这些示例覆盖当前 H1 SDK 的 Python 使用方式：

- `motion_control/`：运动控制和数据采集。
- `camera/`：彩色图、深度图、多相机、状态查询及综合演示。

`camera/01` 至 `camera/05` 显式使用 WebRTC；对应的 `*_dds.py` 文件使用
同机 DDS SHM。每组示例除启动和停止所用的通信协议外，功能、参数和结果处理
保持一致，并分别与 C++ 同名示例对应。

扩展模块按 CPython 3.10 构建。启动脚本优先查找兼容的 `zerith`，没有时自动查找
`PATH` 和其他 Conda 环境，不需要手动激活环境或设置 `PYTHONPATH`。
请在发布包根目录执行：

```bash
./run_python.sh SDK_python/example/camera/04_get_full_state.py --ip 192.168.1.2
./run_python.sh SDK_python/example/camera/01_minimal_color_dds.py --ip 127.0.0.1
./run_python.sh SDK_python/example/camera/02_minimal_depth_dds.py --ip 127.0.0.1
./run_python.sh SDK_python/example/camera/05_full_demo_dds.py --ip 127.0.0.1
```

若在机器人本体上遇到 DDS 共享内存权限错误，可将命令改为
`sudo ./run_python.sh 示例路径 参数`；脚本会按原登录用户查找 Python。
默认日志按运行用户分别写入 `/tmp/h1-sdk-logs-<UID>`。sudo 不自动解决图形桌面
访问权限，也不修复 SDK 退出异常。详细规则见
[发布包 README](../../README.md#python-解释器选择与-sudo)。

与 `SDK_cpp/example/motion_control` C++ 测试函数及参数对齐的示例：

- `motion_control/h1_high_level_complex_new.py`：对应 C++ 高层控制示例中的全部测试函数。
- `motion_control/h1_low_level_simple.py`：对应 C++ 低层简单控制示例。
- `motion_control/h1_low_level_KPKD.py`：对应 C++ KP/KD 参数和分组测试。
- `motion_control/h1_low_level_motor_limit.py`：对应 C++ 全电机机械极限测试。
- `motion_control/h1_data_collection.py`：对应 C++ 数据采集调用顺序和默认配置。
- `motion_control/state_monitor.py`：只连接、不初始化、不发送运动指令的状态监控。

以下运动控制示例可能驱动电机，必须先确认现场安全；状态监控不发送运动指令。
示例用法：

```bash
./run_python.sh SDK_python/example/motion_control/h1_high_level_complex_new.py
./run_python.sh SDK_python/example/motion_control/h1_low_level_simple.py
./run_python.sh SDK_python/example/motion_control/h1_low_level_KPKD.py
./run_python.sh SDK_python/example/motion_control/h1_low_level_motor_limit.py
./run_python.sh SDK_python/example/motion_control/state_monitor.py
```

按当前范围不提供 `h1_low_level_HandControl.py`。
