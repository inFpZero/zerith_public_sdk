# H1 SDK Python

Python Binding 直接使用同一发布包内 `SDK_cpp` 的公开头文件和预编译核心库，模块名
与旧版兼容：`lib_h1_sdk_python`。不依赖包外 SDK 工程。

```bash
cmake -S . -B build -DPython3_EXECUTABLE=/usr/bin/python3.10
cmake --build build --parallel 2
../run_python.sh -c "import lib_h1_sdk_python as h1; print(h1.__doc__)"
```

使用 `../run_python.sh` 运行示例或导入模块时，无需手动设置 `PYTHONPATH`；脚本会自动
加入发布包内的 Python Binding、NumPy 和 cv2 路径。

脚本优先选择用户的 `zerith` 中的 CPython 3.10；没有兼容的 `zerith` 时，依次检查
`PATH` 中的 Python 和用户其他 Conda 环境（不限制环境名称）。显式指定 `PYTHON3`
可覆盖自动选择，指定无效时会报错。脚本同时支持常见 Conda 安装目录和
`~/.conda/environments.txt` 登记的自定义环境，不需要提前 `conda activate`。
未安装兼容解释器时仍需先安装 CPython 3.10。

若机器人本体上的 DDS 共享内存权限要求使用 sudo，可在当前 `SDK_python` 目录执行：

```bash
sudo ../run_python.sh -c 'import sys; print(sys.executable)'
```

sudo 下会查找原登录用户的环境，不要求 root 安装 Conda。将 `-c '...'` 替换为
示例路径及参数即可运行示例；运动示例必须先确认现场安全。
此方式不修复 SDK 退出阶段的异常。完整查找顺序、连接探测和手动覆盖方法见
[发布包 README](../README.md#python-解释器选择与-sudo)。

当前发布扩展的 ABI 为 CPython 3.10：
`lib/lib_h1_sdk_python.cpython-310-x86_64-linux-gnu.so`。

`H1Robot` 是进程单例，Python 端通过以下方式取得：

```python
import lib_h1_sdk_python as h1
robot = h1.H1Robot.get_instance("192.168.1.2")
```

相机传输接口已拆分为两组，不再提供自动选择的
`startCamera()/stopCamera()`：

```python
# 同机：DDS SHM
robot.startCameraDds()
robot.stopCameraDds()

# 同机或异机：WebRTC
robot.startCameraWebRTC()
robot.stopCameraWebRTC()
```

完整示例位于 `example/motion_control/` 和 `example/camera/`。例如：

```bash
../run_python.sh example/camera/04_get_full_state.py \
  --ip 192.168.1.2
```

日志模块同时支持普通日志和独立 EVENT 日志。模块名必须在导入前通过
`ROBOT_LOG_MODULE` 设置；未设置时默认为 `H1_SDK`：

```bash
export ROBOT_LOG_MODULE=H1_SDK
```

`run_python.sh` 默认将日志写入 `/tmp/h1-sdk-logs-<运行用户UID>`，例如 root 为
`/tmp/h1-sdk-logs-0`。不同运行用户使用不同目录，避免日志所有者导致权限冲突。
如需指定其他可写目录，可在运行脚本前设置（自定义目录不会被脚本覆盖）：

```bash
export ROBOT_LOG_ROOT="/tmp/my-h1-sdk-logs-$(id -u)"
```

```python
from logger import logger

logger.info("normal message")
logger.event("Camera %s is ready.", "rs/cam_high")
```

`logger.event()` 支持与 Python `logging` 一致的 `%` 参数写法。
EVENT 会写入独立事件日志，并以原生 `EVENT` 等级同步写入普通日志。
日志分别写入 `$ROBOT_LOG_ROOT/H1_SDK/normal/` 和
`$ROBOT_LOG_ROOT/H1_SDK/event/`，文件名由模块名和日期自动生成。

主要目录：

```text
SDK_python/
├── bind/include/       # binding 声明
├── bind/src/           # pybind11 实现
├── example/            # Python 示例
├── lib/                # 预编译 Python 扩展模块和 .pyi
├── python_deps/        # 包内 NumPy 和 cv2 运行模块
├── third_party/        # Binding 编译所需的包内依赖
```

用户可重新编译 Python Binding；闭源的 `../SDK_cpp/lib/librobot_core.so` 不在用户侧
重新编译。仅支持 Ubuntu 20.04 x86_64 与 CPython 3.10。
