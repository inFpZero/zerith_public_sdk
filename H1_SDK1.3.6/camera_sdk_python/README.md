# Camera WebRTC + gRPC 跨平台 Python SDK
## 1. 简介
本 SDK 是专为机器人视觉、智能监控及工业自动化场景设计的相机客户端开发包。它通过集成 **WebRTC** 高性能媒体传输协议与 gRPC 可靠信令通道，实现了：

* **超低延迟**：基于 WebRTC (H.264) 的实时视频流传输，优化了局域网内的端到端延迟。
* **高精度深度**：支持 16-bit 原始深度图 (Depth) 的亚像素级数据重组与解压。
* **参数透明**：提供接口动态查询服务端相机内参（焦距、主点、畸变系数），方便视觉算法集成。
* **极简集成**：采用 Python 包设计，开发者仅需 import 相关模块即可驱动全流程。

## 2. 交付包目录结构
解压交付包后，您将看到如下标准布局：

```
camera_sdk_python/
├── camera_client.cpython-38-x86_64-linux-gnu.so  # 预编译 C 扩展 (Python 3.8)
├── camera_client.cpython-39-x86_64-linux-gnu.so  # 预编译 C 扩展 (Python 3.9)
├── camera_client.cpython-310-x86_64-linux-gnu.so # 预编译 C 扩展 (Python 3.10)
├── requirements.txt        # 项目依赖清单
├── proto/                  # gRPC 协议定义目录
│   ├── robot_pb2.py       # 基础数据协议定义（自动生成）
│   └── robot_pb2_grpc.py  # gRPC 服务接口定义（自动生成）
├── example/                # 示例脚本目录
│   ├── 01_minimal_color.py      # 获取彩色图最简代码
│   ├── 02_minimal_depth.py      # 深度图读取与伪彩色可视化转换
│   ├── 03_multi_cam_viewer.py   # 动态发现并展示所有在线相机窗口
│   ├── 04_get_full_state.py     # 高级调试工具
│   └── 05_full_demo.py          # 综合功能演示 (含实时 FPS 与 HUD 信息遮罩)
└── README.md               # 本说明文档
```

## 3. 环境要求

### 3.1 操作系统与硬件
- OS: Linux (推荐 Ubuntu 22.04 或 24.04 LTS)
- 架构: x86_64
- Python: 3.x 或更高版本

### 3.2 依赖库清单
集成前，请确保系统中已安装以下第三方库：

| 依赖项 | 建议版本 | 安装命令 (以 Ubuntu 为例) |
|--------|---------|------------------------|
| Python | >= 3.x |  |
| aiortc | >= 1.13.0 | `pip install aiortc` |
| av (PyAV) | >= 14.4.0 | `pip install av` |
| gRPC | >= 1.74.0 | `pip install grpcio` |
| Protobuf | >= 6.33.2 | `pip install protobuf` |
| NumPy | >= 2.4.0 | `pip install numpy` |
| OpenCV | >= 4.10.0 | `pip install opencv-python` |

## 4. 快速安装

### 4.1 基于依赖文件安装（推荐）
```bash
# 进入 SDK 目录
cd camera_sdk_python_v1.0

# 安装项目依赖
pip install -r requirements.txt

# 安装 SDK 包
```

### 4.2 手动安装依赖
```bash
# 安装指定版本的依赖
pip install aiortc av grpcio protobuf numpy opencv-python
```

## 5. 快速上手指南

### 5.1 修改关键参数
在运行示例代码前，请根据您的实际环境修改源文件顶部的配置：
- **GRPC_TARGET**: 服务端的 IP 地址和端口（例如 "192.168.1.100:50051"）
- **CAMERA_NAME**: 服务端挂载的相机 ID（例如 "rs/cam_left_wrist"）

### 5.2 核心 API 参考
- `CameraClient(target)`: 构造函数。初始化服务地址。
- `start()`: 启动后台线程，完成 gRPC 握手与 WebRTC 链路建立。
- `stop()`: 停止所有取流任务并断开网络连接。
- `get_latest_frame(name)`: 获取最新彩色图。返回 `(frame: np.ndarray, timestamp: float)`。
- `get_latest_depth(name)`: 获取最新深度图。返回 `(depth: np.ndarray, timestamp: float)`，深度单位为毫米 (mm)。
- `get_state()`: 获取服务端当前所有相机的详细状态、流配置及精密内参。


## 6. 示例程序说明 (Examples)

- **01_minimal_color.py**: 极其简单的入门示例，展示了从建立连接到获取并显示彩色画面的标准流程。
- **02_minimal_depth.py**: 重点展示如何正确处理 16-bit 原始深度数据，并将其转换为伪彩色（Jet Map）进行直观展示。
- **03_multi_cam_viewer.py**: 演示动态发现机制。它会自动获取服务端在线相机列表，并为每个相机动态生成一个预览窗口。
- **04_full_demo.py**: 综合应用示例。包含了高频帧率统计逻辑、服务端硬件时间戳对齐展示以及美观的 HUD 文字覆盖。
- **05_get_full_state.py**: 高级调试工具。它将以树状结构完整打印出所有相机的分辨率、FPS 以及精密内参（焦距、主点、畸变模型等）。

## 7. 最佳实践与注意事项

- **并发安全**: SDK 内部已实现完善的帧缓冲线程锁。您可以在主线程中渲染 UI，在工作线程中调用 `get_latest_*` 接口，无需额外处理多线程同步。
- **带宽占用**: H.264 编码虽能降低流量，但同时传输多路高清流和深度流仍需较高带宽。建议在千兆有线网络或 Wi-Fi 6 环境下运行。
- **深度图显示**: 原始深度数据通常呈现为极暗的画面。必须通过 `cv2.convertScaleAbs(depth, alpha=0.03)` 或 matplotlib 的彩色映射进行缩放后方可正常观测。
- **时间戳对齐**: SDK 返回的时间戳是服务端传感器采集瞬间的硬件时间戳。在做多传感器融合（如 V-SLAM）时，请务必以此时间戳为准。

## 8. 常见问题 (FAQ)

**Q: 导入时报错 ModuleNotFoundError: No module named 'grpc'?**

A: 请确保已安装 gRPC Python 包。执行 `pip install grpcio grpcio-tools` 后重试。

**Q: 程序启动时报错 Connection refused?**

A: 检查服务端是否已启动；检查客户端与服务端 IP 是否在同一网段；检查防火墙是否放行了 50051 端口。

**Q: Protobuf 版本不兼容？**

A: 确保已安装兼容的 Protobuf。执行 `pip install protobuf --force-reinstall` 重新安装。

**Q: 深度图显示的数值不准？**

A: 默认单位为毫米 (mm)。请确认您的服务端深度缩放单位（Scale）设置是否与 SDK 默认期望一致。

---

© 2025 ZERITH Team.
