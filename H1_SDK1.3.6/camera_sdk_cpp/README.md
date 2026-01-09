# Camera WebRTC + gRPC 跨平台 C++ SDK
## 1. 简介
本 SDK 是专为机器人视觉、智能监控及工业自动化场景设计的相机客户端开发包。它通过集成 **WebRTC** 高性能媒体传输协议与 gRPC 可靠信令通道，实现了： 

* **超低延迟**：基于 WebRTC (H.264) 的实时视频流传输，优化了局域网内的端到端延迟。
* **高精度深度**：支持 16-bit 原始深度图 (Depth) 的亚像素级数据重组与解压。
* **参数透明**：提供接口动态查询服务端相机内参（焦距、主点、畸变系数），方便视觉算法集成。
* **极简集成**：采用头文件扁平化设计，开发者仅需链接一个静态库即可驱动全流程。
## 2. 交付包目录结构
解压交付包后，您将看到如下标准布局：  
```
camera_sdk_cpp/
├── CMakeLists.txt          # 示例程序构建脚本 (一键编译所有 Example)
├── README.md               # 本说明文档
├── include/                # SDK 头文件目录 (已扁平化，支持直接引用)
│   ├── camera_client.h     # 核心接口类入口
│   ├── robot.pb.h          # 基础数据协议定义
│   └── robot.grpc.pb.h     # gRPC 服务接口定义
├── lib/                    # 二进制库目录
│   └── libcamera_client.a  # 预编译静态库 (针对 Linux x86_64 环境编译)
└── examples/               # 示例源代码目录
    ├── 01_minimal_color.cpp   # 获取彩色图最简代码
    ├── 02_minimal_depth.cpp   # 深度图读取与伪彩色可视化转换
    ├── 03_multi_cam_viewer.cpp # 动态发现并展示所有在线相机窗口
    ├── 04_full_demo.cpp       # 综合功能演示 (含实时 FPS 与 HUD 信息遮罩)
    └── 05_get_full_state.cpp  # 查询并解析服务端全量元数据 (如相机内参)
```

## 3. 环境要求

### 3.1 操作系统与硬件
- OS: Linux (推荐 Ubuntu 22.04 或 24.04 LTS)
- 架构: x86_64
- 编译器: 支持 C++17 或更高标准的 GCC (>= 9.0) 或 Clang

### 3.2 依赖库清单
集成前，请确保系统中已安装以下第三方库：

| 依赖项 | 建议版本 | 安装命令 (以 Ubuntu 为例) |
|--------|---------|------------------------|
| OpenCV | >= 4.2 | `sudo apt install libopencv-dev` |
| gRPC | >= 1.30 | `sudo apt install libgrpc++-dev protobuf-compiler-grpc` |
| Protobuf | >= 3.12 | `sudo apt install protobuf-compiler libprotobuf-dev` |
| FFmpeg | >= 4.0 | `sudo apt install libavcodec-dev libavutil-dev libswscale-dev` |
| ZLIB | 1.2.x | `sudo apt install zlib1g-dev` |
| LibDataChannel | 最新 | 建议参考项目主页自行编译安装 |

## 4. 快速构建示例

我们为所有示例程序预置了 CMake 构建系统，您可以快速验证环境：

```bash
# 1. 进入 SDK 交付包根目录
cd camera_sdk_cpp

# 2. 创建并进入构建目录
mkdir build && cd build

# 3. 执行配置与编译
cmake ..
make -j$(nproc)

# 4. 运行示例 (运行前请修改 IP 地址)
./01_minimal_color
```

## 5. 快速上手指南

### 5.1 修改关键参数
在运行示例代码前，请根据您的实际环境修改源文件顶部的配置：
- **GRPC_TARGET**: 服务端的 IP 地址和端口（例如 "192.168.1.100:50051"）
- **CAMERA_NAME**: 服务端挂载的相机 ID（例如 "rs/cam_left_wrist"）

### 5.2 核心 API 参考
- **CameraClient(target)**: 构造函数。初始化服务地址。
- **void start()**: 阻塞式接口。启动后台线程，完成 gRPC 握手与 WebRTC 链路建立。
- **void stop()**: 停止所有取流任务并断开网络连接。
- **getLatestFrame(name)**: 获取最新彩色图。返回 std::optional<std::pair<cv::Mat, double>>。
- **getLatestDepth(name)**: 获取最新深度图。数据格式为 CV_16UC1，单位为毫米 (mm)。
- **get_state()**: 获取服务端当前所有相机的详细状态、流配置及精密内参。

## 6. 示例程序说明 (Examples)

- **01_minimal_color**: 极其简单的入门示例，展示了从建立连接到获取并显示彩色画面的标准流程。
- **02_minimal_depth**: 重点展示如何正确处理 16-bit 原始深度数据，并将其转换为伪彩色（Jet Map）进行直观展示。
- **03_multi_cam_viewer**: 演示动态发现机制。它会自动获取服务端在线相机列表，并为每个相机动态生成一个预览窗口。
- **04_get_full_state**: 高级调试工具。它将以树状结构完整打印出所有相机的分辨率、FPS 以及精密内参（焦距、主点、畸变模型等）。
- **05_full_demo**: 综合应用示例。包含了高频帧率统计逻辑、服务端硬件时间戳对齐展示以及美观的 HUD 文字覆盖。

## 7. 最佳实践与注意事项

- **并发安全**: SDK 内部已实现完善的帧缓冲互斥锁。您可以在主线程中渲染 UI，在工作线程中调用 getLatest 接口，无需额外处理多线程同步。
- **带宽占用**: H.264 编码虽能降低流量，但同时传输多路高清流和深度流仍需较高带宽。建议在千兆有线网络或 Wi-Fi 6 环境下运行。
- **深度图显示**: 原始深度数据通常呈现为极暗的画面。必须通过 `convertTo(viz, CV_8U, 0.03)` 进行缩放后，再配合彩色映射表方可正常观测。
- **时间戳对齐**: SDK 返回的时间戳是服务端传感器采集瞬间的硬件时间戳。在做多传感器融合（如 V-SLAM）时，请务必以此时间戳为准。

## 8. 常见问题 (FAQ)

**Q: 编译时报错提示 fatal error: robot.pb.h: No such file or directory?**

A: 请检查 CMake 包含路径。本 SDK 采用扁平化设计，头文件均在 include/ 下，直接使用 `#include "robot.pb.h"` 即可。

**Q: 程序启动时报错 Connection refused?**

A: 检查服务端是否已启动；检查客户端与服务端 IP 是否在同一网段；检查防火墙是否放行了 50051 端口。

**Q: 深度图显示的数值不准？**

A: 默认单位为毫米 (mm)。请确认您的服务端深度缩放单位（Scale）设置是否与 SDK 默认期望一致。

---

© 2025 ZERITH Team. 