/**
 * @file 04_get_full_state.cpp
 * @brief 【示例 04】服务端全量状态与元数据查询
 * * [功能介绍]
 * 调用 get_state 接口获取服务端所有在线相机的详细配置。
 * 遍历并打印：相机 ID、支持的流类型（Color/Depth）、分辨率、FPS 以及关键的相机内参 (Intrinsics)。
 * 这对于需要根据相机内参进行 3D 重建或视觉测量的开发者至关重要。
 * * [使用说明]
 * 1. 修改 GRPC_TARGET 为服务端真实的 IP 地址和端口（例如 "192.168.1.100:50051"）。
 * 2. 程序运行后会执行单次查询并退出，控制台将输出格式化的配置树。
 * * [关键 API]
 * - CameraClient::get_state(): 获取 RecorderStateResponse 原始 Protobuf 对象。
 * - Protobuf 嵌套访问：演示了如何遍历 CameraConfig -> StreamSpec -> RsIntrinsics。
 */

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include "camera_client.h"

// 辅助函数：打印缩进
void print_indent(int level) {
  for (int i = 0; i < level; ++i) std::cout << "  ";
}

int main() {
  // ---------------------------------------------------------
  // 【用户配置区】请根据实际环境修改以下参数
  // ---------------------------------------------------------
  const std::string GRPC_TARGET = "localhost:50051";
  // ---------------------------------------------------------

  CameraClient client(GRPC_TARGET);

  try {
    std::cout << "[*] 正在连接服务 " << GRPC_TARGET << " 并获取元数据..." << std::endl;
    client.start();

    // 1. 调用接口获取状态 (传入空向量表示获取所有相机)
    auto state = client.get_state({}, 5.0);

    if (state.camera_configs_size() == 0) {
      std::cout << "[WARN] 服务端已连接，但未发现任何在线相机设备。" << std::endl;
    } else {
      std::cout << "[SUCCESS] 发现 " << state.camera_configs_size() << " 个相机配置：\n"
                << std::endl;
    }

    // 2. 遍历每个相机配置
    for (int i = 0; i < state.camera_configs_size(); ++i) {
      const auto& cam = state.camera_configs(i);
      std::cout << "========= 相机 [" << i << "]: " << cam.camera_name() << " =========" << std::endl;

      // 3. 遍历该相机下的所有视频流 (Color, Depth, etc.)
      for (int j = 0; j < cam.streams_size(); ++j) {
        const auto& stream = cam.streams(j);
        print_indent(1);
        std::cout << "> 流类型: " << std::left << std::setw(8) << stream.type()
                  << " | 分辨率: " << stream.width() << "x" << stream.height()
                  << " | 帧率: " << stream.fps() << " FPS" << std::endl;

        // 4. 解析并打印相机内参 (如果存在)
        if (stream.has_intrinsics()) {
          const auto& in = stream.intrinsics();
          print_indent(2);
          std::cout << "[内参详情] 模型: " << in.model() << std::endl;

          print_indent(3);
          std::cout << "焦距 (fx, fy): (" << in.fx() << ", " << in.fy() << ")" << std::endl;

          print_indent(3);
          std::cout << "主点 (ppx, ppy): (" << in.ppx() << ", " << in.ppy() << ")" << std::endl;

          if (in.coeffs_size() > 0) {
            print_indent(3);
            std::cout << "畸变系数: [";
            for (int k = 0; k < in.coeffs_size(); ++k) {
              std::cout << in.coeffs(k) << (k == in.coeffs_size() - 1 ? "" : ", ");
            }
            std::cout << "]" << std::endl;
          }
        }
        std::cout << std::endl;
      }
    }

  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 无法获取服务端信息: " << e.what() << std::endl;
    std::cerr << "[HINT] 请确认 IP [" << GRPC_TARGET << "] 是否可达且防火墙已关闭。" << std::endl;
  }

  client.stop();
  std::cout << "[*] 查询结束。" << std::endl;
  return 0;
}