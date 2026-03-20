/**
 * @file 01_minimal_color.cpp
 * @brief 【示例 01】最简彩色图获取
 * * [功能介绍]
 * 展示如何初始化 CameraClient 并从指定的相机通道实时获取彩色图像 (RGB)。
 * * [使用说明]
 * 1. 修改 GRPC_TARGET 为服务端真实的 IP 地址和端口（例如 "192.168.1.100:50051"）。
 * 2. 修改 CAMERA_NAME 为服务端实际挂载的相机 ID（如 "rs/cam_left_wrist"）。
 * 3. 运行后将弹出窗口显示画面，按 'q' 键退出。
 * * [注意]
 * 如果连接失败，请优先检查网络是否互通（ping）以及防火墙是否放行了 50051 端口。
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera_client.h"

int main() {
  // ---------------------------------------------------------
  // 【用户配置区】请根据实际环境修改以下参数
  // ---------------------------------------------------------
  const std::string GRPC_TARGET = "localhost:50051";    // 服务端 IP:端口
  const std::string CAMERA_NAME = "rs/cam_left_wrist";  // 相机 ID
  // ---------------------------------------------------------

  CameraClient client(GRPC_TARGET);
  try {
    client.start();
    std::cout << "[INFO] SDK 已启动，正在尝试连接: " << GRPC_TARGET << std::endl;
    std::cout << "[INFO] 正在获取相机流: " << CAMERA_NAME << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 启动失败: " << e.what() << std::endl;
    std::cerr << "[HINT] 请检查 IP 地址 [" << GRPC_TARGET << "] 是否正确且服务端已开启。" << std::endl;
    return -1;
  }

  while (true) {
    auto frame_data = client.getLatestFrame(CAMERA_NAME);
    if (frame_data) {
      cv::imshow("Color Viewer", frame_data->first);
    }

    if (cv::waitKey(1) == 'q') break;
  }

  client.stop();
  return 0;
}