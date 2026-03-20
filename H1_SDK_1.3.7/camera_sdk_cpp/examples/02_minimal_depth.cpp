/**
 * @file 02_minimal_depth.cpp
 * @brief 【示例 02】深度图原始数据读取与可视化
 * * [功能介绍]
 * 获取 16位 (uint16_t) 原始深度数据并展示像素读取与伪彩色转换。
 * * [使用说明]
 * 1. 修改 GRPC_TARGET 为服务端真实的 IP 地址和端口（例如 "192.168.1.100:50051"）。
 * 2. 修改 CAMERA_NAME 为服务端实际挂载的相机 ID（如 "rs/cam_left_wrist"）。
 *  *[注意]
 * 如果连接失败，请优先检查网络是否互通（ping）以及防火墙是否放行了 50051 端口。
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "camera_client.h"

int main() {
  // ---------------------------------------------------------
  // 【用户配置区】请根据实际环境修改以下参数
  // ---------------------------------------------------------
  const std::string GRPC_TARGET = "localhost:50051";    // 示例：修改为真实 IP
  const std::string CAMERA_NAME = "rs/cam_left_wrist";  // 深度相机 ID
  // ---------------------------------------------------------

  CameraClient client(GRPC_TARGET);
  client.start();

  while (true) {
    auto depth_data = client.getLatestDepth(CAMERA_NAME);
    if (depth_data) {
      cv::Mat depth_raw = depth_data->first;

      // 可视化处理
      cv::Mat depth_viz;
      depth_raw.convertTo(depth_viz, CV_8U, 0.03);
      cv::applyColorMap(depth_viz, depth_viz, cv::COLORMAP_JET);
      cv::imshow("Color Viewer", depth_viz);
    }

    if (cv::waitKey(1) == 'q') break;
  }

  client.stop();
  return 0;
}