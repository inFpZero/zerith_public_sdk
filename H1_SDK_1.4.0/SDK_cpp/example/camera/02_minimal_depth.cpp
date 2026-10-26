#include "../common/example_cli.hpp"
/**
 * @file 02_minimal_depth.cpp
 * @brief 【示例 02】深度图原始数据读取与可视化
 * * [功能介绍]
 * 获取 16位 (uint16_t) 原始深度数据并展示像素读取与伪彩色转换。
 * * [使用说明]
 * 1. 修改 ROBOT_IP 为机器人真实 IP（例如 "192.168.1.100"）。
 * 2. 修改 CAMERA_NAME 为服务端实际挂载的相机 ID（如 "rs/cam_left_wrist"）。
 *  *[注意]
 * 如果连接失败，请优先检查网络是否互通（ping）以及防火墙是否放行了 50051 端口。
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "H1_Robot.hpp"

int main(int argc, char** argv) {
  // ---------------------------------------------------------
  // 【用户配置区】请根据实际环境修改以下参数
  // ---------------------------------------------------------
  const std::string ROBOT_IP = h1_sdk_examples::parseRobotIp(argc, argv);
  const std::string CAMERA_NAME = "rs/cam_high";  // 深度相机 ID
  // ---------------------------------------------------------

  auto& client = H1Robot::getInstance(ROBOT_IP);
  client.startCameraWebRTC();

  while (true) {
    auto depth_data = client.getLatestCameraDepth(CAMERA_NAME);
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

  client.stopCameraWebRTC();
  return 0;
}
