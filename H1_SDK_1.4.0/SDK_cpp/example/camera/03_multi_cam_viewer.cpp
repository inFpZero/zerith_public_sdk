#include "../common/example_cli.hpp"
/**
 * @file 03_multi_cam_viewer.cpp
 * @brief 【示例 03】自动发现多相机流
 * * [功能介绍]
 * 自动从服务端获取所有可用的相机 ID，无需手动硬编码相机名称。
 * * [使用说明]
 * 1. 修改 ROBOT_IP 为机器人真实 IP（例如 "192.168.1.100"）。
 * 2. 程序会自动调用 getCameraState() 接口拉取配置列表。
 * * [注意]
 * 如果该接口返回列表为空，说明服务端未成功挂载任何相机设备。
 */

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "H1_Robot.hpp"

int main(int argc, char** argv) {
  // ---------------------------------------------------------
  // 【用户配置区】请根据实际环境修改以下参数
  // ---------------------------------------------------------
  const std::string ROBOT_IP = h1_sdk_examples::parseRobotIp(argc, argv);
  // ---------------------------------------------------------

  auto& client = H1Robot::getInstance(ROBOT_IP);
  client.startCameraWebRTC();

  std::vector<std::string> camera_names;
  try {
    // 自动发现：获取所有可用相机
    auto state = client.getCameraState({}, 5.0);
    for (int i = 0; i < state.camera_configs_size(); ++i) {
      camera_names.push_back(state.camera_configs(i).camera_name());
    }
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] 获取状态失败，请检查 IP [" << ROBOT_IP << "]: " << e.what() << std::endl;
    client.stopCameraWebRTC();
    return -1;
  }

  std::cout << "[INFO] 正在开启多窗口预览，按 'q' 退出..." << std::endl;
  while (true) {
    for (const auto& name : camera_names) {
      auto frame = client.getLatestCameraFrame(name);
      if (frame) {
        cv::imshow("Stream: " + name, frame->first);
      }
    }

    if (cv::waitKey(1) == 'q') break;
  }

  cv::destroyAllWindows();
  client.stopCameraWebRTC();
  return 0;
}
