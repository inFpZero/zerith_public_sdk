#pragma once

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>
#include <opencv2/core/mat.hpp>
#include "robot.pb.h"

class CameraClient {
 public:
  explicit CameraClient(const std::string& grpc_target);
  ~CameraClient();

  CameraClient(const CameraClient&) = delete;
  CameraClient& operator=(const CameraClient&) = delete;

  CameraClient(CameraClient&&) noexcept;
  CameraClient& operator=(CameraClient&&) noexcept;

  // 启动客户端（非阻塞，内部启动工作线程）
  void start();

  // 停止客户端并清理资源
  void stop();

  // 获取最新 RGB 帧
  std::optional<std::pair<cv::Mat, double>> getLatestFrame(const std::string& cam_name);

  // 获取最新深度帧
  std::optional<std::pair<cv::Mat, double>> getLatestDepth(const std::string& cam_name);

  // 获取相机状态
  robot::RecorderStateResponse get_state(const std::vector<std::string>& camera_names = {}, float timeout_sec = 5.0);

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};