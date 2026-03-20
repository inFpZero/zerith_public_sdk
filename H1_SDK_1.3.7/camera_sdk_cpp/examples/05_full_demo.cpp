/**
 * @file 05_full_demo.cpp
 * @brief 【示例 05】相机 SDK 全功能综合演示程序 (预览、统计与 UI)
 * * * [功能介绍]
 * 1. 自动发现：连接后动态获取服务端所有 RGB 及 Depth 相机配置。
 * 2. 多路预览：为每个发现的相机自动创建渲染窗口。
 * 3. 性能统计：内置 SourceFpsTracker，基于单调时间戳实时计算服务端原始帧率。
 * 4. HUD 绘制：在视频流上实时绘制相机 ID、当前 FPS、以及服务端硬件时间戳。
 * 5. 深度渲染：自动识别深度流，并应用线性缩放与 JET 伪彩色映射增强可视化效果。
 * 6. 线程模型：展示非阻塞式 getLatest 调用与主线程 UI 渲染的分离。
 * * * [使用说明]
 * 1. 修改 GRPC_TARGET 为服务端真实的 IP 地址和端口（例如 "192.168.1.100:50051"）。
 * 2. 本程序依赖 OpenCV 的高性能 UI 模块，建议在性能较好的 PC 上运行。
 * 3. 运行中按 'q' 键可安全退出并通知服务端释放 WebRTC 资源。
 * * * [注意]
 * - 如果画面出现卡顿，请检查局域网带宽负载（WebRTC 传输彩色+深度对带宽有一定要求）。
 * - 确保服务端已正确配置，否则程序在“发现设备”阶段会提示“未找到可用相机”。
 */

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <iomanip>
#include <thread>
#include <opencv2/opencv.hpp>
#include "camera_client.h"

// ==========================================
//              用户配置区域
// ==========================================
struct Config {
  static constexpr const char* GRPC_TARGET = "localhost:50051";

  // 深度图可视化参数
  static constexpr double DEPTH_SCALE_FACTOR = 0.03;

  // 界面显示设置
  static constexpr double FONT_SCALE = 0.6;
  static inline const cv::Scalar TEXT_COLOR = cv::Scalar(0, 255, 0);  // BGR: 绿色
  static constexpr int TEXT_THICKNESS = 1;

  // 窗口刷新逻辑
  static constexpr int WAIT_KEY_DELAY = 1;  // ms
};

// ==========================================
//              工具类与辅助函数
// ==========================================

class SourceFpsTracker {
 public:
  SourceFpsTracker(double stat_interval = 1.0)
      : stat_interval_(stat_interval) {}

  double update(double timestamp) {
    // 如果时间戳没变（或者是旧帧），直接返回旧FPS
    if (timestamp <= last_ts_) {
      return current_fps_;
    }

    // 初始化第一个时间窗口
    if (window_start_ts_ < 0) {
      window_start_ts_ = timestamp;
      last_ts_ = timestamp;
      return 0.0;
    }

    // 累加帧数
    frames_in_window_++;
    last_ts_ = timestamp;

    // 检查是否达到统计周期
    double duration = timestamp - window_start_ts_;
    if (duration >= stat_interval_) {
      current_fps_ = static_cast<double>(frames_in_window_) / duration;
      // 重置窗口
      window_start_ts_ = timestamp;
      frames_in_window_ = 0;
    }

    return current_fps_;
  }

 private:
  double last_ts_ = -1.0;
  double window_start_ts_ = -1.0;
  int frames_in_window_ = 0;
  double current_fps_ = 0.0;
  double stat_interval_;
};

cv::Mat visualize_depth(const cv::Mat& depth_raw) {
  // 1. 线性缩放: uint16 -> uint8
  cv::Mat depth_8bit;
  depth_raw.convertTo(depth_8bit, CV_8U, Config::DEPTH_SCALE_FACTOR);

  // 2. 伪彩色映射
  cv::Mat depth_color;
  cv::applyColorMap(depth_8bit, depth_color, cv::COLORMAP_JET);
  return depth_color;
}

cv::Mat draw_hud(cv::Mat image, const std::vector<std::string>& text_lines) {
  int x = 10;
  int y = 30;
  int line_height = 25;

  for (const auto& line : text_lines) {
    // 绘制黑色描边 (背景)
    cv::putText(image, line, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                Config::FONT_SCALE, cv::Scalar(0, 0, 0), Config::TEXT_THICKNESS + 2);
    // 绘制彩色文字 (前景)
    cv::putText(image, line, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX,
                Config::FONT_SCALE, Config::TEXT_COLOR, Config::TEXT_THICKNESS);
    y += line_height;
  }
  return image;
}

cv::Mat create_placeholder_image(const std::string& text) {
  cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::putText(img, text, cv::Point(50, 240), cv::FONT_HERSHEY_SIMPLEX,
              1.0, cv::Scalar(200, 200, 200), 2);
  return img;
}

// ==========================================
//                 主程序逻辑
// ==========================================

int main(int argc, char** argv) {
  // 1. 初始化客户端
  CameraClient client(Config::GRPC_TARGET);
  std::cout << "[*] 正在连接 gRPC 服务: " << Config::GRPC_TARGET << " ..." << std::endl;

  robot::RecorderStateResponse state;
  try {
    client.start();
    std::cout << "[+] 连接建立成功，开始握手..." << std::endl;

    // 获取服务端状态
    state = client.get_state(std::vector<std::string>(), 5.0);
  } catch (const std::exception& e) {
    std::cerr << "[!] 连接或初始化失败: " << e.what() << std::endl;
    std::cerr << "    请检查: 1. 服务端是否运行  2. IP端口配置  3. 网络防火墙" << std::endl;
    return -1;
  }

  // 2. 解析相机列表
  std::vector<std::string> rgb_cameras;
  std::vector<std::string> depth_cameras;

  std::cout << "\n--- 发现设备 ---" << std::endl;
  for (int i = 0; i < state.camera_configs_size(); ++i) {
    const auto& cam_config = state.camera_configs(i);
    std::string name = cam_config.camera_name();

    bool is_depth = false;
    bool is_color = false;
    std::string stream_info = "";
    for (int j = 0; j < cam_config.streams_size(); ++j) {
      const auto& s = cam_config.streams(j);
      stream_info += s.type() + " ";

      if (s.type() == "depth") {
        is_depth = true;
      } else if (s.type() == "color") {
        is_color = true;
      }
    }

    if (is_depth) {
      depth_cameras.push_back(name);
    }
    if (is_color) {
      rgb_cameras.push_back(name);
    }
    std::cout << "• 相机: " << std::left << std::setw(15) << name << " 流: " << stream_info << std::endl;
  }
  std::cout << "----------------\n"
            << std::endl;

  if (rgb_cameras.empty()) {
    std::cout << "[!] 未找到可用相机，程序退出。" << std::endl;
    client.stop();
    return 0;
  }

  // 3. 准备 FPS 追踪器
  std::map<std::string, SourceFpsTracker> trackers;
  for (const auto& name : rgb_cameras) trackers.emplace("rgb_" + name, SourceFpsTracker());
  for (const auto& name : depth_cameras) trackers.emplace("depth_" + name, SourceFpsTracker());

  std::cout << ">> 视频流接收中... 按 'q' 键退出。" << std::endl;

  // 4. 主循环 (渲染与显示)
  try {
    while (true) {
      bool any_window_updated = false;

      // --- 处理所有 RGB 相机 ---
      for (const auto& name : rgb_cameras) {
        auto frame_data = client.getLatestFrame(name);
        cv::Mat image_display;

        if (frame_data) {
          cv::Mat image = frame_data->first;
          double timestamp = frame_data->second;

          double fps = trackers.at("rgb_" + name).update(timestamp);

          std::vector<std::string> info_lines = {
              "CAM: " + name + " (RGB)",
              "FPS: " + std::to_string(fps).substr(0, 4),
              "TS : " + std::to_string(timestamp).substr(0, 10)};
          image_display = draw_hud(image, info_lines);
        } else {
          image_display = create_placeholder_image("Waiting: " + name);
        }

        cv::imshow("RGB - " + name, image_display);
        any_window_updated = true;
      }

      // --- 处理所有 Depth 相机 ---
      for (const auto& name : depth_cameras) {
        auto depth_data = client.getLatestDepth(name);
        cv::Mat depth_display;

        if (depth_data) {
          cv::Mat depth_raw = depth_data->first;
          double timestamp = depth_data->second;

          double fps = trackers.at("depth_" + name).update(timestamp);

          depth_display = visualize_depth(depth_raw);

          std::vector<std::string> info_lines = {
              "CAM: " + name + " (Depth)",
              "FPS: " + std::to_string(fps).substr(0, 4)};
          depth_display = draw_hud(depth_display, info_lines);
        } else {
          depth_display = create_placeholder_image("Waiting: " + name);
        }

        cv::imshow("Depth - " + name, depth_display);
        any_window_updated = true;
      }

      // --- 键盘控制 ---
      int key = cv::waitKey(Config::WAIT_KEY_DELAY);
      if ((key & 0xFF) == 'q') {
        std::cout << "\n[+] 用户请求退出..." << std::endl;
        break;
      }

      if (!any_window_updated) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  } catch (...) {
    std::cout << "\n[+] 发生未知中断..." << std::endl;
  }

  // 5. 资源清理
  std::cout << "[*] 正在断开连接与释放资源..." << std::endl;
  cv::destroyAllWindows();
  client.stop();
  std::cout << "[*] 程序已结束。" << std::endl;

  return 0;
}