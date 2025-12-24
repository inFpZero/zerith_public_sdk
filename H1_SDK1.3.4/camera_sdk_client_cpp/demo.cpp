#include "camera_client.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <vector>
#include <string>
#include <unordered_map>
#include <deque>

using namespace unified;

static const std::string GRPC_TARGET = "localhost:50051";

static std::atomic<bool> g_should_exit{false};
static void handle_signal(int sig) {
  if (sig == SIGINT || sig == SIGTERM) {
    g_should_exit = true;
    std::cerr << "\n捕捉到退出信号，正在关闭...\n";
  }
}

static void list_cameras(UnifiedReceiverClient& cli,
                         std::vector<std::string>& rs_names,
                         std::vector<std::string>& v4l2_names) {
  auto st = cli.get_state();
  for (const auto& a : st.actuals()) {
    if (a.has_rs()) {
      rs_names.push_back(a.rs().camera_name());
    } else if (a.has_v4l2()) {
      v4l2_names.push_back(a.v4l2().camera_name());
    }
  }
  std::cerr << "RealSense cameras: ";
  for (auto& n : rs_names) std::cerr << n << " ";
  std::cerr << "\nV4L2 cameras    : ";
  for (auto& n : v4l2_names) std::cerr << n << " ";
  std::cerr << "\n";
}

struct CameraStatus {
  double last_ts = -1.0;
  double fps = 0.0;
  int frame_count = 0;
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point last_update_time;

  CameraStatus() {
    start_time = std::chrono::steady_clock::now();
    last_update_time = start_time;
  }
};

// 辅助函数：水平拼接两个图像
cv::Mat create_horizontal_stack(const cv::Mat& img1, const cv::Mat& img2) {
  // 确保两个图像高度相同
  int max_height = std::max(img1.rows, img2.rows);
  
  cv::Mat resized_img1, resized_img2;
  if (img1.rows < max_height) {
    int pad_bottom = max_height - img1.rows;
    cv::copyMakeBorder(img1, resized_img1, 0, pad_bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  } else {
    resized_img1 = img1;
  }
  
  if (img2.rows < max_height) {
    int pad_bottom = max_height - img2.rows;
    cv::copyMakeBorder(img2, resized_img2, 0, pad_bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
  } else {
    resized_img2 = img2;
  }
  
  cv::Mat result;
  cv::hconcat(resized_img1, resized_img2, result);
  return result;
}

static void show_all_cameras_loop(UnifiedReceiverClient& cli,
                                  const std::vector<std::string>& camera_names,
                                  std::atomic<bool>& stop_event) {
  const std::string window_name = "All Cameras";
  cv::namedWindow(window_name, cv::WINDOW_NORMAL);
  cv::resizeWindow(window_name, 1200, 800);

  // 存储每个摄像头的状态
  std::unordered_map<std::string, CameraStatus> camera_status;
  for (const auto& name : camera_names) {
    camera_status[name] = CameraStatus();
  }

  try {
    while (!stop_event.load()) {
      std::vector<cv::Mat> frames;
      std::vector<std::string> display_texts;

      for (const auto& cam_name : camera_names) {
        auto item = cli.get_latest_frame(cam_name);
        auto& status = camera_status[cam_name];
        auto current_time = std::chrono::steady_clock::now();

        if (item) {
          auto& [bgr, ts] = *item;
          status.frame_count++;

          // 计算实时FPS（基于时间间隔）
          if (status.last_ts > 0 && ts > status.last_ts) {
            double dt = ts - status.last_ts;
            if (dt > 0) {
              double fps = 1.0 / dt;
              // 平滑处理FPS
              status.fps = 0.8 * status.fps + 0.2 * fps;
            }
          }
          status.last_ts = ts;

          // 计算平均FPS（基于帧计数）
          auto elapsed = std::chrono::duration<double>(current_time - status.start_time).count();
          double avg_fps = elapsed > 1 ? status.frame_count / elapsed : 0;

          // 每5秒重置一次平均FPS计数
          if (std::chrono::duration<double>(current_time - status.start_time).count() > 5) {
            status.frame_count = 0;
            status.start_time = current_time;
          }

          // 调整图像大小以适应显示
          int h = bgr.rows, w = bgr.cols;
          cv::Mat resized_bgr = bgr;
          if (w > 640 || h > 480) {
            double scale = std::min(640.0 / w, 480.0 / h);
            int new_w = static_cast<int>(w * scale);
            int new_h = static_cast<int>(h * scale);
            cv::resize(bgr, resized_bgr, cv::Size(new_w, new_h));
          }

          // 添加信息文本
          std::string info1 = cam_name;
          char info2[64], info3[64];
          std::snprintf(info2, sizeof(info2), "FPS: %4.1f", status.fps);
          std::snprintf(info3, sizeof(info3), "Size: %dx%d", w, h);

          cv::putText(resized_bgr, info1, cv::Point(10, 20),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
          cv::putText(resized_bgr, info2, cv::Point(10, 40),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
          cv::putText(resized_bgr, info3, cv::Point(10, 60),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

          frames.push_back(resized_bgr);

          char display_text[128];
          std::snprintf(display_text, sizeof(display_text), "%s: %4.1f FPS", cam_name.c_str(), status.fps);
          display_texts.push_back(display_text);
        } else {
          // 创建等待画面
          cv::Mat placeholder(240, 320, CV_8UC3, cv::Scalar(0, 0, 0));
          cv::putText(placeholder, cam_name, cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
          cv::putText(placeholder, "No signal", cv::Point(10, 60),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
          frames.push_back(placeholder);
          display_texts.push_back(cam_name + ": No signal");
        }
      }

      // 创建多画面拼接
      if (!frames.empty()) {
        cv::Mat composite;

        if (frames.size() == 1) {
          composite = frames[0];
        } else if (frames.size() == 2) {
          // 两个摄像头水平排列
          composite = create_horizontal_stack(frames[0], frames[1]);
        } else if (frames.size() <= 4) {
          // 2x2 网格
          std::vector<cv::Mat> rows;
          for (size_t i = 0; i < frames.size(); i += 2) {
            std::vector<cv::Mat> row_frames;
            for (size_t j = i; j < std::min(i + 2, frames.size()); j++) {
              row_frames.push_back(frames[j]);
            }

            // 确保所有行中的图像高度一致
            int max_h = 0;
            for (const auto& f : row_frames) {
              max_h = std::max(max_h, f.rows);
            }

            std::vector<cv::Mat> padded_frames;
            for (auto& f : row_frames) {
              if (f.rows < max_h) {
                int pad_bottom = max_h - f.rows;
                cv::Mat padded_f;
                cv::copyMakeBorder(f, padded_f, 0, pad_bottom, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
                padded_frames.push_back(padded_f);
              } else {
                padded_frames.push_back(f);
              }
            }

            cv::Mat row;
            cv::hconcat(padded_frames, row);
            rows.push_back(row);
          }

          // 确保所有行宽度一致
          int max_w = 0;
          for (const auto& r : rows) {
            max_w = std::max(max_w, r.cols);
          }

          std::vector<cv::Mat> padded_rows;
          for (auto& r : rows) {
            if (r.cols < max_w) {
              int pad_right = max_w - r.cols;
              cv::Mat padded_r;
              cv::copyMakeBorder(r, padded_r, 0, 0, 0, pad_right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
              padded_rows.push_back(padded_r);
            } else {
              padded_rows.push_back(r);
            }
          }

          cv::vconcat(padded_rows, composite);
        } else {
          // 超过4个摄像头，只显示前4个
          std::vector<cv::Mat> first_four(frames.begin(), frames.begin() + 4);
          cv::Mat top_row, bottom_row;
          cv::hconcat(std::vector<cv::Mat>{first_four[0], first_four[1]}, top_row);
          cv::hconcat(std::vector<cv::Mat>{first_four[2], first_four[3]}, bottom_row);
          cv::vconcat(std::vector<cv::Mat>{top_row, bottom_row}, composite);
        }

        cv::imshow(window_name, composite);
      }

      // 处理按键事件
      int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {
        stop_event.store(true);
        break;
      }

      // 稍微降低循环频率，减少CPU占用
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception& e) {
    std::cerr << "显示循环错误: " << e.what() << "\n";
  }

  try {
    cv::destroyWindow(window_name);
  } catch (...) {
  }
  std::cerr << "显示线程结束\n";
}

static void sync_reinit_all_cameras(UnifiedReceiverClient& cli,
                                    const std::vector<std::string>& rs_cameras,
                                    const std::vector<std::string>& v4l2_cameras,
                                    std::atomic<bool>& stop_event) {
  // RealSense 配置序列 - 仅颜色流
  std::vector<std::vector<UnifiedReceiverClient::RsTargetIn>> rs_configs = {
      // 阶段0: 低分辨率
      [&]() {
        std::vector<UnifiedReceiverClient::RsTargetIn> config;
        for (const auto& cam : rs_cameras) {
          config.push_back({cam, {{"color", 640, 480, 30, "bgr8"}}});
        }
        return config;
      }(),
      // 阶段1: 中等分辨率
      [&]() {
        std::vector<UnifiedReceiverClient::RsTargetIn> config;
        for (const auto& cam : rs_cameras) {
          config.push_back({cam, {{"color", 1280, 720, 30, "bgr8"}}});
        }
        return config;
      }(),
      // 阶段2: 高分辨率
      [&]() {
        std::vector<UnifiedReceiverClient::RsTargetIn> config;
        for (const auto& cam : rs_cameras) {
          config.push_back({cam, {{"color", 1920, 1080, 15, "bgr8"}}});
        }
        return config;
      }()};

  // V4L2 配置序列
  std::vector<std::vector<UnifiedReceiverClient::V4L2TargetIn>> v4l2_configs = {
      [&]() {
        std::vector<UnifiedReceiverClient::V4L2TargetIn> config;
        for (const auto& cam : v4l2_cameras) {
          config.push_back({cam, 640, 480, 30, "MJPG"});
        }
        return config;
      }(),
      [&]() {
        std::vector<UnifiedReceiverClient::V4L2TargetIn> config;
        for (const auto& cam : v4l2_cameras) {
          config.push_back({cam, 1280, 720, 30, "MJPG"});
        }
        return config;
      }(),
      [&]() {
        std::vector<UnifiedReceiverClient::V4L2TargetIn> config;
        for (const auto& cam : v4l2_cameras) {
          config.push_back({cam, 1920, 1080, 15, "MJPG"});
        }
        return config;
      }()};

  // 确保配置序列长度一致
  size_t max_configs = std::max(rs_configs.size(), v4l2_configs.size());

  for (size_t stage = 0; stage < max_configs; ++stage) {
    if (stop_event.load()) break;

    std::cerr << "=== 配置阶段 " << (stage + 1) << "/" << max_configs << " ===\n";

    // 应用 RealSense 配置
    if (stage < rs_configs.size() && !rs_cameras.empty()) {
      auto& rs_config = rs_configs[stage];
      std::cerr << "[RS] 应用配置到 " << rs_cameras.size() << " 个摄像头\n";

      if (stage == 0) {
        // 第一次只重配置不等待
        auto reply = cli.reinit_rs(rs_config);
        std::cerr << "[RS] reinit返回: ok=" << reply.ok() << " msg=" << reply.message() << "\n";
      } else {
        // 后续配置使用等待模式
        auto [reply, matched] = cli.reinit_rs_and_wait(rs_config, 6.0, 0.2);
        std::cerr << "[RS] reinit_and_wait返回: ok=" << reply.ok() << " matched=" << matched
                  << " msg=" << reply.message() << "\n";
      }
    }

    // 应用 V4L2 配置
    if (stage < v4l2_configs.size() && !v4l2_cameras.empty()) {
      auto& v4l2_config = v4l2_configs[stage];
      std::cerr << "[V4L2] 应用配置到 " << v4l2_cameras.size() << " 个摄像头\n";

      if (stage == 0) {
        // 第一次只重配置不等待
        auto reply = cli.reinit_v4l2(v4l2_config);
        std::cerr << "[V4L2] reinit返回: ok=" << reply.ok() << " msg=" << reply.message() << "\n";
      } else {
        // 后续配置使用等待模式
        auto [reply, matched] = cli.reinit_v4l2_and_wait(v4l2_config, 6.0, 0.2);
        std::cerr << "[V4L2] reinit_and_wait返回: ok=" << reply.ok() << " matched=" << matched
                  << " msg=" << reply.message() << "\n";
      }
    }

    // 如果不是最后一个阶段，等待一段时间
    if (stage < max_configs - 1) {
      std::cerr << "等待10秒后切换到下一个配置阶段...\n";
      for (int j = 0; j < 10; ++j) {
        if (stop_event.load()) break;
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }
}

int main() {
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  UnifiedReceiverClient cli(GRPC_TARGET);
  std::atomic<bool> stop_event{false};
  std::thread display_thread;

  try {
    // 建立 WebRTC 连接
    std::cerr << "正在连接服务端...\n";
    cli.start();
    std::cerr << "连接成功\n";

    // 列出所有相机
    std::vector<std::string> rs_cameras, v4l2_cameras;
    list_cameras(cli, rs_cameras, v4l2_cameras);
    std::vector<std::string> all_cameras;
    all_cameras.insert(all_cameras.end(), rs_cameras.begin(), rs_cameras.end());
    all_cameras.insert(all_cameras.end(), v4l2_cameras.begin(), v4l2_cameras.end());

    if (all_cameras.empty()) {
      std::cerr << "没有发现任何摄像头，退出程序\n";
      return 0;
    }

    std::cerr << "发现 " << all_cameras.size() << " 个摄像头: ";
    for (const auto& cam : all_cameras) {
      std::cerr << cam << " ";
    }
    std::cerr << "\n";
    std::cerr << "启动所有摄像头显示...\n";

    // 启动单个显示线程处理所有摄像头
    display_thread = std::thread([&]() {
      show_all_cameras_loop(cli, all_cameras, stop_event);
    });
    std::cerr << "显示线程已启动\n";

    // 等待一下让窗口初始化完成
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 同步重配置所有摄像头
    sync_reinit_all_cameras(cli, rs_cameras, v4l2_cameras, stop_event);

    std::cerr << "重配置演示完成，按'q'关闭窗口或Ctrl+C退出程序\n";

    // 主线程等待退出信号或显示线程结束
    while (!stop_event.load() && display_thread.joinable()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

  } catch (const std::exception& e) {
    std::cerr << "程序运行出错: " << e.what() << "\n";
  }

  // 清理资源
  std::cerr << "正在清理资源...\n";
  stop_event.store(true);

  // 等待显示线程结束
  if (display_thread.joinable()) {
    display_thread.join();
    std::cerr << "显示线程已结束\n";
  }

  // 关闭所有OpenCV窗口
  try {
    cv::destroyAllWindows();
  } catch (...) {
  }

  // 停止客户端
  try {
    cli.stop();
  } catch (...) {
  }

  std::cerr << "程序退出完成\n";
  return 0;
}