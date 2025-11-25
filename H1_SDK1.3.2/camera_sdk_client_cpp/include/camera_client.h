#pragma once
#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <optional>
#include <utility>
#include <atomic>

#include <opencv2/opencv.hpp>

// gRPC
#include <grpcpp/grpcpp.h>

// Protobuf stubs (generated from your .proto)
#include "robot.pb.h"
#include "robot.grpc.pb.h"

// GStreamer
#include <gst/gst.h>
#include <gst/sdp/gstsdpmessage.h>
#include <gst/webrtc/webrtc.h>

namespace unified {

// IMPORTANT: alias inside our namespace to avoid clashing with Protobuf's global ::pb
namespace pb = robot;

class UnifiedReceiverClient {
 public:
  // Python signature: UnifiedReceiverClient(grpc_target="localhost:50051", connect_timeout=10.0)
  explicit UnifiedReceiverClient(const std::string& grpc_target = "localhost:50051",
                                 double connect_timeout = 10.0);
  ~UnifiedReceiverClient();

  // start()/stop() identical semantics
  void start();
  void stop();

  // get_latest_frame(cam_name) -> (bgr8 mat, timestamp) or empty
  std::optional<std::pair<cv::Mat, double>> get_latest_frame(const std::string& cam_name);

  // get_latest_depth(cam_name) -> (depth mat, timestamp) or empty
  std::optional<std::pair<cv::Mat, double>> get_latest_depth(const std::string& cam_name);

  // get_state(camera_names=None, timeout=5.0) -> pb::RecorderGetStateReply
  pb::RecorderGetStateReply get_state(const std::vector<std::string>& camera_names = {},
                                      double timeout_sec = 5.0);

  // --- RealSense reinit ---
  struct RsStreamSpecIn {
    std::string type;
    int width{}, height{}, fps{};
    std::string fmt;
  };
  struct RsTargetIn {
    std::string camera_name;
    std::vector<RsStreamSpecIn> streams;
  };
  pb::RecorderReinitReply reinit_rs(const std::vector<RsTargetIn>& targets,
                                    double timeout_sec = 5.0);

  // reinit_rs_and_wait(targets, wait_timeout=6.0, poll_interval=0.2) -> (reply, matched)
  std::pair<pb::RecorderReinitReply, bool> reinit_rs_and_wait(const std::vector<RsTargetIn>& targets,
                                                              double wait_timeout = 6.0,
                                                              double poll_interval = 0.2);

  // --- V4L2 reinit ---
  struct V4L2TargetIn {
    std::string camera_name;
    int width{}, height{}, fps{};
    std::string fourcc;
  };
  pb::RecorderReinitReply reinit_v4l2(const std::vector<V4L2TargetIn>& targets, double timeout_sec = 5.0);
  std::pair<pb::RecorderReinitReply, bool> reinit_v4l2_and_wait(const std::vector<V4L2TargetIn>& targets,
                                                                double wait_timeout = 6.0,
                                                                double poll_interval = 0.2);

 private:
  struct FrameItem {
    cv::Mat bgr;
    double ts{};
  };

  struct DepthFrameItem {
    cv::Mat depth;
    double ts{};
  };

  // Depth frame reassembler
  class DepthFrameReassembler {
   public:
    DepthFrameReassembler(double timeout_ms = 1500.0);
    void add_header(const std::string& metadata_json);
    std::optional<std::pair<std::unordered_map<std::string, std::string>, std::vector<uint8_t>>>
    add_chunk(const std::vector<uint8_t>& packet);
    void garbage_collect();

   private:
    struct FrameState {
      std::unordered_map<std::string, std::string> metadata;
      std::vector<uint8_t> buffer;
      int chunks_count{};
      int chunks_received{};
      double start_time{};
    };

    std::unordered_map<int, FrameState> frames_;
    double timeout_ms_;
  };

  // Zlib depth decoder
  cv::Mat decode_depth_zlib(const std::vector<uint8_t>& raw_data, int width, int height, int dtype = CV_16U);

  // Handshake (WebRTC) helpers
  void bootstrap_async();
  void mainloop_wait_stop();
  void shutdown_async();

  // gRPC ControlVideo handshake steps (server offers, client answers)
  bool grpc_request_offer(std::string& sdp_type, std::string& sdp);
  bool grpc_send_answer(const std::string& sdp_type, const std::string& sdp);

  // GStreamer webrtc helpers
  bool gst_setup_webrtc();
  bool gst_set_remote_offer_and_create_answer(const std::string& offer_sdp_text,
                                              std::string& out_answer_sdp_text);

  // Pad/linking handlers
  static void on_pad_added(GstElement* webrtc, GstPad* pad, gpointer user_data);
  static void on_decodebin_pad_added(GstElement* dbin, GstPad* pad, gpointer user_data);
  static GstFlowReturn on_new_sample(GstElement* appsink, gpointer user_data);

  // Data channel handlers
  static void on_data_channel(GstElement* webrtc, GstWebRTCDataChannel* channel, gpointer user_data);
  static void on_data_channel_message(GstWebRTCDataChannel* channel, GBytes* data, gpointer user_data);
  static void on_data_channel_open(GstWebRTCDataChannel* channel, gpointer user_data);
  static void on_data_channel_close(GstWebRTCDataChannel* channel, gpointer user_data);

  // Utilities
  static std::vector<std::string> parse_track_ids_from_sdp(const std::string& sdp_text);
  static std::string caps_to_string(const GstCaps* caps);
  static double now_monotonic_sec();

 private:
  std::string grpc_target_;
  double connect_timeout_{};

  // gRPC sync channel + stub (state/reinit + ControlVideo handshake)
  std::shared_ptr<grpc::Channel> channel_sync_;
  std::unique_ptr<robot::RobotService::Stub> stub_sync_;

  // Frame buffers: cam_name -> last frame
  std::unordered_map<std::string, FrameItem> frame_bufs_;
  std::unordered_map<std::string, DepthFrameItem> depth_bufs_;
  std::mutex buf_mu_;

  // Background thread & flags
  std::thread th_;
  std::atomic<bool> stop_req_{false};

  // GLib main loop & pipeline
  GMainLoop* loop_ = nullptr;
  GstElement* pipeline_ = nullptr;
  GstElement* webrtcbin_ = nullptr;
  GMainContext* main_context_ = nullptr;

  // Track id bookkeeping from SDP offer (order of m=video sections)
  std::vector<std::string> track_ids_;
  std::atomic<int> track_video_index_{0};

  // Depth data channel
  GstWebRTCDataChannel* depth_channel_ = nullptr;
  DepthFrameReassembler depth_reassembler_;
};

}  // namespace unified