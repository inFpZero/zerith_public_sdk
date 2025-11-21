cmake_minimum_required(VERSION 3.16)
project(vr_collect_demo CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-webrtc-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
  gstreamer-pbutils-1.0
  gstreamer-sdp-1.0
)


# ===== 头文件路径 =====
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    /usr/local/include        # gRPC / protobuf / OpenCV 通常在这里
)

include_directories(${GST_INCLUDE_DIRS})
# ===== 库路径 =====
link_directories(
    ${CMAKE_SOURCE_DIR}/lib
    /usr/local/lib
)

# ===== 外部依赖 =====
find_package(PkgConfig REQUIRED)

find_package(OpenCV REQUIRED)
find_package(X11 REQUIRED)

# ===== demo 可执行文件 =====
add_executable(demo demo.cpp)

# ===== 链接库 =====
target_link_libraries(demo
  PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/libcamera_client.a  # 你的静态库
    ${GST_LIBRARIES}
    ${OpenCV_LIBS}
    X11
    jsoncpp
    grpc++
    protobuf
    gpr
    absl_base
    absl_synchronization
    absl_strings
    absl_time
    absl_throw_delegate
    pthread
    dl
    z  
)

# ===== 运行时 RPATH（仅对共享库有用，可留可删）=====
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
list(APPEND CMAKE_BUILD_RPATH
    "${CMAKE_SOURCE_DIR}/lib"
    "/usr/local/lib"
)
