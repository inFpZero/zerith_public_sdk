# ---------------------------------------------------------------------------
# 项目基础配置
# ---------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.10)
project(camera_client_sdk_user CXX)

# 使用 C++17 标准，确保与 OpenCV 4.6 及新版 gRPC 兼容
set(CMAKE_CXX_STANDARD 17)

# ---------------------------------------------------------------------------
# 1. 定义三方静态库路径（基于 SDK 交付目录结构）
# ---------------------------------------------------------------------------
# 交付后的结构假设：
# camera_sdk_cpp/
# ├── CMakeLists.txt  (本文件)
# ├── include/        (包含 camera_client.h 及生成的 .pb.h)
# ├── lib/            (包含 libcamera_client.a)
# ├── examples/        (示例源码)
# └── 3rd_party/      (包含所有依赖的静态库文件夹)

set(SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")
set(THIRD_LIBS_ROOT "${SDK_ROOT}/3rd_party")

message(STATUS "----------------------------------------------------")
message(STATUS "SDK_ROOT: ${SDK_ROOT}")
message(STATUS "THIRD_LIBS_ROOT: ${THIRD_LIBS_ROOT}")
message(STATUS "----------------------------------------------------")

set(OPENCV_ROOT  "${THIRD_LIBS_ROOT}/opencv_static_lib")
set(FFMPEG_ROOT  "${THIRD_LIBS_ROOT}/ffmpeg_static_lib")
set(GRPC_ROOT    "${THIRD_LIBS_ROOT}/grpc_static_lib")
set(RTC_ROOT     "${THIRD_LIBS_ROOT}/libdatachannel_static_lib")
set(OPENSSL_ROOT "${THIRD_LIBS_ROOT}/openssl_static_lib")

# 将自定义路径加入 CMake 搜索优先级列表
set(CMAKE_PREFIX_PATH 
    ${GRPC_ROOT}
    "${GRPC_ROOT}/lib/cmake/protobuf"  # 显式指向 protobuf 配置文件
    "${GRPC_ROOT}/lib/cmake/grpc"      # 显式指向 grpc 配置文件
    ${OPENCV_ROOT}
    ${FFMPEG_ROOT}
    ${RTC_ROOT}
    ${OPENSSL_ROOT}
    ${CMAKE_PREFIX_PATH}
)

# 【关键】强制 CMake 优先寻找 .a 后缀的静态库，忽略系统自带的 .so 动态库
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")

# ---------------------------------------------------------------------------
# 2. 依赖查找 (使用 NO_DEFAULT_PATH 确保不被宿主机环境污染)
# ---------------------------------------------------------------------------
# 【版本对齐】强制使用我们自己编译的 protoc 工具，防止生成的代码与链接的库版本不匹配
set(PROTOC_EXE "${GRPC_ROOT}/bin/protoc")
set(GRPC_CPP_PLUGIN_EXE "${GRPC_ROOT}/bin/grpc_cpp_plugin")
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenCV REQUIRED CONFIG)
find_package(LibDataChannel CONFIG REQUIRED)
find_package(Threads REQUIRED)

# 【手动查找】针对不提供 CMake Config 的 FFmpeg 和 OpenSSL
find_library(AVCODEC_LIB NAMES libavcodec.a PATHS "${FFMPEG_ROOT}/lib")
find_library(AVUTIL_LIB  NAMES libavutil.a  PATHS "${FFMPEG_ROOT}/lib")
find_library(SWSCALE_LIB NAMES libswscale.a PATHS "${FFMPEG_ROOT}/lib")

find_library(SSL_LIB     NAMES libssl.a     PATHS "${OPENSSL_ROOT}/lib" NO_DEFAULT_PATH)
find_library(CRYPTO_LIB  NAMES libcrypto.a  PATHS "${OPENSSL_ROOT}/lib" NO_DEFAULT_PATH)

# GUI 支持（通常需要宿主机提供）
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)

# ---------------------------------------------------------------------------
# 3. 导入预编译好的 Camera SDK 核心库
# ---------------------------------------------------------------------------
add_library(camera_client STATIC IMPORTED)

set_target_properties(camera_client PROPERTIES
    IMPORTED_LOCATION "${SDK_ROOT}/lib/libcamera_client.a"
    # 将 SDK 自己的 include 和所有三方件 include 封装在一起提供给用户
    INTERFACE_INCLUDE_DIRECTORIES 
    "${SDK_ROOT}/include;${OPENSSL_ROOT}/include;${FFMPEG_ROOT}/include"
)

# ---------------------------------------------------------------------------
# 4. 定义 SDK 完整的链接链 (静态链接的“生命线”)
# ---------------------------------------------------------------------------
# 因为是静态链接，用户程序必须按顺序链接所有底层依赖
set(SDK_LIBS
    camera_client                 # 1. 业务静态库
    ${OpenCV_LIBS}                # 2. 图像处理
    LibDataChannel::LibDataChannel # 3. WebRTC
    ${AVCODEC_LIB}                # 4. 编解码
    ${AVUTIL_LIB}
    ${SWSCALE_LIB}
    gRPC::grpc++_unsecure         # 5. 通讯
    gRPC::grpc_unsecure
    gRPC::gpr
    protobuf::libprotobuf
    ${GTK2_LIBRARIES}             # 6. 系统界面
    X11
    ${SSL_LIB}                    # 7. 加密
    ${CRYPTO_LIB}
    z m dl rt lzma bz2            # 8. 操作系统底层 (必须在最后)
    Threads::Threads
)

# ---------------------------------------------------------------------------
# 5. 自动构建示例程序
# ---------------------------------------------------------------------------
file(GLOB EXAMPLES "${SDK_ROOT}/examples/*.cpp")

foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXE_NAME ${EXAMPLE} NAME_WE)

    add_executable(${EXE_NAME} ${EXAMPLE})

    # 用户只需链接这个集合，路径和宏定义会通过 INTERFACE 属性自动透传
    target_link_libraries(${EXE_NAME} PRIVATE ${SDK_LIBS})

    message(STATUS "Found and configured target: ${EXE_NAME}")
endforeach()