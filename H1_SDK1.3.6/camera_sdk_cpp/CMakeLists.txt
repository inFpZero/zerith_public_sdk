cmake_minimum_required(VERSION 3.16)
project(camera_sdk_examples)

set(CMAKE_CXX_STANDARD 17)

# --- 查找依赖 ---
find_package(OpenCV REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ZLIB REQUIRED)
find_package(LibDataChannel CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC REQUIRED libavcodec)
pkg_check_modules(AVUTIL REQUIRED libavutil)
pkg_check_modules(SWSCALE REQUIRED libswscale)

# --- SDK 包含目录 ---
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${AVCODEC_INCLUDE_DIRS} ${AVUTIL_INCLUDE_DIRS} ${SWSCALE_INCLUDE_DIRS})

# --- 定义 SDK 静态库 ---
# 客户直接链接我们提供的编译好的库
set(SDK_LIB ${CMAKE_SOURCE_DIR}/lib/libcamera_client.a)

# --- 自动编译示例 ---
file(GLOB EXAMPLE_SOURCES "${CMAKE_SOURCE_DIR}/examples/*.cpp")

foreach(SOURCE ${EXAMPLE_SOURCES})
    get_filename_component(NAME ${SOURCE} NAME_WLE)
    add_executable(${NAME} ${SOURCE})
    
    # 链接顺序：业务代码 -> SDK库 -> 第三方依赖库
    target_link_libraries(${NAME} 
        PRIVATE 
        ${SDK_LIB}
        ${OpenCV_LIBS}
        LibDataChannel::LibDataChannel
        ${AVCODEC_LIBRARIES}
        ${AVUTIL_LIBRARIES}
        ${SWSCALE_LIBRARIES}
        ZLIB::ZLIB
        ${Protobuf_LIBRARIES}
        grpc++
        gpr
        absl_synchronization
    )
endforeach()
