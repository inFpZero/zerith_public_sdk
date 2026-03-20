cmake_minimum_required(VERSION 3.10)
project(H1_SDK_Public CXX)
set(CMAKE_CXX_STANDARD 17)

# --- 1) 第三方库根路径（绝对路径） ---
set(THIRD_LIBS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/3rd_party")
set(ZCM_ROOT    "${THIRD_LIBS_ROOT}/zcm_static_lib")
set(SPDLOG_ROOT "${THIRD_LIBS_ROOT}/spdlog_static_lib")
set(GRPC_ROOT   "${THIRD_LIBS_ROOT}/grpc_static_lib")

# 让 CMake 能找到包配置
set(Protobuf_DIR "${GRPC_ROOT}/lib/cmake/protobuf")
set(gRPC_DIR     "${GRPC_ROOT}/lib/cmake/grpc")
set(CMAKE_PREFIX_PATH 
    ${GRPC_ROOT}
    ${GRPC_ROOT}/lib/cmake
    ${GRPC_ROOT}/lib/cmake/protobuf  # 显式指向 protobuf 配置文件
    ${GRPC_ROOT}/lib/cmake/grpc      # 显式指向 grpc 配置文件
    ${SPDLOG_ROOT}
    ${ZCM_ROOT}
    ${CMAKE_PREFIX_PATH}
)

# 优先静态库
set(CMAKE_FIND_LIBRARY_SUFFIXES ".a;${CMAKE_FIND_LIBRARY_SUFFIXES}")

# --- 2) 查找依赖包 ---
find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(spdlog REQUIRED CONFIG)
find_package(Threads REQUIRED)

# ZCM 手动找静态库
find_library(ZCM_LIB       NAMES libzcm.a       PATHS "${ZCM_ROOT}/lib" NO_DEFAULT_PATH)
find_library(ZCM_JSON_LIB  NAMES libzcm_json.a  PATHS "${ZCM_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ZCM_LIB OR NOT ZCM_JSON_LIB)
  message(FATAL_ERROR "ZCM static libs not found in ${ZCM_ROOT}/lib")
endif()

# --- 3) gRPC 头文件生成（从对外包的 proto） ---
set(PROTOC_EXE "${GRPC_ROOT}/bin/protoc")
set(GRPC_CPP_PLUGIN_EXE "${GRPC_ROOT}/bin/grpc_cpp_plugin")
set(PROTO_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/GRPC_Data")
set(GEN_SRC_DIR   "${CMAKE_CURRENT_BINARY_DIR}/generated_grpc")
file(MAKE_DIRECTORY ${GEN_SRC_DIR})

set(PROTO_FILE "${PROTO_SRC_DIR}/ip_service.proto")

add_custom_command(
    OUTPUT "${GEN_SRC_DIR}/ip_service.pb.cc"
           "${GEN_SRC_DIR}/ip_service.pb.h"
           "${GEN_SRC_DIR}/ip_service.grpc.pb.cc"
           "${GEN_SRC_DIR}/ip_service.grpc.pb.h"
    COMMAND ${PROTOC_EXE}
    ARGS --cpp_out=${GEN_SRC_DIR}
         --grpc_out=${GEN_SRC_DIR}
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXE}
         -I ${PROTO_SRC_DIR}
         ${PROTO_FILE}
    DEPENDS ${PROTO_FILE}
    COMMENT "Generating gRPC/Protobuf headers from ${PROTO_FILE}"
)
add_custom_target(grpc_codegen
    DEPENDS
      "${GEN_SRC_DIR}/ip_service.pb.cc"
      "${GEN_SRC_DIR}/ip_service.pb.h"
      "${GEN_SRC_DIR}/ip_service.grpc.pb.cc"
      "${GEN_SRC_DIR}/ip_service.grpc.pb.h"
)

# --- 4) 查找已打包的核心静态库 ---
find_library(ROBOT_CORE_LIB NAMES robot_core PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib" NO_DEFAULT_PATH)
if(NOT ROBOT_CORE_LIB)
  message(FATAL_ERROR "robot_core.a not found in ${CMAKE_CURRENT_SOURCE_DIR}/lib")
endif()

# --- 5) 编译示例 ---
file(GLOB EXAMPLES "${CMAKE_CURRENT_SOURCE_DIR}/example/*.cpp")

foreach(EXAMPLE ${EXAMPLES})
  get_filename_component(EXE_NAME ${EXAMPLE} NAME_WE)
  add_executable(${EXE_NAME} ${EXAMPLE})
  add_dependencies(${EXE_NAME} grpc_codegen)

  target_include_directories(${EXE_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ZCM_Data
    ${CMAKE_CURRENT_SOURCE_DIR}/include/GRPC_Data
    ${CMAKE_CURRENT_SOURCE_DIR}/include/log
    ${GEN_SRC_DIR}
    ${Protobuf_INCLUDE_DIRS}
    ${gRPC_INCLUDE_DIRS}
    ${ZCM_ROOT}/include
  )

  target_link_libraries(${EXE_NAME} PRIVATE
    ${ROBOT_CORE_LIB}

    # ZCM 依赖（保持注册逻辑）
    -Wl,--whole-archive
    ${ZCM_LIB}
    -Wl,--no-whole-archive
    ${ZCM_JSON_LIB}

    # gRPC / Protobuf
    gRPC::grpc++_unsecure
    gRPC::grpc_unsecure
    gRPC::gpr
    protobuf::libprotobuf

    # spdlog
    spdlog::spdlog

    # 系统依赖（按内部 SDK 保持一致）
    zmq
    efivar
    Threads::Threads
    rt
    m
    ${CMAKE_DL_LIBS}
  )
endforeach()