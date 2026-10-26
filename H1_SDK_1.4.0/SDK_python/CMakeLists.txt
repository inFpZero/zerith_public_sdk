cmake_minimum_required(VERSION 3.16)
project(H1_SDK_python LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PYBIND11_FINDPYTHON ON)

get_filename_component(H1_CPP_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../SDK_cpp" ABSOLUTE)
set(H1_PYBIND11_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/third_party/pybind11")
set(H1_PROTOBUF_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/third_party/protobuf")
set(H1_SPDLOG_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/third_party/spdlog")
set(H1_OPENCV_ROOT
  "${H1_CPP_ROOT}/third_party/opencv_shared_lib")

set(H1_REQUIRED_FILES
  "${H1_CPP_ROOT}/include/H1_Robot.hpp"
  "${H1_CPP_ROOT}/include/log.hpp"
  "${H1_CPP_ROOT}/lib/librobot_core.so"
  "${H1_PYBIND11_ROOT}/include/pybind11/pybind11.h"
  "${H1_PYBIND11_ROOT}/share/cmake/pybind11/pybind11Config.cmake"
  "${H1_PROTOBUF_ROOT}/lib/libprotobuf.a"
  "${H1_SPDLOG_ROOT}/lib/libspdlog.a"
  "${H1_CPP_ROOT}/third_party/include/google/protobuf/message.h"
  "${H1_CPP_ROOT}/third_party/include/spdlog/spdlog.h"
  "${H1_OPENCV_ROOT}/lib/cmake/opencv4/OpenCVConfig.cmake")
foreach(required_file IN LISTS H1_REQUIRED_FILES)
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR "Missing release-local build input: ${required_file}")
  endif()
endforeach()

if(CMAKE_VERSION VERSION_LESS 3.18)
  set(H1_PYTHON_COMPONENTS Interpreter Development)
else()
  set(H1_PYTHON_COMPONENTS Interpreter Development.Module)
endif()
find_package(Python3 3.10 EXACT COMPONENTS
  ${H1_PYTHON_COMPONENTS} REQUIRED)
if(NOT Python3_VERSION_MAJOR EQUAL 3 OR NOT Python3_VERSION_MINOR EQUAL 10)
  message(FATAL_ERROR
    "CPython 3.10 is required, found ${Python3_VERSION}")
endif()

find_package(pybind11 CONFIG REQUIRED
  PATHS "${H1_PYBIND11_ROOT}/share/cmake/pybind11"
  NO_DEFAULT_PATH)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core
  PATHS "${H1_OPENCV_ROOT}/lib/cmake/opencv4"
  NO_DEFAULT_PATH)
foreach(opencv_library IN LISTS OpenCV_LIBS)
  if(TARGET "${opencv_library}")
    get_target_property(opencv_library_type "${opencv_library}" TYPE)
    if(opencv_library_type STREQUAL "STATIC_LIBRARY")
      message(FATAL_ERROR
        "The Python binding requires the bundled shared OpenCV runtime; "
        "static target ${opencv_library} is not allowed.")
    endif()
  elseif(IS_ABSOLUTE "${opencv_library}" AND opencv_library MATCHES "\\.a$")
    message(FATAL_ERROR "The Python binding requires shared OpenCV.")
  endif()
endforeach()
find_package(Threads REQUIRED)

add_library(H1::SDK SHARED IMPORTED GLOBAL)
set_target_properties(H1::SDK PROPERTIES
  IMPORTED_LOCATION "${H1_CPP_ROOT}/lib/librobot_core.so"
  INTERFACE_INCLUDE_DIRECTORIES "${H1_CPP_ROOT}/include")

add_library(H1::Protobuf STATIC IMPORTED GLOBAL)
set_target_properties(H1::Protobuf PROPERTIES
  IMPORTED_LOCATION "${H1_PROTOBUF_ROOT}/lib/libprotobuf.a"
  INTERFACE_INCLUDE_DIRECTORIES
    "${H1_CPP_ROOT}/third_party/include")

add_library(H1::Spdlog STATIC IMPORTED GLOBAL)
set_target_properties(H1::Spdlog PROPERTIES
  IMPORTED_LOCATION "${H1_SPDLOG_ROOT}/lib/libspdlog.a"
  INTERFACE_INCLUDE_DIRECTORIES
    "${H1_CPP_ROOT}/third_party/include")

pybind11_add_module(lib_h1_sdk_python MODULE
  bind/src/bindings.cpp)
target_include_directories(lib_h1_sdk_python PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/bind/include"
  "${H1_CPP_ROOT}/include")
target_link_libraries(lib_h1_sdk_python PRIVATE
  H1::SDK
  H1::Protobuf
  H1::Spdlog
  ${OpenCV_LIBS}
  Threads::Threads
  dl rt m)
target_link_options(lib_h1_sdk_python PRIVATE
  "-Wl,-rpath-link,${H1_CPP_ROOT}/lib"
  "-Wl,--disable-new-dtags")

set(H1_PYTHON_OUTPUT_DIR "${CMAKE_BINARY_DIR}/lib" CACHE PATH
  "Output directory for the rebuilt H1 Python extension")
file(RELATIVE_PATH H1_CPP_LIB_FROM_PYTHON_OUTPUT
  "${H1_PYTHON_OUTPUT_DIR}" "${H1_CPP_ROOT}/lib")
set_target_properties(lib_h1_sdk_python PROPERTIES
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY "${H1_PYTHON_OUTPUT_DIR}"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN/${H1_CPP_LIB_FROM_PYTHON_OUTPUT}"
  INSTALL_RPATH_USE_LINK_PATH FALSE)
