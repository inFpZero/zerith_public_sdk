cmake_minimum_required(VERSION 3.16)
project(H1_SDK_ubuntu20_examples LANGUAGES CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

set(H1_REQUIRED_HEADERS
  ClientTypes.hpp H1_Robot.hpp MotorStateMonitor.hpp config.hpp
  log.hpp log_stream.hpp robot.pb.h)
foreach(header IN LISTS H1_REQUIRED_HEADERS)
  if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}")
    message(FATAL_ERROR "Missing SDK header: ${header}")
  endif()
endforeach()

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/lib/librobot_core.so")
  message(FATAL_ERROR "Missing lib/librobot_core.so")
endif()

set(OpenCV_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opencv_shared_lib/lib/cmake/opencv4")
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc highgui imgcodecs)
find_package(Threads REQUIRED)

foreach(opencv_library IN LISTS OpenCV_LIBS)
  if(TARGET "${opencv_library}")
    get_target_property(opencv_library_type "${opencv_library}" TYPE)
    if(opencv_library_type STREQUAL "STATIC_LIBRARY")
      message(FATAL_ERROR
        "Static OpenCV target ${opencv_library} is not supported. "
        "This release requires its bundled shared OpenCV runtime.")
    endif()
  elseif(IS_ABSOLUTE "${opencv_library}" AND opencv_library MATCHES "\\.a$")
    message(FATAL_ERROR
      "Static OpenCV library ${opencv_library} is not supported.")
  endif()
endforeach()

add_library(H1::SDK SHARED IMPORTED GLOBAL)
set_target_properties(H1::SDK PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/librobot_core.so")

file(GLOB CAMERA_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/example/camera/*.cpp")
file(GLOB MOTION_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/example/motion_control/*.cpp")
list(APPEND ALL_EXAMPLE_SOURCES ${CAMERA_SOURCES} ${MOTION_SOURCES})
list(LENGTH ALL_EXAMPLE_SOURCES EXAMPLE_COUNT)
if(NOT EXAMPLE_COUNT EQUAL 16)
  message(FATAL_ERROR "Expected exactly 16 examples, found ${EXAMPLE_COUNT}")
endif()
foreach(source IN LISTS ALL_EXAMPLE_SOURCES)
  if(source MATCHES "HandControl")
    message(FATAL_ERROR "HandControl must not be included")
  endif()
endforeach()

file(RELATIVE_PATH H1_LIB_FROM_BIN
  "${CMAKE_BINARY_DIR}/bin" "${CMAKE_CURRENT_SOURCE_DIR}/lib")

function(add_h1_example target source)
  add_executable(${target} "${source}")
  target_include_directories(${target} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/include")
  target_link_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_link_options(${target} PRIVATE
    "-Wl,-rpath-link,${CMAKE_CURRENT_SOURCE_DIR}/lib"
    "-Wl,--disable-new-dtags")
  target_link_libraries(${target} PRIVATE
    H1::SDK ${OpenCV_LIBS} Threads::Threads dl rt m)
  set_target_properties(${target} PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/${H1_LIB_FROM_BIN}"
    INSTALL_RPATH_USE_LINK_PATH FALSE)
endfunction()

foreach(source IN LISTS CAMERA_SOURCES)
  get_filename_component(stem "${source}" NAME_WE)
  add_h1_example("camera_${stem}" "${source}")
endforeach()
foreach(source IN LISTS MOTION_SOURCES)
  get_filename_component(stem "${source}" NAME_WE)
  add_h1_example("${stem}" "${source}")
endforeach()
