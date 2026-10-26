# H1 SDK C++ for Ubuntu 20.04

This package contains the H1 C++ SDK, DDS middleware, `librobot_core.so`, and
the source code for 16 examples. It does not ship prebuilt example
executables. The SDK and bundled runtime libraries were built in an Ubuntu
20.04 x86_64 container with GCC 9.4. The rebuildable CPython 3.10 binding is
available in the sibling `SDK_python` directory.

## Rebuild all examples

Requirements: Ubuntu 20.04 x86_64, CMake 3.16 or newer, and a C++17 compiler.
OpenCV 4.6, FFmpeg, OpenSSL, DDS, Protobuf, GTK2, and other non-base build and
runtime dependencies are included in the release package.

```bash
cmake -S . -B build
cmake --build build --parallel 2
```

The build defaults to `Release`. The 16 rebuilt programs are written to
`build/bin`.

The rebuilt programs use package-relative runtime paths, so
`LD_LIBRARY_PATH` is not required. Camera and robot examples require the
corresponding device and network environment.

## Included examples

Camera: `camera_01_minimal_color`, `camera_01_minimal_color_dds`,
`camera_02_minimal_depth`, `camera_02_minimal_depth_dds`,
`camera_03_multi_cam_viewer`, `camera_03_multi_cam_viewer_dds`,
`camera_04_get_full_state`, `camera_04_get_full_state_dds`,
`camera_05_full_demo`, and `camera_05_full_demo_dds`.

Motion/state: `h1_data_collection`, `h1_high_level_complex_new`,
`h1_low_level_KPKD`, `h1_low_level_motor_limit`, `h1_low_level_simple`, and
`state_monitor`.
