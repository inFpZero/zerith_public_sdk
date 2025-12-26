project(H1Examples)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

# 设置头文件路径
include_directories(${CMAKE_SOURCE_DIR}/include/H1)
include_directories(${CMAKE_SOURCE_DIR}/include/H1/ZCM_Data)
include_directories(${CMAKE_SOURCE_DIR}/)

# 查找预编译的库
find_library(ROBOT_CORE_LIB robot_core PATHS ${CMAKE_SOURCE_DIR}/lib/H1 NO_DEFAULT_PATH)



# 自动查找 example 目录下所有 cpp 文件
file(GLOB EXAMPLES ${CMAKE_SOURCE_DIR}/example/*.cpp)

# 为每个 example 生成可执行文件
foreach(EXAMPLE ${EXAMPLES})
    get_filename_component(EXE_NAME ${EXAMPLE} NAME_WE)
    add_executable(${EXE_NAME} ${EXAMPLE})
    target_link_libraries(${EXE_NAME}
        ${ROBOT_CORE_LIB}
        zcm
        grpc++
        protobuf
        gpr
        absl_synchronization
    )
endforeach()


#用户自行编译指定代码的示例
# 编译示例程序1
#add_executable(h1_sdk_example_high_level ${CMAKE_SOURCE_DIR}/example/h1_sdk_example_high_level.cpp)
#target_link_libraries(h1_sdk_example_high_level                         
#                        ${ROBOT_CORE_LIB} 
#                        zcm 
#                        grpc++
#                        protobuf
#                        gpr
#                        absl_synchronization
#                        )

