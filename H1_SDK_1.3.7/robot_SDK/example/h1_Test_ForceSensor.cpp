#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

int main() {

    std::cout << std::fixed << std::setprecision(6);// 设置std::cout输出精度为6位小数


    //实例化机器人
    // H1Robot robot("192.168.2.39");//SDK运行环境为用户主机，无线连接方式，ip不固定
    // H1Robot robot("172.31.200.1");//SDK运行环境为用户主机，有线连接方式，ip固定
    H1Robot robot;       //SDK运行环境为机器人主机

    // 尝试连接机器人
    if(!robot.robot_connect()) { 
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }


    //状态获取无需初始化
    // //选择低层控制模式
    // if(!robot.switchControlMode(MotorControlMode::GRAVITY_COMPENSATION_LEVEL)) {
    //     std::cerr << "Failed to switch to LOW_LEVEL mode." << std::endl;
    //     return 1;
    // }

    // // 尝试初始化机器人（动作）
    // if(robot.robot_init()) {
    //     std::cout << "Robot initialized successfully." << std::endl;
    // } else {
    //     std::cerr << "Robot initialization failed." << std::endl;
    //     return 1;
    // }
    
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // // 创建并启动电机状态监控


    MotorStateMonitor monitor(robot);
    monitor.start();

    ForceSensorState force_sensor_state;
    //开始执行控制，控制的起点必须是初始化后的位姿
    int k=0;
    while(true) {       
        // if(k >=1) {
        //     break; // 测试k次后退出
        // }

        // std::cout << "Test iteration: " << k + 1 << std::endl;
        k++;
        robot.getForceSensorState(force_sensor_state);
        std::cout << k <<": "<<"Force Sensor Data: "
                  << "[0] " << force_sensor_state.Fx[0] << " "
                  << "" << force_sensor_state.Fy[0] << " "
                  << "" << force_sensor_state.Fz[0] << " "
                  << "" << force_sensor_state.Tx[0] << " "
                  << "" << force_sensor_state.Ty[0] << " "
                  << "" << force_sensor_state.Tz[0] << " "
                  << "Err " << force_sensor_state.Error_flag[0] << " "
                  << "[1] " << force_sensor_state.Fx[0] << " "
                  << "" << force_sensor_state.Fy[0] << " "
                  << "" << force_sensor_state.Fz[0] << " "
                  << "" << force_sensor_state.Tx[0] << " "
                  << "" << force_sensor_state.Ty[0] << " "
                  << "" << force_sensor_state.Tz[0] << " "
                  << "Err " << force_sensor_state.Error_flag[0] << " "
                  <<std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if(k%99==0){
            robot.resetForceSensorData(1);
        }
        if(k%180==0){
            robot.resetForceSensorData(0);
            
        }
    }

    // 停止监控
    monitor.stop();

    // 尝试反初始化机器人（动作）
    if(robot.robot_deinit()) {
        std::cout << "Robot deinitialized successfully." << std::endl;
    } else {
        std::cerr << "Robot deinitialization failed." << std::endl;
        return 1;
    }

    return 0;
}