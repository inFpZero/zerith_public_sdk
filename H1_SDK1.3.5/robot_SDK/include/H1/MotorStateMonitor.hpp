#ifndef MOTOR_STATE_MONITOR_HPP
#define MOTOR_STATE_MONITOR_HPP

#include "H1_Robot.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


// 前向声明，而不是包含头文件
class H1Robot;
class MotorStateMonitor {
private:
    std::atomic<bool> is_printing_{false};
    std::atomic<bool> should_exit_{false};
    std::thread monitor_thread_;
    std::thread keyboard_thread_;
    H1Robot* robot_ptr_;

public:
    explicit MotorStateMonitor(H1Robot& robot);
    ~MotorStateMonitor();
    
    // 禁用拷贝构造和赋值
    MotorStateMonitor(const MotorStateMonitor&) = delete;
    MotorStateMonitor& operator=(const MotorStateMonitor&) = delete;
    
    void start();
    void stop();
    void startPrinting();
    void stopPrinting();
    bool isRunning() const;
    bool isPrinting() const;

private:
    void monitorThreadFunc();
    void keyboardThreadFunc();
    void handleKey(char key);
    void printMotorStates(int loop_count);
    void printZCMData(int loop_count);
    char getKey();
};

#endif // MOTOR_STATE_MONITOR_HPP