#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

const float motor_kp[23] = {0, 0, 0, 500, 500, 20, 20, 250, 250, 250, 250, 30, 40, 40, 10, 250, 250, 250, 250, 30, 40, 40, 10};
const float motor_kd[23] = {0, 0, 0, 5, 5, 1, 1, 5, 5, 5, 5, 1, 1, 1, 1, 5, 5, 5, 5, 1, 1, 1, 1};

// KP 默认 -> 最大 -> 默认 -> 最小 -> 默认
void TestMotorKP(H1Robot& robot, EtherCAT_Motor_Index motor_id, float init_pos, int steps = 10, float kp_min_coeff = 0.8f, float kp_max_coeff = 1.2f) {
    if (motor_kp[motor_id] == 0) {
        std::cout << "Motor " << motor_id << " does not support KP modification." << std::endl;
        return;
    }
    Motor_Control control;
    Motor_Information state;
    control.Position = init_pos;
    control.Speed = 0;
    control.Torque = 0;
    control.KD = motor_kd[motor_id];

    float kp_default = motor_kp[motor_id];
    float kp_max = kp_default * kp_max_coeff;
    float kp_min = kp_default * kp_min_coeff;

    // 1. 默认 -> 最大
    for (int i = 0; i <= steps; ++i) {
        control.KP = kp_default + (kp_max - kp_default) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 2. 最大 -> 默认
    for (int i = 0; i <= steps; ++i) {
        control.KP = kp_max - (kp_max - kp_default) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 3. 默认 -> 最小
    for (int i = 0; i <= steps; ++i) {
        control.KP = kp_default - (kp_default - kp_min) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 4. 最小 -> 默认
    for (int i = 0; i <= steps; ++i) {
        control.KP = kp_min + (kp_default - kp_min) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    robot.getMotorState(motor_id, state);
    std::cout << "Motor " << motor_id << " KP test finished. Final KP=" << control.KP << std::endl;
}

// KD 浮动 默认 -> 最大 -> 默认 -> 最小 -> 默认
void TestMotorKD(H1Robot& robot, EtherCAT_Motor_Index motor_id, float init_pos, int steps = 10, float kd_min_coeff = 0.8f, float kd_max_coeff = 1.2f) {
    if (motor_kd[motor_id] == 0) {
        std::cout << "Motor " << motor_id << " does not support KD modification." << std::endl;
        return;
    }
    Motor_Control control;
    Motor_Information state;
    control.Position = init_pos;
    control.Speed = 0;
    control.Torque = 0;
    control.KP = motor_kp[motor_id];

    float kd_default = motor_kd[motor_id];
    float kd_max = kd_default * kd_max_coeff;
    float kd_min = kd_default * kd_min_coeff;

    // 1. 默认 -> 最大
    for (int i = 0; i <= steps; ++i) {
        control.KD = kd_default + (kd_max - kd_default) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 2. 最大 -> 默认
    for (int i = 0; i <= steps; ++i) {
        control.KD = kd_max - (kd_max - kd_default) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 3. 默认 -> 最小
    for (int i = 0; i <= steps; ++i) {
        control.KD = kd_default - (kd_default - kd_min) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 4. 最小 -> 默认
    for (int i = 0; i <= steps; ++i) {
        control.KD = kd_min + (kd_default - kd_min) * i / steps;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    robot.getMotorState(motor_id, state);
    std::cout << "Motor " << motor_id << " KD test finished. Final KD=" << control.KD << std::endl;
}


void TestHeadKP(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> head_ids = {MOTOR_HEAD_DOWN, MOTOR_HEAD_UP};
    for (auto id : head_ids) {
        std::cout << "Testing Head KP for motor " << id << std::endl;
    TestMotorKP(robot, id, 0.0f, steps, 0.0f, 1.2f); // 示例：头部KP范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}
void TestHeadKD(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> head_ids = {MOTOR_HEAD_DOWN, MOTOR_HEAD_UP};
    for (auto id : head_ids) {
        std::cout << "Testing Head KD for motor " << id << std::endl;
    TestMotorKD(robot, id, 0.0f, steps, 0.0f, 1.2f); // 示例：头部KD范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}

void TestArmKP(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> arm_ids = {
        MOTOR_LEFT_ARM_1, MOTOR_LEFT_ARM_2, MOTOR_LEFT_ARM_3, MOTOR_LEFT_ARM_4,
        MOTOR_LEFT_ARM_5, MOTOR_LEFT_ARM_6, MOTOR_LEFT_ARM_7, 
        MOTOR_RIGHT_ARM_1, MOTOR_RIGHT_ARM_2, MOTOR_RIGHT_ARM_3, MOTOR_RIGHT_ARM_4,
        MOTOR_RIGHT_ARM_5, MOTOR_RIGHT_ARM_6, MOTOR_RIGHT_ARM_7
    };
    for (auto id : arm_ids) {
        std::cout << "Testing Arm KP for motor " << id << std::endl;
    TestMotorKP(robot, id, 0.0f, steps, 0.3f, 1.2f); // 示例：手臂KP范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}
void TestArmKD(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> arm_ids = {
        MOTOR_LEFT_ARM_1, MOTOR_LEFT_ARM_2, MOTOR_LEFT_ARM_3, MOTOR_LEFT_ARM_4,
        MOTOR_LEFT_ARM_5, MOTOR_LEFT_ARM_6, MOTOR_LEFT_ARM_7, 
        MOTOR_RIGHT_ARM_1, MOTOR_RIGHT_ARM_2, MOTOR_RIGHT_ARM_3, MOTOR_RIGHT_ARM_4,
        MOTOR_RIGHT_ARM_5, MOTOR_RIGHT_ARM_6, MOTOR_RIGHT_ARM_7
    };
    for (auto id : arm_ids) {
        std::cout << "Testing Arm KD for motor " << id << std::endl;
    TestMotorKD(robot, id, 0.0f, steps, 0.3f, 1.2f); // 示例：手臂KD范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}

void TestGripperKP(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> gripper_ids = {MOTOR_LEFT_ARM_8, MOTOR_RIGHT_ARM_8};
    for (auto id : gripper_ids) {
        std::cout << "Testing Gripper KP for motor " << id << std::endl;
    TestMotorKP(robot, id, 0.0f, steps, 0.0f, 1.2f); // 示例：手爪KP范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}
void TestGripperKD(H1Robot& robot, int steps = 10) {
    std::vector<EtherCAT_Motor_Index> gripper_ids = {MOTOR_LEFT_ARM_8, MOTOR_RIGHT_ARM_8};
    for (auto id : gripper_ids) {
        std::cout << "Testing Gripper KD for motor " << id << std::endl;
    TestMotorKD(robot, id, 0.0f, steps, 0.0f, 1.2f); // 示例：手爪KD范围
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
}



int main() {

    std::cout << std::fixed << std::setprecision(6);// 设置std::cout输出精度为6位小数


    // H1Robot robot("172.31.200.1");//SDK运行环境为用户主机，有线连接方式
    // H1Robot robot("192.168.2.165");//SDK运行环境为用户主机，有线连接方式
    H1Robot robot;       //SDK运行环境为机器人主机

    // 尝试连接机器人
    if(!robot.robot_connect()) { 
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }

    //选择低层控制模式
    if(!robot.switchControlMode(MotorControlMode::LOW_LEVEL)) {
        std::cerr << "Failed to switch to LOW_LEVEL mode." << std::endl;
        return 1;
    }

    // 尝试初始化机器人（动作）
    if(robot.robot_init()) {
        std::cout << "Robot initialized successfully." << std::endl;
    } else {
        std::cerr << "Robot initialization failed." << std::endl;
        return 1;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // 创建并启动电机状态监控
    MotorStateMonitor monitor(robot);
    monitor.start();

    
    // 只测试某一部位
    // TestHeadKP(robot);
    // TestHeadKD(robot);

    // TestArmKP(robot);
    // TestArmKD(robot);

    // TestGripperKP(robot);
    // TestGripperKD(robot);


    std::this_thread::sleep_for(std::chrono::milliseconds(30000));

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