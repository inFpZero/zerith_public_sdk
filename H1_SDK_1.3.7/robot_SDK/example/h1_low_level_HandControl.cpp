#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

void HandControlTest1(H1Robot& robot) {
    HandControl control1, control2;
    
    // 控制值
    // 左手       
    // 模式1：位置速度控制    
    control1.hand_mode = 1;             
    // 左手关节1~6的逻辑位置(0~65535)
    control1.hand_control[0] = 0;       // 大拇指弯曲  
    control1.hand_control[1] = 65535;   // 食指
    control1.hand_control[2] = 65535;   // 中指
    control1.hand_control[3] = 65535;   // 无名指
    control1.hand_control[4] = 65535;   // 小拇指
    control1.hand_control[5] = 0;       // 大拇指旋转
    // 左手关节1~6的逻辑速度(0~65535)       
    control1.hand_speed[0] = 65535;     // 大拇指弯曲  
    control1.hand_speed[1] = 65535;     // 食指
    control1.hand_speed[2] = 65535;     // 中指
    control1.hand_speed[3] = 65535;     // 无名指
    control1.hand_speed[4] = 65535;     // 小拇指
    control1.hand_speed[5] = 65535;     // 大拇指旋转
    
    // 右手  
    // 模式2：角度速度控制      
    control2.hand_mode = 2;      
    // 右手关节1~6的角度位置(实际角度*100)
    control2.hand_control[0] = 3676;    // 大拇指弯曲 (226~3676)      
    control2.hand_control[1] = 10022;   // 食指 (10022~17837)
    control2.hand_control[2] = 9781;    // 中指 (9781~17606)
    control2.hand_control[3] = 10138;   // 无名指 (10138~17654)
    control2.hand_control[4] = 9884;    // 小拇指 (9884~17486)
    control2.hand_control[5] = 0;    // 大拇指旋转 (0~9000)
    // 右手关节1~6的逻辑速度(0~65535)       
    control2.hand_speed[0] = 65535;     // 大拇指弯曲    
    control2.hand_speed[1] = 65535;     // 食指
    control2.hand_speed[2] = 65535;     // 中指
    control2.hand_speed[3] = 65535;     // 无名指
    control2.hand_speed[4] = 65535;     // 小拇指
    control2.hand_speed[5] = 65535;     // 大拇指旋转

    robot.setHand_low(ArmAction::LEFT_ARM, control1);
    robot.setHand_low(ArmAction::RIGHT_ARM, control2);
}

void HandControlTest2(H1Robot& robot) {
    HandControl control1, control2;
    
    // 控制值
    // 左手       
    // 模式1：位置速度控制    
    control1.hand_mode = 1;             
    // 左手关节1~6的逻辑位置(0~65535)
    control1.hand_control[0] = 0;       // 大拇指弯曲  
    control1.hand_control[1] = 0;       // 食指
    control1.hand_control[2] = 0;       // 中指
    control1.hand_control[3] = 0;       // 无名指
    control1.hand_control[4] = 0;       // 小拇指
    control1.hand_control[5] = 0;       // 大拇指旋转
    // 左手关节1~6的逻辑速度(0~65535)       
    control1.hand_speed[0] = 65535;     // 大拇指弯曲  
    control1.hand_speed[1] = 65535;     // 食指
    control1.hand_speed[2] = 65535;     // 中指
    control1.hand_speed[3] = 65535;     // 无名指
    control1.hand_speed[4] = 65535;     // 小拇指
    control1.hand_speed[5] = 65535;         // 大拇指旋转
    
    // 右手  
    // 模式2：角度速度控制      
    control2.hand_mode = 2;      
    // 右手关节1~6的角度位置(实际角度*100)
    control2.hand_control[0] = 3676;    // 大拇指弯曲 (226~3676)      
    control2.hand_control[1] = 17837;   // 食指 (10022~17837)
    control2.hand_control[2] = 17606;   // 中指 (9781~17606)
    control2.hand_control[3] = 17654;   // 无名指 (10138~17654)
    control2.hand_control[4] = 17486;   // 小拇指 (9884~17486)
    control2.hand_control[5] = 0;       // 大拇指旋转 (0~9000)
    // 右手关节1~6的逻辑速度(0~65535)       
    control2.hand_speed[0] = 65535;     // 大拇指弯曲    
    control2.hand_speed[1] = 65535;     // 食指
    control2.hand_speed[2] = 65535;     // 中指
    control2.hand_speed[3] = 65535;     // 无名指
    control2.hand_speed[4] = 65535;     // 小拇指
    control2.hand_speed[5] = 65535;     // 大拇指旋转

    robot.setHand_low(ArmAction::LEFT_ARM, control1);
    robot.setHand_low(ArmAction::RIGHT_ARM, control2);
}

void HandStateTest(H1Robot& robot) {
    HandState state1, state2;
    robot.getHandState(ArmAction::LEFT_ARM, state1);
    robot.getHandState(ArmAction::RIGHT_ARM, state2);

    //打印
    printf("STATE:%d,%d,%d,%d\n", state1.hand_type, state1.error_flag, state2.hand_type, state2.error_flag);
    for(int i=0; i<6; i++)
    {
        printf("L%d:%d\t%d\t%d\t%d\n", i, state1.hand_position[i], state1.hand_angle[i], state1.hand_current[i], state1.hand_status[i]);
        printf("R%d:%d\t%d\t%d\t%d\n", i, state2.hand_position[i], state2.hand_angle[i], state2.hand_current[i], state2.hand_status[i]);
    }
}

int main() {

    std::cout << std::fixed << std::setprecision(6);// 设置std::cout输出精度为6位小数

    //实例化机器人
    // H1Robot robot("192.168.2.199");//SDK运行环境为用户主机，无线连接方式，ip不固定
    // H1Robot robot("172.31.200.1");//SDK运行环境为用户主机，有线连接方式，ip固定
    H1Robot robot;       //SDK运行环境为机器人主机

    // 尝试连接机器人
    if(!robot.robot_connect()) { 
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }

    //选择低层控制模式
    //底层控制模式和重力补偿模式任选其一，都可以使用low level接口，区别是底层控制模式下电机力矩控制不受限，重力补偿模式下电机力矩控制受限，且会自动补偿重力影响
    if(!robot.switchControlMode(MotorControlMode::LOW_LEVEL)) {
    // if(!robot.switchControlMode(MotorControlMode::GRAVITY_COMPENSATION_LEVEL)) {
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

    //开始执行控制，控制的起点必须是初始化后的位姿
    int k=0;
    while(true) {       
        if(k >=10) {
            break; // 测试k次后退出
        }

        std::cout << "Test iteration: " << k + 1 << std::endl;
        HandControlTest1(robot); // 测试灵巧手控制-闭合
        HandStateTest(robot);    // 测试灵巧手状态读取,可以单开线程进行读取
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        HandControlTest2(robot); // 测试灵巧手控制-张开
        HandStateTest(robot);    // 测试灵巧手状态读取,可以单开线程进行读取
        k++;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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