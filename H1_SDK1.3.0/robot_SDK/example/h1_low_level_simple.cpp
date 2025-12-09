#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


int rate = 500; // 1秒钟1000次循环

int time_sleep = 1000*1000 / rate; // 每次循环的时间间隔（微秒）

//底盘控制
//底盘电机仅支持速度控制
//左轮ID0，右轮ID1
void ChassisTest(H1Robot& robot) {
    Motor_Control control, control1, control2;
    Motor_Information state;

    // =============================================底盘 ，左右轮共同前进 =======================================
    control.Speed = 1; 

    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT << " Left Wheel State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT << " Right Wheel State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;


    // 停止
    control2.Speed = 0; // 停止
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control2);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    
    // =============================================底盘 ，左右轮共同后退 =======================================
    control1.Speed =-1;

    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control1);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control1);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT << " Left Wheel State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT << " Right Wheel State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;

    // 停止
    control2.Speed = 0; // 停止
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control2);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control2);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}


void WaistTest(H1Robot& robot) {
    Motor_Control control,control1,control2;
    Motor_Information state;

    //============================================腰部升降 ,0~0.8 =======================================
     // 升降仅支持位置模式
    control.Position = 0.5; //升高
    // control.Speed = 0;
    // control.Torque = 0;
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::cout << EtherCAT_Motor_Index::MOTOR_LIFT << " Lift State: Position=" << state.Position_Actual 
            << ", Speed=" << state.Speed_Actual 
            << ", Torque=" << state.Torque_Actual << std::endl;

    control.Position = 0;  //降到

    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::cout << EtherCAT_Motor_Index::MOTOR_LIFT << " Lift State: Position=" << state.Position_Actual 
        << ", Speed=" << state.Speed_Actual 
        << ", Torque=" << state.Torque_Actual << std::endl;

    //============================================腰部俯仰 ,0~1.48 =======================================

    //俯
    control1.Position = 0; 
    control1.Speed = 0;
    control1.Torque = 0;
    for(int i = 0; i < 1000; ++i) {
        control1.Position += 0.0002; //0->0.2  #这里过大可能会导致手臂触地
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WAIST_DOWN, control1);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WAIST_DOWN, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WAIST_DOWN << " Waist Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //仰
    for(int i = 0; i < 1000; ++i) {
        control1.Position -= 0.0005; //0.5->0
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WAIST_DOWN, control1);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WAIST_DOWN, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WAIST_DOWN << " Waist Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    //============================================腰部旋转 ,-0.7~0.7 =======================================

    //0->左
    control2.Position = 0; 
    control2.Speed = 0;
    control2.Torque = 0;
    for (int i = 0; i < 500; i++)
    {
        control2.Position += 0.0005; //0->0.25

        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WAIST_UP, control2);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WAIST_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WAIST_UP << " Waist up State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    //左->右
    for (int i = 0; i < 1000; i++)
    {
        control2.Position -= 0.0005; //0.25->-0.25
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WAIST_UP, control2);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WAIST_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WAIST_UP << " Waist Up State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    //右->0
    for (int i = 0; i < 500; i++)
    {
        control2.Position += 0.0005; //-0.25->0
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WAIST_UP, control2);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_WAIST_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_WAIST_UP << " Waist Up State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    //=============================================腰部旋转测试结束 =======================================
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
}

void HeadTest(H1Robot& robot) {
    Motor_Control control,control1,control2;
    Motor_Information state;
    //============================================头部俯仰 ,-0.175~0.262 =======================================
    control.Position = 0; 
    control.Speed = 0;
    control.Torque = 0;
    for(int i = 0; i < 1000; ++i) {
        control.Position += 0.0002; //0->0.2
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); //头部上
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_UP << " Up State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    for(int i = 0; i < 1800; ++i) {
        control.Position -= 0.0002; //0.2->-0.16
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); //头部上
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_DOWN << " Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;

    for(int i = 0; i < 1000; ++i) {
        control.Position += 0.0002; //-0.2->0
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); //头部上   
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_UP, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_UP << " Up State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
        
   //=============================================头部旋转 ,-1.57~1.57 =======================================

    // 头部旋转
    control1.Position = 0; 
    control1.Speed = 0;
    control1.Torque = 0;
    for(int i = 0; i < 1000; ++i) {
        control1.Position += 0.001; //0->1
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); //头部下
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_DOWN << " Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;

    for(int i = 0; i < 2000; ++i) {
        control1.Position -= 0.001; //1->-1
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); //头部下
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_DOWN << " Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;
    for(int i = 0; i < 1000; ++i) {
        control1.Position += 0.001; //-1->0
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); //头部下
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, state);
    std::cout << EtherCAT_Motor_Index::MOTOR_HEAD_DOWN << " Down State: Position=" << state.Position_Actual 
              << ", Speed=" << state.Speed_Actual 
              << ", Torque=" << state.Torque_Actual << std::endl;


    
    //=============================================头部测试结束 =======================================
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    
}


//手臂测试建议采用微分的形式，否则会导致控制不稳定
//要改的只有arm的电机编号和控制参数

void ArmTest(H1Robot& robot){
    Motor_Control  control, control1,control2;
    Motor_Information  state, state1 ,state2;
    //============================================左右臂肘部  =======================================
    control.Position = 0; 
    control.Speed = 0;
    control.Torque = 0;
    
    for(int i = 0; i < 500; ++i) {
        control.Position -= 0.001;     //0->-0.5
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, control); //手腕    
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, state1);
    std::cout << EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7 << " Up State: Position=" << state1.Position_Actual 
              << ", Speed=" << state1.Speed_Actual 
              << ", Torque=" << state1.Torque_Actual << std::endl;
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, state2);
    std::cout << EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7 << " Up State: Position=" << state2.Position_Actual 
              << ", Speed=" << state2.Speed_Actual 
              << ", Torque=" << state2.Torque_Actual << std::endl;

    for(int i = 0; i < 1000; ++i) {
        control.Position += 0.001; //-0.5->0.5
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, control); //手腕
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, state1);
    std::cout << EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7 << " Up State: Position=" << state1.Position_Actual 
              << ", Speed=" << state1.Speed_Actual 
              << ", Torque=" << state1.Torque_Actual << std::endl;
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, state2);
    std::cout << EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7 << " Up State: Position=" << state2.Position_Actual 
              << ", Speed=" << state2.Speed_Actual 
              << ", Torque=" << state2.Torque_Actual << std::endl;
    for(int i = 0; i < 500; ++i) {
        control.Position -= 0.001; //0.5->0
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, control); //手腕
        robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7, state1);
    std::cout << EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7 << " Up State: Position=" << state1.Position_Actual 
              << ", Speed=" << state1.Speed_Actual 
              << ", Torque=" << state1.Torque_Actual << std::endl;
    robot.getMotorState(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7, state2);
    std::cout << EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7 << " Up State: Position=" << state2.Position_Actual 
              << ", Speed=" << state2.Speed_Actual 
              << ", Torque=" << state2.Torque_Actual << std::endl;

}

void GripperTest(H1Robot& robot) {
        Motor_Control  control1, control2;
        Motor_Information  state1 ,state2;
        
        control1.Position = 0; 
        control1.Speed = 0;
        control1.Torque = 0;
        
        control2.Position = 1; 
        control2.Speed = 0;
        control2.Torque = 0;

        //============================================左右臂手腕  =======================================



        for(int i = 0; i < 5; ++i) {

            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control1);
            
            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control1);
    
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            
            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control2);
            
            robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control2);

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        }

        // robot.getMotorState(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, state1);
        // std::cout << EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8 << " Up State: Position=" << state1.Position_Actual 
        //           << ", Speed=" << state1.Speed_Actual 
        //           << ", Torque=" << state1.Torque_Actual << std::endl;
        // robot.getMotorState(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, state2);
        // std::cout << EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8 << " Up State: Position=" << state2.Position_Actual 
        //           << ", Speed=" << state2.Speed_Actual 
        //           << ", Torque=" << state2.Torque_Actual << std::endl;


        // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

        //         robot.getMotorState(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, state1);
        // std::cout << EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8 << " Up State: Position=" << state1.Position_Actual 
        //           << ", Speed=" << state1.Speed_Actual 
        //           << ", Torque=" << state1.Torque_Actual << std::endl;
        // robot.getMotorState(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, state2);
        // std::cout << EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8 << " Up State: Position=" << state2.Position_Actual 
        //           << ", Speed=" << state2.Speed_Actual 
        //           << ", Torque=" << state2.Torque_Actual << std::endl;
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


    //开始执行控制，控制的起点必须是初始化后的位姿
    int k=0;
    while(true) {       
        if(k >=1) {
            break; // 测试k次后退出
        }

        std::cout << "Test iteration: " << k + 1 << std::endl;
        ChassisTest(robot); // 测试底盘
        WaistTest(robot); // 测试腰部
        HeadTest(robot); // 测试头部
        ArmTest(robot); // 测试手臂  
        GripperTest(robot); // 测试手爪
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