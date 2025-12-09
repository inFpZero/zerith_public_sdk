#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>

//此demo为机器人极限位置测试，需谨慎使用
//适用软件版本：v1.0.0

int rate = 500; // 1秒钟1000次循环

int time_sleep = 1000*1000 / rate; // 每次循环的时间间隔（微秒）

int action_time = 3000;      //单个电机动作时间间隔，单位ms

int action_frequency = action_time*rate/1000; //单个电机动作的循环插值次数

const float motor_limit_max[23] = {0, 0, 0.8, 1.3,  0.7,   1.5,  0.25,  1.5,  2.0,  2.9,  1.5,  2.9,  1.0,  1.5,  1.5,  1.5,  0.3,  2.9,  1.5,  2.9,  1.0,  1.5, 1.5};
const float motor_limit_min[23] = {0, 0, 0,   0,   -0.7,  -1.5, -0.17, -2.7, -0.3, -2.9, -1.3, -2.9, -1.0, -1.5,  0,   -2.7, -2.0, -2.9, -1.3, -2.9, -1.0, -1.5, 0  };

void MoveZeroToMax(H1Robot& robot, EtherCAT_Motor_Index motor_id) {
    Motor_Control control;
    control.Position = 0;
    control.Speed = 0;
    control.Torque = 0;
    float max_limit = motor_limit_max[motor_id];
    for (int i = 0; i < action_frequency; ++i) {
        control.Position += max_limit / action_frequency;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MoveMaxToZero(H1Robot& robot, EtherCAT_Motor_Index motor_id) {
    Motor_Control control;
    control.Position = motor_limit_max[motor_id];
    control.Speed = 0;
    control.Torque = 0;
    float max_limit = motor_limit_max[motor_id];
    for (int i = 0; i < action_frequency; ++i) {
        control.Position -= max_limit / action_frequency;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MoveZeroToMin(H1Robot& robot, EtherCAT_Motor_Index motor_id) {
    Motor_Control control;
    control.Position = 0;
    control.Speed = 0;
    control.Torque = 0;
    float min_limit = motor_limit_min[motor_id];
    for (int i = 0; i < action_frequency; ++i) {
        control.Position += min_limit / action_frequency;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MoveMinToZero(H1Robot& robot, EtherCAT_Motor_Index motor_id) {
    Motor_Control control;
    control.Position = motor_limit_min[motor_id];
    control.Speed = 0;
    control.Torque = 0;
    float min_limit = motor_limit_min[motor_id];
    for (int i = 0; i < action_frequency; ++i) {
        control.Position -= min_limit / action_frequency;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void MovePointAToB(H1Robot& robot, EtherCAT_Motor_Index motor_id , float point_a, float point_b) {
    Motor_Control control;
    control.Position = point_a;
    control.Speed = 0;
    control.Torque = 0;
    float distance = point_b - point_a;    
    for (int i = 0; i < action_frequency; ++i) {
        control.Position += distance / action_frequency;
        robot.setMotorControl_low(motor_id, control);
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}
//底盘控制
//底盘电机仅支持速度控制
//左轮ID0，右轮ID1
void ChassisTest(H1Robot& robot) {
    std::cout << "Start ChassisTest." << std::endl;
    Motor_Control control, control1, control2;
    // =============================================底盘 ，左右轮共同前进 =======================================
    control.Speed = 1; 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
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
    // 停止
    control2.Speed = 0; // 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control2);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // =============================================底盘，左转 =======================================
    control.Speed = 1; 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
    // 停止
    control2.Speed = 0; 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_LEFT, control2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // =============================================底盘 ，右转 =======================================
    control.Speed = 1; 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
    // 停止
    control2.Speed = 0; // 停止
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_WHEEL_RIGHT, control2);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void WaistTest_lift_up(H1Robot& robot){
    std::cout << "Start WaistTest_lift_up." << std::endl;
    Motor_Control control,control1,control2;
    //============================================腰部升降=======================================
    // 升降仅支持位置模式
    control.Position = motor_limit_max[EtherCAT_Motor_Index::MOTOR_LIFT]; //升高
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(8000));
    //============================================腰部升降=======================================
    // 升降仅支持位置模式
    control.Position = motor_limit_max[EtherCAT_Motor_Index::MOTOR_LIFT]-0.2; //下降
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void WaistTest_lift_down(H1Robot& robot){
    std::cout << "Start WaistTest_lift_down." << std::endl;
    Motor_Control control,control1,control2;
    //============================================腰部升降=======================================
     // 升降仅支持位置模式
    control.Position = motor_limit_min[EtherCAT_Motor_Index::MOTOR_LIFT]; //下降到0
    // control.Speed = 0;
    // control.Torque = 0;
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LIFT, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void WaistTest_bend(H1Robot& robot){
    std::cout << "Start WaistTest_bend." << std::endl;
    //俯
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_WAIST_DOWN);
}

void WaistTest_raiseup(H1Robot& robot){
    //仰
    std::cout << "Start WaistTest_raiseup." << std::endl;
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_WAIST_DOWN);
}

void WaistTest_turn(H1Robot& robot){
    std::cout << "Start WaistTest_turn." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_WAIST_UP);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_WAIST_UP);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_WAIST_UP);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_WAIST_UP);
}

void HeadTest(H1Robot& robot) {
    std::cout << "Start HeadTest." << std::endl;
    //俯仰
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_HEAD_UP);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_HEAD_UP);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_HEAD_UP);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_HEAD_UP);

    //旋转
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_HEAD_DOWN);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_HEAD_DOWN);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_HEAD_DOWN);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_HEAD_DOWN);
    
}

void ArmTest_all_arm1(H1Robot& robot) {
    std::cout << "Start ArmTest_shoulders." << std::endl;
    // 左右肩关节分别做四个动作
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_1);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_1);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_1);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_1);

    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_1);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_1);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_1);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_1);
}

void ArmTest_left_arm2(H1Robot& robot){
    std::cout << "Start ArmTest_left_arm2." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_2);
}

void ArmTest_right_arm2(H1Robot& robot){
    std::cout << "Start ArmTest_right_arm2." << std::endl;
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_2);
}

void ArmTest_left_arm2_restore(H1Robot& robot){
    std::cout << "Start ArmTest_left_arm2_restore." << std::endl;
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_2);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_2);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_2);
}

void ArmTest_right_arm2_restore(H1Robot& robot){
    std::cout << "Start ArmTest_right_arm2_restore." << std::endl;
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_2);
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_2);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_2);
}

void ArmTest_left_arm3(H1Robot& robot){
    std::cout << "Start ArmTest_left_arm3." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_3);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_3);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_3);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_3);
}

void ArmTest_right_arm3(H1Robot& robot){
    std::cout << "Start ArmTest_right_arm3." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_3);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_3);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_3);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_3);
}

void ArmTest_left_arm4(H1Robot& robot){
    std::cout << "Start ArmTest_left_arm4." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_4);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_4);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_4);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_4);
}

void ArmTest_right_arm4(H1Robot& robot){
    std::cout << "Start ArmTest_right_arm4." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_4);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_4);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_4);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_4);
}

void ArmTest_all_arm5(H1Robot& robot){
    std::cout << "Start ArmTest_all_arm5." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_5);
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_5);

    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_5);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_5);

    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_5);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_5);

    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_5);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_5);
}

void ArmTest_all_arm6(H1Robot& robot){
    std::cout << "Start ArmTest_all_arm6." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_6);
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_6);

    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_6);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_6);

    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_6);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_6);

    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_6);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_6);
}

void ArmTest_all_arm7(H1Robot& robot){
    std::cout << "Start ArmTest_all_arm7." << std::endl;
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7);
    MoveZeroToMax(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7);

    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7);
    MoveMaxToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7);

    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7);
    MoveZeroToMin(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7);

    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_LEFT_ARM_7);
    MoveMinToZero(robot, EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_7);
}

void GripperTest(H1Robot& robot) {
    std::cout << "Start GripperTest." << std::endl;
    Motor_Control  control;
    control.Position = 0; 
    control.Speed = 0;
    control.Torque = 0;
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    control.Position = 1.5; 
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control);
    robot.setMotorControl_low(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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

    // 创建并启动电机状态监控
    MotorStateMonitor monitor(robot);
    monitor.start();

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
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));


    //开始执行控制，控制的起点必须是初始化后的位姿
    int k=0;
    while(true) {
        //对除夹爪和升降电机外的电机的控制建议采用微分插值的形式传递目标数据，否则会导致控制不稳定   
        // ChassisTest(robot); // 测试底盘
        WaistTest_lift_up(robot);//腰部上升
        WaistTest_bend(robot);//下腰
        WaistTest_turn(robot);//腰部旋转
        WaistTest_raiseup(robot);//抬头
        ArmTest_left_arm2(robot);// 此处到arm7全部为手部运动
        ArmTest_right_arm2(robot);
        ArmTest_all_arm1(robot); // 测试肩部时需要arm2先打开，否则arm1的转动会剐蹭到立柱
        ArmTest_left_arm3(robot);
        ArmTest_right_arm3(robot);
        ArmTest_left_arm4(robot);
        ArmTest_right_arm4(robot);
        ArmTest_all_arm5(robot);
        ArmTest_all_arm6(robot);
        ArmTest_all_arm7(robot);
        ArmTest_left_arm2_restore(robot);//手臂测试完成后左手臂收回
        ArmTest_right_arm2_restore(robot);  //手臂测试完成后右手臂收回
        HeadTest(robot); // 测试头部
        GripperTest(robot); // 测试手爪
        WaistTest_lift_down(robot);//腰部下降
        k++;
        if(k >=1) {
            break; //k值为循环测试次数
        }   
    }

    // 尝试反初始化机器人（动作）
    if(robot.robot_deinit()) {
        std::cout << "Robot deinitialized successfully." << std::endl;
    } else {
        std::cerr << "Robot deinitialization failed." << std::endl;
        return 1;
    }

    return 0;
}