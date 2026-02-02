#include "H1_Robot.hpp"
#include "MotorStateMonitor.hpp" 
#include <iostream>
#include <thread>
#include <chrono>



int rate = 500; // 1秒钟1000次循环

int time_sleep = 1000*1000 / rate; // 每次循环的时间间隔（微秒）
//===========================================================测试值修改区域=====================================================
//底盘参数修改
//（advance，retreat）线速度，控制前后，-1~1m/s， （left，right）旋转调节，角速度，控制左右，-6.28~6.28rad/s
float advance = 0.2; //底盘前进速度 m/s
float retreat = -0.2; //底盘旋转速度 rad/s
float stop = 0; //底盘停止速度 m/s
float left =0.1; //左转修改值
float right =-0.1; //右转修改值
//==============================================================腰部参数配置=====================================================
//腰部升降配置
float lift_up = 0.3; //升降上升位置 0~0.8m
//滑轨电机下降
float lift_down = 0; //升降下降位置 0~0.8m
//腰部俯仰缓冲值
float waist_pitch_buffer = 0.0005; //腰部俯仰缓冲值
//腰部旋转缓冲值
float waist_yaw_buffer = 0.0002; //腰部旋转缓冲值
//==============================================================手臂参数配置=======================================================
float x=0.0001;
float y=0.0001; //手臂x,y缓冲值
float z=0.0001; //手臂z缓冲值
float roll=0.0010; //手臂roll缓冲值
float pitch=0.0010; //手臂pitch缓冲值
float yaw=0.00010; //手臂yaw缓冲值
//夹爪参数配置
float gripper_open = 0; //夹爪张开位置 0~1.45
float gripper_close =1.45; //夹爪闭合位置 0~1.45

//==============================================================头部参数配置=======================================================
//头部俯仰缓冲值
float head_pitch_buffer = 0.0002; //头部俯仰缓冲值
//头部旋转缓冲值
float head_yaw_buffer = 0.001; //头部旋转缓冲值

//=============================================================测试值修改区域=======================================================

void ChassisTest_advance(H1Robot& robot){

    std::cout << "Start ChassisTest_advance" << std::endl;

    float speed_x ;
    float speed_y ;
    // =============================================底盘 ，左右轮共同前进和后退 =======================================
    //参数1线速度，控制前后，-1~1m/s，参数2，角速度，控制左右，-6.28~6.28rad/s

    // 前进
    speed_x = advance; 
    speed_y = stop;
    robot.setChassis_high(speed_x, speed_y);  
    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 

    // 停止
    speed_x = stop; 
    speed_y = stop;
    robot.setChassis_high(speed_x, speed_x);    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // 后退
    speed_x = retreat; 
    speed_y =stop;
    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // 停止
    speed_x = stop; 
    speed_y = stop;
    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    
}

void ChassisTest_turn(H1Robot& robot){

    std::cout << "Start ChassisTest_turn" << std::endl;

    float speed_x ;
    float speed_y ;
    speed_x = stop; 
    speed_y = left;

    // 左转
    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));   

    // 停止
    speed_x = stop; 
    speed_y = stop;

    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    //右转
    speed_x = stop;
    speed_y = right;

    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));


    // 停止
    speed_x = stop; 
    speed_y = stop;

    robot.setChassis_high(speed_x, speed_y);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

void WaistTest(H1Robot& robot) {

    std::cout << "Start WaistTest" << std::endl;

    ArmPose action;

    //============================================腰部升降=======================================
     // 升降仅支持位置模式
    // 腰部控制参数，采用和手臂相同的结构来控制
    action.x = 0; //x 无意义
    action.y = 0; //y 无意义
    action.z = 0; //z  升降电机高度
    action.roll = 0; //roll 无意义
    action.pitch = 0; //pitch
    action.yaw = 0; //yaw

    // //升高
    for(int i = 0; i < 2000; ++i) {
        action.z += 0.00005;  //这里过大可能会导致手臂触地
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    //俯
    for(int i = 0; i < 1000; ++i) {
        action.pitch += 0.0015;  //这里过大可能会导致手臂触地
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
    //0->左 旋转
    for (int i = 0; i < 500; i++)
    {
        action.yaw += 0.001; 

        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //左->右 旋转
    for (int i = 0; i < 1000; i++)
    {
        action.yaw -= 0.001; 
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //右->0 旋转
    for (int i = 0; i < 500; i++)
    {
        action.yaw += 0.001; 
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //仰
    for(int i = 0; i < 1000; ++i) {
        action.pitch -= 0.0015; 
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //降低
    for(int i = 0; i < 2000; ++i) {
        action.z -= 0.00005;  //这里过大可能会导致手臂触地
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    //=============================================腰部旋转测试结束=======================================
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
}

void WaistTest_rise(H1Robot& robot) {

    std::cout << "Start WaistTest_rise" << std::endl;

    ArmPose action;
        //滑轨电机上升
    action.x = 0; //x 无意义
    action.y = 0; //y 无意义
    action.z = lift_up; //升降上升位置 0~0.8m
    action.roll = 0; //roll 无意义
    action.pitch = 0; //pitch
    action.yaw = 0; //yaw

    //============================================腰部上升=======================================
     // 升降仅支持位置模式
    // 腰部控制参数，采用和手臂相同的结构来控制

    robot.setWaist_high(robot.armPoseToArmEndPose(action));
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
}

void WaistTest_descend(H1Robot& robot){

    std::cout << "Start WaistTest_descend" << std::endl;

    ArmPose action;
    action.x = 0; //x 无意义
    action.y = 0; //y 无意义
    action.z = lift_down; //升降上升位置 0~0.8m
    action.roll = 0; //roll 无意义
    action.pitch = 0; //pitch
    action.yaw = 0; //yaw

    //============================================腰部下降=======================================
     // 升降仅支持位置模式
    robot.setWaist_high(robot.armPoseToArmEndPose(action));
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));

}

void WaistTest_uplift(H1Robot& robot){

    std::cout << "Start WaistTest_uplift" << std::endl;

    ArmPose action; 
    action.x = 0; //x 无意义
    action.y = 0; //y 无意义
    action.z = 0; //升降上升位置 0~0.8m
    action.roll = 0; //roll 无意义
    action.pitch = 0; //pitch
    action.yaw = 0; //yaw

    for (int i = 0; i < 500; i++)
    {
        action.yaw += waist_yaw_buffer;

        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

}

void WaistTest_leftspin(H1Robot& robot){

    std::cout << "Start WaistTest_leftspin" << std::endl;

    ArmPose action;   
    action.x = 0; //x 无意义
    action.y = 0; //y 无意义
    action.z = 0; //升降上升位置 0~0.8m
    action.roll = 0; //roll 无意义
    action.pitch = 0; //pitch
    action.yaw = 0; //yaw

    //0->左
    for (int i = 0; i < 500; i++)
    {
        action.yaw += waist_yaw_buffer; 

        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //左->右
    for (int i = 0; i < 1000; i++)
    {
        action.yaw -= waist_yaw_buffer; 
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //右->0
    for (int i = 0; i < 500; i++)
    {
        action.yaw += waist_yaw_buffer; //-0.25->0
        robot.setWaist_high(robot.armPoseToArmEndPose(action));
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    //=============================================腰部旋转测试结束=======================================
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

}

void HeadTest(H1Robot& robot) {

    std::cout << "Start HeadTest" << std::endl;

    Motor_Control control,control1,control2;
    //============================================头部俯仰=======================================
    control.Position = 0;
    control.Speed = 0;
    control.Torque = 0;
    for(int i = 0; i < 1000; ++i) {
        control.Position += head_pitch_buffer; 
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(int i = 0; i < 2000; ++i) {
        control.Position -= head_pitch_buffer; 
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(int i = 0; i < 1000; ++i) {
        control.Position +=head_pitch_buffer; 
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_UP, control); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

   //=============================================头部旋转=======================================

    // 头部旋转
    control1.Position = 0;
    control1.Speed = 0;
    control1.Torque = 0;
    for(int i = 0; i < 1000; ++i) {
        control1.Position += head_yaw_buffer; 
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(int i = 0; i < 2000; ++i) {
        control1.Position -=head_yaw_buffer; 
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    for(int i = 0; i < 1000; ++i) {
        control1.Position += head_yaw_buffer;
        robot.setHead_high(EtherCAT_Motor_Index::MOTOR_HEAD_DOWN, control1); 
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    //=============================================头部测试结束=======================================
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

}

void ArmTest(H1Robot& robot){

    std::cout << "Start ArmTest" << std::endl;

    ArmPose control, control1;
    //============================================左臂=======================================
    control.x = 0; //x
    control.y = 0; //y
    control.z = 0; //z
    control.roll = 0; //row
    control.pitch = 0; //pitch
    control.yaw = 0; //yaw
    
    control1.x = 0; //x
    control1.y = 0; //y
    control1.z = 0; //z
    control1.roll = 0; //row
    control1.pitch = 0; //pitch
    control1.yaw = 0; //yaw

    for(int i = 0; i < 1000; ++i) {
        control.z += z;     
        control1.z +=z;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.x += x;    
        control1.x += x;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.y += y;  
        control1.y -=y;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }


    for(int i = 0; i < 1000; ++i) {
        control.roll += roll;     
        control1.roll -=roll;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.roll -= roll;     
        control1.roll +=roll;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.pitch += pitch;    
        control1.pitch +=pitch;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.pitch -= pitch; 
        control1.pitch -=pitch;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.yaw += yaw; //x      10cm
        control1.yaw -=yaw;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }
    for(int i = 0; i < 1000; ++i) {
        control.yaw -= yaw; //x      10cm
        control1.yaw +=yaw;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.y -= y;   
        control1.y +=y;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.x -= x;   
        control1.x -=x;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }

    for(int i = 0; i < 1000; ++i) {
        control.z -= z;     
        control1.z -=z;
        robot.setArm_high(ArmAction::LEFT_ARM, robot.armPoseToArmEndPose(control)); //左臂 
        robot.setArm_high(ArmAction::RIGHT_ARM, robot.armPoseToArmEndPose(control1)); //右臂
        std::this_thread::sleep_for(std::chrono::microseconds(time_sleep));
    }


    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
}

void GripperTest(H1Robot& robot) {

    std::cout << "Start GripperTest" << std::endl;

    Motor_Control  control1, control2;

    control1.Position = gripper_open;
    control1.Speed = 0;
    control1.Torque = 0;

    control2.Position =gripper_close;
    control2.Speed = 0;
    control2.Torque = 0;

    //============================================左右臂夹爪=======================================
    robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control1);
    robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control1);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_LEFT_ARM_8, control2);
    robot.setGripper_high(EtherCAT_Motor_Index::MOTOR_RIGHT_ARM_8, control2);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

}

void HighLevelControlTest_arm(H1Robot& robot) {

    std::cout << "Start HighLevelControlTest_arm" << std::endl;
    HighLevelControl control1;
    control1.control_type = 1; // 单次控制
    control1.control_part = 0; // 右手
    control1.control_coordinate = 0;     // 默认相对初始坐标系（相对电机零位）
    for (int i = 0; i < 3; ++i) {
        // zcm_data_.high_level_control.waist_position[i] = 0.0f;  //默认腰部为0
       control1.middle_position[i] = 0.0f; //默认moveL,非0时为moveC
    }
    ArmPose armPose{0.2, 0, 0, 0, 0, 0};
    auto quat = robot.armPoseToArmEndPose(armPose); // 默认姿态不变

     for(int i=0;i<7;i++){  
        if(i<3)
            control1.hand_pose[i]=quat.PositionData[i];
        else
            control1.hand_pose[i]=quat.RotationData[i-3];
    }

    // 默认单位四元数 (qw = 1)  
    control1.eef_velocity = 0.0f;        // 默认为0，使用默认速度
    control1.eef_acceleration = 0.0f;    // 默认为0，使用默认加速度
    control1.block = false;//true;              // 默认非阻塞
    control1.duration = 10.0f;            // 默认为0，自动计算运动时间

    robot.setHighLevelControl(control1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    //在此处添加复杂动作测试代码

}


void HighLevelControlTest_arm2(H1Robot& robot) {
    std::cout << "Start HighLevelControlTest_arm" << std::endl;
    
    // 定义多组ArmPose数据
    std::vector<ArmPose> armPoses = {
        {0.1, -0.1, 0.2, 0, 0, 0},    // 第一组数据
         {0.1, 0.1, 0.2, 0, 0, 0}, // 第二组数据
         {0.1, 0.2, 0.2, 0, 0, 0}, // 第三组数据
        {0, 0.2, 0, 0.2, 0, 0},
         {0.1, 0, 0.2, 0, 0, 0},  // 第四组数据
         {0.1, 0, 0.2, 0, 0, 0} , // 第四组数据
        //  {-0.1, 0.2, 0.2, 0, 0, 0}, // 第三组数据
        //   {-0.1, 0, 0.1, 0, 0, 0} , // 第四组数据
        //  {-0.1, 0, 0.2, 0, 0, 0}  // 第四组数据

        // 可以继续添加更多组数据
    };
    
    // 循环执行每一组ArmPose数据
    for (const auto& armPose : armPoses) {
        HighLevelControl control1;
        control1.control_type = 1; // 单次控制
        control1.control_part = 1; // 右手
        control1.control_coordinate = 0; // 默认相对初始坐标系
        
        for (int i = 0; i < 3; ++i) {
            control1.middle_position[i] = 0.0f; // 默认moveL,非0时为moveC
        }
        
        auto quat = robot.armPoseToArmEndPose(armPose); // 转换位姿
        
        for(int i = 0; i < 7; i++){  
            if(i < 3)
                control1.hand_pose[i] = quat.PositionData[i];
            else
                control1.hand_pose[i] = quat.RotationData[i-3];
        }
        
        control1.eef_velocity = 0.0f;
        control1.eef_acceleration = 0.0f;
        control1.block = true;//true;
        control1.duration = 0.0f;
        
        std::cout << "执行ArmPose: {" << armPose.x << ", " << armPose.y << ", " 
                  << armPose.z << ", " << armPose.roll << ", " << armPose.pitch 
                  << ", " << armPose.yaw << "}" << std::endl;
        
        robot.setHighLevelControl(control1);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }
    
    std::cout << "所有ArmPose执行完成" << std::endl;
}



void HighLevelControlTest_arm1(H1Robot& robot) {

    std::cout << "Start HighLevelControlTest_arm" << std::endl;
    HighLevelControl control1;
    control1.control_type = 1; // 单次控制
    control1.control_part = 1; // 右手
    control1.control_coordinate = 0;     // 默认相对初始坐标系（相对电机零位）
    for (int i = 0; i < 3; ++i) {
        // zcm_data_.high_level_control.waist_position[i] = 0.0f;  //默认腰部为0
       control1.middle_position[i] = 0.0f; //默认moveL,非0时为moveC
    }
    ArmPose armPose{0, -0.1, 0, 0, 0, 0};
    auto quat = robot.armPoseToArmEndPose(armPose); // 默认姿态不变

     for(int i=0;i<7;i++){  
        if(i<3)
            control1.hand_pose[i]=quat.PositionData[i];
        else
            control1.hand_pose[i]=quat.RotationData[i-3];
    }

    // 默认单位四元数 (qw = 1)  
    control1.eef_velocity = 0.0f;        // 默认为0，使用默认速度
    control1.eef_acceleration = 0.0f;    // 默认为0，使用默认加速度
    control1.block = true;              // 默认非阻塞
    control1.duration = 0.0f;            // 默认为0，自动计算运动时间

    robot.setHighLevelControl(control1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    //在此处添加复杂动作测试代码

}

void HighLevelControlTest_waist(H1Robot& robot) {

    std::cout << "Start HighLevelControlTest_waist" << std::endl;
    HighLevelControl control1;
    control1.control_type = 1; // 单次控制
    control1.control_part = 2; // 腰
    control1.control_coordinate = 0;     // 默认相对初始坐标系（相对电机零位）
    for (int i = 0; i < 3; ++i) {
        // zcm_data_.high_level_control.waist_position[i] = 0.0f;  //默认腰部为0
       control1.middle_position[i] = 0.0f; //默认moveL,非0时为moveC
    }
    control1.waist_position[0] = 0.2f; // z
    control1.waist_position[1] = 0.1f; // pitch
    control1.waist_position[2] = 0.0f; // yaw

    // 默认单位四元数 (qw = 1)  
    control1.eef_velocity = 0.0f;        // 默认为0，使用默认速度
    control1.eef_acceleration = 0.0f;    // 默认为0，使用默认加速度
    control1.block =true;               // 默认非阻塞
    control1.duration = 0.0f;            // 默认为0，自动计算运动时间

    robot.setHighLevelControl(control1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

}

int main() {
    
    std::cout << std::fixed << std::setprecision(6);// 设置std::cout输出精度为6位小数

    //实例化机器人
    // H1Robot robot("192.168.3.43");//SDK运行环境为用户主机，无线连接方式，ip不固定
    // H1Robot robot("172.31.200.1");//SDK运行环境为用户主机，有线连接方式，ip固定
    H1Robot robot;       //SDK运行环境为机器人主机

    // 创建并启动电机状态监控
    MotorStateMonitor monitor(robot);
    monitor.start();

    if(!robot.robot_connect()) { // 尝试连接机器人
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }

        //选择高层控制模式
    if(!robot.switchControlMode(MotorControlMode::HIGH_LEVEL)) {
        std::cerr << "Failed to switch to HIGH_LEVEL mode." << std::endl;
        return 1;
    }

    if(robot.robot_init()) {// 尝试初始化机器人（动作）
        std::cout << "Robot initialized successfully." << std::endl;
    } else {
        std::cerr << "Robot initialization failed." << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    int k=0;

    while(true) {

        if(k >=1) {
            break; 
        }
        // 对除夹爪和升降电机外的电机的控制建议采用微分插值的形式传递目标数据，否则会导致控制不稳定
        // ChassisTest_advance(robot); //底盘前后
        // ChassisTest_turn(robot);//底盘转向
        // WaistTest(robot);//腰部三个自由度测试
        // HeadTest(robot);//头部测试
        // ArmTest(robot);//手臂控制
        // GripperTest(robot);//夹爪控制

        // HighLevelControlTest_arm(robot);//高层控制测试
        // WaistTest_rise(robot);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // HighLevelControlTest_arm2(robot);

        // HighLevelControlTest_arm(robot);

        std::cout << "Test iteration: " << k + 1 << std::endl;
        k++;
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    if(robot.robot_deinit()) {// 尝试初始化机器人（动作）
        std::cout << "Robot deinitialized successfully." << std::endl;
    } else {
        std::cerr << "Robot deinitialization failed." << std::endl;
        return 1;
    }

    return 0;
}