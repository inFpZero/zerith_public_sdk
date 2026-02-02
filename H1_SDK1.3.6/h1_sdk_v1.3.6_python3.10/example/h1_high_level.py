import os, sys, time

# 将 lib 目录加入 sys.path 以便导入编译出的 lib_h1_sdk_python.so
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 不要加入 lib 目录，而是加入 root 目录
if root not in sys.path:
    sys.path.insert(0, root)

from state_monitor import start_state_monitor 

from lib.lib_h1_sdk_python import (
    H1Robot,
    MotorControlMode,
    Motor_Control,
    EtherCAT_Motor_Index,
    ArmAction,       # 枚举: LEFT_ARM / RIGHT_ARM
    ArmPose,         # 三维数据+欧拉姿态输入
    ArmEndPose       # 三维数据+四元数姿态输入
)

RATE_HZ = 500
DT = 1.0 / RATE_HZ

def wait_mode(robot, target, timeout=5.0, interval=0.1):
    """
    轮询等待控制模式切换完成
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        if robot.getCurrentMode() == target:
            return True
        time.sleep(interval)
    return False

def ensure_high_level(robot):
    """
    确保进入 HIGH_LEVEL:
    若还在 UNINITIALIZED 则先初始化，再切换模式
    """
    if robot.getCurrentMode() == MotorControlMode.UNINITIALIZED:
        if hasattr(robot, "robot_init"):
            robot.robot_init()
            time.sleep(1.0)
    if robot.getCurrentMode() != MotorControlMode.HIGH_LEVEL:
        robot.switchControlMode(MotorControlMode.HIGH_LEVEL)
        wait_mode(robot, MotorControlMode.HIGH_LEVEL, 3)

# ---------------- 高层控制测试函数 ----------------

def chassis_test(robot):
    """
    高层底盘速度向量: (speed_x, speed_y/旋转)
    示例：前进 / 旋转 / 后退
    """
    print("[Chassis] forward")
    robot.setChassis_high(0.1, 0.0); time.sleep(2)
    robot.setChassis_high(0.0, 0.0); time.sleep(1)
    print("[Chassis] 0->left")
    robot.setChassis_high(0.0, 0.15); time.sleep(2)
    robot.setChassis_high(0.0, 0.0); time.sleep(1)
    print("[Chassis] left->right")
    robot.setChassis_high(0.0, -0.15); time.sleep(4)
    robot.setChassis_high(0.0, 0.0); time.sleep(1)
    print("[Chassis] right->0")
    robot.setChassis_high(0.0, 0.15); time.sleep(2)
    robot.setChassis_high(0.0, 0.0); time.sleep(1)
    print("[Chassis] backward")
    robot.setChassis_high(-0.1, 0.0); time.sleep(2)
    robot.setChassis_high(0.0, 0.0); time.sleep(1)

def waist_test(robot):
    """
    腰部高层测试：
    使用 ArmPose(x,y,z,roll,pitch,yaw) 转换为 ArmEndPose(位置+四元数)后调用 setWaist_high。
    腰部当前仅有效：
      z  -> 升降
      pitch -> 俯仰
      yaw -> 旋转
    其余保持 0。
    """
    waist_pose = ArmPose()
    waist_pose.x = waist_pose.y = 0.0
    waist_pose.z = 0.0
    waist_pose.roll = 0.0
    waist_pose.pitch = 0.0
    waist_pose.yaw = 0.0

    def send():
        end = robot.armPoseToArmEndPose(waist_pose)
        robot.setWaist_high(end)

    # 升降 z: 0 -> 0.4 -> 0
    print("[Waist] lift z 0->0.4")
    for _ in range(400):
        waist_pose.z += 0.001
        send(); time.sleep(DT)
    print("[Waist] lift z 0.4->0")
    for _ in range(400):
        waist_pose.z -= 0.001
        send(); time.sleep(DT)

    # 俯仰 pitch: 0 -> 0.3 -> 0
    print("[Waist] pitch 0->0.3")
    for _ in range(300):
        waist_pose.pitch += 0.001
        send(); time.sleep(DT)
    print("[Waist] pitch 0.3->0")
    for _ in range(300):
        waist_pose.pitch -= 0.001
        send(); time.sleep(DT)

    # 旋转 yaw: 0 -> 0.1 -> -0.1 -> 0
    print("[Waist] yaw 0->0.1")
    for _ in range(100):
        waist_pose.yaw += 0.001
        send(); time.sleep(DT)
    print("[Waist] yaw 0.1->-0.1")
    for _ in range(200):
        waist_pose.yaw -= 0.001
        send(); time.sleep(DT)
    print("[Waist] yaw -0.1->0")
    for _ in range(100):
        waist_pose.yaw += 0.001
        send(); time.sleep(DT)

def head_test(robot):
    """
    头部高层：接口与低层一致 (位置控制示意)
    """
    pitch = Motor_Control(); yaw = Motor_Control()
    pitch.Position = 0.0
    print("[Head] pitch 0->0.18")
    for _ in range(900):
        pitch.Position += 0.0002
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_UP, pitch)
        time.sleep(DT)
    print("[Head] pitch 0.18->-0.12")
    for _ in range(1500):
        pitch.Position -= 0.0002
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_UP, pitch)
        time.sleep(DT)
    print("[Head] pitch ->0")
    for _ in range(600):
        pitch.Position += 0.0002
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_UP, pitch)
        time.sleep(DT)

    yaw.Position = 0.0
    print("[Head] yaw 0->0.8->-0.8->0")
    for _ in range(800):
        yaw.Position += 0.001
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, yaw); time.sleep(DT)
    for _ in range(1600):
        yaw.Position -= 0.001
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, yaw); time.sleep(DT)
    for _ in range(800):
        yaw.Position += 0.001
        robot.setHead_high(EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, yaw); time.sleep(DT)

def arm_test(robot):
    """
    手臂高层：
    先在 Python 侧维护 ArmPose(x,y,z,roll,pitch,yaw)，
    每步用 robot.armPoseToArmEndPose 转为 ArmEndPose 再 setArm_high 下发。
    """
    poseL = ArmPose(); poseR = ArmPose()
    poseL.x = poseL.y = poseL.z = 0.0
    poseL.roll = poseL.pitch = poseL.yaw = 0.0
    poseR.x = poseR.y = poseR.z = 0.0
    poseR.roll = poseR.pitch = poseR.yaw = 0.0

    def send():
        endL = robot.armPoseToArmEndPose(poseL)
        endR = robot.armPoseToArmEndPose(poseR)
        robot.setArm_high(ArmAction.LEFT_ARM, endL)
        robot.setArm_high(ArmAction.RIGHT_ARM, endR)

    print("[Arm] x 0->0.2")
    for _ in range(2000):
        poseL.x += 0.0001
        poseR.x += 0.0001
        send(); time.sleep(DT)

    print("[Arm] y 0->0.1")
    for _ in range(1000):
        poseL.y += 0.0001
        poseR.y -= 0.0001
        send(); time.sleep(DT)

    print("[Arm] z 0->0.1")
    for _ in range(1000):
        poseL.z += 0.0001
        poseR.z += 0.0001
        send(); time.sleep(DT)

    print("[Arm] roll 0->0.1")
    for _ in range(1000):
        poseL.roll += 0.0001
        poseR.roll -= 0.0001
        send(); time.sleep(DT)

    print("[Arm] roll 0.1->0")
    for _ in range(1000):
        poseL.roll -= 0.0001
        poseR.roll += 0.0001
        send(); time.sleep(DT)

    print("[Arm] pitch 0->0.1")
    for _ in range(1000):
        poseL.pitch += 0.0001
        poseR.pitch += 0.0001
        send(); time.sleep(DT)

    print("[Arm] pitch 0.1->0")
    for _ in range(1000):
        poseL.pitch -= 0.0001
        poseR.pitch -= 0.0001
        send(); time.sleep(DT)

    print("[Arm] yaw 0->0.1")
    for _ in range(1000):
        poseL.yaw -= 0.0001
        poseR.yaw += 0.0001
        send(); time.sleep(DT)

    print("[Arm] yaw 0.1->0")
    for _ in range(1000):
        poseL.yaw -= 0.0001
        poseR.yaw += 0.0001
        send(); time.sleep(DT)

    print("[Arm] z 0.1->0")
    for _ in range(1000):
        poseL.z -= 0.0001
        poseR.z -= 0.0001
        send(); time.sleep(DT)

    print("[Arm] y 0.1->0")
    for _ in range(1000):
        poseL.y -= 0.0001
        poseR.y += 0.0001
        send(); time.sleep(DT)

    print("[Arm] x 0.2->0")
    for _ in range(2000):
        poseL.x -= 0.0001
        poseR.x -= 0.0001
        send(); time.sleep(DT)

def gripper_test(robot):
    """
    抓手高层：与低层类似，位置 0/1
    """
    open_cmd = Motor_Control(); open_cmd.Position = 0.0
    close_cmd = Motor_Control(); close_cmd.Position = 1.5
    print("[Gripper] cycles")
    for _ in range(2):
        robot.setGripper_high(EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8, open_cmd)
        robot.setGripper_high(EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8, open_cmd)
        time.sleep(1.0)
        robot.setGripper_high(EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8, close_cmd)
        robot.setGripper_high(EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8, close_cmd)
        time.sleep(1.0)

def print_arm_target(robot):
    """
    打印 ArmEndPose: position[3], rotation[4](qx,qy,qz,qw)
    """
    ok, st = robot.getArmTargetState(ArmAction.LEFT_ARM)
    if not ok:
        print("getArmTargetState returned failure for LEFT_ARM"); return
    pos = getattr(st, "position", None) or getattr(st, "PositionData", None)
    rot = getattr(st, "rotation", None) or getattr(st, "RotationData", None)
    print("Left arm target -> position:", pos, "rotation(qx,qy,qz,qw):", rot)

def main():
    """
    高层示例主流程:
      1) 构造 H1Robot
      2) 连接，                        所有状态获取接口在这一步之后即可执行
      3) 切到 HIGH_LEVEL
      4) robot_init
      5) 调用测试函数
      6) robot_deinit
      7) Ctrl+C 直接中断 (finally 里 return 已阻止清理)
      注意: finally 里的 return 使得 robot_deinit 永不执行, 仅用于你当前“非优雅退出”需求。
    """
    try:
        # 实例化机器人
        # robot = H1Robot("192.168.2.39")   #SDK运行环境为用户主机，无线连接方式，ip不固定
        # robot = H1Robot("172.31.200.1")   #SDK运行环境为用户主机，有线连接方式，ip固定
        robot = H1Robot()   #SDK运行环境为机器人主机

        if not robot.robot_connect():
            print("connect failed"); return
        print("connected =", robot.isRobotConnected())

        # 状态监控，参考state_monitor.py，可以在控制内调用或单独调用
        monitor = start_state_monitor(robot, EtherCAT_Motor_Index, period=1.0)  

        if not robot.switchControlMode(MotorControlMode.HIGH_LEVEL):
            print("switch HIGH_LEVEL failed"); return
        print("mode =", robot.getCurrentMode())

        if not robot.robot_init():
            print("robot_init failed"); return
        print("robot initialized")
        time.sleep(3.0)

        # 按需启用单项测试，或都开启后顺序执行测试
        chassis_test(robot)
        waist_test(robot)      
        head_test(robot)
        arm_test(robot)
        gripper_test(robot)
        print_arm_target(robot)
        if not robot.robot_deinit():
            print("robot_deinit failed"); return
        print("robot deinitialized")

    except KeyboardInterrupt:
        print("捕获到ctrl+C 中断 (未做清理)")
    finally:
        return
        # 若要恢复优雅退出, 删除上面 return 并取消注释:
        # if hasattr(robot, "robot_deinit"):
        #     robot.robot_deinit()
        # print("done")

if __name__ == "__main__":
    main()