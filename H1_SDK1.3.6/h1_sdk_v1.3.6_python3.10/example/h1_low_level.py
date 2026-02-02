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
    Motor_Information,
    EtherCAT_Motor_Index,
)


# 低层控制循环频率与时间步
RATE_HZ = 500
DT = 1.0 / RATE_HZ

def send_motor(robot, idx, pos=None, speed=None, torque=None):
    """
    构造一个 MotorControl 并发送到指定电机:
    - 只设置你给出的字段, 其他保持默认(通常 0)
    - 对底盘轮子使用 speed (速度控制)
    - 对关节使用 pos (位置控制)
    """
    mc = Motor_Control()
    if pos is not None:    mc.Position = pos
    if speed is not None:  mc.Speed = speed
    if torque is not None: mc.Torque = torque
    robot.setMotorControl_low(idx, mc)

def get_state(robot, idx):
    """
    获取单个电机状态的鲁棒尝试:
    兼容三种可能的绑定签名:
      getMotorState(idx) -> MotorInformation
      getMotorState(idx, out_obj) -> bool0
      getMotorState() -> 序列  (再用 idx 取)
    """
    if not hasattr(robot, "getMotorState"):
        return None
    m = robot.getMotorState
    # 尝试直接返回对象
    try:
        ret = m(idx)
        if isinstance(ret, Motor_Information):
            return ret
        if isinstance(ret, (list, tuple)):
            try:
                return ret[idx]
            except Exception:
                pass
        return ret
    except TypeError:
        pass
    # 尝试 (idx, out)
    try:
        out = Motor_Information()
        ok = m(idx, out)
        if isinstance(ok, Motor_Information):
            return ok
        return out
    except TypeError:
        pass
    # 尝试无参调用
    try:
        ret = m()
        if isinstance(ret, (list, tuple)):
            return ret[idx]
        return ret
    except TypeError:
        return None

# ---------------- 下面是各部位低层测试函数 ----------------

def chassis_test(robot):
    """
    底盘：左右轮速度控制 (仅支持速度)
    演示 前进 -> 停 -> 后退 -> 停
    """
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT,  speed=1)
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT, speed=1)
    time.sleep(3)
    stL = get_state(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT)
    stR = get_state(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT)
    print("Chassis forward L:", getattr(stL, "Position_Actual", None),
          "R:", getattr(stR, "Position_Actual", None))
    # 停
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT,  speed=0)
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT, speed=0)
    time.sleep(2)
    # 后退
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT,  speed=-1)
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT, speed=-1)
    time.sleep(3)
    stL = get_state(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT)
    stR = get_state(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT)
    print("Chassis backward L:", getattr(stL, "Position_Actual", None),
          "R:", getattr(stR, "Position_Actual", None))
    # 停
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_LEFT,  speed=0)
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_WHEEL_RIGHT, speed=0)
    time.sleep(2)

def waist_test(robot):
    """
    腰部/升降：
    LIFT: 升降电机
    WAIST_DOWN: 俯仰
    WAIST_UP:   旋转
    使用位置模式做往返
    """
    # 升降 0 -> 0.5
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_LIFT, pos=0.5)
    time.sleep(3)
    st = get_state(robot, EtherCAT_Motor_Index.MOTOR_LIFT)
    print("Lift mid pos:", getattr(st, "Position_Actual", None))

    # 俯仰
    pos = 0.0
    steps = 1000
    inc = 0.5 / steps
    for _ in range(steps):
        pos += inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_WAIST_DOWN, pos=pos)
        time.sleep(DT)
    time.sleep(1.5)
    for _ in range(steps):
        pos -= inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_WAIST_DOWN, pos=pos)
        time.sleep(DT)
    time.sleep(1.5)

    # 旋转
    pos = 0.0
    inc = 0.20 / 500
    for _ in range(500):
        pos += inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_WAIST_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.5)
    for _ in range(1000):
        pos -= inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_WAIST_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.5)
    for _ in range(500):
        pos += inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_WAIST_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.5)

    # 升降 0.5 -> 0
    send_motor(robot, EtherCAT_Motor_Index.MOTOR_LIFT, pos=0.0)
    time.sleep(3)
    st = get_state(robot, EtherCAT_Motor_Index.MOTOR_LIFT)
    print("Lift zero pos:", getattr(st, "Position_Actual", None))


def head_test(robot):
    """
    头部俯仰/旋转位置往返
    """
    # 俯仰
    pos = 0.0
    inc_up = 0.2 / 1000
    for _ in range(1000):
        pos += inc_up
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)
    inc_down = (0.2 + 0.16) / 1800
    for _ in range(1800):
        pos -= inc_down
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)
    back_steps = int(abs(pos) / inc_up)
    for _ in range(back_steps):
        pos += inc_up
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_UP, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)

    # 旋转
    pos = 0.0
    inc = 1.0 / 1000
    for _ in range(1000):
        pos += inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)
    for _ in range(2000):
        pos -= inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)
    for _ in range(1000):
        pos += inc
        send_motor(robot, EtherCAT_Motor_Index.MOTOR_HEAD_DOWN, pos=pos)
        time.sleep(DT)
    time.sleep(1.0)

def arm_test(robot):
    """
    手臂 7 号关节（假设末端某自由度）做 -0.5 -> 0.5 -> 0 往返
    左右臂同步
    """
    idxL = EtherCAT_Motor_Index.MOTOR_LEFT_ARM_7
    idxR = EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_7
    pos = 0.0
    inc = 0.5 / 500
    for _ in range(500):
        pos -= inc
        send_motor(robot, idxL, pos=pos)
        send_motor(robot, idxR, pos=pos)
        time.sleep(DT)
    for _ in range(1000):
        pos += inc
        send_motor(robot, idxL, pos=pos)
        send_motor(robot, idxR, pos=pos)
        time.sleep(DT)
    for _ in range(500):
        pos -= inc
        send_motor(robot, idxL, pos=pos)
        send_motor(robot, idxR, pos=pos)
        time.sleep(DT)

def gripper_test(robot):
    """
    抓手开闭循环 (Position=0 -> 1)
    """
    idxL = EtherCAT_Motor_Index.MOTOR_LEFT_ARM_8
    idxR = EtherCAT_Motor_Index.MOTOR_RIGHT_ARM_8
    for _ in range(5):
        send_motor(robot, idxL, pos=0.0)
        send_motor(robot, idxR, pos=0.0)
        time.sleep(1.5)
        send_motor(robot, idxL, pos=1.0)
        send_motor(robot, idxR, pos=1.0)
        time.sleep(1.5)

def main():
    """
    低层示例主流程:
      1) 构造 H1Robot
      2) 连接，                         所有状态获取接口在这一步之后即可执行
      3) 切到 LOW_LEVEL
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
        
        # 状态监控，参考state_monitor.py，可以关闭
        monitor = start_state_monitor(robot, EtherCAT_Motor_Index, period=1.0)  # 新增

        if not robot.switchControlMode(MotorControlMode.LOW_LEVEL):
            print("switch LOW_LEVEL failed"); return
        print("mode =", robot.getCurrentMode())

        if not robot.robot_init():
            print("robot_init failed"); return
        print("robot initialized")
        time.sleep(3.0)

        # 按需启用单项测试，或都开启后顺序执行测试
        # chassis_test(robot)
        waist_test(robot)
        head_test(robot)
        arm_test(robot)
        gripper_test(robot)

        if not robot.robot_deinit():
            print("robot_deinit failed"); return
        print("robot deinitialized")

    except KeyboardInterrupt:
        print("捕获到ctrl+C 中断 (未做清理)")
    finally:
        return
        # 如果以后需要恢复优雅退出, 删除上面 'return' 并取消注释:
        # if hasattr(robot, "robot_deinit"):
        #     robot.robot_deinit()
        # print("done")

if __name__ == "__main__":
    main()