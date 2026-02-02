import os, sys, time

# 将 lib 目录加入 sys.path 以便导入编译出的 lib_h1_sdk_python.so
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 不要加入 lib 目录，而是加入 root 目录
if root not in sys.path:
    sys.path.insert(0, root)


from lib.lib_h1_sdk_python import (
    H1Robot,
    MotorControlMode,
    ArmAction,
    HandControl,
    HandState,
)

def get_hand_state(robot, arm):
    """
    兼容两种绑定签名:
    - getHandState(arm) -> (ok, HandState)
    - getHandState(arm, out) -> bool
    """
    m = robot.getHandState
    try:
        ok, st = m(arm)
        return bool(ok), st
    except TypeError:
        try:
            st = HandState()
            ok = m(arm, st)
            if isinstance(ok, bool):
                return ok, st
            return True, st
        except Exception:
            return False, None

def hand_close(robot):
    # 左手 模式1：位置+速度(0~65535)
    c1 = HandControl()
    c1.hand_mode = 1
    c1.hand_control[:] = [0, 65535, 65535, 65535, 65535, 0]
    c1.hand_speed[:]   = [65535]*6

    # 右手 模式2：角度+速度(角度*100)
    c2 = HandControl()
    c2.hand_mode = 2
    c2.hand_control[:] = [3676, 10022, 9781, 10138, 9884, 0]
    c2.hand_speed[:]   = [65535]*6

    robot.setHand_low(ArmAction.LEFT_ARM, c1)
    robot.setHand_low(ArmAction.RIGHT_ARM, c2)

def hand_open(robot):
    # 左手张开（模式1）
    c1 = HandControl()
    c1.hand_mode = 1
    c1.hand_control[:] = [0, 0, 0, 0, 0, 0]
    c1.hand_speed[:]   = [65535]*6

    # 右手张开（模式2）
    c2 = HandControl()
    c2.hand_mode = 2
    c2.hand_control[:] = [3676, 17837, 17606, 17654, 17486, 0]
    c2.hand_speed[:]   = [65535]*6

    robot.setHand_low(ArmAction.LEFT_ARM, c1)
    robot.setHand_low(ArmAction.RIGHT_ARM, c2)

def print_hand_state(robot):
    okL, stL = get_hand_state(robot, ArmAction.LEFT_ARM)
    okR, stR = get_hand_state(robot, ArmAction.RIGHT_ARM)
    if not (okL and okR):
        print("getHandState 失败")
        return
    print(f"STATE: L(type={stL.hand_type}, err={stL.error_flag}) "
          f"R(type={stR.hand_type}, err={stR.error_flag})")
    for i in range(6):
        print(f"L{i}: pos={stL.hand_position[i]} ang={stL.hand_angle[i]} cur={stL.hand_current[i]} st={stL.hand_status[i]}")
        print(f"R{i}: pos={stR.hand_position[i]} ang={stR.hand_angle[i]} cur={stR.hand_current[i]} st={stR.hand_status[i]}")

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

        # 低层/重力补偿 二选一；此处用低层
        if not robot.switchControlMode(MotorControlMode.LOW_LEVEL):
            print("switch LOW_LEVEL failed"); return
        print("mode =", robot.getCurrentMode())

        if not robot.robot_init():
            print("robot_init failed"); return


        time.sleep(2.0)
        for k in range(5):
            print(f"[{k+1}] 手闭合")
            hand_close(robot)
            time.sleep(1.0)
            print_hand_state(robot)

            print(f"[{k+1}] 手张开")
            hand_open(robot)
            time.sleep(1.0)
            print_hand_state(robot)

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