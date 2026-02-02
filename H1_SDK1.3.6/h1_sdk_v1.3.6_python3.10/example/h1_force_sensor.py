import os, sys, time

# 将 lib 目录加入 sys.path 以便导入编译出的 lib_h1_sdk_python.so
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 不要加入 lib 目录，而是加入 root 目录
if root not in sys.path:
    sys.path.insert(0, root)

from lib.lib_h1_sdk_python import (
    H1Robot,
    # ForceSensorState 可能在绑定里，也可能通过返回值直接获得
)

def get_force_state(robot):
    """
    兼容三种绑定签名:
    - getForceSensorState() -> (ok, state)
    - getForceSensorState(out) -> bool
    - getForceSensorState() -> state
    """
    m = robot.getForceSensorState
    try:
        ok, st = m()
        return bool(ok), st
    except TypeError:
        try:
            # 如果已绑定 ForceSensorState，可解开注释:
            # st = ForceSensorState()
            # ok = m(st)
            # return (ok if isinstance(ok, bool) else True), st
            st = m()  # 直接返回对象
            return True, st
        except Exception:
            return False, None

def print_force(st, idx):
    Fx = getattr(st, "Fx")[idx]
    Fy = getattr(st, "Fy")[idx]
    Fz = getattr(st, "Fz")[idx]
    Tx = getattr(st, "Tx")[idx]
    Ty = getattr(st, "Ty")[idx]
    Tz = getattr(st, "Tz")[idx]
    Err = getattr(st, "Error_flag")[idx]
    print(f"[{idx}] Fx={Fx:.4f} Fy={Fy:.4f} Fz={Fz:.4f} "
          f"Tx={Tx:.4f} Ty={Ty:.4f} Tz={Tz:.4f} Err={Err}")

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

        k = 0
    
        while True:
            k += 1
            ok, st = get_force_state(robot)
            if not ok or st is None:
                print("读取力传感器失败")
                time.sleep(0.1)
                continue

            # 打印两个传感器通道（若仅一个存在，请只看 [0]）
            print_force(st, 0)
            try:
                print_force(st, 1)
            except Exception:
                pass

            # 定期清零（根据硬件通道号调整 0/1）
            if k % 99 == 0 and hasattr(robot, "resetForceSensorData"):
                robot.resetForceSensorData(1)
            if k % 180 == 0 and hasattr(robot, "resetForceSensorData"):
                robot.resetForceSensorData(0)

            time.sleep(0.1)

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