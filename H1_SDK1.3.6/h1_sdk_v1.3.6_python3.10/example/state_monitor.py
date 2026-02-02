import threading, time, json, traceback, inspect
import os, sys, time

# 将 lib 目录加入 sys.path 以便导入编译出的 lib_h1_sdk_python.so
root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 不要加入 lib 目录，而是加入 root 目录
if root not in sys.path:
    sys.path.insert(0, root)


from lib.lib_h1_sdk_python import (
    H1Robot,
    MotorControlMode,
    Motor_Control,
    Motor_Information,
    EtherCAT_Motor_Index,
    ArmAction,       # 枚举: LEFT_ARM / RIGHT_ARM
    ArmPose,         

)

# 默认刷新周期（秒）
DEFAULT_PERIOD = 1.0

def _struct_to_dict(obj):
    """
    粗糙提取 pybind 返回结构的公开属性 -> 基本类型 / list / tuple
    避免递归过深
    """
    basic_types = (int, float, bool, str, type(None))
    if isinstance(obj, (dict, list, tuple)) or isinstance(obj, basic_types):
        return obj
    out = {}
    for name in dir(obj):
        if name.startswith("_"):
            continue
        try:
            val = getattr(obj, name)
        except:
            continue
        if isinstance(val, basic_types):
            out[name] = val
        elif isinstance(val, (list, tuple)):
            # 截断过长数组
            if len(val) > 32:
                out[name] = list(val[:32]) + ["..."]
            else:
                out[name] = list(val)
        elif hasattr(val, "__int__"):
            try:
                out[name] = int(val)
            except Exception:
                pass
    return out

def _collect_motor_indices(enum_cls):
    members = []
    for k in dir(enum_cls):
        if k.startswith("_"):
            continue
        v = getattr(enum_cls, k)
        if inspect.isclass(v) or inspect.isfunction(v):
            continue
        if hasattr(v, "__int__"):
            members.append((k, v))
    return members

class StateMonitor(threading.Thread):
    # def __init__(self, robot, EtherCAT_Motor_Index,
    #              period=DEFAULT_PERIOD, stop_event=None, print_func=print):
    #     super().__init__(daemon=True)
    # 为 robot 参数添加 : H1Robot 标注

    def __init__(self, 
                 robot: H1Robot, 
                 motor_index_cls: EtherCAT_Motor_Index,
                 period=DEFAULT_PERIOD, 
                 stop_event=None, 
                 print_func=print):
        super().__init__(daemon=True)
        self.robot = robot
        self.EtherCAT_Motor_Index = EtherCAT_Motor_Index
        self.period = period
        self._stop = stop_event or threading.Event()    
        self.print = print_func
        self.motor_indices = _collect_motor_indices(EtherCAT_Motor_Index)

        # print("EtherCAT_Motor_Index dir:", dir(EtherCAT_Motor_Index))
        # print("motor_indices:", self.motor_indices)

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                snapshot = {}

                # 初始化状态 / 控制模式 / 连接
                if hasattr(self.robot, "getInitState"):
                    snapshot["init_state"] = self.robot.getInitState()
                if hasattr(self.robot, "getCurrentMode"):
                    snapshot["control_mode"] = self.robot.getCurrentMode()
                if hasattr(self.robot, "isRobotConnected"):
                    snapshot["connected"] = self.robot.isRobotConnected()

                # 电源
                # 新接口：getPowerChargeState；兼容旧接口：getPowerState
                if hasattr(self.robot, "getPowerChargeState"):
                    try:
                        ok, pcs = self.robot.getPowerChargeState()
                        if ok:
                            snapshot["power"] = _struct_to_dict(pcs)
                    except Exception:
                        pass
                elif hasattr(self.robot, "getPowerState"):
                    try:
                        ok, ps = self.robot.getPowerState()
                        if ok:
                            snapshot["power"] = _struct_to_dict(ps)
                    except Exception:
                        pass

                # IMU
                if hasattr(self.robot, "getIMU_State"):
                    ok, imu = self.robot.getIMU_State()
                    if ok: snapshot["imu"] = _struct_to_dict(imu)

                # 手柄
                if hasattr(self.robot, "getJoystickState"):
                    ok, js = self.robot.getJoystickState()
                    if ok: snapshot["joystick"] = _struct_to_dict(js)

                # 机器人基本信息（新）
                if hasattr(self.robot, "getRobotInfo"):
                    try:
                        ok, info = self.robot.getRobotInfo()
                        if ok:
                            snapshot["robot_info"] = _struct_to_dict(info)
                    except Exception:
                        pass

                # 通讯板（若后续绑定了 getCommunicationBoardState，可在此添加）
                if hasattr(self.robot, "getCommunicationBoardState"):
                    try:
                        ok, comm = self.robot.getCommunicationBoardState()
                        if ok: snapshot["communication_board"] = _struct_to_dict(comm)
                    except Exception:
                        pass

                # 夹爪控制模式（新）
                if hasattr(self.robot, "getGripperControlMode"):
                    try:
                        ok, mode = self.robot.getGripperControlMode()
                        if ok:
                            snapshot["gripper_mode"] = mode
                    except Exception:
                        pass

                # 推杆状态（新）
                if hasattr(self.robot, "getFixedRodState"):
                    try:
                        ok, is_open = self.robot.getFixedRodState()
                        if ok:
                            snapshot["fixed_rod"] = is_open
                    except Exception:
                        pass

                # 所有电机
                motors_data = {}
                if hasattr(self.robot, "getMotorState"):

                    for name, idx in self.motor_indices:
                        try:
                            ok, ms = self.robot.getMotorState(idx)
                            # print(f"getMotorState({name}={idx}) -> ok={ok}, ms={ms}")
                            if ok:
                                motors_data[name] = {
                                    "pos": ms.Position_Actual,
                                    "spd": ms.Speed_Actual,
                                    "tq": ms.Torque_Actual,
                                    "kp": ms.KP_Actual,
                                    "kd": ms.KD_Actual,
                                    "err": ms.Error_flag
                                }
                        except Exception as e:
                            print(f"getMotorState({name}={idx}) exception: {e}")
                            continue
                    snapshot["motors"] = motors_data

                # 底盘速度状态（新）
                if hasattr(self.robot, "getChassisSpeedState"):
                    try:
                        ok, speed_actual, speed_algo = self.robot.getChassisSpeedState()
                        if ok:
                            # speed_actual: [左轮, 右轮]；speed_algo: [线速度, 角速度]
                            snapshot["chassis_speed"] = {
                                "wheel": list(speed_actual),
                                "algo": list(speed_algo)
                            }
                    except Exception:
                        pass

                # 手臂末端目标位姿（始终尝试左右臂）
                if hasattr(self.robot, "getArmTargetState"):
                    arm_targets = {}
                    # 直接使用已导入的 ArmAction 枚举，失败则使用整数回退
                    try:
                        left_val = ArmAction.LEFT_ARM
                        right_val = ArmAction.RIGHT_ARM
                    except Exception:
                        left_val, right_val = 0, 1
                    for name, side in [("LEFT_ARM", left_val), ("RIGHT_ARM", right_val)]:
                        try:
                            ok, pose = self.robot.getArmTargetState(side)
                            if ok:
                                arm_targets[name] = _struct_to_dict(pose)
                        except Exception:
                            continue
                    if arm_targets:
                        snapshot["arm_targets"] = arm_targets

                # 灵巧手状态（左右臂）
                if hasattr(self.robot, "getHandState"):
                    hands = {}
                    try:
                        left_val = ArmAction.LEFT_ARM
                        right_val = ArmAction.RIGHT_ARM
                    except Exception:
                        left_val, right_val = 0, 1
                    for name, side in [("LEFT_ARM", left_val), ("RIGHT_ARM", right_val)]:
                        try:
                            ok, hs = self.robot.getHandState(side)
                            if ok:
                                hands[name] = _struct_to_dict(hs)
                        except Exception:
                            continue
                    if hands:
                        snapshot["hands"] = hands

                # 六维力传感器状态
                if hasattr(self.robot, "getForceSensorState"):
                    try:
                        ok, fs = self.robot.getForceSensorState()
                        if ok:
                            snapshot["force6"] = _struct_to_dict(fs)
                    except Exception:
                        pass

                # 打印（模块一行，换行分隔）
                self.print(_format_snapshot_multiline(_to_jsonable(snapshot)))
            except Exception:
                self.print("[StateMonitor] exception:\n" + traceback.format_exc())

            self._stop.wait(self.period)

def start_state_monitor(robot, EtherCAT_Motor_Index, period=DEFAULT_PERIOD):
    mon = StateMonitor(robot, EtherCAT_Motor_Index, period=period)
    mon.start()
    return mon

def _to_jsonable(obj):
    # 基本类型直接返回
    if isinstance(obj, (int, float, bool, str)) or obj is None:
        return obj
    # pybind11 枚举对象
    if hasattr(obj, "__int__"):
        try:
            return int(obj)
        except Exception:
            pass
    # list/tuple
    if isinstance(obj, (list, tuple)):
        return [_to_jsonable(x) for x in obj]
    # dict
    if isinstance(obj, dict):
        return {k: _to_jsonable(v) for k, v in obj.items()}
    # 结构体
    if not isinstance(obj, (set,)):
        d = {}
        for name in dir(obj):
            if name.startswith("_"):
                continue
            try:
                v = getattr(obj, name)
            except Exception:
                continue
            if callable(v):
                continue
            d[name] = _to_jsonable(v)
        if d:
            return d
    # 兜底
    return str(obj)

def _format_snapshot_multiline(snapshot):
    """将顶层每个键的内容压缩成一行，键之间以换行分隔。
    例如:
    [StateMonitor]\ninit_state:0\npower:{"soc":55,"temperature":300}\n...
    """
    if not isinstance(snapshot, dict):
        try:
            return "[StateMonitor]\n" + json.dumps(snapshot, ensure_ascii=False)
        except Exception:
            return f"[StateMonitor]\n{snapshot}"
    lines = ["[StateMonitor]"]
    for k, v in snapshot.items():
        # 特殊处理: motors / arm_targets 拆成多行
        if k == "motors" and isinstance(v, dict):
            lines.append("motors:")
            for m_name, m_state in v.items():
                try:
                    m_txt = json.dumps(m_state, ensure_ascii=False, separators=(",", ":"))
                except Exception:
                    m_txt = str(m_state)
                lines.append(f"  {m_name}: {m_txt}")
            continue
        if k == "arm_targets" and isinstance(v, dict):
            lines.append("arm_targets:")
            for arm_name, pose in v.items():
                try:
                    pose_txt = json.dumps(pose, ensure_ascii=False, separators=(",", ":"))
                except Exception:
                    pose_txt = str(pose)
                lines.append(f"  {arm_name}: {pose_txt}")
            continue
        # 新增：灵巧手状态多行打印
        if k == "hands" and isinstance(v, dict):
            lines.append("hands:")
            for arm_name, hs in v.items():
                try:
                    hs_txt = json.dumps(hs, ensure_ascii=False, separators=(",", ":"))
                except Exception:
                    hs_txt = str(hs)
                lines.append(f"  {arm_name}: {hs_txt}")
            continue
        # 新增：六维力多行打印（按左右两侧）
        if k == "force6" and isinstance(v, dict):
            fx = v.get("Fx", []) or v.get("fx", [])
            fy = v.get("Fy", []) or v.get("fy", [])
            fz = v.get("Fz", []) or v.get("fz", [])
            tx = v.get("Tx", []) or v.get("tx", [])
            ty = v.get("Ty", []) or v.get("ty", [])
            tz = v.get("Tz", []) or v.get("tz", [])
            err = v.get("error_flag", v.get("Error_flag", []))
            n = min(len(fx), len(fy), len(fz), len(tx), len(ty), len(tz)) if all(isinstance(a, (list, tuple)) for a in [fx, fy, fz, tx, ty, tz]) else 0
            labels = ["LEFT", "RIGHT"] if n == 2 else [str(i) for i in range(n)]
            lines.append("force6:")
            for i, label in enumerate(labels):
                try:
                    e = (err[i] if isinstance(err, (list, tuple)) and i < len(err) else err) if err is not None else "NA"
                    lines.append(
                        f"  {label}: F=[{fx[i]:.3f},{fy[i]:.3f},{fz[i]:.3f}], "
                        f"T=[{tx[i]:.3f},{ty[i]:.3f},{tz[i]:.3f}], err={e}"
                    )
                except Exception:
                    # 退化为原始对象打印
                    try:
                        v_txt = json.dumps(v, ensure_ascii=False, separators=(",", ":"))
                    except Exception:
                        v_txt = str(v)
                    lines.append(f"  raw: {v_txt}")
                    break
            continue
        # joystick: 正常完整打印
        if k == "joystick" and isinstance(v, dict):
            try:
                v_txt = json.dumps(v, ensure_ascii=False, separators=(",", ":"))
            except Exception:
                v_txt = str(v)
            lines.append(f"joystick: {v_txt}")
            continue

        try:
            if isinstance(v, (dict, list, tuple)):
                v_txt = json.dumps(v, ensure_ascii=False, separators=(",", ":"))
            else:
                v_txt = json.dumps(v, ensure_ascii=False)
        except Exception:
            v_txt = str(v)
        lines.append(f"{k}: {v_txt}")
    return "\n".join(lines)



def main():
    """
    低层示例主流程:
      1) 构造 H1Robot
      2) 连接 ,                                 所有状态获取接口在这一步之后即可执行
      3) 切到 LOW_LEVEL
      4) robot_init
      5) 调用测试函数
      6) robot_deinit
      7) Ctrl+C 直接中断 (finally 里 return 已阻止清理)
    注意: finally 里的 return 使得 robot_deinit 永不执行, 仅用于你当前“非优雅退出”需求。
    """
    try:
        # 实例化机器人
        # robot = H1Robot("192.168.2.165")   #SDK运行环境为用户主机，无线连接方式，ip不固定
        # robot = H1Robot("172.31.200.1")   #SDK运行环境为用户主机，有线连接方式，ip固定
        robot = H1Robot()   #SDK运行环境为机器人主机
        
        if not robot.robot_connect():
            print("connect failed"); return

        print("connected =", robot.isRobotConnected())
        
        # 状态监控，参考state_monitor.py，可以关闭
        monitor = start_state_monitor(robot, EtherCAT_Motor_Index, period=1.0)  # 新增

        while True:
            # 临时调试片段（交互式）
            # ok, m = robot.getMotorState(EtherCAT_Motor_Index.MOTOR_LEFT_ARM_1)
            # print(ok, m.KP_Actual, m.KD_Actual)
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("捕获到ctrl+C 中断 (未做清理)")
    finally:
        return

if __name__ == "__main__":
    main()