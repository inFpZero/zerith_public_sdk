/**
 * @file state_monitor.cpp
 * @brief 只读机器人综合状态监控，与 Python state_monitor.py 查询范围一致。
 *
 * 该示例只建立状态连接，不申请控制权限、不切换控制模式、不初始化机器人，
 * 也不调用任何运动控制接口。
 */

#include "H1_Robot.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

struct Options {
  std::string robot_ip = "127.0.0.1";
  double period = 1.0;
  long long count = 0;
  std::string motors = "all";
};

const std::array<const char*, MOTOR_COUNT> kMotorNames = {
    "MOTOR_WHEEL_LEFT", "MOTOR_WHEEL_RIGHT", "MOTOR_LIFT",
    "MOTOR_WAIST_DOWN", "MOTOR_WAIST_UP", "MOTOR_HEAD_DOWN",
    "MOTOR_HEAD_UP", "MOTOR_LEFT_ARM_1", "MOTOR_LEFT_ARM_2",
    "MOTOR_LEFT_ARM_3", "MOTOR_LEFT_ARM_4", "MOTOR_LEFT_ARM_5",
    "MOTOR_LEFT_ARM_6", "MOTOR_LEFT_ARM_7", "MOTOR_LEFT_ARM_8",
    "MOTOR_RIGHT_ARM_1", "MOTOR_RIGHT_ARM_2", "MOTOR_RIGHT_ARM_3",
    "MOTOR_RIGHT_ARM_4", "MOTOR_RIGHT_ARM_5", "MOTOR_RIGHT_ARM_6",
    "MOTOR_RIGHT_ARM_7", "MOTOR_RIGHT_ARM_8"};

void printUsage(const char* program) {
  std::cout << "Usage: " << program
            << " [--ip IP] [--period SECONDS] [--count N]"
               " [--motors all|NAME,NAME]\n";
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++i];
    if (argument == "--ip") {
      options.robot_ip = value;
    } else if (argument == "--period") {
      options.period = std::stod(value);
    } else if (argument == "--count") {
      options.count = std::stoll(value);
    } else if (argument == "--motors") {
      options.motors = value;
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.period <= 0.0) {
    throw std::invalid_argument("--period must be greater than zero");
  }
  if (options.count < 0) {
    throw std::invalid_argument("--count cannot be negative");
  }
  return options;
}

std::vector<EtherCAT_Motor_Index> selectMotors(const std::string& specification) {
  std::unordered_map<std::string, EtherCAT_Motor_Index> members;
  for (int index = 0; index < MOTOR_COUNT; ++index) {
    members.emplace(kMotorNames[index], static_cast<EtherCAT_Motor_Index>(index));
  }
  if (specification == "all") {
    std::vector<EtherCAT_Motor_Index> result;
    for (int index = 0; index < MOTOR_COUNT; ++index) {
      result.push_back(static_cast<EtherCAT_Motor_Index>(index));
    }
    return result;
  }

  std::vector<EtherCAT_Motor_Index> result;
  std::istringstream stream(specification);
  std::string name;
  while (std::getline(stream, name, ',')) {
    const auto first = name.find_first_not_of(" \t");
    const auto last = name.find_last_not_of(" \t");
    if (first == std::string::npos) continue;
    name = name.substr(first, last - first + 1);
    const auto found = members.find(name);
    if (found == members.end()) {
      throw std::invalid_argument("unknown motor: " + name);
    }
    result.push_back(found->second);
  }
  if (result.empty()) {
    throw std::invalid_argument("--motors cannot be empty");
  }
  return result;
}

void printPose(const char* name, bool available, const ArmEndPose& pose) {
  std::cout << name << ": ";
  if (!available) {
    std::cout << "unavailable\n";
    return;
  }
  std::cout << "position=[" << pose.PositionData[0] << ", "
            << pose.PositionData[1] << ", " << pose.PositionData[2]
            << "] rotation=[" << pose.RotationData[0] << ", "
            << pose.RotationData[1] << ", " << pose.RotationData[2] << ", "
            << pose.RotationData[3] << "] timestamp_ns=" << pose.timestamp_ns
            << '\n';
}

void printSnapshot(H1Robot& robot,
                   const std::vector<EtherCAT_Motor_Index>& motors,
                   long long sequence) {
  const double timestamp = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  std::cout << "\n========== state snapshot " << sequence << " ==========\n";
  std::cout << std::fixed << std::setprecision(6);
  std::cout << "timestamp: " << timestamp << '\n';
  std::cout << "connected: " << std::boolalpha << robot.isRobotConnected() << '\n';
  std::cout << "system_init: " << static_cast<int>(robot.getInitState()) << '\n';

  const auto init_status = robot.getSystemInitStatus();
  std::cout << "system_init_status: last=" << static_cast<int>(init_status.last_state)
            << " current=" << static_cast<int>(init_status.current_state)
            << " reason=" << static_cast<int>(init_status.change_reason)
            << " error=" << init_status.error_code
            << " changed_at_ns=" << init_status.state_changed_at_ns
            << " timestamp_ns=" << init_status.timestamp_ns << '\n';

  std::cout << "control_mode: " << static_cast<int>(robot.getCurrentMode()) << '\n';
  const auto mode_status = robot.getControlModeStatus();
  std::cout << "control_mode_status: last=" << static_cast<int>(mode_status.last_state)
            << " current=" << static_cast<int>(mode_status.current_state)
            << " changed_at_ns=" << mode_status.state_changed_at_ns
            << " timestamp_ns=" << mode_status.timestamp_ns << '\n';

  RobotInfo robot_info;
  if (robot.getRobotInfo(robot_info)) {
    std::cout << "robot_info: name=" << robot_info.robot_name
              << " type=" << robot_info.robot_type
              << " firmware=" << robot_info.firmware_version
              << " hardware=" << robot_info.hardware_version
              << " software=" << robot_info.software_version
              << " manufacturer=" << robot_info.manufacturer
              << " timestamp_ns=" << robot_info.timestamp_ns << '\n';
  } else {
    std::cout << "robot_info: unavailable\n";
  }

  power_charge_state power;
  if (robot.getPowerChargeState(power)) {
    std::cout << "power: status=" << static_cast<int>(power.status)
              << " temperature=" << power.temperature << " soc=" << power.soc
              << '\n';
  } else {
    std::cout << "power: unavailable\n";
  }

  ImuData imu;
  if (robot.getIMU_State(imu)) {
    std::cout << "imu: rpy=[" << imu.rpy[0] << ", " << imu.rpy[1] << ", "
              << imu.rpy[2] << "] omega=[" << imu.omega[0] << ", "
              << imu.omega[1] << ", " << imu.omega[2] << "] accel=["
              << imu.accel[0] << ", " << imu.accel[1] << ", " << imu.accel[2]
              << "] quat=[" << imu.quat[0] << ", " << imu.quat[1] << ", "
              << imu.quat[2] << ", " << imu.quat[3] << "] error="
              << imu.error_flag << " timestamp_ns=" << imu.timestamp_ns << '\n';
  } else {
    std::cout << "imu: unavailable\n";
  }

  XboxMap joystick;
  if (robot.getJoystickState(joystick)) {
    std::cout << "joystick: a=" << joystick.a << " b=" << joystick.b
              << " x=" << joystick.x << " y=" << joystick.y
              << " shoulders=[" << joystick.leftShoulder << ", "
              << joystick.rightShoulder << "] stick_buttons=["
              << joystick.leftStick << ", " << joystick.rightStick
              << "] start=" << joystick.start << " back=" << joystick.back
              << " dpad=[" << joystick.dpadUp << ", " << joystick.dpadDown
              << ", " << joystick.dpadLeft << ", " << joystick.dpadRight << "]"
              << " left_xy=[" << joystick.leftX << ", " << joystick.leftY
              << "] right_xy=[" << joystick.rightX << ", " << joystick.rightY
              << "] triggers=[" << joystick.leftTrigger << ", "
              << joystick.rightTrigger << "] error=" << joystick.error_flag
              << " timestamp_ns=" << joystick.timestamp_ns << '\n';
  } else {
    std::cout << "joystick: unavailable\n";
  }

  HighLevelState high_level;
  if (robot.getHighLevelState(high_level)) {
    std::cout << "high_level: state=" << high_level.state
              << " progress=" << high_level.progress
              << " timestamp_ns=" << high_level.timestamp_ns << '\n';
  } else {
    std::cout << "high_level: unavailable\n";
  }

  std::array<float, 2> speed_actual{};
  std::array<float, 2> speed_algo{};
  if (robot.getChassisSpeedState(speed_actual, speed_algo)) {
    std::cout << "chassis_speed: actual=[" << speed_actual[0] << ", "
              << speed_actual[1] << "] algorithm=[" << speed_algo[0] << ", "
              << speed_algo[1] << "]\n";
  } else {
    std::cout << "chassis_speed: unavailable\n";
  }

  int gripper_mode = 0;
  std::cout << "gripper_mode: ";
  if (robot.getGripperControlMode(gripper_mode)) std::cout << gripper_mode;
  else std::cout << "unavailable";
  std::cout << '\n';

  int fixed_rod = 0;
  std::cout << "fixed_rod: ";
  if (robot.getFixedRodState(fixed_rod)) std::cout << fixed_rod;
  else std::cout << "unavailable";
  std::cout << '\n';

  ArmEndPose pose;
  printPose("head_relative", robot.getHeadRelative(pose), pose);
  pose = {};
  printPose("head_camera_relative", robot.getHeadCameraRelative(pose), pose);
  for (const auto& arm : std::array<std::pair<const char*, ArmAction>, 2>{
           std::make_pair("left_arm", LEFT_ARM),
           std::make_pair("right_arm", RIGHT_ARM)}) {
    pose = {};
    const std::string relative_name = std::string(arm.first) + ".hand_relative";
    printPose(relative_name.c_str(), robot.getHandRelative(arm.second, pose), pose);
    pose = {};
    const std::string camera_name = std::string(arm.first) + ".hand_camera_relative";
    printPose(camera_name.c_str(),
              robot.getHandCameraRelative(arm.second, pose), pose);
  }

  std::cout << "motors:\n";
  for (const auto motor : motors) {
    Motor_Information state{};
    const int index = static_cast<int>(motor);
    std::cout << "  " << kMotorNames[index] << ": ";
    if (robot.getMotorState(motor, state)) {
      std::cout << "position=" << state.Position_Actual
                << " speed=" << state.Speed_Actual
                << " torque=" << state.Torque_Actual
                << " kp=" << state.KP_Actual << " kd=" << state.KD_Actual
                << " error=" << state.Error_flag;
    } else {
      std::cout << "unavailable";
    }
    std::cout << '\n';
  }
  std::cout << std::noboolalpha;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    const auto motors = selectMotors(options.motors);
    auto& robot = H1Robot::getInstance(options.robot_ip);
    if (!robot.robot_connect()) {
      std::cerr << "Robot connection failed.\n";
      return 1;
    }

    long long emitted = 0;
    while (options.count == 0 || emitted < options.count) {
      printSnapshot(robot, motors, emitted + 1);
      ++emitted;
      if (options.count == 0 || emitted < options.count) {
        std::this_thread::sleep_for(std::chrono::duration<double>(options.period));
      }
    }
  } catch (const std::exception& error) {
    std::cerr << "state_monitor: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
