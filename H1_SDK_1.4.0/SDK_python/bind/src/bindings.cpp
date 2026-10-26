#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "bindings.hpp"
#include "H1_Robot.hpp"
#include "log.hpp"

namespace py = pybind11;

const char* python_log_module() {
  const char* module = std::getenv("ROBOT_LOG_MODULE");
  return (module && *module) ? module : "H1_SDK";
}

template <typename T, std::size_t N>
py::list array_to_list(const T (&values)[N]) {
  py::list result;
  for (const auto& value : values) result.append(value);
  return result;
}

template <typename T, std::size_t N>
void sequence_to_array(const py::sequence& values, T (&target)[N], const char* name) {
  if (static_cast<std::size_t>(py::len(values)) != N) {
    throw std::invalid_argument(std::string(name) + " must contain " + std::to_string(N) + " values");
  }
  for (std::size_t i = 0; i < N; ++i) target[i] = values[i].cast<T>();
}

template <typename State, typename Getter>
py::tuple state_result(H1Robot& robot, Getter getter) {
  State state{};
  const bool ok = (robot.*getter)(state);
  return py::make_tuple(ok, state);
}

py::object mat_to_numpy(const cv::Mat& input) {
  if (input.empty()) return py::none();
  auto* owner = new cv::Mat(input);
  py::capsule lifetime(owner, [](void* pointer) {
    delete static_cast<cv::Mat*>(pointer);
  });

  py::dtype dtype;
  switch (owner->depth()) {
    case CV_8U: dtype = py::dtype::of<std::uint8_t>(); break;
    case CV_16U: dtype = py::dtype::of<std::uint16_t>(); break;
    case CV_32F: dtype = py::dtype::of<float>(); break;
    default: throw std::runtime_error("Unsupported OpenCV frame depth");
  }

  std::vector<py::ssize_t> shape;
  std::vector<py::ssize_t> strides;
  if (owner->channels() == 1) {
    shape = {owner->rows, owner->cols};
    strides = {
        static_cast<py::ssize_t>(owner->step[0]),
        static_cast<py::ssize_t>(owner->elemSize1())};
  } else {
    shape = {owner->rows, owner->cols, owner->channels()};
    strides = {
        static_cast<py::ssize_t>(owner->step[0]),
        static_cast<py::ssize_t>(owner->elemSize()),
        static_cast<py::ssize_t>(owner->elemSize1())};
  }

  py::array result(dtype, shape, strides, owner->data, lifetime);
  result.attr("setflags")(false);
  return result;
}

py::object camera_sample_to_python(
    const std::optional<std::pair<cv::Mat, double>>& sample) {
  if (!sample) return py::none();
  return py::make_tuple(mat_to_numpy(sample->first), sample->second);
}

py::dict camera_state_to_dict(const robot::RecorderStateResponse& response) {
  py::dict result;
  py::list cameras;
  for (const auto& camera : response.camera_configs()) {
    py::dict camera_dict;
    camera_dict["camera_name"] = camera.camera_name();
    py::list streams;
    for (const auto& stream : camera.streams()) {
      py::dict stream_dict;
      stream_dict["type"] = stream.type();
      stream_dict["width"] = stream.width();
      stream_dict["height"] = stream.height();
      stream_dict["fps"] = stream.fps();
      if (stream.has_intrinsics()) {
        const auto& intrinsics = stream.intrinsics();
        py::dict intrinsics_dict;
        intrinsics_dict["width"] = intrinsics.width();
        intrinsics_dict["height"] = intrinsics.height();
        intrinsics_dict["fx"] = intrinsics.fx();
        intrinsics_dict["fy"] = intrinsics.fy();
        intrinsics_dict["ppx"] = intrinsics.ppx();
        intrinsics_dict["ppy"] = intrinsics.ppy();
        intrinsics_dict["model"] = intrinsics.model();
        intrinsics_dict["coeffs"] = intrinsics.coeffs();
        stream_dict["intrinsics"] = std::move(intrinsics_dict);
      }
      streams.append(std::move(stream_dict));
    }
    camera_dict["streams"] = std::move(streams);
    cameras.append(std::move(camera_dict));
  }
  result["camera_configs"] = std::move(cameras);
  return result;
}

void write_python_log(spdlog::level::level_enum level,
                      const std::string& message,
                      bool event = false) {
  const spdlog::source_loc location{"<python>", 0, "python"};
  if (event) {
    auto logger = logx::event_logger();
    if (!logger) {
      throw std::runtime_error(
          "EVENT logging is not initialized");
    }
    logger->log(location, spdlog::level::info, "{}", message);
    spdlog::log(location, spdlog::level::info, "[EVENT] {}", message);
    return;
  }
  spdlog::log(location, level, "{}", message);
}

std::string format_python_log_message(const std::string& format,
                                      const py::args& arguments) {
  if (arguments.empty()) return format;
  const py::tuple values = py::reinterpret_borrow<py::tuple>(arguments);
  return py::str(py::str(format).attr("__mod__")(values));
}

struct PythonLoggerFacade {};

void bind_enums(py::module_& m) {
  py::enum_<InitCommand>(m, "InitCommand")
      .value("Wait_Init", Wait_Init).value("Init", Init)
      .value("Deinit", Deinit).value("ReInit", ReInit);
  py::enum_<InitState>(m, "InitState")
      .value("Uninit", Uninit).value("Initializing", Initializing)
      .value("Init_Complete", Init_Complete).value("Deinitializing", Deinitializing)
      .value("Deinit_Complete", Deinit_Complete).value("Error_State", Error_State);
  py::enum_<ArmAction>(m, "ArmAction")
      .value("LEFT_ARM", LEFT_ARM).value("RIGHT_ARM", RIGHT_ARM);
  py::enum_<MotorControlMode>(m, "MotorControlMode")
      .value("UNINITIALIZED", UNINITIALIZED).value("LOW_LEVEL", LOW_LEVEL)
      .value("HIGH_LEVEL", HIGH_LEVEL)
      .value("GRAVITY_COMPENSATION_LEVEL", GRAVITY_COMPENSATION_LEVEL);
  py::enum_<EtherCAT_Motor_Index>(m, "EtherCAT_Motor_Index")
      .value("MOTOR_WHEEL_LEFT", MOTOR_WHEEL_LEFT)
      .value("MOTOR_WHEEL_RIGHT", MOTOR_WHEEL_RIGHT)
      .value("MOTOR_LIFT", MOTOR_LIFT)
      .value("MOTOR_WAIST_DOWN", MOTOR_WAIST_DOWN)
      .value("MOTOR_WAIST_UP", MOTOR_WAIST_UP)
      .value("MOTOR_HEAD_DOWN", MOTOR_HEAD_DOWN)
      .value("MOTOR_HEAD_UP", MOTOR_HEAD_UP)
      .value("MOTOR_LEFT_ARM_1", MOTOR_LEFT_ARM_1)
      .value("MOTOR_LEFT_ARM_2", MOTOR_LEFT_ARM_2)
      .value("MOTOR_LEFT_ARM_3", MOTOR_LEFT_ARM_3)
      .value("MOTOR_LEFT_ARM_4", MOTOR_LEFT_ARM_4)
      .value("MOTOR_LEFT_ARM_5", MOTOR_LEFT_ARM_5)
      .value("MOTOR_LEFT_ARM_6", MOTOR_LEFT_ARM_6)
      .value("MOTOR_LEFT_ARM_7", MOTOR_LEFT_ARM_7)
      .value("MOTOR_LEFT_ARM_8", MOTOR_LEFT_ARM_8)
      .value("MOTOR_RIGHT_ARM_1", MOTOR_RIGHT_ARM_1)
      .value("MOTOR_RIGHT_ARM_2", MOTOR_RIGHT_ARM_2)
      .value("MOTOR_RIGHT_ARM_3", MOTOR_RIGHT_ARM_3)
      .value("MOTOR_RIGHT_ARM_4", MOTOR_RIGHT_ARM_4)
      .value("MOTOR_RIGHT_ARM_5", MOTOR_RIGHT_ARM_5)
      .value("MOTOR_RIGHT_ARM_6", MOTOR_RIGHT_ARM_6)
      .value("MOTOR_RIGHT_ARM_7", MOTOR_RIGHT_ARM_7)
      .value("MOTOR_RIGHT_ARM_8", MOTOR_RIGHT_ARM_8)
      .value("MOTOR_COUNT", MOTOR_COUNT);
}

void bind_types(py::module_& m) {
  py::class_<SystemInitStatus>(m, "SystemInitStatus").def(py::init<>())
      .def_readwrite("last_state", &SystemInitStatus::last_state)
      .def_readwrite("current_state", &SystemInitStatus::current_state)
      .def_readwrite("change_reason", &SystemInitStatus::change_reason)
      .def_readwrite("error_code", &SystemInitStatus::error_code)
      .def_readwrite("state_changed_at_ns", &SystemInitStatus::state_changed_at_ns)
      .def_readwrite("timestamp_ns", &SystemInitStatus::timestamp_ns);
  py::class_<ControlModeStatus>(m, "ControlModeStatus").def(py::init<>())
      .def_readwrite("last_state", &ControlModeStatus::last_state)
      .def_readwrite("current_state", &ControlModeStatus::current_state)
      .def_readwrite("state_changed_at_ns", &ControlModeStatus::state_changed_at_ns)
      .def_readwrite("timestamp_ns", &ControlModeStatus::timestamp_ns);
  py::class_<CommunicationBoardState>(m, "CommunicationBoardState")
      .def(py::init<>())
      .def_property(
          "error_flag",
          [](CommunicationBoardState& v) {
            return array_to_list(v.error_flag);
          },
          [](CommunicationBoardState& v, const py::sequence& s) {
            sequence_to_array(s, v.error_flag, "error_flag");
          })
      .def_readwrite("timestamp_ns", &CommunicationBoardState::timestamp_ns);
  py::class_<Motor_Information>(m, "Motor_Information").def(py::init<>())
      .def_readwrite("Position_Actual", &Motor_Information::Position_Actual)
      .def_readwrite("Speed_Actual", &Motor_Information::Speed_Actual)
      .def_readwrite("Torque_Actual", &Motor_Information::Torque_Actual)
      .def_readwrite("KP_Actual", &Motor_Information::KP_Actual)
      .def_readwrite("KD_Actual", &Motor_Information::KD_Actual)
      .def_readwrite("Error_flag", &Motor_Information::Error_flag);
  py::class_<Motor_Control>(m, "Motor_Control").def(py::init<>())
      .def_readwrite("Position", &Motor_Control::Position)
      .def_readwrite("Speed", &Motor_Control::Speed)
      .def_readwrite("Torque", &Motor_Control::Torque)
      .def_readwrite("KP", &Motor_Control::KP).def_readwrite("KD", &Motor_Control::KD);
  py::class_<ArmPose>(m, "ArmPose").def(py::init<>())
      .def_readwrite("x", &ArmPose::x).def_readwrite("y", &ArmPose::y)
      .def_readwrite("z", &ArmPose::z).def_readwrite("roll", &ArmPose::roll)
      .def_readwrite("pitch", &ArmPose::pitch).def_readwrite("yaw", &ArmPose::yaw);
  py::class_<power_charge_state>(m, "power_charge_state").def(py::init<>())
      .def_property("status", [](const power_charge_state& s) { return static_cast<int>(s.status); },
                    [](power_charge_state& s, int value) {
                      s.status = static_cast<power_charge_state::ChargeStatus>(value);
                    })
      .def_readwrite("temperature", &power_charge_state::temperature)
      .def_readwrite("soc", &power_charge_state::soc);

  py::class_<ArmEndPose>(m, "ArmEndPose").def(py::init<>())
      .def_property("position", [](ArmEndPose& v) { return array_to_list(v.PositionData); },
                    [](ArmEndPose& v, const py::sequence& s) { sequence_to_array(s, v.PositionData, "position"); })
      .def_property("rotation", [](ArmEndPose& v) { return array_to_list(v.RotationData); },
                    [](ArmEndPose& v, const py::sequence& s) { sequence_to_array(s, v.RotationData, "rotation"); })
      .def_readwrite("timestamp_ns", &ArmEndPose::timestamp_ns);
  py::class_<HighLevelState>(m, "HighLevelState").def(py::init<>())
      .def_readwrite("state", &HighLevelState::state)
      .def_readwrite("progress", &HighLevelState::progress)
      .def_readwrite("timestamp_ns", &HighLevelState::timestamp_ns);
  py::class_<RobotInfo>(m, "RobotInfo").def(py::init<>())
      .def_readwrite("robot_name", &RobotInfo::robot_name)
      .def_readwrite("robot_type", &RobotInfo::robot_type)
      .def_readwrite("firmware_version", &RobotInfo::firmware_version)
      .def_readwrite("hardware_version", &RobotInfo::hardware_version)
      .def_readwrite("software_version", &RobotInfo::software_version)
      .def_readwrite("manufacturer", &RobotInfo::manufacturer)
      .def_readwrite("timestamp_ns", &RobotInfo::timestamp_ns);
  py::class_<ImuData>(m, "ImuData").def(py::init<>())
      .def_property("rpy", [](ImuData& v) { return array_to_list(v.rpy); },
                    [](ImuData& v, const py::sequence& s) { sequence_to_array(s, v.rpy, "rpy"); })
      .def_property("omega", [](ImuData& v) { return array_to_list(v.omega); },
                    [](ImuData& v, const py::sequence& s) { sequence_to_array(s, v.omega, "omega"); })
      .def_property("accel", [](ImuData& v) { return array_to_list(v.accel); },
                    [](ImuData& v, const py::sequence& s) { sequence_to_array(s, v.accel, "accel"); })
      .def_property("quat", [](ImuData& v) { return array_to_list(v.quat); },
                    [](ImuData& v, const py::sequence& s) { sequence_to_array(s, v.quat, "quat"); })
      .def_readwrite("error_flag", &ImuData::error_flag)
      .def_readwrite("timestamp_ns", &ImuData::timestamp_ns);
  py::class_<XboxMap>(m, "XboxMap").def(py::init<>())
      .def_readwrite("a", &XboxMap::a).def_readwrite("b", &XboxMap::b)
      .def_readwrite("x", &XboxMap::x).def_readwrite("y", &XboxMap::y)
      .def_readwrite("rightShoulder", &XboxMap::rightShoulder)
      .def_readwrite("leftShoulder", &XboxMap::leftShoulder)
      .def_readwrite("leftStick", &XboxMap::leftStick).def_readwrite("rightStick", &XboxMap::rightStick)
      .def_readwrite("start", &XboxMap::start).def_readwrite("back", &XboxMap::back)
      .def_readwrite("dpadUp", &XboxMap::dpadUp).def_readwrite("dpadDown", &XboxMap::dpadDown)
      .def_readwrite("dpadLeft", &XboxMap::dpadLeft).def_readwrite("dpadRight", &XboxMap::dpadRight)
      .def_readwrite("leftX", &XboxMap::leftX).def_readwrite("leftY", &XboxMap::leftY)
      .def_readwrite("rightX", &XboxMap::rightX).def_readwrite("rightY", &XboxMap::rightY)
      .def_readwrite("leftTrigger", &XboxMap::leftTrigger)
      .def_readwrite("rightTrigger", &XboxMap::rightTrigger)
      .def_readwrite("error_flag", &XboxMap::error_flag)
      .def_readwrite("timestamp_ns", &XboxMap::timestamp_ns);
}

void bind_robot(py::module_& m) {
  using NoDelete = std::unique_ptr<H1Robot, py::nodelete>;
  py::class_<H1Robot, NoDelete>(m, "H1Robot")
      .def_static("get_instance", &H1Robot::getInstance, py::arg("robot_ip") = "127.0.0.1",
                  py::return_value_policy::reference)
      .def("robot_connect", &H1Robot::robot_connect, py::call_guard<py::gil_scoped_release>())
      .def("getControlPermission", &H1Robot::getControlPermission,
           py::call_guard<py::gil_scoped_release>())
      .def("releaseControlPermission", &H1Robot::releaseControlPermission,
           py::call_guard<py::gil_scoped_release>())
      .def("hasControlPermission", &H1Robot::hasControlPermission)
      .def("switchControlMode", &H1Robot::switchControlMode)
      .def("robot_init", &H1Robot::robot_init, py::call_guard<py::gil_scoped_release>())
      .def("robot_deinit", &H1Robot::robot_deinit, py::call_guard<py::gil_scoped_release>())
      .def("setMotorControl_low", &H1Robot::setMotorControl_low)
      .def("setChassis_low", &H1Robot::setChassis_low)
      .def("setWaist_low", &H1Robot::setWaist_low)
      .def("setHead_low", &H1Robot::setHead_low)
      .def("setArm_low", &H1Robot::setArm_low)
      .def("setGripper_low", &H1Robot::setGripper_low, py::arg("gripper_id"), py::arg("control"),
           py::arg("is_hold_torque") = true)
      .def("setFixedRod_low", &H1Robot::setFixedRod_low)
      .def("setChassis_high", &H1Robot::setChassis_high)
      .def("setWaist_high", &H1Robot::setWaist_high)
      .def("setHead_high", &H1Robot::setHead_high)
      .def("setArm_high", &H1Robot::setArm_high)
      .def("setArmMove_high", &H1Robot::setArmMove_high, py::arg("arm"), py::arg("action"),
           py::arg("eef_velocity") = 0.0f, py::arg("eef_acceleration") = 0.0f,
           py::arg("duration") = 0.0f, py::arg("block") = false)
      .def("setGripper_high", &H1Robot::setGripper_high, py::arg("gripper_id"), py::arg("control"),
           py::arg("is_hold_torque") = true)
      .def("setFixedRod_high", &H1Robot::setFixedRod_high)
      .def("armPoseToArmEndPose", &H1Robot::armPoseToArmEndPose)
      .def("CreateCollectSession", &H1Robot::CreateCollectSession,
           py::arg("config"),
           py::call_guard<py::gil_scoped_release>())
      .def("Start", &H1Robot::Start,
           py::call_guard<py::gil_scoped_release>())
      .def("NextStep", &H1Robot::NextStep,
           py::call_guard<py::gil_scoped_release>())
      .def("Pause", &H1Robot::Pause,
           py::call_guard<py::gil_scoped_release>())
      .def("Resume", &H1Robot::Resume,
           py::call_guard<py::gil_scoped_release>())
      .def("Stop", &H1Robot::Stop,
           py::call_guard<py::gil_scoped_release>())
      .def("Close", &H1Robot::Close,
           py::call_guard<py::gil_scoped_release>())
      .def("getRobotInfo", [](H1Robot& r) { return state_result<RobotInfo>(r, &H1Robot::getRobotInfo); })
      .def("getCommunicationBoardState", [](H1Robot& r) {
        return state_result<CommunicationBoardState>(
            r, &H1Robot::getCommunicationBoardState);
      })
      .def("getMotorState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getMotorState(id, s), s); })
      .def("getChassisState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getChassisState(id, s), s); })
      .def("getChassisSpeedState", [](H1Robot& r) { std::array<float, 2> a{}, b{}; return py::make_tuple(r.getChassisSpeedState(a, b), a, b); })
      .def("getWaistState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getWaistState(id, s), s); })
      .def("getHeadState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getHeadState(id, s), s); })
      .def("getArmState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getArmState(id, s), s); })
      .def("getHandRelative", [](H1Robot& r, ArmAction arm) { ArmEndPose s{}; return py::make_tuple(r.getHandRelative(arm, s), s); })
      .def("getHeadRelative", [](H1Robot& r) { return state_result<ArmEndPose>(r, &H1Robot::getHeadRelative); })
      .def("getHandCameraRelative", [](H1Robot& r, ArmAction arm) { ArmEndPose s{}; return py::make_tuple(r.getHandCameraRelative(arm, s), s); })
      .def("getHeadCameraRelative", [](H1Robot& r) { return state_result<ArmEndPose>(r, &H1Robot::getHeadCameraRelative); })
      .def("getGripperState", [](H1Robot& r, EtherCAT_Motor_Index id) { Motor_Information s{}; return py::make_tuple(r.getGripperState(id, s), s); })
      .def("getGripperControlMode", [](H1Robot& r) { int s{}; return py::make_tuple(r.getGripperControlMode(s), s); })
      .def("getFixedRodState", [](H1Robot& r) { int s{}; return py::make_tuple(r.getFixedRodState(s), s); })
      .def("getHighLevelState", [](H1Robot& r) { return state_result<HighLevelState>(r, &H1Robot::getHighLevelState); })
      .def("getJoystickState", [](H1Robot& r) { return state_result<XboxMap>(r, &H1Robot::getJoystickState); })
      .def("getIMU_State", [](H1Robot& r) { return state_result<ImuData>(r, &H1Robot::getIMU_State); })
      .def("getPowerChargeState", [](H1Robot& r) { return state_result<power_charge_state>(r, &H1Robot::getPowerChargeState); })
      .def("getCurrentMode", &H1Robot::getCurrentMode)
      .def("getControlModeStatus", &H1Robot::getControlModeStatus)
      .def("getInitState", &H1Robot::getInitState)
      .def("getSystemInitStatus", &H1Robot::getSystemInitStatus)
      .def("isRobotConnected", &H1Robot::isRobotConnected)
      .def("startCameraDds", &H1Robot::startCameraDds,
           py::call_guard<py::gil_scoped_release>())
      .def("stopCameraDds", &H1Robot::stopCameraDds,
           py::call_guard<py::gil_scoped_release>())
      .def("startCameraWebRTC", &H1Robot::startCameraWebRTC,
           py::call_guard<py::gil_scoped_release>())
      .def("stopCameraWebRTC", &H1Robot::stopCameraWebRTC,
           py::call_guard<py::gil_scoped_release>())
      .def("getLatestCameraFrame", [](H1Robot& r, const std::string& name) { return camera_sample_to_python(r.getLatestCameraFrame(name)); })
      .def("getLatestCameraDepth", [](H1Robot& r, const std::string& name) { return camera_sample_to_python(r.getLatestCameraDepth(name)); })
      .def("getCameraState", [](H1Robot& r, const std::vector<std::string>& names, float timeout) {
        return camera_state_to_dict(r.getCameraState(names, timeout));
      }, py::arg("camera_names") = std::vector<std::string>{}, py::arg("timeout_sec") = 5.0f);
}

void bind_logging(py::module_& m) {
  py::enum_<spdlog::level::level_enum>(m, "LogLevel")
      .value("TRACE", spdlog::level::trace)
      .value("DEBUG", spdlog::level::debug)
      .value("INFO", spdlog::level::info)
      .value("WARN", spdlog::level::warn)
      .value("ERROR", spdlog::level::err)
      .value("CRITICAL", spdlog::level::critical)
      .value("OFF", spdlog::level::off);

  m.def("init_logging", []() { logx::init_once(python_log_module()); });
  m.def("is_logging_initialized", &logx::is_initialized);
  m.def("set_log_level", &logx::set_level, py::arg("level"));
  m.def("flush_log_on", &logx::flush_on, py::arg("level"));
  m.def("log_trace", [](const std::string& message) {
    write_python_log(spdlog::level::trace, message);
  });
  m.def("log_debug", [](const std::string& message) {
    write_python_log(spdlog::level::debug, message);
  });
  m.def("log_info", [](const std::string& message) {
    write_python_log(spdlog::level::info, message);
  });
  m.def("log_warn", [](const std::string& message) {
    write_python_log(spdlog::level::warn, message);
  });
  m.def("log_error", [](const std::string& message) {
    write_python_log(spdlog::level::err, message);
  });
  m.def("log_critical", [](const std::string& message) {
    write_python_log(spdlog::level::critical, message);
  });
  m.def("log_event", [](const std::string& message) {
    write_python_log(spdlog::level::info, message, true);
  });

  py::class_<PythonLoggerFacade>(m, "Logger")
      .def("trace", [](PythonLoggerFacade&, const std::string& format,
                       const py::args& arguments) {
        write_python_log(spdlog::level::trace,
                         format_python_log_message(format, arguments));
      })
      .def("debug", [](PythonLoggerFacade&, const std::string& format,
                       const py::args& arguments) {
        write_python_log(spdlog::level::debug,
                         format_python_log_message(format, arguments));
      })
      .def("info", [](PythonLoggerFacade&, const std::string& format,
                      const py::args& arguments) {
        write_python_log(spdlog::level::info,
                         format_python_log_message(format, arguments));
      })
      .def("warning", [](PythonLoggerFacade&, const std::string& format,
                         const py::args& arguments) {
        write_python_log(spdlog::level::warn,
                         format_python_log_message(format, arguments));
      })
      .def("warn", [](PythonLoggerFacade&, const std::string& format,
                      const py::args& arguments) {
        write_python_log(spdlog::level::warn,
                         format_python_log_message(format, arguments));
      })
      .def("error", [](PythonLoggerFacade&, const std::string& format,
                       const py::args& arguments) {
        write_python_log(spdlog::level::err,
                         format_python_log_message(format, arguments));
      })
      .def("critical", [](PythonLoggerFacade&, const std::string& format,
                          const py::args& arguments) {
        write_python_log(spdlog::level::critical,
                         format_python_log_message(format, arguments));
      })
      .def("fatal", [](PythonLoggerFacade&, const std::string& format,
                       const py::args& arguments) {
        write_python_log(spdlog::level::critical,
                         format_python_log_message(format, arguments));
      })
      .def("event", [](PythonLoggerFacade&, const std::string& format,
                       const py::args& arguments) {
        write_python_log(spdlog::level::info,
                         format_python_log_message(format, arguments), true);
      });

  static PythonLoggerFacade logger;
  m.attr("logger") = py::cast(&logger, py::return_value_policy::reference);
}

PYBIND11_MODULE(lib_h1_sdk_python, m) {
  // ROBOT_LOG_MODULE 必须在 import 前设置；日志目录和文件名由 logx 统一生成。
  logx::init_once(python_log_module());
  m.doc() = "H1 Robot SDK Python bindings (DDS singleton client)";
  bind_enums(m);
  bind_types(m);
  bind_robot(m);
  bind_logging(m);
}
