#include "../common/example_cli.hpp"
#include "H1_Robot.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace {

using namespace std::chrono_literals;

bool CheckCommand(bool accepted, const char* command) {
    if (!accepted) {
        std::cerr << "Data collection command failed: " << command << std::endl;
    }
    return accepted;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string robot_ip = h1_sdk_examples::parseRobotIp(argc, argv);
    auto& robot = H1Robot::getInstance(robot_ip);
    if (!robot.robot_connect()) {
        std::cerr << "Robot connection failed." << std::endl;
        return 1;
    }

    const std::string config = R"({
        "root_dir":"/data/zerith_data",
        "task_id":990001,
        "task_name":"sdk_demo",
        "subtask_num":2,
        "max_episode_time":60,
        "frequency":30,
        "min_free_disk_gb":0
    })";
    if (!robot.CreateCollectSession(config)) {
        std::cerr << "Failed to create data collection session." << std::endl;
        return 1;
    }
    if (!robot.Start()) {
        std::cerr << "Failed to start data collection." << std::endl;
        return 1;
    }

    std::cout << "Collecting subtask 1 for 5 seconds..." << std::endl;
    std::this_thread::sleep_for(5s);
    if (!CheckCommand(robot.NextStep(), "NextStep (subtask 1)")) {
        robot.Close();
        return 1;
    }

    // Give DataEngine time to commit the first subtask transition before pause.
    std::this_thread::sleep_for(1s);
    if (!CheckCommand(robot.Pause(), "Pause")) {
        robot.Close();
        return 1;
    }

    std::cout << "Collection paused for 3 seconds..." << std::endl;
    std::this_thread::sleep_for(3s);
    if (!CheckCommand(robot.Resume(), "Resume")) {
        robot.Close();
        return 1;
    }

    // Give DataEngine time to apply resume before recording the second subtask.
    std::this_thread::sleep_for(1s);
    std::cout << "Collecting subtask 2 for 5 seconds..." << std::endl;
    std::this_thread::sleep_for(5s);
    if (!CheckCommand(robot.NextStep(), "NextStep (subtask 2)")) {
        robot.Close();
        return 1;
    }

    // The second NextStep completes subtask_num=2. DataEngine then finalizes
    // HDF5/video output and publishes EPISODE_END. Stop is intentionally not
    // sent because its semantics are to end the episode as user_stopped.
    std::cout << "All subtasks completed; waiting for DataEngine to finalize "
                 "the episode..." << std::endl;
    std::this_thread::sleep_for(10s);
    robot.Close();

    std::cout << "Collection flow finished. Confirm EPISODE_END contains "
                 "is_success=true in the log above." << std::endl;
    return 0;
}
