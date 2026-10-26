#pragma once

#include <cstdlib>
#include <iostream>
#include <string>

namespace h1_sdk_examples {

// Edit this value to change the source-level default for every C++ example.
inline constexpr const char* kDefaultRobotIp = "127.0.0.1";

inline void printIpUsage(const char* program) {
  std::cout << "Usage: " << program << " [--ip IP]\n"
            << "  --ip IP  Robot IP address (default: " << kDefaultRobotIp
            << ")\n";
}

inline std::string parseRobotIp(int argc, char** argv) {
  std::string robot_ip = kDefaultRobotIp;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      printIpUsage(argv[0]);
      std::exit(0);
    }
    if (argument != "--ip") {
      std::cerr << "Unknown argument: " << argument << '\n';
      printIpUsage(argv[0]);
      std::exit(2);
    }
    if (++i >= argc || std::string(argv[i]).empty()) {
      std::cerr << "--ip requires a non-empty value\n";
      printIpUsage(argv[0]);
      std::exit(2);
    }
    robot_ip = argv[i];
  }
  return robot_ip;
}

}  // namespace h1_sdk_examples
