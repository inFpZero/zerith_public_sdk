#pragma once
#include <zcm/zcm-cpp.hpp>
#include <memory>
#include <atomic>
#include <functional>
#include "config.hpp"
#include <iostream>


// RobotBase 类定义
// 这是一个抽象基类，定义了轮臂机器人的基本接口和通用功能
class RobotBase {
public:
    explicit RobotBase();
    virtual ~RobotBase() = default;

};
