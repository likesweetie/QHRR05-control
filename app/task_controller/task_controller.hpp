#pragma once

#include "motion_controllers/homing_controller.hpp"
#include <memory>

namespace qhrr05 {
class TaskController {
public:
    TaskController(const YAML::Node& settings, int fd);
    ~TaskController();
    TaskController(const TaskController&) = delete;
    TaskController& operator=(const TaskController&) = delete;
    bool ReadInput();
    void Control();
    void WriteOutput();
private:
    int fd_;
    std::unique_ptr<HomingController> homing_;
    // time, position[3], velocity[3], bias[3]; torque[3], reference[3].
    std::array<double, 10> input_{};
    std::array<double, 6> output_{};
};
}  // namespace qhrr05
