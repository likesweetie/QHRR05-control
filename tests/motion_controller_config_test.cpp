#include "motion_controllers/homing_controller.hpp"
#include "motion_controllers/gravity_compensation_controller.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main() {
    std::string temporary_directory;
    try {
        char directory[] = "/tmp/qhrr05-config-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        const std::string path = temporary_directory+"/controller config.yaml";
        const auto save = [&](const YAML::Node& config) {
            std::ofstream output(path);
            output << config;
            output.close();
            Require(!output.fail(), "config write failed");
        };
        const auto reject = [](auto construct) {
            bool rejected = false;
            try { construct(); } catch (const std::exception&) { rejected = true; }
            Require(rejected, "controller accepted invalid or missing own settings");
        };
        auto homing = YAML::Load(R"(
robot: qhrr-05
homing:
  kp: [10,20,30]
  kd: 2
  initial_position_rad: [0,.3,-1]
  position_rad: [0,.9,-1.8]
  duration_sec: 2
)");
        save(homing);  // No gravity section: homing is responsible only for itself.
        qhrr05::HomingController home(path);
        double q[] = {0,.3,-1}, dq[] = {.1,.2,.3}, torque[3], reference[3];
        home.Compute(2, q, dq, torque, reference);
        const double expected_home[] = {-.2,11.6,-24.6};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i]-expected_home[i]) < 1e-10, "homing file parameters were not applied");
        homing["kp"] = 999;
        homing["homing"].remove("kp");
        save(homing);
        reject([&] { qhrr05::HomingController invalid(path); });  // No old root-gain fallback.
        homing["homing"]["kp"] = 10;
        homing["homing"]["duration_sec"] = 0;
        save(homing);
        reject([&] { qhrr05::HomingController invalid(path); });

        auto gravity = YAML::Load(R"(
robot: qhrr-05
gravity_compensation:
  kp_motor: 50
  kd_motor: [1,2,3]
  alpha: .5
  task_stiffness: [2,4,6]
)");
        save(gravity);  // No homing section; use independent motor gains.
        qhrr05::GravityCompensationController compensation(path);
        const double desired[] = {.1,.2,.3}, position[] = {.2,.4,.6}, g[] = {1,2,3};
        compensation.Start(desired);
        compensation.Compute(position, dq, g, torque, reference);
        const double expected_gravity[] = {.2,-.2,-1.2};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i]-expected_gravity[i]) < 1e-10, "gravity file parameters were not applied");
        gravity["kd"] = 999;
        gravity["gravity_compensation"].remove("kd_motor");
        save(gravity);
        reject([&] { qhrr05::GravityCompensationController invalid(path); });
        gravity["gravity_compensation"]["kd_motor"] = YAML::Load("[1,2]");
        save(gravity);
        reject([&] { qhrr05::GravityCompensationController invalid(path); });
        reject([&] { qhrr05::HomingController missing(path+".missing"); });
        reject([&] { qhrr05::GravityCompensationController missing(path+".missing"); });
        std::filesystem::remove_all(temporary_directory);
        std::cout << "PASS: path constructors, independent sections/gains, controller-owned validation, no fallback\n";
        return 0;
    } catch (const std::exception& error) {
        if (!temporary_directory.empty()) std::filesystem::remove_all(temporary_directory);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
