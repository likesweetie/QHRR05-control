#include <yaml-cpp/yaml.h>
#include <array>
#include "motion_controllers/homing_controller.hpp"
#include "simulation/simulation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <memory>
#include <stdexcept>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    std::string temporary_directory;

    try {
        char directory[] = "/tmp/qhrr05-homing-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        const std::string config_path = temporary_directory + "/control.yaml";
        const auto save = [&](const YAML::Node& config) {
            std::ofstream output(config_path);
            output << config;
            output.close();
            Require(!output.fail(), "test config write failed");
        };

        Require(argc == 3, "expected model and control config");
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, error);
        auto config = YAML::LoadFile(argv[2]);
        const auto initial =
            qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        auto invalid = YAML::Load("[0,100,-1.8]");
        bool rejected = false;

        try {
            qhrr05::ReadPosition(invalid, model.get());
        } catch (const std::runtime_error&) {
            rejected = true;
        }

        Require(rejected, "home outside joint limits must be rejected");
        const double duration = config["homing"]["duration_sec"].as<double>();
        const auto original_home = YAML::Clone(config["homing"]["position_rad"]);

        for (int scenario = 0; scenario < 6; ++scenario) {
            config["homing"]["position_rad"] = scenario % 2
                ? YAML::Load("[.1,.6,-1.5]") : YAML::Clone(original_home);
            config["homing"]["torque_control"] = scenario >= 2;
            config["homing"]["differentiate_position"] = scenario >= 4;
            config["homing_velocity_lpf"]["enabled"] = scenario >= 4;
            config["homing_velocity_lpf"]["cutoff_hz"] = 20;

            const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
            save(config);
            qhrr05::HomingController controller(config_path);
            std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
                mj_makeData(model.get()), mj_deleteData);
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            const auto start = controller.Reference(0);
            const auto finish = controller.Reference(duration);

            for (int i = 0; i < 3; ++i) {
                Require(start.position[i] == initial[i], "trajectory starts at wrong pose");
                Require(std::abs(finish.position[i] - home[i]) < 1e-12,
                    "trajectory ends at wrong pose");
                Require(start.velocity[i] == 0 && finish.velocity[i] == 0,
                    "endpoint velocity must be zero");
            }

            double max_torque = 0;
            std::array<std::uint64_t, 3> sequence{}, sample_ns{};

            while (data->time < duration + 4) {
                mj_forward(model.get(), data.get());
                std::array<double, 3> position_reference{}, torque{};
                for (int i = 0; i < 3; ++i) {
                    ++sequence[i];
                    sample_ns[i] = static_cast<std::uint64_t>(std::llround(data->time * 1e9)) + 1;
                }
                controller.Compute(data->time, data->qpos, data->qvel, sequence, sample_ns,
                    position_reference, torque);
                for (int i = 0; i < 3; ++i) {
                    const double kp = controller.kp()[i];
                    const double limit = config["limits"]["torque_nm"][i].as<double>();
                    const double position = position_reference[i] + torque[i] / kp;
                    const MitCommand command = controller.torque_control()
                        ? MitCommand{position_reference[i], 0, 0, 0, torque[i]}
                        : MitCommand{std::clamp(position,
                            data->qpos[i] - limit / kp, data->qpos[i] + limit / kp),
                            0, kp, controller.kd()[i], 0};
                    data->ctrl[i] = command.kp * (command.position - data->qpos[i]) -
                                    command.kd * data->qvel[i] + command.tau_ff;
                }

                if (data->time == 0)
                    for (int i = 0; i < 3; ++i)
                        Require(data->ctrl[i] == 0,
                            "homing must not compensate gravity at its initial target");

                for (int i = 0; i < 3; ++i)
                    data->ctrl[i] = std::clamp(data->ctrl[i],
                        model->actuator_ctrlrange[2 * i],
                        model->actuator_ctrlrange[2 * i + 1]);

                mj_step(model.get(), data.get());
                mj_forward(model.get(), data.get());

                for (int i = 0; i < 3; ++i) {
                    Require(std::isfinite(data->qpos[i]), "non-finite homing state");
                    Require(data->ctrl[i] >= model->actuator_ctrlrange[2 * i] &&
                                data->ctrl[i] <= model->actuator_ctrlrange[2 * i + 1],
                        "torque limit exceeded");
                    max_torque = std::max(max_torque, std::abs(data->actuator_force[i]));
                }
            }

            double max_error = 0;

            for (int i = 0; i < 3; ++i) {
                max_error = std::max(max_error, std::abs(data->qpos[i] - home[i]));
                Require(std::abs(data->qvel[i]) < .01, "home pose has not settled");
            }

            std::cout << "scenario=" << scenario << " home_error=" << max_error
                      << " peak_torque=" << max_torque << '\n';
            // Pure PD has a static gravity deflection. Bound it by the actual
            // gravitational load plus dry friction, rather than assuming zero error.
            const auto gains = config["homing"]["kp"].as<std::vector<double>>();

            for (int i = 0; i < 3; ++i)
                Require(gains[i] * std::abs(data->qpos[i] - home[i]) <=
                            std::abs(data->qfrc_bias[i]) + model->dof_frictionloss[i] + .05,
                    "PD hold error exceeds gravity/friction equilibrium bound");

            for (int i = 0; i < mjNWARNING; ++i)
                Require(data->warning[i].number == 0, "MuJoCo warning");
        }

        std::filesystem::remove_all(temporary_directory);

        return 0;
    } catch (const std::exception& e) {
        if (!temporary_directory.empty())
            std::filesystem::remove_all(temporary_directory);

        std::cerr << e.what() << '\n';

        return 1;
    }
}
