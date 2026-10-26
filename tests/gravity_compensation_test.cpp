#include <yaml-cpp/yaml.h>
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "mujoco_simulate/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <memory>
#include <stdexcept>

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    std::string temporary_directory;

    try {
        char directory[] = "/tmp/qhrr05-gravity-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        const std::string config_path = temporary_directory + "/control.yaml";
        const auto save = [&](const YAML::Node& config) {
            std::ofstream output(config_path);
            output << config;
            output.close();
            Require(!output.fail(), "test config write failed");
        };

        Require(argc == 3, "expected model and config");
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity(
            mj_makeData(model.get()), mj_deleteData);
        auto config = YAML::LoadFile(argv[2]);
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        double reference[3];

        for (double alpha : {0.0, .5, 1.0}) {
            config["gravity_compensation"]["alpha"] = alpha;
            config["gravity_compensation"]["task_stiffness"] = YAML::Load("[3,4,5]");
            save(config);
            qhrr05::GravityCompensationController controller(config_path);
            controller.Start(home.data());
            double q[3], dq[3] = {.1, -.2, .3}, g[3] = {2, -3, 1}, torque[3];

            for (int i = 0; i < 3; ++i)
                q[i] = home[i] + .02 * (i + 1);

            controller.Compute(q, dq, g, torque, reference);

            for (int i = 0; i < 3; ++i) {
                const auto kd = config["gravity_compensation"]["kd_motor"];
                const double damping = kd.IsScalar() ? kd.as<double>() : kd[i].as<double>();
                Require(std::abs(torque[i] - (alpha * g[i] + (3 + i) * (home[i] - q[i]) -
                                                 damping * dq[i])) < 1e-10,
                    "surrogate position servo gives wrong gravity/task/damping torque");
            }
        }

        // With no task stiffness, compensation supports weight but permits motion.
        config["gravity_compensation"]["task_stiffness"] = 0;

        for (int scenario = 0; scenario < 3; ++scenario) {
            const double alpha = scenario == 1 ? 0.0 : 1.0;
            config["gravity_compensation"]["alpha"] = alpha;
            save(config);
            qhrr05::GravityCompensationController controller(config_path);
            controller.Start(home.data());
            qhrr05::ResetSimulation(home, model.get(), data.get());

            while (data->time < 3) {
                mju_copy(gravity->qpos, data->qpos, 3);
                mju_zero(gravity->qvel, 3);
                mj_forward(model.get(), gravity.get());
                controller.Compute(
                    data->qpos, data->qvel, gravity->qfrc_bias, data->ctrl, reference);

                for (int i = 0; i < 3; ++i)
                    data->ctrl[i] = std::clamp(data->ctrl[i],
                        model->actuator_ctrlrange[2 * i],
                        model->actuator_ctrlrange[2 * i + 1]);

                data->qfrc_applied[0] = scenario == 2 && data->time < .5 ? 3.0 : 0.0;
                mj_step(model.get(), data.get());
            }

            double drift = 0;

            for (int i = 0; i < 3; ++i)
                drift = std::max(drift, std::abs(data->qpos[i] - home[i]));

            std::cout << "alpha=" << alpha << " external_push=" << (scenario == 2)
                      << " drift=" << drift << '\n';
            Require(scenario == 0 ? drift < 1e-7 : drift > .05,
                "compensated leg must hold under gravity; uncompensated leg must fall");

            if (scenario == 2)
                Require(std::abs(data->qpos[0] - home[0]) > .1,
                    "zero-stiffness compensation must permit displacement by external force");

            for (int i = 0; i < mjNWARNING; ++i)
                Require(!data->warning[i].number, "MuJoCo warning");
        }

        config["gravity_compensation"]["alpha"] = 1.1;
        bool rejected = false;
        save(config);

        try {
            qhrr05::GravityCompensationController invalid(config_path);
        } catch (const std::runtime_error&) {
            rejected = true;
        }

        Require(rejected, "out-of-range alpha must be rejected");
        std::filesystem::remove_all(temporary_directory);

        return 0;
    } catch (const std::exception& e) {
        if (!temporary_directory.empty())
            std::filesystem::remove_all(temporary_directory);

        std::cerr << e.what() << '\n';

        return 1;
    }
}
