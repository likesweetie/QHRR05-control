#include <yaml-cpp/yaml.h>
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/magnetic_touch_controller.hpp"
#include "simulation/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <unistd.h>
#include <memory>
#include <limits>
#include <stdexcept>

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

qhrr05::RobotModelState State(const mjModel* model, const mjData* data, const mjData* gravity) {
    qhrr05::RobotModelState state;
    std::copy_n(data->qpos, 3, state.joint_position.begin());
    std::copy_n(data->qvel, 3, state.joint_velocity.begin());
    std::copy_n(data->qfrc_bias, 3, state.bias.begin());
    std::copy_n(gravity->qfrc_bias, 3, state.gravity.begin());
    std::copy_n(data->qfrc_passive, 3, state.passive.begin());
    std::copy_n(model->dof_damping, 3, state.damping.begin());
    std::copy_n(model->dof_frictionloss, 3, state.friction.begin());
    return state;
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

        // Evaluate both choices at a moving pose, where bias differs from gravity.
        {
            qhrr05::ResetSimulation(home, model.get(), data.get());
            const std::array<double, 3> speed{1.2, -.8, 1.5};
            std::copy(speed.begin(), speed.end(), data->qvel);
            std::array<double, 3> damping, friction;
            std::copy_n(model->dof_damping, 3, damping.begin());
            std::copy_n(model->dof_frictionloss, 3, friction.begin());
            shm_utils::Mapping mapping("/qhrr05-bias-test-" + std::to_string(getpid()), true);
            auto& shm = *mapping.get();
            std::array<double, 3> io_reference;
            qhrr05::UpdateControlIO(
                shm, model.get(), data.get(), gravity.get(), io_reference, 0, nullptr, true);
            const auto model_state = shm.model[shm.model_sequence % kShmBufferCount];
            std::unique_ptr<mjData, decltype(&mj_deleteData)> rest(
                mj_makeData(model.get()), mj_deleteData);
            std::copy(home.begin(), home.end(), rest->qpos);
            mj_forward(model.get(), rest.get());
            double nonlinear_difference = 0;

            for (int i = 0; i < 3; ++i) {
                Require(std::abs(model_state.gravity_torque[i] - rest->qfrc_bias[i]) < 1e-12,
                    "split forward pipeline did not isolate pure gravity");
                Require(model_state.bias_torque[i] == data->qfrc_bias[i],
                    "moving-state bias was not preserved");
                Require(data->qvel[i] == speed[i] && gravity->qvel[i] == 0 &&
                            model->dof_damping[i] == damping[i] &&
                            model->dof_frictionloss[i] == friction[i],
                    "gravity calculation changed plant velocity/friction/damping");
                nonlinear_difference +=
                    std::abs(model_state.bias_torque[i] - model_state.gravity_torque[i]);
            }

            Require(nonlinear_difference > 1e-5, "test pose has no velocity-dependent bias");
            auto options = YAML::Clone(config);
            options["gravity_compensation"]["alpha"] = .6;
            options["gravity_compensation"]["task_stiffness"] = 3;
            std::array<double, 3> desired = home;

            for (double& value : desired)
                value += .02;

            for (bool gravity_only : {true, false}) {
                options["gravity_compensation"]["gravity_only"] = gravity_only;
                options["magnetic_touch"]["gravity_only"] = gravity_only;
                save(options);
                const auto& selected =
                    gravity_only ? model_state.gravity_torque : model_state.bias_torque;
                qhrr05::GravityCompensationController gravity_controller(config_path);
                gravity_controller.Start(desired.data());
                std::array<double, 3> torque, reference;
                gravity_controller.Compute(
                    State(model.get(), data.get(), rest.get()), torque.data(), reference.data());

                for (int i = 0; i < 3; ++i) {
                    const auto kd = options["gravity_compensation"]["kd_motor"];
                    const double gain = kd.IsScalar() ? kd.as<double>() : kd[i].as<double>();
                    const double expected =
                        .6 * selected[i] + 3 * (desired[i] - home[i]) - gain * speed[i];
                    Require(std::abs(torque[i] - expected) < 1e-10,
                        "gravity controller selected the wrong compensation or lost its PD terms");
                }

                const std::array<double, 3> acceleration{}, motor_torque{};
                qhrr05::MagneticTouchController magnetic(config_path);
                magnetic.Start(0, model_state.toe_position);
                magnetic.Compute(.001,
                    home,
                    speed,
                    acceleration,
                    motor_torque,
                    model_state.gravity_torque,
                    model_state.bias_torque,
                    model_state.passive_torque,
                    model_state.friction_limit,
                    model_state.mass,
                    model_state.jacobian,
                    model_state.toe_position,
                    model_state.joint_min,
                    model_state.joint_max,
                    0,
                    .001,
                    3,
                    torque,
                    reference);

                for (int i = 0; i < 3; ++i) {
                    const double gain =
                        options["magnetic_touch"]["joint_damping_nm_s_rad"][i].as<double>();
                    const double limit = options["limits"]["torque_nm"][i].as<double>();
                    const double expected =
                        std::clamp(selected[i] - gain * speed[i], -limit, limit);
                    Require(std::abs(torque[i] - expected) < 1e-10,
                        "magnetic controller selected the wrong compensation or lost damping");
                }
            }

            for (const auto bad : {YAML::Node("invalid"), YAML::Node()}) {
                options["magnetic_touch"]["gravity_only"] = bad;
                save(options);
                bool rejected = false;

                try {
                    qhrr05::MagneticTouchController invalid(config_path);
                } catch (const std::exception&) {
                    rejected = true;
                }

                Require(rejected, "magnetic gravity_only must be an explicit valid boolean");
            }
        }

        for (double alpha : {0.0, .5, 1.0}) {
            config["gravity_compensation"]["alpha"] = alpha;
            config["gravity_compensation"]["task_stiffness"] = YAML::Load("[3,4,5]");
            save(config);
            qhrr05::GravityCompensationController controller(config_path);
            controller.Start(home.data());
            double q[3], dq[3] = {.1, -.2, .3}, g[3] = {2, -3, 1}, torque[3];

            for (int i = 0; i < 3; ++i)
                q[i] = home[i] + .02 * (i + 1);

            qhrr05::RobotModelState state;
            std::copy_n(q, 3, state.joint_position.begin());
            std::copy_n(dq, 3, state.joint_velocity.begin());
            std::copy_n(g, 3, state.gravity.begin());
            state.bias = state.gravity;
            controller.Compute(state, torque, reference);

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
                    State(model.get(), data.get(), gravity.get()), data->ctrl, reference);

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

        // The new drag compensation is controller-side: check both directions,
        // exact 50% viscous remainder, smooth zero crossing and residual dissipation.
        auto assisted = YAML::Clone(config);
        auto node = assisted["gravity_compensation"];
        node["alpha"] = 1;
        node["gravity_only"] = false;
        node["task_stiffness"] = 0;
        node["kd_motor"] = .05;
        node["damping_ratio"] = .5;
        node["friction_compensation"]["coulomb_ratio"] = .85;
        save(assisted);
        qhrr05::GravityCompensationController drag(config_path);
        qhrr05::RobotModelState state;
        state.damping.fill(2);
        state.friction.fill(.2);
        drag.Start(state.joint_position.data());
        double torque[3];
        for (double speed : {-.5, -.000001, 0.0, .000001, .5}) {
            state.joint_velocity.fill(speed);
            drag.Compute(state, torque, reference);
            for (int i = 0; i < 3; ++i) {
                const double expected = .95 * speed + .17 * std::tanh(speed / .03);
                Require(std::abs(torque[i] - expected) < 1e-12, "wrong signed drag compensation");
                const double net = torque[i] - 2 * speed - .2 * std::tanh(speed / .03);
                Require(net * speed <= 0, "drag compensation injected energy into free motion");
                if (std::abs(speed) < 1e-5)
                    Require(std::abs(torque[i]) < 1e-5, "dry compensation jumps near zero speed");
            }
        }
        std::cout
            << "PASS: signed 50% damping compensation, smooth partial dry friction, zero creep\n";

        node["friction_compensation"]["enabled"] = false;
        save(assisted);
        qhrr05::GravityCompensationController without_friction(config_path);
        state.joint_velocity = {.5, -.5, 0};
        state.bias = {1, -2, 3};
        without_friction.Start(state.joint_position.data());
        without_friction.Compute(state, torque, reference);
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - (state.bias[i] - .05 * state.joint_velocity[i])) < 1e-12,
                "friction OFF changed bias support or retained viscous/dry compensation");
        node["friction_compensation"]["enabled"] = true;
        std::cout
            << "PASS: friction OFF removes both compensation terms and retains bias/motor damping\n";

        const auto pushed_travel = [&](double damping_ratio, double coulomb_ratio) {
            node["damping_ratio"] = damping_ratio;
            node["friction_compensation"]["coulomb_ratio"] = coulomb_ratio;
            save(assisted);
            qhrr05::GravityCompensationController controller(config_path);
            qhrr05::ResetSimulation(home, model.get(), data.get());
            controller.Start(home.data());
            while (data->time < .6) {
                mj_forward(model.get(), data.get());
                controller.Compute(
                    State(model.get(), data.get(), gravity.get()), data->ctrl, reference);
                data->qfrc_applied[0] = data->time < .2 ? 1.0 : 0.0;
                mj_step(model.get(), data.get());
            }
            for (int i = 0; i < mjNWARNING; ++i)
                Require(!data->warning[i].number, "friction-compensated plant became unstable");
            return std::abs(data->qpos[0] - home[0]);
        };
        const double original_travel = pushed_travel(1, 0);
        const double compensated_travel = pushed_travel(.5, .85);
        Require(compensated_travel > 1.1 * original_travel,
            "same MuJoCo plant/push did not move more freely with controller friction compensation");
        std::cout << "PASS: same-plant push travel " << original_travel << " -> "
                  << compensated_travel << " rad; physical damping/friction unchanged\n";
        node["damping_ratio"] = .5;
        node["friction_compensation"]["coulomb_ratio"] = .85;

        // Keep output saturation and malformed configuration checks after removing the observer.
        state.bias.fill(100);
        state.joint_velocity.fill(0);
        drag.Compute(state, torque, reference);
        Require(torque[0] == assisted["limits"]["torque_nm"][0].as<double>(),
            "gravity effort exceeded configured torque limit");
        const auto rejects = [&](const YAML::Node& bad) {
            save(bad);
            bool rejected = false;
            try {
                qhrr05::GravityCompensationController invalid(config_path);
            } catch (const std::exception&) {
                rejected = true;
            }
            Require(rejected, "invalid gravity configuration was silently accepted");
        };
        for (const auto* key : {"damping_ratio", "friction_compensation"}) {
            auto bad = YAML::Clone(assisted);
            bad["gravity_compensation"].remove(key);
            rejects(bad);
        }
        auto missing_switch = YAML::Clone(assisted);
        missing_switch["gravity_compensation"]["friction_compensation"].remove("enabled");
        rejects(missing_switch);
        auto invalid_switch = YAML::Clone(assisted);
        invalid_switch["gravity_compensation"]["friction_compensation"]["enabled"] = "invalid";
        rejects(invalid_switch);
        for (double value : {-.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
            auto bad = YAML::Clone(assisted);
            bad["gravity_compensation"]["damping_ratio"] = value;
            rejects(bad);
        }
        auto obsolete = YAML::Clone(assisted);
        obsolete["gravity_compensation"]["virtual_inertia"]["enabled"] = false;
        rejects(obsolete);
        std::cout << "PASS: total torque limit, invalid ratios and removed option rejection\n";

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
