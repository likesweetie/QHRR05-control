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
    mj_fullM(model, state.mass.data(), data->qM);
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
                gravity_controller.Compute(State(model.get(), data.get(), rest.get()),
                    {},
                    torque.data(),
                    reference.data());

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
                    const double limit =
                        options["magnetic_touch"]["torque_limit_nm"][i].as<double>();
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
            controller.Compute(state, {}, torque, reference);

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
                    State(model.get(), data.get(), gravity.get()), {}, data->ctrl, reference);

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
            drag.Compute(state, {}, torque, reference);
            for (int i = 0; i < 3; ++i) {
                const double expected = .95 * speed + .17 * std::tanh(speed / .03);
                Require(std::abs(torque[i] - expected) < 1e-12, "wrong signed drag compensation");
                const double net = torque[i] - 2 * speed - .2 * std::tanh(speed / .03);
                Require(net * speed <= 0, "drag compensation injected energy into free motion");
                if (std::abs(speed) < 1e-5)
                    Require(std::abs(torque[i]) < 1e-5, "dry compensation jumps near zero speed");
            }
        }
        Require(drag.inertia_assist() == std::array<double, 3>{},
            "disabled virtual inertia produced assistance");
        std::cout
            << "PASS: signed 50% damping compensation, smooth partial dry friction, zero creep\n";

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
                    State(model.get(), data.get(), gravity.get()), {}, data->ctrl, reference);
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

        // Isolated measured-state plant: actual applied effort and measured
        // acceleration reconstruct the external torque; no GT external-force input.
        node["kd_motor"] = 0;
        node["virtual_inertia"]["enabled"] = true;
        node["virtual_inertia"]["external_torque_deadband_nm"] = 0;
        node["virtual_inertia"]["maximum_velocity_rad_s"] = 20;
        save(assisted);
        qhrr05::GravityCompensationController inertia(config_path);
        state = {};
        state.mass = {1, 0, 0, 0, 2, 0, 0, 0, 3};
        inertia.Start(state.joint_position.data());
        qhrr05::GravityFeedback feedback;
        feedback.dt = .001;
        feedback.acceleration = {.5, -.25, 1.0 / 6};
        const std::array<double, 3> external{.5, -.5, .5};
        for (int step = 1; step <= 1000; ++step) {
            feedback.sequence = step;
            inertia.Compute(state, feedback, torque, reference);
            for (int i = 0; i < 3; ++i) {
                feedback.motor_torque[i] = torque[i];
                feedback.acceleration[i] = (torque[i] + external[i]) / state.mass[4 * i];
                state.joint_velocity[i] += feedback.dt * feedback.acceleration[i];
                state.joint_position[i] += feedback.dt * state.joint_velocity[i];
            }
        }
        for (int i = 0; i < 3; ++i) {
            Require(std::abs(inertia.inertia_assist()[i] - external[i]) < 1e-8,
                "half-inertia control did not double steady external-force acceleration");
            Require(
                std::abs(state.joint_velocity[i]) > 1.6 * std::abs(external[i]) / state.mass[4 * i],
                "virtual inertia did not lighten the measured-state plant response");
        }
        const auto before_duplicate = inertia.inertia_assist();
        feedback.acceleration.fill(1000);  // A duplicate must not update observer state.
        inertia.Compute(state, feedback, torque, reference);
        Require(inertia.inertia_assist() == before_duplicate,
            "duplicate sample changed inertia observer");
        inertia.Start(state.joint_position.data());
        Require(inertia.inertia_assist() == std::array<double, 3>{},
            "mode reentry retained inertia assist");
        std::cout << "PASS: measured-state half inertia, duplicate filtering, mode reset\n";

        // Deadband, aid limits, velocity gate, elapsed-time filtering and invalid
        // inputs are explicit even though production defaults leave inertia OFF.
        node["virtual_inertia"]["ramp_sec"] = .001;
        node["virtual_inertia"]["external_torque_deadband_nm"] = .15;
        node["virtual_inertia"]["maximum_velocity_rad_s"] = 2;
        save(assisted);
        qhrr05::GravityCompensationController limited(config_path);
        state = {};
        state.mass = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        limited.Start(state.joint_position.data());
        feedback = {};
        feedback.dt = .01;
        feedback.acceleration.fill(.1);
        for (int step = 1; step <= 100; ++step) {
            feedback.sequence = step;
            limited.Compute(state, feedback, torque, reference);
        }
        Require(limited.inertia_assist() == std::array<double, 3>{},
            "observer noise bypassed deadband");
        feedback.acceleration.fill(1000);
        for (int step = 101; step <= 120; ++step) {
            feedback.sequence = step;
            limited.Compute(state, feedback, torque, reference);
        }
        Require(
            limited.inertia_assist()[0] == 3, "inertia assistance exceeded or missed its limit");
        state.joint_velocity[0] = 2;
        limited.Compute(state, feedback, torque, reference);
        Require(
            limited.inertia_assist()[0] == 0, "velocity gate did not remove accelerating assist");
        state.joint_velocity[0] = -2;
        limited.Compute(state, feedback, torque, reference);
        Require(limited.inertia_assist()[0] == 3, "velocity gate removed braking assist");
        state.bias.fill(100);
        limited.Compute(state, feedback, torque, reference);
        Require(torque[0] == 53.7, "gravity total effort exceeded configured torque limit");
        state.bias.fill(0);
        limited.Start(state.joint_position.data());
        state.joint_velocity.fill(0);
        feedback.sequence = 1;
        feedback.acceleration.fill(1);
        feedback.dt = .002;
        limited.Compute(state, feedback, torque, reference);
        feedback.sequence += 10;
        limited.Compute(state, feedback, torque, reference);
        const double expected_filtered = -std::expm1(-2 * 3.141592653589793 * 8 * .004);
        Require(std::abs(limited.inertia_assist()[0] - (expected_filtered - .15)) < 1e-12,
            "observer multiplied elapsed dt by skipped sequence count");
        const auto rejects = [&](auto action) {
            bool rejected = false;
            try {
                action();
            } catch (const std::exception&) {
                rejected = true;
            }
            Require(rejected, "invalid gravity configuration/measurement was silently accepted");
        };
        feedback.sequence = 0;
        rejects([&] {
            limited.Compute(state, feedback, torque, reference);
        });
        feedback.sequence = 12;
        feedback.dt = .1;
        rejects([&] {
            limited.Compute(state, feedback, torque, reference);
        });
        feedback.dt = .001;
        feedback.acceleration[0] = std::numeric_limits<double>::quiet_NaN();
        rejects([&] {
            limited.Compute(state, feedback, torque, reference);
        });
        for (const auto* key : {"damping_ratio", "friction_compensation", "virtual_inertia"}) {
            auto bad = YAML::Clone(assisted);
            bad["gravity_compensation"].remove(key);
            save(bad);
            rejects([&] {
                qhrr05::GravityCompensationController invalid(config_path);
            });
        }
        for (double value : {-.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
            auto bad = YAML::Clone(assisted);
            bad["gravity_compensation"]["damping_ratio"] = value;
            save(bad);
            rejects([&] {
                qhrr05::GravityCompensationController invalid(config_path);
            });
        }
        auto bad = YAML::Clone(assisted);
        bad["gravity_compensation"]["gravity_only"] = true;
        save(bad);
        rejects([&] {
            qhrr05::GravityCompensationController invalid(config_path);
        });
        std::cout
            << "PASS: inertia limits, deadband, speed gate, elapsed dt, explicit invalid-input rejection\n";

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
