#include "motion_controllers/homing_controller.hpp"
#include "motion_controllers/gravity_compensation_controller.hpp"
#include "motion_controllers/ball_following_controller.hpp"
#include "motion_controllers/magnetic_touch_controller.hpp"
#include "motion_controllers/tracking_controller.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void Near(const std::array<double, 3>& actual, const std::array<double, 3>& expected,
    const char* message) {
    for (int i = 0; i < 3; ++i)
        Require(std::abs(actual[i] - expected[i]) < 1e-10, message);
}
}

int main(int argc, char** argv) {
    std::filesystem::path temporary;
    try {
        Require(argc == 2, "expected control configuration");
        auto config = YAML::LoadFile(argv[1]);
        auto backend = qhrr05::MakeMujocoBackend(argv[1]);
        const auto values = config["homing"]["position_rad"].as<std::vector<double>>();
        const std::array<double, 3> q{values[0], values[1], values[2]};
        const std::array<double, 3> dq{.1, -.2, .15}, zero{};
        const auto state = backend->Update(q, dq);
        std::array<double, 3> position_reference{}, torque{};

        qhrr05::HomingController homing(argv[1]);
        homing.Start(0, q.data());
        auto displaced = q;
        displaced[0] += .02;
        homing.Compute(10, displaced.data(), dq.data(), {1, 1, 1}, {1000000, 1000000, 1000000},
            position_reference, torque);
        Near(position_reference, q, "homing altered its position trajectory");
        Near(torque, zero, "homing leaked position/velocity feedback into torque");

        qhrr05::GravityCompensationController gravity(argv[1]);
        gravity.Start(displaced.data());
        gravity.Compute(state, position_reference, torque);
        Near(position_reference, q, "gravity accidentally holds its entry position");
        Near(torque, state.gravity, "gravity mixed position error into compensation");

        qhrr05::BallFollowingController ball(argv[1], *backend);
        ball.Start(q);
        ball.Compute(.01, {1, 0, 0}, 0, 0, true, displaced, dq, state.gravity, position_reference, torque);
        Near(position_reference, ball.joint_reference(), "ball mixed torque into position");
        Near(torque, state.gravity, "ball mixed position/velocity error into torque");

        qhrr05::MagneticTouchController magnetic(argv[1]);
        magnetic.Start(0, state.toe_position);
        magnetic.Compute(.001, q, dq, zero, state.gravity, state.bias, state.passive,
            state.friction, state.mass, {}, state.jacobian, state.toe_position, state.joint_min,
            state.joint_max, 0, .001, 3, position_reference, torque);
        Near(position_reference, q, "magnetic torque mode created a joint position servo");
        Near(torque, state.gravity, "free magnetic mode altered compensation");

        temporary = std::filesystem::temp_directory_path() /
                    ("qhrr-joint-reference-" + std::to_string(getpid()) + ".yaml");
        config["model_path"] = std::filesystem::canonical(
            std::filesystem::path(argv[1]).parent_path() /
            config["model_path"].as<std::string>()).string();
        {
            auto runtime = YAML::Clone(config);
            runtime["control"]["frequency_hz"] = 1000;
            runtime["tracking"]["trajectory"]["period_sec"] = 1;
            runtime["tracking"]["trajectory"]["ramp_sec"] = .02;
            runtime["tracking"]["ilc"] = YAML::Load(R"(
enabled: true
phase_bins: 16
warmup_cycles: 0
position_gain_nm_rad: [1, 1, 1]
retention: 1
phase_lead_bins: 0
smoothing_passes: 0
correction_limit_nm: [100, 100, 100]
update_limit_nm: [100, 100, 100]
blend_sec: 0.01
)");
            std::ofstream(temporary) << runtime;
            qhrr05::TrackingILC ilc(temporary.string());
            constexpr double turn = 6.2831853071795864769;
            const auto first = [&](int i) { return 1 + .25 * std::cos(turn * (i + .5) / 16); };
            const auto second = [&](int i) { return .2 * std::sin(turn * (i + .5) / 16); };
            for (int i = 0; i < 16; ++i) {
                ilc.Compute(1.01 + (i + .5) / 16);
                ilc.Record({first(i), 0, 0}, 0, false);
            }
            for (int i = 0; i < 16; ++i) {
                ilc.Compute(2.01 + (i + .5) / 16);
                if (i == 8) {
                    const auto before = ilc.torque();
                    const auto updates = ilc.updates();
                    ilc.SetParameters(2, 1, .5);
                    Near(ilc.torque(), before, "ILC parameter edit caused an immediate torque jump");
                    Require(ilc.updates() == updates && updates == 1,
                        "ILC parameter edit reset the learned table");
                    for (const auto& values : std::array<std::array<double, 3>, 5>{{
                        {-1, 1, .5}, {5, 1, .5}, {2, 33, .5}, {2, 1, 0}, {2, 1, 1.1}}}) {
                        bool rejected = false;
                        try { ilc.SetParameters(static_cast<int>(values[0]),
                            static_cast<int>(values[1]), values[2]); }
                        catch (const std::exception&) { rejected = true; }
                        Require(rejected && ilc.phase_lead_bins() == 2 &&
                            ilc.smoothing_passes() == 1 && ilc.retention() == .5,
                            "invalid ILC update was accepted or partially applied");
                    }
                }
                ilc.Record({second(i), 0, 0}, 0, false);
            }
            ilc.Compute(3.01 + 4.5 / 16);
            const auto candidate = [&](int i) { return .5 * first(i) + second((i + 2) % 16); };
            const double expected = .25 * candidate(3) + .5 * candidate(4) + .25 * candidate(5);
            Near(ilc.torque(), {expected, 0, 0},
                "next ILC update did not use the new lead, smoothing and retention together");
            Require(ilc.updates() == 2, "mid-cycle ILC edit lost collected samples");
            ilc.Reset();
            Require(ilc.phase_lead_bins() == 2 && ilc.smoothing_passes() == 1 &&
                ilc.retention() == .5 && ilc.updates() == 0,
                "Tracking reentry lost runtime ILC parameters");
        }
        config["tracking"]["ilc"]["enabled"] = false;
        for (const char* mode : {"inverse_dynamics", "impedance", "ctc"}) {
            config["tracking"]["mode"] = mode;
            std::ofstream(temporary) << config;
            qhrr05::TrackingController tracking(temporary.string(), *backend);
            tracking.Start(0, state.toe_position, q);
            for (int tick = 0; tick < 4097 && tracking.checking(); ++tick)
                tracking.Compute(0, q, dq, state.gravity, state.jacobian, state.toe_position, position_reference, torque);
            Require(!tracking.checking() && !tracking.ik_failed(), "tracking preflight failed");
            tracking.Compute(.4, q, dq, state.gravity, state.jacobian, state.toe_position, position_reference, torque);
            if (tracking.method() == TrackingMethod::CTC) {
                Near(position_reference, q, "CTC added a second joint position servo");
                auto acceleration = tracking.joint_acceleration_reference();
                for (int i = 0; i < 3; ++i)
                    acceleration[i] += config["tracking"]["ctc"]["kp_per_s2"][i].as<double>() *
                        (tracking.joint_reference()[i] - q[i]) +
                        config["tracking"]["ctc"]["kd_per_s"][i].as<double>() *
                        (tracking.joint_velocity_reference()[i] - dq[i]);
                const auto expected = backend->InverseDynamics(q, dq, acceleration);
                Near(tracking.ctc_torque(), expected, "CTC lost its measured-state PD acceleration feedback");

                auto undamped = YAML::Clone(config);
                undamped["tracking"]["ctc"]["kd_per_s"] = YAML::Load("[0,0,0]");
                std::ofstream(temporary) << undamped;
                qhrr05::TrackingController zero_damping(temporary.string(), *backend);
                zero_damping.Start(0, state.toe_position, q);
                for (int tick = 0; tick < 4097 && zero_damping.checking(); ++tick)
                    zero_damping.Compute(0, q, dq, state.gravity, state.jacobian,
                        state.toe_position, position_reference, torque);
                Require(!zero_damping.checking() && !zero_damping.ik_failed(),
                    "CTC with Kd=0 could not start");
                zero_damping.Compute(.4, q, dq, state.gravity, state.jacobian,
                    state.toe_position, position_reference, torque);
                Near(zero_damping.joint_velocity_reference(), tracking.joint_velocity_reference(),
                    "CTC damping changed the reference trajectory");
                std::array<double, 3> damping_torque{}, expected_difference{};
                for (int i = 0; i < 3; ++i) {
                    damping_torque[i] = tracking.ctc_torque()[i] - zero_damping.ctc_torque()[i];
                    for (int j = 0; j < 3; ++j)
                        expected_difference[i] += state.mass[3 * i + j] *
                            config["tracking"]["ctc"]["kd_per_s"][j].as<double>() *
                            (tracking.joint_velocity_reference()[j] - dq[j]);
                }
                Near(damping_torque, expected_difference,
                    "CTC velocity feedback must be multiplied by the full mass matrix exactly once");
                Require(std::abs(damping_torque[0]) + std::abs(damping_torque[1]) +
                            std::abs(damping_torque[2]) > 1e-6,
                    "CTC damping test did not exercise a nonzero velocity error");
            } else {
                Near(position_reference, q, "Cartesian tracking added a joint position servo");
            }
            tracking.Compute(.5, q, dq, state.gravity, {}, state.toe_position, position_reference, torque);
            Require(tracking.ik_failed(), "singular Jacobian did not enter HOLD");
            tracking.Compute(.6, displaced, dq, state.gravity, {}, state.toe_position, position_reference, torque);
            Near(position_reference, q, "HOLD position changed with measured joint position");
            Near(torque, state.gravity, "HOLD mixed position error into torque");
        }

        // Motor kp=0 is valid only when torque travels through the native field.
        // Check each constructor independently, including every joint index.
        const auto gains = [&](int controller) -> YAML::Node {
            switch (controller) {
            case 0:
                return config["homing"]["kp"];
            case 1:
                return config["gravity_compensation"]["kp_motor"];
            case 2:
                return config["magnetic_touch"]["kp_motor"];
            case 3:
                return config["tracking"]["motor"]["kp_nm_rad"];
            case 4:
                return config["ball_following"]["kp_motor"];
            default:
                throw std::runtime_error("invalid test controller");
            }
        };
        const auto construct = [&](int controller) {
            switch (controller) {
            case 0: {
                qhrr05::HomingController instance(temporary.string());
                break;
            }
            case 1: {
                qhrr05::GravityCompensationController instance(temporary.string());
                break;
            }
            case 2: {
                qhrr05::MagneticTouchController instance(temporary.string());
                break;
            }
            case 3: {
                qhrr05::TrackingController instance(temporary.string(), *backend);
                break;
            }
            case 4: {
                qhrr05::BallFollowingController instance(temporary.string(), *backend);
                break;
            }
            }
        };
        for (int controller = 0; controller < 5; ++controller) {
            const auto original = YAML::Clone(gains(controller));
            for (int axis = 0; axis < 3; ++axis) {
                gains(controller) = YAML::Clone(original);
                gains(controller)[axis] = 0;
                config["torque_reference_mode"] = "torque_ref_as_position";
                std::ofstream(temporary) << config;
                bool rejected = false;
                try {
                    construct(controller);
                } catch (const std::exception& error) {
                    const std::string message = error.what();
                    rejected = message.find("kp") != std::string::npos &&
                               message.find("torque_ref_as_position") != std::string::npos;
                }
                Require(rejected, "position-torque mode accepted kp=0 at construction");

                config["torque_reference_mode"] = "native_torque_ref";
                std::ofstream(temporary) << config;
                construct(controller);
            }
            for (const auto invalid : {YAML::Node(-1), YAML::Node(".nan")}) {
                gains(controller)[0] = invalid;
                std::ofstream(temporary) << config;
                bool rejected = false;
                try {
                    construct(controller);
                } catch (const std::exception&) {
                    rejected = true;
                }
                if (!rejected)
                    throw std::runtime_error("native torque mode accepted invalid kp: controller=" +
                        std::to_string(controller) + ", gains=" + YAML::Dump(gains(controller)));
            }
            gains(controller) = YAML::Clone(original);
        }

        std::filesystem::remove(temporary);
        std::cout << "PASS: independent references and mode-aware kp validation in all constructors\n";
        return 0;
    } catch (const std::exception& error) {
        if (!temporary.empty())
            std::filesystem::remove(temporary);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
