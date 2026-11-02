#include "control/control.hpp"
#include "motor_can_protocol/openrobot_protocol.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>

namespace {
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void Near(double actual, double expected, const char* message) {
    if (std::abs(actual - expected) > 1e-9)
        throw std::runtime_error(std::string(message) + ": " + std::to_string(actual) +
            " != " + std::to_string(expected));
}
void Save(const std::filesystem::path& path, const YAML::Node& config) {
    std::ofstream file(path);
    file << config;
    file.close();
    Require(!file.fail(), "cannot write isolated test config");
}
}

int main(int argc, char** argv) {
    std::filesystem::path temporary;
    try {
        Require(argc == 3, "expected fixture control YAML and robot model");
        char directory[] = "/tmp/qhrr-anticogging-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "cannot create test directory");
        temporary = directory;
        const auto path = temporary / "control.yaml", motor_path = temporary / "motor.yaml";
        auto config = YAML::LoadFile(argv[1]);
        config["model_path"] = std::filesystem::absolute(argv[2]).string();
        auto motor = YAML::Load("mit: {gear_ratio: 5, kt_output_nm_a: 2}");
        Save(motor_path, motor);
        config["anticogging"] = YAML::Load(R"(
enabled: true
motor_config: motor.yaml
scale: 0.2
max_motor_comp_nm: 0.1
max_comp_current_a: 0.7
motor_zero_rad: [0.1, -0.2, 0.3]
amp21_motor_nm: [0.02, 0.03, 0.04]
phase21_rad: [0.4, -0.7, 1.0]
amp36_motor_nm: [0.01, 0.02, 0.03]
phase36_rad: [0.3, 0.9, -0.8]
)");
        Save(path, config);
        qhrr05::TwoOrderAnticogging model(path.string());
        const double turn = 2 * std::acos(-1.0);
        for (int axis = 0; axis < 3; ++axis) for (double q : {-2.4, .11, 1.9}) {
            const auto s = config["anticogging"];
            const double theta = 5 * q + s["motor_zero_rad"][axis].as<double>();
            const double disturbance = s["amp21_motor_nm"][axis].as<double>() *
                std::sin(21 * theta + s["phase21_rad"][axis].as<double>()) +
                s["amp36_motor_nm"][axis].as<double>() *
                std::sin(36 * theta + s["phase36_rad"][axis].as<double>());
            Near(model.motorDisturbanceNm(axis, q), disturbance, "two orders or per-axis phase changed");
            Near(model.compensationIqA(axis, q), -.2 * disturbance / .4, "opposing current or motor Kt wrong");
            Near(model.compensationOutputNm(axis, q), -disturbance, "output torque conversion wrong");
            Near(model.compensationOutputNm(axis, q + turn / 5),
                model.compensationOutputNm(axis, q), "motor mechanical periodicity changed");
        }
        Near(model.compensationOutputNm(0, std::numeric_limits<double>::quiet_NaN()), 0,
            "nonfinite measurement did not suppress compensation");
        const auto original = YAML::Clone(config);
        // Fixed-q experiments isolate phase tuning from real shaft motion.
        auto phase_config = YAML::Clone(config);
        phase_config["anticogging"]["scale"] = 1.0;
        phase_config["anticogging"]["phase_slew_rad_s"] = 5.0;
        phase_config["anticogging"]["motor_zero_rad"] = 0;
        phase_config["anticogging"]["amp21_motor_nm"] = .05;
        phase_config["anticogging"]["amp36_motor_nm"] = 0;
        phase_config["anticogging"]["phase21_rad"] = 0;
        phase_config["anticogging"]["phase36_rad"] = 0;
        auto phase_motor = YAML::Load("mit: {gear_ratio: 9, kt_output_nm_a: 1.298}");
        Save(motor_path, phase_motor);
        const double one_step_bound = 2 * 9 * .05 * std::sin(.5 / 2);
        double largest_slew = 0, largest_can_step = 0;
        for (int order : {21, 36}) {
            phase_config["anticogging"]["amp21_motor_nm"] = order == 21 ? .05 : 0;
            phase_config["anticogging"]["amp36_motor_nm"] = order == 36 ? .05 : 0;
            Save(path, phase_config);
            qhrr05::TwoOrderAnticogging phased(path.string());
            const double q = -.25 / (order * 9);
            const double before = phased.compensationOutputNm(0, q);
            auto jump_config = YAML::Clone(phase_config);
            jump_config["anticogging"][order == 21 ? "phase21_rad" : "phase36_rad"] = .5;
            Save(path, jump_config);
            const double after_jump = qhrr05::TwoOrderAnticogging(path.string()).compensationOutputNm(0, q);
            Near(std::abs(after_jump - before), one_step_bound, "instantaneous phase jump estimate wrong");
            phased.setPhaseTarget(0, order, .5);
            Near(phased.phase(0, order), 0, "target update instantly changed applied phase");
            Near(phased.compensationOutputNm(0, q), before, "target update instantly changed torque");
            double previous = before, encoded_previous = openrobot::DecodeMitCommand(
                openrobot::EncodeMitCommand({0, 0, 0, 0, before}, 45, 1, 3), 45, 1, 3).tau_ff;
            for (int step = 1; step <= 100; ++step) {
                phased.update(.001);
                const double next = phased.compensationOutputNm(0, q);
                const double change = std::abs(next - previous);
                Require(change <= 9 * .05 * 5 * .001 + 1e-12, "phase slew caused a torque step");
                largest_slew = std::max(largest_slew, change);
                const double encoded = openrobot::DecodeMitCommand(
                    openrobot::EncodeMitCommand({0, 0, 0, 0, next}, 45, 1, 3), 45, 1, 3).tau_ff;
                largest_can_step = std::max(largest_can_step, std::abs(encoded - encoded_previous));
                Require(std::abs(encoded - encoded_previous) <= 6.0 / 255 + 1e-12,
                    "smooth phase command skipped a native CAN quantization level");
                previous = next; encoded_previous = encoded;
            }
            Near(phased.phase(0, order), .5, "0.5-rad phase change did not finish in 0.1 sec");
            Near(phased.phase(1, order), 0, "phase tuning affected another axis");
            phased.setPhaseTarget(0, order, -.5);
            const double held = phased.compensationOutputNm(0, q);
            phased.update(0);
            Near(phased.compensationOutputNm(0, q), held, "zero elapsed time changed torque");
            phased.update(5);
            Near(phased.phase(0, order), .45, "delayed loop caught up with a large phase jump");
        }
        phase_config["anticogging"]["phase21_rad"] = turn - .2;
        phase_config["anticogging"]["amp21_motor_nm"] = .05;
        Save(path, phase_config);
        qhrr05::TwoOrderAnticogging wrapped(path.string());
        wrapped.setPhaseTarget(0, 21, .1);
        double previous_torque = wrapped.compensationOutputNm(0, 0);
        for (int i = 0; i < 60; ++i) {
            wrapped.update(.001);
            Require(wrapped.phase(0, 21) >= 0 && wrapped.phase(0, 21) < turn,
                "applied phase escaped [0, 2pi)");
            const double next = wrapped.compensationOutputNm(0, 0);
            Require(std::abs(next - previous_torque) <= 9 * .05 * 5 * .001 + 1e-12,
                "2pi boundary created a torque discontinuity");
            previous_torque = next;
        }
        Near(wrapped.phase(0, 21), .1, "zero boundary did not take the shortest phase path");
        wrapped.setPhaseTarget(0, 21, turn);
        Near(wrapped.phase(0, 21, true), 0, "2pi was not normalized to zero");
        wrapped.setPhaseTarget(0, 21, -.5);
        Near(wrapped.phase(0, 21, true), turn - .5, "negative target was not wrapped");
        std::cout << "PHASE CHECK: raw 0.5 rad worst jump=" << one_step_bound
                  << " Nm; observed 1ms slew=" << largest_slew
                  << " Nm; native CAN step=" << largest_can_step << " Nm\n";
        Save(motor_path, motor);
        auto amp_config = YAML::Clone(phase_config);
        amp_config["anticogging"]["amp21_motor_nm"] = 0;
        amp_config["anticogging"]["amp36_motor_nm"] = 0;
        amp_config["anticogging"]["phase21_rad"] = turn / 4;
        amp_config["anticogging"]["phase36_rad"] = 0;
        amp_config["anticogging"]["amplitude_slew_nm_s"] = .1;
        amp_config["anticogging"]["motor_zero_rad"] = 0;
        Save(motor_path, phase_motor);
        Save(path, amp_config);
        qhrr05::TwoOrderAnticogging amplitudes(path.string());
        amplitudes.setAmplitudeTarget(0, 21, .01);
        Near(amplitudes.amplitude(0, 21), 0, "amplitude target caused an immediate jump");
        double amp_previous = amplitudes.compensationOutputNm(0, 0);
        for (int i = 0; i < 100; ++i) {
            amplitudes.update(.001);
            const double next = amplitudes.compensationOutputNm(0, 0);
            Require(std::abs(next - amp_previous) <= 9 * .1 * .001 + 1e-12,
                "amplitude transition exceeded configured slew");
            amp_previous = next;
        }
        Near(amplitudes.amplitude(0, 21), .01, "0.01 motor Nm did not settle in 0.1 sec");
        Near(amplitudes.compensationOutputNm(0, 0), -.09, "amplitude did not reach torque output");
        Near(amplitudes.amplitude(1, 21), 0, "amplitude tuning changed another axis");
        bool negative_rejected = false;
        try { amplitudes.setAmplitudeTarget(0, 21, -.01); }
        catch (const std::exception&) { negative_rejected = true; }
        Require(negative_rejected, "negative amplitude accepted");
        Save(motor_path, motor);
        config = YAML::Clone(original);
        config["anticogging"]["amp21_motor_nm"] = 1;
        config["anticogging"]["amp36_motor_nm"] = 0;
        config["anticogging"]["phase21_rad"] = turn / 4;
        config["anticogging"]["motor_zero_rad"] = 0;
        Save(path, config);
        unsigned clipping = 0;
        Near(qhrr05::TwoOrderAnticogging(path.string()).compensationOutputNm(0, 0, &clipping), -.1,
            "motor-side torque cap was not applied before scale");
        Require(clipping == qhrr05::TwoOrderAnticogging::MotorTorque, "motor torque clip diagnostic wrong");
        config["anticogging"]["max_motor_comp_nm"] = 10;
        config["anticogging"]["max_comp_current_a"] = .03;
        Save(path, config);
        Near(qhrr05::TwoOrderAnticogging(path.string()).compensationOutputNm(0, 0, &clipping), -.012,
            "current cap was not applied before scale");
        Require(clipping == qhrr05::TwoOrderAnticogging::MotorCurrent, "current clip diagnostic wrong");
        config["anticogging"]["scale"] = 0;
        Save(path, config);
        Near(qhrr05::TwoOrderAnticogging(path.string()).compensationOutputNm(0, 0), 0, "scale zero failed");
        config["anticogging"]["enabled"] = false;
        config["anticogging"]["motor_config"] = "nonexistent.yaml";
        Save(path, config);
        Require(!qhrr05::TwoOrderAnticogging(path.string()).enabled(), "disabled config still loaded motor");
        config.remove("anticogging");
        Save(path, config);
        Near(qhrr05::TwoOrderAnticogging(path.string()).compensationOutputNm(1, 0), 0,
            "missing optional configuration changed existing behavior");
        for (const char* key : {"scale", "max_motor_comp_nm", "max_comp_current_a", "amp21_motor_nm",
                 "amp36_motor_nm", "motor_zero_rad", "phase21_rad", "phase36_rad"}) {
            config = YAML::Clone(original);
            config["anticogging"][key] = std::numeric_limits<double>::quiet_NaN();
            Save(path, config);
            bool rejected = false;
            try { qhrr05::TwoOrderAnticogging invalid(path.string()); }
            catch (const std::exception&) { rejected = true; }
            Require(rejected, "nonfinite anticogging configuration was accepted");
        }
        for (const char* key : {"scale", "max_motor_comp_nm", "max_comp_current_a", "amp21_motor_nm", "amp36_motor_nm"}) {
            config = YAML::Clone(original);
            config["anticogging"][key] = -1;
            Save(path, config);
            bool rejected = false;
            try { qhrr05::TwoOrderAnticogging invalid(path.string()); }
            catch (const std::exception&) { rejected = true; }
            Require(rejected, "negative anticogging magnitude was accepted");
        }
        config = YAML::Clone(original);
        Save(path, config);
        for (const char* key : {"gear_ratio", "kt_output_nm_a"}) {
            auto invalid_motor = YAML::Clone(motor);
            invalid_motor["mit"][key] = 0;
            Save(motor_path, invalid_motor);
            bool rejected = false;
            try { qhrr05::TwoOrderAnticogging invalid(path.string()); }
            catch (const std::exception&) { rejected = true; }
            Require(rejected, "invalid motor YAML was accepted");
        }
        motor["mit"]["gear_ratio"] = 9;
        motor["mit"]["kt_output_nm_a"] = 1.298;
        Save(motor_path, motor);
        std::cout << "PASS: signed 21/36 orders, motor YAML, limits, scale, disabled and invalid config\n";

        auto normal = YAML::Clone(original);
        normal["gravity_compensation"]["alpha"] = 0;
        normal["gravity_compensation"]["friction_compensation"]["enabled"] = false;
        normal["magnetic_touch"]["gravity_only"] = true;
        normal["tracking"]["ilc"]["enabled"] = false;
        normal["ball_following"]["workspace_grid_points"] = 9;
        const auto q = normal["homing"]["position_rad"].as<std::vector<double>>();
        normal["anticogging"]["amp21_motor_nm"] = YAML::Load("[0.01, 0.02, 0.03]");
        normal["anticogging"]["amp36_motor_nm"] = 0;
        normal["anticogging"]["phase21_rad"] = turn / 4;
        for (int i = 0; i < 3; ++i)
            normal["anticogging"]["motor_zero_rad"][i] = -9 * q[i];
        int run = 0;
        for (const char* output_mode : {"torque_ref_as_position", "native_torque_ref"})
          for (bool limited : {false, true}) for (bool enabled : {false, true}) {
            config = YAML::Clone(normal);
            config["torque_reference_mode"] = output_mode;
            config["anticogging"]["enabled"] = enabled;
            config["limits"]["torque_nm"] = limited ? .01 : 50;
            if (limited) {
                config["zero_set"]["contact_torque_nm"] = YAML::Load("[0.0025, 0.0025, 0.0025]");
                config["zero_set"]["maximum_push_nm"] = YAML::Load("[0.005, 0.005, 0.005]");
            }
            Save(path, config);
            const double limit = limited ? .01 : 50;
            const auto name = "/qhrr-anticogging-" + std::to_string(::getpid()) + "-" + std::to_string(run++);
            shm_utils::Mapping mapping(name, true);
            auto& shm = *mapping.get();
            shm.can_enabled = true;
            auto& feedback = shm.motors[0];
            feedback.running = true;
            feedback.supported.fill(kCanSignals);
            feedback.valid.fill(kCanSignals);
            feedback.coordinate_valid.fill(true);
            feedback.enabled.fill(true);
            feedback.profile_verified.fill(true);
            for (int i = 0; i < 3; ++i) feedback.feedback[i].position = q[i];
            qhrr05::TaskController controller(path.string(), name);
            const auto cycle = [&] {
                {
                    shm_utils::Lock lock(shm);
                    auto& data = shm_utils::Next(shm.motors, shm.motor_sequence);
                    for (int i = 0; i < 3; ++i) {
                        ++data.received_sequence[i];
                        data.received_ns[i] = data.measurement_ns[i] = shm_utils::MonotonicNs();
                    }
                    ++shm.motor_sequence;
                }
                Require(controller.ReadInput(), "isolated controller stopped");
                controller.Control(); controller.WriteOutput();
                return shm.motors[shm.motor_sequence % kShmBufferCount];
            };
            for (const auto& command : cycle().command) Near(command.tau_ff, 0, "disarmed torque was added");
            qhrr05::InputInterface::RequestDrive(shm, DriveAction::Arm);
            for (int mode : {5, 0, 1, 2, 3, 4, 6}) {
                {
                    shm_utils::Lock lock(shm);
                    auto& request = shm_utils::Next(shm.control, shm.control_sequence);
                    request.requested_mode = mode;
                    ++request.motion_sequence; ++shm.control_sequence;
                }
                const auto motors = cycle();
                const auto& state = shm.control[shm.control_sequence % kShmBufferCount];
                Require(state.mode == mode && state.drive_state == DriveState::Active,
                    "test failed to enter requested motion");
                for (int i = 0; i < 3; ++i) {
                    const auto& command = motors.command[i];
                    if (mode == 6) {
                        Near(command.tau_ff, 0, "anticogging entered Zero set");
                        continue;
                    }
                    const double anti = enabled ? -.018 * (i + 1) : 0;
                    if (mode == 0 || mode == 1 || mode == 5)
                        Near(motors.debug_torque[i], std::clamp(anti, -limit, limit),
                            "compensation was not added exactly once before limiting");
                    const double requested = command.kp * (motors.reference[i] - q[i]) + motors.debug_torque[i];
                    const double applied = command.kp * (command.position - q[i]) + command.tau_ff;
                    Near(applied, std::clamp(requested, -limit, limit), "combined P plus compensation bypassed limit");
                    Near(command.tau_ff, mode == 5 || std::string(output_mode) == "native_torque_ref"
                        ? motors.debug_torque[i] : 0, "compensation used wrong position/torque output path");
                    Near(command.velocity, 0, "compensation introduced velocity target");
                }
            }
            // Damping can run before calibration, but phase-dependent compensation cannot.
            {
                shm_utils::Lock lock(shm);
                auto& data = shm_utils::Next(shm.motors, shm.motor_sequence);
                data.coordinate_valid.fill(false); ++shm.motor_sequence;
                auto& request = shm_utils::Next(shm.control, shm.control_sequence);
                request.requested_mode = 5; ++request.motion_sequence; ++shm.control_sequence;
            }
            for (const auto& command : cycle().command) Near(command.tau_ff, 0, "uncalibrated phase was used");
            qhrr05::InputInterface::RequestDrive(shm, DriveAction::Release);
            for (const auto& command : cycle().command) Near(command.tau_ff, 0, "release kept compensation");
        }
        std::filesystem::remove_all(temporary);
        std::cout << "PASS: all motion paths, both output modes, on/off, shared limits and exclusions\n";
        return 0;
    } catch (const std::exception& error) {
        if (!temporary.empty()) std::filesystem::remove_all(temporary);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
