#include "safety/safety_layer.hpp"
#include "interfaces/input/simulation.hpp"

#include <iostream>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace {
void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "expected control config");
        qhrr05::SafetyLayer safety(argv[1]);
        const double limit =
            YAML::LoadFile(argv[1])["magnetic_touch"]["maximum_force_n"].as<double>();
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (double value : {0., 5., 100.}) safety.ValidateRmdDampingConstant(value);
        for (double value : {-1., 101., 5.5, nan}) {
            bool invalid = false;
            try { safety.ValidateRmdDampingConstant(value); }
            catch (const std::runtime_error&) { invalid = true; }
            Require(invalid, "invalid RMD damping constant was accepted");
        }
        const auto rejected =
            [&](const ControlData& input, bool stopping = false, bool failed = false) {
                try {
                    safety.Validate(input, stopping, failed);
                    return false;
                } catch (const std::runtime_error&) {
                    return true;
                }
            };

        ControlData input;
        for (int axis = 0; axis < 3; ++axis) for (int order : {21, 36})
            for (double delta : {-.5, -.25, -.1, .1, .25, .5}) safety.ValidateAnticoggingPhase(axis, order, delta);
        for (const auto parameters : {std::array<double, 3>{-1, 21, .5}, {3, 21, .5},
                 {0, 22, .5}, {0, 36, .2}, {0, 36, .05}, {0, 36, 0}, {0, 21, nan}}) {
            bool rejected_phase = false;
            try { safety.ValidateAnticoggingPhase(int(parameters[0]), int(parameters[1]), parameters[2]); }
            catch (const std::runtime_error&) { rejected_phase = true; }
            Require(rejected_phase, "invalid live phase command accepted");
        }

        for (int axis = 0; axis < 3; ++axis) for (int order : {21, 36})
            for (double delta : {-.01, .01}) safety.ValidateAnticoggingAmplitude(axis, order, delta);
        for (const auto values : {std::array<double, 3>{-1, 21, .01}, {0, 22, .01},
                {0, 21, .1}, {0, 36, 0}, {0, 21, nan}}) {
            bool rejected_amplitude = false;
            try { safety.ValidateAnticoggingAmplitude(int(values[0]), int(values[1]), values[2]); }
            catch (const std::runtime_error&) { rejected_amplitude = true; }
            Require(rejected_amplitude, "invalid amplitude command accepted");
        }
        for (int mode = 0; mode <= 6; ++mode) {
            input.requested_mode = mode;
            safety.Validate(input, false, false);
        }

        Require(rejected(input, true), "stopped session accepted input");
        Require(rejected(input, false, true), "failed session accepted input");

        for (int mode : {-1, 7}) {
            auto invalid = input;
            invalid.requested_mode = mode;
            Require(rejected(invalid), "invalid mode accepted");
        }

        for (int plane : {-1, 3}) {
            auto invalid = input;
            invalid.requested_tracking_plane = static_cast<TrackingPlane>(plane);
            Require(rejected(invalid), "invalid plane accepted");
        }

        for (double force : {-1.0, limit + .1, nan, std::numeric_limits<double>::infinity()}) {
            auto invalid = input;
            invalid.target_force = force;
            Require(rejected(invalid), "invalid target force accepted");
        }

        input.target_force = limit;
        safety.Validate(input, false, false);
        input.target_force = 0;
        safety.Validate(input, false, false);

        for (int axis = 0; axis < 3; ++axis) {
            for (double value : {-1.01, 1.01, nan}) {
                auto invalid = input;
                invalid.joystick_axes[axis] = value;
                Require(rejected(invalid), "invalid joystick axis accepted");
                invalid = input;
                invalid.requested_ball_position[axis] = nan;
                Require(rejected(invalid), "non-finite ball target accepted");
            }
        }

        for (double value : {-0.01, 1.01, nan}) {
            auto invalid = input;
            invalid.joystick_l2 = value;
            Require(rejected(invalid), "invalid L2 trigger accepted");
            invalid = input;
            invalid.joystick_r2 = value;
            Require(rejected(invalid), "invalid R2 trigger accepted");
        }

        SimulationData simulation;
        SimulationApproval approval;
        SimulationCommand operation;
        const auto rejected_simulation = [&] {
            try {
                qhrr05::ValidateSimulationInput(operation, simulation, approval, 100);
                return false;
            } catch (const std::runtime_error&) {
                return true;
            }
        };
        operation.action = SimulationAction::Step;
        Require(rejected_simulation(), "absent simulator accepted input");
        simulation.published_ns = 1;
        Require(rejected_simulation(), "running simulator accepted single step");
        approval.pause_sequence = 1;
        approval.paused = true;
        qhrr05::ValidateSimulationInput(operation, simulation, approval, 100);
        operation.action = SimulationAction::Perturb;
        auto& perturb = operation.perturb;
        perturb.owner = 7;
        perturb.active = 1;
        const char body[] = "FL_calf";
        std::copy(std::begin(body), std::end(body), perturb.body.begin());
        qhrr05::ValidateSimulationInput(operation, simulation, approval, 100);
        perturb.reference_position[0] = nan;
        Require(rejected_simulation(), "non-finite perturbation accepted");
        perturb.reference_position[0] = 0;
        perturb.reference_quaternion[0] = 0;
        Require(rejected_simulation(), "zero quaternion accepted");
        perturb.reference_quaternion[0] = 1;
        approval.perturb = perturb;
        approval.perturb_expires_ns = 200;
        perturb.owner = 8;
        Require(rejected_simulation(), "another viewer took an active drag");
        qhrr05::ValidateSimulationInput(operation, simulation, approval, 201);

        std::cout
            << "PASS: basic safety only, all modes without homing gate, session/shape/range/finite checks\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
