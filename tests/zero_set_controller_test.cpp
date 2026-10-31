#include "motion_controllers/zero_set_controller.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

namespace {
void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    const auto path = "/tmp/qhrr-zero-unit-" + std::to_string(getpid()) + ".yaml";
    try {
        Require(argc == 2, "expected control config");
        auto config = YAML::LoadFile(argv[1]);
        const auto save = [&] {
            std::ofstream(path) << config;
        };
        save();
        qhrr05::ZeroSetController zero(path);
        std::array<double, 3> q{2, -1, 1}, dq{}, tau{};
        std::array<std::uint64_t, 3> seq{1, 1, 1};
        double time = 0;
        zero.Start(time, q);
        const auto tick = [&](double dt = .01) {
            time += dt;
            for (auto& s : seq)
                ++s;
            zero.Update(time, q, dq, tau, seq);
        };
        for (int axis = 0; axis < 3; ++axis) {
            const double direction = config["zero_set"]["direction"][axis].as<double>();
            Require(zero.telemetry().axis == axis, "wrong calibration order");
            Require(zero.telemetry().phase == ZeroSetPhase::Seeking, "not seeking next axis");
            tick();
            Require(
                direction * (zero.reference()[axis] - q[axis]) > 0, "wrong incremental direction");
            for (int i = axis + 1; i < 3; ++i)
                Require(zero.reference()[i] == q[i], "inactive joint reference moved");
            tau[axis] = direction * 7;
            tick();  // Single high-torque sample must not calibrate.
            tau[axis] = 0;
            tick();
            Require(!zero.telemetry().calibrated[axis], "torque spike calibrated a joint");
            tau[axis] = -direction * 7;
            for (int i = 0; i < 20; ++i)
                tick();
            Require(!zero.telemetry().calibrated[axis], "wrong signed torque calibrated a joint");
            tau[axis] = direction * 7;
            dq[axis] = .1;
            for (int i = 0; i < 20; ++i)
                tick();
            Require(!zero.telemetry().calibrated[axis], "moving joint calibrated");
            dq[axis] = 0;
            for (int i = 0; i < 18; ++i)
                tick();
            Require(
                zero.telemetry().phase == ZeroSetPhase::Backoff, "persistent stop not detected");
            const double expected =
                config["zero_set"]["stop_position_rad"][axis].as<double>() - q[axis];
            Require(
                std::abs(zero.telemetry().offset[axis] - expected) < 1e-12, "bad runtime offset");
            tau[axis] = 0;
            for (int i = 0; i < 100 && zero.telemetry().phase == ZeroSetPhase::Backoff; ++i) {
                q[axis] = zero.reference()[axis];
                tick();
            }
        }
        Require(zero.telemetry().phase == ZeroSetPhase::Complete, "calibration did not complete");
        Require(
            zero.telemetry().calibrated == std::array<bool, 3>{true, true, true}, "missing axis");

        zero.Start(time, q);
        auto frozen = seq;
        zero.Update(time + .3, q, dq, tau, frozen);
        zero.Update(time + .6, q, dq, tau, frozen);
        Require(
            zero.telemetry().failure == ZeroSetFailure::FeedbackStale, "stale feedback accepted");
        zero.Start(time, q);
        q[0] += 2;
        tick();
        Require(zero.telemetry().failure == ZeroSetFailure::TravelLimit, "travel overrun accepted");
        config["zero_set"]["timeout_sec"] = YAML::Load("[0.2, 0.2, 0.2]");
        save();
        qhrr05::ZeroSetController timed(path);
        timed.Start(0, q);
        for (int i = 1; i <= 22; ++i) {
            for (auto& s : seq)
                ++s;
            timed.Update(i * .01, q, dq, tau, seq);
        }
        Require(timed.telemetry().failure == ZeroSetFailure::Timeout, "axis timeout ignored");
        config["zero_set"]["direction"][0] = 0;
        save();
        bool rejected = false;
        try {
            qhrr05::ZeroSetController invalid(path);
        } catch (const std::exception&) {
            rejected = true;
        }
        Require(rejected, "invalid direction accepted");
        std::filesystem::remove(path);
        std::cout
            << "PASS: sequential incremental calibration, debounce, direction, backoff, faults and config validation\n";
        return 0;
    } catch (const std::exception& e) {
        std::filesystem::remove(path);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
