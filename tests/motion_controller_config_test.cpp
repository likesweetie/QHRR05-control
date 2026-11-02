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
    if (!value)
        throw std::runtime_error(message);
}

int main() {
    std::string temporary_directory;

    try {
        char directory[] = "/tmp/qhrr05-config-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        const std::string path = temporary_directory + "/controller config.yaml";
        const auto save = [&](const YAML::Node& config) {
            std::ofstream output(path);
            output << config;
            output.close();
            Require(!output.fail(), "config write failed");
        };

        const auto reject = [](auto construct) {
            bool rejected = false;

            try {
                construct();
            } catch (const std::exception&) {
                rejected = true;
            }

            Require(rejected, "controller accepted invalid or missing own settings");
        };
        auto homing = YAML::Load(R"(
torque_reference_mode: torque_ref_as_position
robot: qhrr-05
limits:
  torque_nm: 53.7
homing:
  kp: [10,20,30]
  kd: 2
  initial_position_rad: [0,.3,-1]
  position_rad: [0,.9,-1.8]
  duration_sec: 2
)");
        save(homing);  // No gravity section: homing is responsible only for itself.
        qhrr05::HomingController home(path);
        double q[] = {0, .3, -1}, dq[] = {.1, .2, .3};
        const std::array<std::uint64_t, 3> sequence{1, 1, 1}, sample_ns{1000000, 1000000, 1000000};
        std::array<double, 3> reference{}, torque{};


        home.Compute(2, q, dq, sequence, sample_ns, reference, torque);
        const double expected_home[] = {0, 0, 0};
        const std::array<double, 3> expected_position{0, .9, -1.8};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(reference[i] - expected_position[i]) < 1e-12,
                "homing did not keep position separate from torque");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - expected_home[i]) < 1e-10,
                "homing file parameters were not applied");

        Require(!home.torque_control(), "legacy homing must default to MIT position control");
        homing["homing"]["torque_control"] = true;
        save(homing);
        qhrr05::HomingController torque_home(path);
        torque_home.Start(1, q);
        torque_home.Compute(1, q, dq, sequence, sample_ns, reference, torque);
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] + 2 * dq[i]) < 1e-12,
                "torque homing must start at measured position with measured-velocity damping");
        torque_home.Compute(2, q, dq, sequence, sample_ns, reference, torque);
        const double midpoint_torque[] = {-.2, 5.6, -12.6};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - midpoint_torque[i]) < 1e-10,
                "torque homing lost PD or introduced trajectory velocity feedforward");
        torque_home.Compute(3, q, dq, sequence, sample_ns, reference, torque);
        const double hold_torque[] = {-.2, 11.6, -24.6};
        Require(torque_home.Reference(3).holding,
            "torque homing must retain its target after the trajectory finishes");
        for (int i = 0; i < 3; ++i) {
            Require(std::abs(reference[i] - expected_position[i]) < 1e-12,
                "torque homing changed the final target position");
            Require(std::abs(torque[i] - hold_torque[i]) < 1e-10,
                "torque homing stopped applying PD at HOLD");
        }
        homing["limits"]["torque_nm"] = YAML::Load("[.1,.2,.3]");
        save(homing);
        qhrr05::HomingController limited_torque_home(path);
        limited_torque_home.Compute(2, q, dq, sequence, sample_ns, reference, torque);
        const double limited_pd[] = {-.1, .2, -.3};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - limited_pd[i]) < 1e-12,
                "torque homing bypassed shared per-axis torque limits");
        homing["limits"]["torque_nm"] = 53.7;

        // Isolate the host D term to check the actual encoder sampling path.
        {
            auto difference = YAML::Clone(homing);
            difference["torque_reference_mode"] = "native_torque_ref";
            difference["limits"]["torque_nm"] = 10000;
            difference["homing"]["kp"] = 0;
            difference["homing"]["kd"] = 1;
            difference["homing"]["differentiate_position"] = true;
            difference["homing_velocity_lpf"]["enabled"] = false;
            difference["homing_velocity_lpf"]["cutoff_hz"] = 0;
            save(difference);
            qhrr05::HomingController differentiated(path);
            double measured[] = {6.28, .3, -1};
            const double reported_velocity[] = {11, -12, 13};
            std::array<std::uint64_t, 3> seq{1, 1, 1}, ns{1000000000, 1000000000, 1000000000};
            const auto check = [&](qhrr05::HomingController& controller,
                                   const std::array<double, 3>& expected, const char* message) {
                controller.Compute(1, measured, reported_velocity, seq, ns, reference, torque);
                for (int i = 0; i < 3; ++i)
                    Require(std::abs(torque[i] - expected[i]) < 1e-9, message);
            };
            differentiated.Start(0, measured);
            check(differentiated, {0, 0, 0}, "first difference sample must have zero velocity");
            for (int i = 0; i < 3; ++i) {
                measured[i] += (i == 1 ? -1 : 1) * .01 * (i + 1);
                ++seq[i];
                ns[i] += 10000000 * (i + 1);
            }
            check(differentiated, {-1, 1, -1},
                "difference must use accumulated q across 2*pi and each axis sample interval");
            check(differentiated, {-1, 1, -1}, "duplicate samples must retain estimated velocity");
            measured[1] -= .03;
            ++seq[1];
            ns[1] += 10000000;
            check(differentiated, {-1, 3, -1}, "one fresh axis must not update the other axes");

            // A restart or non-increasing timestamp seeds new history, never a spike.
            for (double& value : measured)
                value += .5;
            ++seq[0];  // Same timestamp.
            ++seq[1];
            ns[1] -= 50000000;
            seq[2] = 1;
            ns[2] += 10000000;
            check(differentiated, {0, 0, 0}, "sample discontinuity must reset differentiation");
            for (int i = 0; i < 3; ++i) {
                measured[i] += i == 1 ? -.01 : .01;
                ++seq[i];
                ns[i] += 10000000;
            }
            check(differentiated, {-1, 1, -1}, "differentiation failed after sample reset");
            differentiated.Start(1, measured);
            check(differentiated, {0, 0, 0}, "homing re-entry must reset differentiation");

            for (double cutoff : {20.0, 40.0}) {
                difference["homing_velocity_lpf"]["enabled"] = true;
                difference["homing_velocity_lpf"]["cutoff_hz"] = cutoff;
                save(difference);
                qhrr05::HomingController filtered(path);
                filtered.Start(0, measured);
                check(filtered, {0, 0, 0}, "filtered difference must start at zero");
                for (int i = 0; i < 3; ++i) {
                    measured[i] += .02;
                    ++seq[i];
                    ns[i] += 10000000;
                }
                const double alpha = -std::expm1(-2 * 3.14159265358979323846 * cutoff * .01);
                const double expected = -2 * alpha;
                check(filtered, {expected, expected, expected}, "configured LPF step response is wrong");
                check(filtered, {expected, expected, expected}, "duplicate samples advanced the LPF");
                for (int i = 0; i < 3; ++i) {
                    ++seq[i];
                    ns[i] += 10000000;
                }
                const double decayed = expected * (1 - alpha);
                check(filtered, {decayed, decayed, decayed}, "stationary samples must decay filtered velocity");
            }
            difference["homing"]["differentiate_position"] = false;
            difference["homing_velocity_lpf"]["cutoff_hz"] = 20;
            save(difference);
            qhrr05::HomingController filtered_feedback(path);
            filtered_feedback.Start(0, measured);
            check(filtered_feedback, {-11, 12, -13},
                "feedback mode must initialize from the reported motor velocity");
            for (int i = 0; i < 3; ++i) {
                measured[i] += .1;
                ++seq[i];
                ns[i] += 10000000;
            }
            check(filtered_feedback, {-11, 12, -13},
                "feedback mode must not differentiate position");

            for (const auto& cutoff : {YAML::Node(0), YAML::Node(-1), YAML::Load(".nan")}) {
                difference["homing_velocity_lpf"]["cutoff_hz"] = cutoff;
                save(difference);
                reject([&] { qhrr05::HomingController invalid(path); });
            }
            std::cout << "PASS: homing per-axis difference, LPF, sample reset and feedback selection\n";
        }
        homing["homing"].remove("torque_control");

        homing["kp"] = 999;
        homing["homing"].remove("kp");
        save(homing);
        reject([&] {
            qhrr05::HomingController invalid(path);
        });  // No old root-gain fallback.

        homing["homing"]["kp"] = 10;
        homing["homing"]["duration_sec"] = 0;
        save(homing);
        reject([&] {
            qhrr05::HomingController invalid(path);
        });

        auto gravity = YAML::Load(R"(
torque_reference_mode: torque_ref_as_position
robot: qhrr-05
limits:
  torque_nm: 53.7
gravity_compensation:
  gravity_only: true
  kp_motor: 50
  kd_motor: [1,2,3]
  alpha: .5
  friction_compensation:
    enabled: true
    coulomb_ratio: 0
    velocity_scale_rad_s: .03
)");
        save(gravity);  // No homing section; use independent motor gains.
        qhrr05::GravityCompensationController compensation(path);
        const double desired[] = {.1, .2, .3}, position[] = {.2, .4, .6}, g[] = {1, 2, 3};
        compensation.Start(desired);
        qhrr05::RobotModelState state;
        std::copy_n(position, 3, state.joint_position.begin());
        std::copy_n(dq, 3, state.joint_velocity.begin());
        std::copy_n(g, 3, state.gravity.begin());
        state.bias = state.gravity;
        compensation.Compute(state, reference, torque);
        const double expected_gravity[] = {.5, 1, 1.5};
        Require(reference == state.joint_position,
            "gravity must follow the measured position without an entry-pose servo");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - expected_gravity[i]) < 1e-10,
                "gravity file parameters were not applied");

        auto limited_gravity = YAML::Clone(gravity);
        limited_gravity["limits"]["torque_nm"] = YAML::Load("[.05,.1,.2]");
        save(limited_gravity);
        qhrr05::GravityCompensationController limited(path);
        limited.Start(desired);
        limited.Compute(state, reference, torque);
        const double clipped_gravity[] = {.05, .1, .2};
        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - clipped_gravity[i]) < 1e-12,
                "shared per-axis torque limits were not applied");

        auto limited_homing = YAML::Load(R"(
torque_reference_mode: torque_ref_as_position
robot: qhrr-05
limits: {torque_nm: .1}
homing:
  kp: [10,20,30]
  kd: 2
  initial_position_rad: [0,.3,-1]
  position_rad: [0,.9,-1.8]
  duration_sec: 2
)");
        save(limited_homing);
        qhrr05::HomingController limited_home(path);
        limited_home.Compute(2, q, dq, sequence, sample_ns, reference, torque);
        for (double value : torque)
            Require(std::abs(value) <= .1, "homing bypassed shared output limit");

        for (const char* section :
            {"homing", "gravity_compensation", "magnetic_touch", "tracking", "ball_following"}) {
            auto obsolete = YAML::Clone(gravity);
            obsolete[section][section == std::string("tracking") ? "output_torque_limit_nm"
                                                                 : "torque_limit_nm"] = 100;
            save(obsolete);
            reject([&] {
                qhrr05::GravityCompensationController invalid(path);
            });
        }
        for (const auto& value : {YAML::Node(),
                 YAML::Node(0),
                 YAML::Node(-1),
                 YAML::Load("[1,2]"),
                 YAML::Load("[1,.nan,3]")}) {
            auto bad = YAML::Clone(gravity);
            bad["limits"]["torque_nm"] = value;
            save(bad);
            reject([&] {
                qhrr05::GravityCompensationController invalid(path);
            });
        }
        std::cout
            << "PASS: shared scalar/axis limits, homing clipping and legacy/invalid limits rejection\n";

        gravity["gravity_compensation"]["gravity_only"] = "invalid";
        save(gravity);
        reject([&] {
            qhrr05::GravityCompensationController invalid(path);
        });

        gravity["gravity_compensation"].remove("gravity_only");
        save(gravity);
        reject([&] {
            qhrr05::GravityCompensationController invalid(path);
        });

        gravity["gravity_compensation"]["gravity_only"] = true;
        gravity["kd"] = 999;
        gravity["gravity_compensation"].remove("kd_motor");
        save(gravity);
        reject([&] {
            qhrr05::GravityCompensationController invalid(path);
        });

        gravity["gravity_compensation"]["kd_motor"] = YAML::Load("[1,2]");
        save(gravity);
        reject([&] {
            qhrr05::GravityCompensationController invalid(path);
        });

        reject([&] {
            qhrr05::HomingController missing(path + ".missing");
        });

        reject([&] {
            qhrr05::GravityCompensationController missing(path + ".missing");
        });

        std::filesystem::remove_all(temporary_directory);
        std::cout
            << "PASS: path constructors, independent sections/gains, controller-owned validation, no fallback\n";
        return 0;
    } catch (const std::exception& error) {
        if (!temporary_directory.empty())
            std::filesystem::remove_all(temporary_directory);

        std::cerr << error.what() << '\n';

        return 1;
    }
}
