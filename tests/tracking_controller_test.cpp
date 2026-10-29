#include "motion_controllers/tracking_controller.hpp"
#include "simulation/simulation.hpp"
#include "simulation/telemetry.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <unistd.h>

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void CheckILC(const YAML::Node& config, const std::string& directory) {
    auto settings = YAML::Clone(config);
    settings["tracking"]["trajectory"]["period_sec"] = 1;
    settings["tracking"]["trajectory"]["ramp_sec"] = 1;
    auto ilc = settings["tracking"]["ilc"];
    ilc["enabled"] = true;
    ilc["phase_bins"] = 16;
    ilc["warmup_cycles"] = 0;
    ilc["position_gain_nm_rad"] = YAML::Load("[1,1,1]");
    ilc["velocity_gain_nm_s_rad"] = YAML::Load("[0,0,0]");
    ilc["smoothing_passes"] = 0;
    ilc["correction_limit_nm"] = YAML::Load("[.2,.2,.2]");
    ilc["update_limit_nm"] = YAML::Load("[.1,.1,.1]");
    const auto path = directory + "/ilc-check.yaml";
    std::ofstream(path) << settings;
    qhrr05::TrackingILC learning(path);
    const std::array<double, 3> zero{}, error{1, 0, 0};
    learning.Compute(1.49);
    learning.Record(error, zero, .01, false);
    Require(learning.state() == TrackingILCState::Warmup && learning.torque() == zero,
        "ILC learned during the ramp/partial initial lap");

    for (int lap = 0; lap < 5; ++lap) {
        for (int bin = 0; bin < 16; ++bin) {
            if (lap == 2 && bin == 5)
                continue;  // An incomplete lap must never be stitched into a valid trial.

            learning.Compute(1.5 + lap + (bin + .5) / 16);
            learning.Record(error, zero, .01, lap == 1);
            Require(std::abs(learning.torque()[0]) <= .2 + 1e-12,
                "ILC exceeded its independent torque bound");
        }

        learning.Compute(2.5 + lap);

        if (lap == 0) {
            Require(learning.updates() == 1 && learning.limited() && learning.torque() == zero,
                "ILC updated within a lap, ignored its step cap, or jumped at the first update");
            learning.Compute(2.500001);
            Require(std::abs(learning.torque()[0]) < 1e-8,
                "ILC update blend is discontinuous at the cycle boundary");
        } else if (lap == 1) {
            Require(learning.updates() == 1 && learning.skipped_cycles() == 1 &&
                        learning.state() == TrackingILCState::SkippedSaturation &&
                        std::abs(learning.torque()[0] - .1) < 1e-12,
                "saturated ILC trial grew the table or reverted to an older waveform");
        } else if (lap == 2) {
            Require(learning.updates() == 1 && learning.skipped_cycles() == 2 &&
                        learning.state() == TrackingILCState::SkippedSamples,
                "ILC learned from missing phase samples");
        }
    }

    Require(learning.updates() == 3 && learning.limited(), "ILC torque cap did not limit continued learning");
    learning.Hold();
    learning.Compute(100);
    Require(learning.state() == TrackingILCState::IKHold && learning.torque() == zero,
        "ILC continued output after IK hold");
    learning.Reset();
    Require(learning.updates() == 0 && learning.skipped_cycles() == 0 && learning.torque() == zero,
        "ILC reset retained learned corrections/counters");
    std::cout << "PASS: ILC full-lap learning, ramp exclusion, blend, caps, saturation/missing-bin skip, IK hold/reset\n";
}

int main(int argc, char** argv) {
    std::string temporary_directory;

    try {
        Require(argc == 3, "expected config and model paths");
        auto plots = std::make_unique<qhrr05::viewer::LivePlots>(2.0);
        qhrr05::viewer::Sample sample;
        sample.mode = 3;
        sample.dynamics_enabled = true;
        sample.dynamics_torque = {-60, 12, -7};
        sample.commanded_torque = {53, -20, 8};
        plots->Push(sample);
        sample.time = 1;
        sample.dynamics_torque = {10, -30, 5};
        plots->Push(sample);
        std::array<double, 3> peak;
        Require(plots->DynamicsPeak(peak) && peak == std::array<double, 3>{60, 30, 7},
            "IDFF plot peak lost signed torque magnitude");
        const auto headroom = plots->TorqueHeadroom({53, 53, 53}, 100);
        Require(headroom.find("-7.0 (-13%) OVER") != std::string::npos &&
                    headroom.find("Ctrl cmd left now [Nm]: +0.0 / +33.0 / +45.0") != std::string::npos,
            "IDFF headroom did not preserve negative margin or separate total command");
        sample.time = 2.01;
        plots->Push(sample);
        Require(plots->DynamicsPeak(peak) && peak == std::array<double, 3>{10, 30, 5},
            "expired IDFF peaks remained in the sliding window");
        sample.dynamics_enabled = false;
        sample.dynamics_torque.fill(0);
        plots->Push(sample);
        Require(plots->DynamicsPeak(peak) && peak[1] == 30 &&
                    plots->TorqueHeadroom({53, 53, 53}, 100).find("OFF - current N/A") != std::string::npos,
            "IDFF off/hold silently replaced historical peaks with zero");
        ++sample.motion_sequence;
        plots->Push(sample);
        Require(!plots->DynamicsPeak(peak), "Tracking restart reused old torque peaks");
        sample.tracking_method = TrackingMethod::CTC;
        sample.dynamics_enabled = true;
        sample.dynamics_torque = {5, 6, 7};
        plots->Push(sample);
        Require(plots->DynamicsPeak(peak) && peak == std::array<double, 3>{5, 6, 7} &&
                    plots->TorqueHeadroom({53, 53, 53}, 100).find("CTC raw (before clipping)") != std::string::npos,
            "CTC torque headroom was mislabeled as IDFF or mixed old samples");
        plots->Clear();
        Require(!plots->DynamicsPeak(peak), "clearing plots retained a torque peak");
        std::cout << "PASS: IDFF sampled window, negative headroom, off/hold, restart/reset\n";

        for (int mode : {2, 4}) {
            plots->Clear();
            sample = {};
            sample.mode = mode;
            sample.time = 1;
            plots->Push(sample);
            sample.time = 1.01;
            sample.toe_position[0] = .01;
            plots->Push(sample);
            mjvGeom geometry[4]{};
            mjvScene scene{};
            scene.geoms = geometry;
            scene.maxgeom = 4;
            plots->AddToeTrail(scene);
            Require(scene.ngeom == 1, "current mode history disappeared");
            ++sample.motion_sequence;
            sample.time = 1.02;
            sample.toe_position[0] = .02;
            plots->Push(sample);
            scene.ngeom = 0;
            plots->AddToeTrail(scene);
            Require(scene.ngeom == 0, "Magnetic/Ball reentry retained previous plot history");
        }

        plots->Clear();
        std::cout << "PASS: Magnetic/Ball plot and trail history resets on reentry\n";

        const auto config = YAML::LoadFile(argv[1]);
        qhrr05::TrackingController tracking_on(argv[1]);
        char directory[] = "/tmp/qhrr05-tracking-XXXXXX";
        Require(::mkdtemp(directory) != nullptr, "temporary directory failed");
        temporary_directory = directory;
        CheckILC(config, temporary_directory);
        const std::string off_path = temporary_directory + "/control.yaml";
        auto off_config = YAML::Clone(config);
        off_config["model_path"] = std::filesystem::absolute(argv[2]).string();
        off_config["tracking"]["mode"] = "impedance";
        std::ofstream(off_path) << off_config;
        qhrr05::TrackingController tracking_off(off_path);
        auto constant_speed_config = YAML::Clone(off_config);
        constant_speed_config["tracking"]["trajectory"]["timing"] = "constant_speed";
        const auto constant_speed_path = temporary_directory + "/constant-speed.yaml";
        std::ofstream(constant_speed_path) << constant_speed_config;
        qhrr05::TrackingController constant_speed(constant_speed_path);
        auto faster_config = YAML::Clone(off_config);
        faster_config["tracking"]["trajectory"]["period_sec"] = .3;
        const auto faster_path = temporary_directory + "/faster.yaml";
        std::ofstream(faster_path) << faster_config;
        qhrr05::TrackingController faster(faster_path);
        auto ctc_config = YAML::Clone(config);
        ctc_config["model_path"] = std::filesystem::absolute(argv[2]).string();
        ctc_config["tracking"]["mode"] = "ctc";
        ctc_config["tracking"]["inverse_dynamics"]["alpha"] = 0;
        const auto ctc_path = temporary_directory + "/ctc.yaml";
        std::ofstream(ctc_path) << ctc_config;
        qhrr05::TrackingController tracking_ctc(ctc_path);
        Require(tracking_ctc.method() == TrackingMethod::CTC && tracking_ctc.joint_ik_enabled() &&
                    !tracking_ctc.inverse_dynamics_enabled(),
            "CTC incorrectly depends on the IDFF mode/alpha options");
        auto ilc_config = YAML::Clone(config);
        ilc_config["model_path"] = std::filesystem::absolute(argv[2]).string();
        ilc_config["tracking"]["ilc"]["enabled"] = true;
        const auto id_ilc_path = temporary_directory + "/id-ilc.yaml";
        std::ofstream(id_ilc_path) << ilc_config;
        qhrr05::TrackingController tracking_id_ilc(id_ilc_path);
        ilc_config["tracking"]["mode"] = "ctc";
        const auto ctc_ilc_path = temporary_directory + "/ctc-ilc.yaml";
        std::ofstream(ctc_ilc_path) << ilc_config;
        qhrr05::TrackingController tracking_ctc_ilc(ctc_ilc_path);
        ilc_config["tracking"]["mode"] = "impedance";
        const auto impedance_ilc_path = temporary_directory + "/impedance-ilc.yaml";
        std::ofstream(impedance_ilc_path) << ilc_config;
        qhrr05::TrackingController tracking_impedance_ilc(impedance_ilc_path);
        Require(tracking_impedance_ilc.method() == TrackingMethod::Impedance &&
                    tracking_impedance_ilc.ilc().enabled() && tracking_impedance_ilc.joint_ik_enabled() &&
                    !tracking_impedance_ilc.inverse_dynamics_enabled(),
            "impedance + ILC was disabled or changed to inverse dynamics");
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> gravity_data(
            mj_makeData(model.get()), mj_deleteData);

        std::unique_ptr<mjData, decltype(&mj_deleteData)> desired_data(
            mj_makeData(model.get()), mj_deleteData);

        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        const std::array<double, 3> zero{};
        const std::array<double, 9> zero_jacobian{};
        std::array<double, 3> torque{};
        const double period = config["tracking"]["trajectory"]["period_sec"].as<double>();
        const double ramp_time = config["tracking"]["trajectory"]["ramp_sec"].as<double>();
        qhrr05::ResetSimulation(home, model.get(), data.get());
        const int toe_site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        std::array<double, 3> home_toe;
        std::array<double, 9> home_jacobian;
        std::copy_n(data->site_xpos + 3 * toe_site, 3, home_toe.begin());
        mj_jacSite(model.get(), data.get(), home_jacobian.data(), nullptr, toe_site);

        const auto prepare = [&](qhrr05::TrackingController& controller, double time,
                                 const std::array<double, 3>& toe, const std::array<double, 3>& q,
                                 TrackingPlane plane = TrackingPlane::XY) {
            controller.Start(time, toe, q, plane);
            std::copy(q.begin(), q.end(), desired_data->qpos);
            mj_forward(model.get(), desired_data.get());
            std::array<double, 9> actual_jacobian;
            mj_jacSite(model.get(), desired_data.get(), actual_jacobian.data(), nullptr, toe_site);

            for (int tick = 0; tick < 4097 && controller.checking(); ++tick)
                controller.Compute(time, q, zero, zero, actual_jacobian, toe, torque);

            Require(!controller.checking(), "trajectory check did not finish in its sample budget");
        };

        for (const auto plane : {TrackingPlane::XY, TrackingPlane::XZ, TrackingPlane::YZ}) {
            const auto axes = TrackingPlaneAxes(plane);
            const double a = config["tracking"]["trajectory"]["amplitude_m"][axes[0]].as<double>();
            const double b = config["tracking"]["trajectory"]["amplitude_m"][axes[1]].as<double>();
            double length = 0;

            // Independent fine polygon length, not the controller's Simpson table.
            for (int i = 0; i < 20000; ++i) {
                const double p = 2 * std::acos(-1.0) * i / 20000;
                const double q = 2 * std::acos(-1.0) * (i + 1) / 20000;
                length += std::hypot(a * (std::sin(q) - std::sin(p)),
                    b * (std::sin(2 * q) - std::sin(2 * p)));
            }

            const double speed = length / period;
            prepare(constant_speed, 0, home_toe, home, plane);
            prepare(tracking_off, 0, home_toe, home, plane);

            for (auto* controller : {&constant_speed, &tracking_off}) {
                controller->Compute(0, home, zero, zero, home_jacobian, home_toe, torque);
                Require(controller->toe_reference() == home_toe &&
                            controller->toe_velocity_reference() == zero &&
                            controller->toe_acceleration_reference() == zero,
                    "timing profile did not start at rest at the center crossing");
            }

            std::array<double, 3> previous{};
            double minimum_chord_speed = speed, maximum_chord_speed = speed;
            double acceleration_sum[2]{}, acceleration_squared[2]{}, acceleration_peak[2]{};
            const double dt = period / 4096;

            for (int i = 0; i <= 4096; ++i) {
                constant_speed.Compute(ramp_time + (i + .37) * dt,
                    home, zero, zero, home_jacobian, home_toe, torque);
                const auto& position = constant_speed.toe_reference();
                const auto& velocity = constant_speed.toe_velocity_reference();
                const auto& acceleration = constant_speed.toe_acceleration_reference();
                Require(std::abs(mju_norm3(velocity.data()) / speed - 1) < 1e-6,
                    "reference speed is not constant along the figure eight");
                Require(std::abs(mju_dot3(velocity.data(), acceleration.data())) < 1e-9,
                    "constant-speed reference retained tangential acceleration");

                if (i) {
                    std::array<double, 3> delta;

                    for (int axis = 0; axis < 3; ++axis)
                        delta[axis] = position[axis] - previous[axis];

                    const double chord_speed = mju_norm3(delta.data()) / dt;
                    minimum_chord_speed = std::min(minimum_chord_speed, chord_speed);
                    maximum_chord_speed = std::max(maximum_chord_speed, chord_speed);
                }

                previous = position;

                tracking_off.Compute(ramp_time + (i + .37) * dt,
                    home, zero, zero, home_jacobian, home_toe, torque);
                const double u = (tracking_off.toe_reference()[axes[0]] - home_toe[axes[0]]) / a;
                const double v = (tracking_off.toe_reference()[axes[1]] - home_toe[axes[1]]) / (2 * b);
                Require(std::abs(v * v - u * u * (1 - u * u)) < 1e-12,
                    "acceleration timing changed the figure-eight geometry");

                if (i < 4096) {
                    const double magnitudes[2]{mju_norm3(acceleration.data()),
                        mju_norm3(tracking_off.toe_acceleration_reference().data())};

                    for (int profile = 0; profile < 2; ++profile) {
                        acceleration_sum[profile] += magnitudes[profile];
                        acceleration_squared[profile] += magnitudes[profile] * magnitudes[profile];
                        acceleration_peak[profile] = std::max(acceleration_peak[profile], magnitudes[profile]);
                    }
                }
            }

            Require((maximum_chord_speed - minimum_chord_speed) / speed < .001,
                "sampled positions do not advance at uniform arc speed");

            double cv[2];

            for (int profile = 0; profile < 2; ++profile) {
                const double mean = acceleration_sum[profile] / 4096;
                cv[profile] = std::sqrt(acceleration_squared[profile] / 4096 - mean * mean) / mean;
            }

            Require(cv[1] < .2 * cv[0] && cv[1] < .15 && acceleration_peak[1] < .65 * acceleration_peak[0],
                "uniform-acceleration profile did not reduce acceleration variation and peak");
            std::cout << TrackingPlaneName(plane) << " acceleration CV=" << cv[0] << " -> " << cv[1]
                      << ", peak=" << acceleration_peak[0] << " -> " << acceleration_peak[1] << " m/s^2\n";

            // Check both profiles at ramp completion, table nodes and the periodic seam.
            for (auto* controller : {&constant_speed, &tracking_off}) {
                for (const double time : {ramp_time * .25, ramp_time, ramp_time * 1.37,
                           .5 * ramp_time + 2 * period, .5 * ramp_time + period / 4}) {
                    constexpr double epsilon = 1e-6;
                    controller->Compute(time - epsilon, home, zero, zero, home_jacobian, home_toe, torque);
                    const auto before = controller->toe_reference();
                    const auto before_velocity = controller->toe_velocity_reference();
                    controller->Compute(time + epsilon, home, zero, zero, home_jacobian, home_toe, torque);
                    const auto after = controller->toe_reference();
                    const auto after_velocity = controller->toe_velocity_reference();
                    controller->Compute(time, home, zero, zero, home_jacobian, home_toe, torque);

                    for (int axis = 0; axis < 3; ++axis) {
                        Require(std::abs((after[axis] - before[axis]) / (2 * epsilon) -
                                             controller->toe_velocity_reference()[axis]) < 1e-6,
                            "timing position/velocity derivatives disagree");
                        Require(std::abs((after_velocity[axis] - before_velocity[axis]) / (2 * epsilon) -
                                             controller->toe_acceleration_reference()[axis]) < 1e-4,
                            "timing velocity/acceleration derivatives disagree");
                    }
                }
            }

            std::cout << "PASS: " << TrackingPlaneName(plane) << " arc speed=" << speed
                      << " m/s, sampled variation="
                      << 100 * (maximum_chord_speed - minimum_chord_speed) / speed << "%\n";

            prepare(faster, 0, home_toe, home, plane);
            const double speed_ratio = period / .3;
            const double cycles = std::ceil(ramp_time / std::min(period, .3)) + 2.137;
            tracking_off.Compute(.5 * ramp_time + cycles * period,
                home, zero, zero, home_jacobian, home_toe, torque);
            faster.Compute(.5 * ramp_time + cycles * .3,
                home, zero, zero, home_jacobian, home_toe, torque);

            for (int axis = 0; axis < 3; ++axis) {
                Require(std::abs(faster.toe_reference()[axis] - tracking_off.toe_reference()[axis]) < 1e-12 &&
                            std::abs(faster.toe_velocity_reference()[axis] -
                                speed_ratio * tracking_off.toe_velocity_reference()[axis]) < 1e-10 &&
                            std::abs(faster.toe_acceleration_reference()[axis] -
                                speed_ratio * speed_ratio * tracking_off.toe_acceleration_reference()[axis]) < 1e-8,
                    "profile period scaling changed the path or omitted velocity/acceleration scaling");
            }
        }

        auto fixed_config = YAML::Clone(config);
        fixed_config["model_path"] = std::filesystem::absolute(argv[2]).string();

        // The current pose is well conditioned, but a later section is not.
        auto guarded_config = YAML::Clone(fixed_config);
        // Between the current CAD model's entry condition and its path peak.
        guarded_config["workspace"]["maximum_condition_number"] =
            qhrr05::numeric::SingularValues(home_jacobian).condition * 1.02;
        const auto guarded_path = temporary_directory + "/guarded.yaml";
        std::ofstream(guarded_path) << guarded_config;
        qhrr05::TrackingController guarded(guarded_path);
        guarded.Start(0, home_toe, home);
        const std::array<double, 3> check_gravity{1, 2, 3};
        guarded.Compute(0, home, zero, check_gravity, home_jacobian, home_toe, torque);
        Require(guarded.checking() && guarded.check_progress() > 0 && guarded.check_progress() < 1 &&
                    torque == check_gravity && guarded.joint_reference() == home &&
                    guarded.toe_velocity_reference() == zero && guarded.inverse_dynamics_torque() == zero,
            "checking did not hold the entry pose or ran trajectory feedforward early");

        for (int tick = 0; tick < 4097 && guarded.checking(); ++tick)
            guarded.Compute(0, home, zero, zero, home_jacobian, home_toe, torque);

        Require(guarded.ik_failed() && guarded.joint_reference() == home,
            "unsafe later trajectory segment was not rejected before any motion");
        guarded.Compute(10, home, zero, zero, home_jacobian, home_toe, torque);
        Require(torque == zero && guarded.toe_reference() == home_toe &&
                    guarded.inverse_dynamics_torque() == zero,
            "rejected trajectory moved or produced feedforward torque");

        // An actual singular pose is rejected in every tracking method, including pure impedance.
        const std::array<double, 3> singular_q{0, .9 + std::acos(-1.0) / 2, -1.8};
        std::copy(singular_q.begin(), singular_q.end(), desired_data->qpos);
        mj_forward(model.get(), desired_data.get());
        std::array<double, 3> singular_toe;
        std::copy_n(desired_data->site_xpos + 3 * toe_site, 3, singular_toe.begin());

        for (auto* controller : {&tracking_on, &tracking_ctc, &tracking_off, &tracking_impedance_ilc}) {
            prepare(*controller, 0, singular_toe, singular_q);
            Require(controller->ik_failed() && controller->joint_reference() == singular_q,
                "singular entry was accepted by a tracking method");
        }

        std::cout << "PASS: full-path condition rejection before motion; singular entry blocked across methods\n";

        auto unreachable_config = YAML::Clone(fixed_config);
        unreachable_config["tracking"]["trajectory"]["amplitude_m"] = YAML::Load("[5,5,5]");
        const auto unreachable_path = temporary_directory + "/unreachable.yaml";
        std::ofstream(unreachable_path) << unreachable_config;
        qhrr05::TrackingController unreachable(unreachable_path);
        prepare(unreachable, 0, home_toe, home);
        Require(unreachable.ik_failed() && unreachable.joint_reference() == home,
            "unreachable full trajectory was not rejected at entry");

        const auto reject_config = [&](const YAML::Node& invalid) {
            const auto path = temporary_directory + "/invalid.yaml";
            std::ofstream(path) << invalid;
            bool rejected = false;

            try {
                qhrr05::TrackingController controller(path);
            } catch (const std::exception&) {
                rejected = true;
            }

            Require(rejected, "invalid adaptive DLS/arc-length config was accepted");
        };
        auto invalid = YAML::Clone(fixed_config);
        invalid["workspace"]["maximum_condition_number"] = .5;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["ik"]["trajectory_samples"] = 0;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["trajectory"]["arc_length_samples"] = 0;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["trajectory"].remove("arc_length_samples");
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["trajectory"]["timing"] = "unknown";
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["trajectory"].remove("timing");
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["trajectory"]["arc_length_samples"] = 64;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["ik"]["damping_max_m"] = .0001;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["ik"]["singularity_threshold_m"] = 0;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["mode"] = "unknown";
        reject_config(invalid);
        invalid = YAML::Clone(ctc_config);
        invalid["tracking"]["ctc"]["kp_per_s2"] = YAML::Load("[1, 2]");
        reject_config(invalid);
        invalid["tracking"]["ctc"]["kp_per_s2"] = YAML::Load("[1, -1, 1]");
        reject_config(invalid);
        invalid = YAML::Clone(ilc_config);
        invalid["tracking"]["ilc"]["phase_bins"] = 0;
        reject_config(invalid);
        invalid = YAML::Clone(ilc_config);
        invalid["tracking"]["ilc"]["retention"] = 2;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["inverse_dynamics"]["enabled"] = false;
        reject_config(invalid);
        invalid = YAML::Clone(fixed_config);
        invalid["tracking"]["ilc"]["learning_rate"] = .2;
        reject_config(invalid);

        auto active_only = YAML::Clone(ctc_config);
        active_only["tracking"].remove("inverse_dynamics");
        active_only["tracking"].remove("impedance");
        const auto active_path = temporary_directory + "/active-only.yaml";
        std::ofstream(active_path) << active_only;
        Require(qhrr05::TrackingController(active_path).method() == TrackingMethod::CTC,
            "CTC requires parameters belonging to the inverse-dynamics/impedance modes");
        active_only["tracking"]["mode"] = "impedance";
        active_only["tracking"]["impedance"] = config["tracking"]["impedance"];
        active_only["tracking"].remove("ctc");
        std::ofstream(active_path) << active_only;
        Require(!qhrr05::TrackingController(active_path).joint_ik_enabled(),
            "pure impedance unexpectedly enabled runtime joint IK/IDFF");
        active_only["tracking"].remove("ik");
        reject_config(active_only);  // Preflight requires explicit IK settings in every mode.

        std::array<std::array<double, 3>, 3> baseline_rmse{};

        for (const auto plane : {TrackingPlane::XY, TrackingPlane::XZ, TrackingPlane::YZ}) {
            for (int variant : {0, 1, 2, 3, 4, 5}) {
                const bool use_model = variant != 0;
                const bool ctc = variant == 2 || variant == 4;
                const bool use_ilc = variant >= 3;
                auto& tracking = variant == 5 ? tracking_impedance_ilc
                                : use_ilc ? (ctc ? tracking_ctc_ilc : tracking_id_ilc)
                                        : ctc ? tracking_ctc : use_model ? tracking_on : tracking_off;
                qhrr05::ResetSimulation(home, model.get(), data.get());
                const int site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
                std::array<double, 3> center, position, velocity{}, gravity{}, torque{};
                std::array<double, 9> jacobian{};
                std::copy_n(data->site_xpos + 3 * site, 3, center.begin());

                prepare(tracking, 10, center, home, plane);
                tracking.Compute(10, home, velocity, gravity, home_jacobian, center, torque);
                Require(tracking.toe_reference() == center &&
                            tracking.toe_velocity_reference() == velocity,
                    "entry reference is discontinuous");

                if (!use_ilc) {
                    const double period = config["tracking"]["trajectory"]["period_sec"].as<double>();
                    const double ramp = config["tracking"]["trajectory"]["ramp_sec"].as<double>();
                    const double probe = 10 + ramp + .137 * period;
                    tracking.Compute(probe, home, velocity, gravity, home_jacobian, center, torque);
                    const auto reference = tracking.toe_reference();
                    const auto reference_velocity = tracking.toe_velocity_reference();
                    const auto reference_acceleration = tracking.toe_acceleration_reference();
                    tracking.Compute(probe + period, home, velocity, gravity, home_jacobian, center, torque);

                    for (int i = 0; i < 3; ++i)
                        Require(std::abs(tracking.toe_reference()[i] - reference[i]) < 1e-12,
                            "figure eight does not repeat at configured period");

                    tracking.Compute(probe + 1e-6, home, velocity, gravity, home_jacobian, center, torque);

                    for (int i = 0; i < 3; ++i)
                        Require(std::abs((tracking.toe_reference()[i] - reference[i]) / 1e-6 -
                                             reference_velocity[i]) < 1e-5,
                            "reference velocity is not the trajectory derivative");

                    for (int i = 0; i < 3; ++i)
                        Require(std::abs((tracking.toe_velocity_reference()[i] - reference_velocity[i]) / 1e-6 -
                                             reference_acceleration[i]) < 1e-3,
                            "reference acceleration is not the velocity derivative");
                }

                // 500 Hz controller / ZOH + quantized MIT position servo at 1 kHz.
                // Desired-state inverse dynamics + task impedance; no contact truth in control.
                prepare(tracking, 0, center, home, plane);
                MitCommand commands[3]{};
                double squared_error = 0, maximum_error = 0;
                int count = 0, saturated = 0;
                const auto axes = TrackingPlaneAxes(plane);
                const int fixed_axis = 3 - axes[0] - axes[1];
                double minimum[2] = {center[axes[0]], center[axes[1]]};
                double maximum[2] = {center[axes[0]], center[axes[1]]};

                for (int step = 0; step < (use_ilc ? 14000 : 6000); ++step) {
                    mj_forward(model.get(), data.get());

                    if (step % 2 == 0) {
                        std::copy_n(data->site_xpos + 3 * site, 3, position.begin());
                        std::copy_n(data->qvel, 3, velocity.begin());
                        mju_copy(gravity_data->qpos, data->qpos, 3);
                        mju_zero(gravity_data->qvel, 3);
                        mj_fwdPosition(model.get(), gravity_data.get());
                        mj_fwdVelocity(model.get(), gravity_data.get());
                        std::copy_n(gravity_data->qfrc_bias, 3, gravity.begin());
                        mj_jacSite(model.get(), data.get(), jacobian.data(), nullptr, site);
                        const std::array<double, 3> measured_q{data->qpos[0], data->qpos[1], data->qpos[2]};
                        tracking.Compute(data->time, measured_q, velocity, gravity, jacobian, position, torque);
                        saturated += tracking.torque_saturated();
                        Require(tracking.toe_reference()[fixed_axis] == center[fixed_axis] &&
                                    tracking.toe_velocity_reference()[fixed_axis] == 0 &&
                                    tracking.toe_acceleration_reference()[fixed_axis] == 0,
                            "trajectory moved the axis outside the selected plane");

                        if (use_model && step % 100 == 0) {
                            std::copy(tracking.joint_reference().begin(), tracking.joint_reference().end(),
                                desired_data->qpos);
                            std::copy(tracking.joint_velocity_reference().begin(),
                                tracking.joint_velocity_reference().end(), desired_data->qvel);
                            mj_forward(model.get(), desired_data.get());
                            double mass[9], desired_jacobian[9], desired_jacobian_dot[9];
                            mj_fullM(model.get(), mass, desired_data->qM);
                            mj_jacSite(model.get(), desired_data.get(), desired_jacobian, nullptr, site);
                            mj_jacDot(model.get(), desired_data.get(), desired_jacobian_dot, nullptr,
                                desired_data->site_xpos + 3 * site, model->site_bodyid[site]);

                            for (int i = 0; i < 3; ++i) {
                                double expected = desired_data->qfrc_bias[i];
                                double expected_velocity = 0, expected_acceleration = 0;

                                for (int j = 0; j < 3; ++j) {
                                    expected += mass[3 * i + j] * tracking.joint_acceleration_reference()[j];
                                    expected_velocity += desired_jacobian[3 * i + j] *
                                                         tracking.joint_velocity_reference()[j];
                                    expected_acceleration += desired_jacobian[3 * i + j] *
                                                                 tracking.joint_acceleration_reference()[j] +
                                                             desired_jacobian_dot[3 * i + j] *
                                                                 tracking.joint_velocity_reference()[j];
                                }

                                if (tracking.inverse_dynamics_enabled())
                                    Require(std::abs(expected - tracking.inverse_dynamics_torque()[i]) < 1e-9,
                                        "RNEA differs from full M*ddq+bias or includes passive/contact forces");
                                Require(std::abs(desired_data->site_xpos[3 * site + i] -
                                                     tracking.toe_reference()[i]) < 1.1e-5,
                                    "desired joint pose does not reach Cartesian reference");
                                const double velocity_error =
                                    expected_velocity - tracking.toe_velocity_reference()[i];
                                const double acceleration_error =
                                    expected_acceleration - tracking.toe_acceleration_reference()[i];
                                Require(std::abs(velocity_error) < 1e-4 &&
                                            std::abs(acceleration_error) < 2e-3,
                                    "differential IK lost Jdot*dq or reference derivatives");
                            }

                            if (ctc) {
                                double current_mass[9];
                                mj_fullM(model.get(), current_mass, data->qM);

                                for (int i = 0; i < 3; ++i) {
                                    double expected = data->qfrc_bias[i];

                                    for (int j = 0; j < 3; ++j) {
                                        const double acceleration = tracking.joint_acceleration_reference()[j] +
                                            config["tracking"]["ctc"]["kp_per_s2"][j].as<double>() *
                                                (tracking.joint_reference()[j] - data->qpos[j]) +
                                            config["tracking"]["ctc"]["kd_per_s"][j].as<double>() *
                                                (tracking.joint_velocity_reference()[j] - data->qvel[j]);
                                        expected += current_mass[3 * i + j] * acceleration;
                                    }

                                    const double limit = config["tracking"]["output_torque_limit_nm"][i].as<double>();
                                    Require(std::abs(expected - tracking.ctc_torque()[i]) < 1e-9 &&
                                                std::abs(torque[i] - std::clamp(expected + tracking.ilc().torque()[i], -limit, limit)) < 1e-9 &&
                                                tracking.inverse_dynamics_torque()[i] == 0,
                                        "CTC is not current-state M*(ddq_d+Kp*e+Kd*de)+bias or adds duplicate torque");
                                }
                            }
                        }

                        if (tracking.method() == TrackingMethod::Impedance) {
                            Require(tracking.inverse_dynamics_torque() == zero && tracking.ctc_torque() == zero,
                                "impedance + ILC computed an unwanted dynamics feedforward torque");

                            for (int joint = 0; joint < 3; ++joint) {
                                double expected = gravity[joint] - tracking.kd_motor()[joint] * velocity[joint] +
                                    tracking.ilc().torque()[joint];

                                for (int axis = 0; axis < 3; ++axis) {
                                    double toe_velocity = 0;

                                    for (int j = 0; j < 3; ++j)
                                        toe_velocity += jacobian[3 * axis + j] * velocity[j];

                                    expected += jacobian[3 * axis + joint] *
                                        (config["tracking"]["impedance"]["stiffness_n_m"][axis].as<double>() *
                                             (tracking.toe_reference()[axis] - position[axis]) +
                                         config["tracking"]["impedance"]["damping_ns_m"][axis].as<double>() *
                                             (tracking.toe_velocity_reference()[axis] - toe_velocity));
                                }

                                const double limit = config["tracking"]["output_torque_limit_nm"][joint].as<double>();
                                Require(std::abs(torque[joint] - std::clamp(expected, -limit, limit)) < 1e-9,
                                    "impedance torque is not gravity + task feedback + ILC - motor damping");
                            }
                        }

                        for (int i = 0; i < 3; ++i) {
                            MitCommand command{
                                data->qpos[i] - home[i] +
                                    (torque[i] + tracking.kd_motor()[i] * velocity[i]) / tracking.kp_motor()[i],
                                0, tracking.kp_motor()[i], tracking.kd_motor()[i], 0};
                            commands[i] = openrobot::DecodeMitCommand(
                                openrobot::EncodeMitCommand(command, 45, 1), 45, 1);
                        }

                        if (data->time > (use_ilc ? 12 : 2)) {
                            double error_squared = 0;

                            for (int i = 0; i < 3; ++i)
                                error_squared += std::pow(tracking.toe_reference()[i] - position[i], 2);

                            squared_error += error_squared;
                            maximum_error = std::max(maximum_error, std::sqrt(error_squared));
                            ++count;
                            for (int component = 0; component < 2; ++component) {
                                minimum[component] = std::min(minimum[component], position[axes[component]]);
                                maximum[component] = std::max(maximum[component], position[axes[component]]);
                            }
                        }
                    }

                    for (int i = 0; i < 3; ++i) {
                        const auto& command = commands[i];
                        data->ctrl[i] = std::clamp(
                            command.kp * (home[i] + command.position - data->qpos[i]) -
                                command.kd * data->qvel[i],
                            model->actuator_ctrlrange[2 * i], model->actuator_ctrlrange[2 * i + 1]);

                        Require(std::isfinite(data->qpos[i]) &&
                                    data->qpos[i] > model->jnt_range[2 * i] &&
                                    data->qpos[i] < model->jnt_range[2 * i + 1],
                            "tracking left joint limits or diverged");
                    }

                    mj_step(model.get(), data.get());

                    for (int i = 0; i < mjNWARNING; ++i)
                        Require(data->warning[i].number == 0, "MuJoCo warning during tracking");
                }

                const double rmse = std::sqrt(squared_error / count);
                std::cout << "tracking " << TrackingPlaneName(plane)
                          << (ctc ? " CTC" : tracking.inverse_dynamics_enabled() ? " IDFF" : " IMPEDANCE")
                          << (use_ilc ? " + ILC" : "") << ": RMSE=" << rmse * 1000
                          << " mm max=" << maximum_error * 1000 << " mm spans="
                          << (maximum[0] - minimum[0]) * 1000 << ","
                          << (maximum[1] - minimum[1]) * 1000 << " mm saturated=" << saturated << '\n';
                Require(rmse < .015 && maximum_error < .03, "tracking error exceeds demo tolerance");

                const int method_index = static_cast<int>(tracking.method());

                if (!use_ilc)
                    baseline_rmse[static_cast<int>(plane)][method_index] = rmse;

                if (use_ilc) {
                    Require(tracking.ilc().updates() >= 10 &&
                                rmse < .9 * baseline_rmse[static_cast<int>(plane)][method_index],
                        "ILC did not learn complete laps or improve late-lap tracking by at least 10%");
                }

                for (int component = 0; component < 2; ++component) {
                    const double amplitude = config["tracking"]["trajectory"]["amplitude_m"][axes[component]].as<double>();
                    // Uniform arc speed removes the sine trajectory's endpoint
                    // dwell. Require at least half the full reference span; the
                    // RMS/maximum error bounds above still verify physical tracking.
                    Require(maximum[component] - minimum[component] > amplitude,
                        "toe did not draw both axes of the figure eight");
                }

                mj_forward(model.get(), data.get());
                std::copy_n(data->site_xpos + 3 * site, 3, position.begin());
                const std::array<double, 3> current_joints{data->qpos[0], data->qpos[1], data->qpos[2]};
                prepare(tracking, 7, position, current_joints, plane);
                tracking.Compute(7, current_joints, velocity, gravity, jacobian, position, torque);
                Require(tracking.toe_reference() == position, "re-entry retained the previous center");
                Require(tracking.ilc().updates() == 0 && tracking.ilc().torque() == zero,
                    "Tracking reentry retained a learned ILC table");
            }
        }

        // A mathematically reachable goal is not silently clipped to joint limits.
        std::array<double, 3> outside_joints = home, outside_toe;
        outside_joints[2] = model->jnt_range[5] + .03;
        std::copy(outside_joints.begin(), outside_joints.end(), desired_data->qpos);
        mj_forward(model.get(), desired_data.get());
        std::copy_n(desired_data->site_xpos + 3 * toe_site, 3, outside_toe.begin());
        auto geometric_config = YAML::Clone(fixed_config);
        geometric_config["tracking"]["trajectory"]["amplitude_m"] = YAML::Load("[.0001,.0001,.0001]");
        const auto geometric_path = temporary_directory + "/geometric.yaml";
        std::ofstream(geometric_path) << geometric_config;
        qhrr05::TrackingController geometric(geometric_path);
        prepare(geometric, 0, outside_toe, home);
        geometric.Compute(0, home, zero, zero, home_jacobian, outside_toe, torque);
        Require(geometric.joint_reference()[2] > model->jnt_range[5] && geometric.ik_error() <= 1e-5,
            "IK silently projected the target into physical joint limits");

        // Force failure after a valid update. Freeze the measured pose, not the
        // preceding desired pose or the IK solver's last unsuccessful iterate.
        auto failure_config = YAML::Clone(config);

        failure_config["model_path"] =
            std::filesystem::absolute(argv[2]).string();
        const auto failure_path = temporary_directory + "/failure.yaml";
        std::ofstream(failure_path) << failure_config;
        qhrr05::TrackingController failure(failure_path);
        qhrr05::ResetSimulation(home, model.get(), data.get());
        std::array<double, 3> current_toe;
        std::copy_n(data->site_xpos + 3 * toe_site, 3, current_toe.begin());
        prepare(failure, 0, current_toe, home);
        failure.Compute(0, home, zero, zero, home_jacobian, current_toe, torque);
        Require(!failure.ik_failed(), "valid IK entered hold unexpectedly");
        std::array<double, 3> stopped_q = home;
        stopped_q[1] += .04;
        failure.Compute(1.137, stopped_q, zero, zero, zero_jacobian, current_toe, torque);
        Require(failure.ik_failed() && failure.joint_reference() == stopped_q,
            "IK failure did not latch the measured pose");
        const auto stopped_toe = failure.toe_reference();
        Require(failure.joint_velocity_reference() == zero &&
                    failure.joint_acceleration_reference() == zero &&
                    failure.toe_velocity_reference() == zero &&
                    failure.toe_acceleration_reference() == zero &&
                    failure.inverse_dynamics_torque() == zero,
            "IK failure retained velocity/acceleration/FF commands");
        auto displaced_q = stopped_q;
        displaced_q[0] += .02;
        const std::array<double, 3> damping_velocity{.1, -.2, .15};
        const std::array<double, 3> hold_gravity{1, 2, 3};
        failure.Compute(20, displaced_q, damping_velocity, hold_gravity, zero_jacobian,
            current_toe, torque);

        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - (hold_gravity[i] + failure.kp_motor()[i] *
                        (stopped_q[i] - displaced_q[i]) - failure.kd_motor()[i] * damping_velocity[i])) < 1e-10,
                "hold did not provide gravity-compensated joint PD");

        Require(failure.ik_failed() && failure.joint_reference() == stopped_q &&
                    failure.toe_reference() == stopped_toe,
            "latched hold automatically resumed or moved its target");

        // Hold is an active position servo, not zero torque: perturb the physical
        // leg and verify it settles without disabling motors or restarting IK.
        qhrr05::ResetSimulation(stopped_q, model.get(), data.get());
        data->qpos[0] += .02;
        data->qvel[1] = .2;
        MitCommand hold_commands[3]{};

        for (int step = 0; step < 2000; ++step) {
            mj_forward(model.get(), data.get());

            if (step % 2 == 0) {
                std::array<double, 3> q, dq, gravity;
                std::copy_n(data->qpos, 3, q.begin());
                std::copy_n(data->qvel, 3, dq.begin());
                std::copy_n(data->site_xpos + 3 * toe_site, 3, current_toe.begin());
                mju_copy(gravity_data->qpos, data->qpos, 3);
                mju_zero(gravity_data->qvel, 3);
                mj_forward(model.get(), gravity_data.get());
                std::copy_n(gravity_data->qfrc_bias, 3, gravity.begin());
                failure.Compute(21 + data->time, q, dq, gravity, zero_jacobian, current_toe, torque);

                for (int i = 0; i < 3; ++i) {
                    const MitCommand command{
                        q[i] - stopped_q[i] + (torque[i] + failure.kd_motor()[i] * dq[i]) / failure.kp_motor()[i],
                        0, failure.kp_motor()[i], failure.kd_motor()[i], 0};
                    hold_commands[i] = openrobot::DecodeMitCommand(
                        openrobot::EncodeMitCommand(command, 45, 1), 45, 1);
                }
            }

            for (int i = 0; i < 3; ++i) {
                const auto& command = hold_commands[i];
                data->ctrl[i] = command.kp * (stopped_q[i] + command.position - data->qpos[i]) -
                                command.kd * data->qvel[i];
            }

            mj_step(model.get(), data.get());
        }

        for (int i = 0; i < 3; ++i)
            Require(std::abs(data->qpos[i] - stopped_q[i]) < .015 && std::abs(data->qvel[i]) < .03,
                "IK hold did not settle at its captured measured pose");

        Require(failure.ik_failed(), "hold latch cleared without explicit restart");
        mj_forward(model.get(), data.get());
        const std::array<double, 3> recovered_q{data->qpos[0], data->qpos[1], data->qpos[2]};
        std::copy_n(data->site_xpos + 3 * toe_site, 3, current_toe.begin());
        std::array<double, 9> recovered_jacobian;
        mj_jacSite(model.get(), data.get(), recovered_jacobian.data(), nullptr, toe_site);
        prepare(failure, 30, current_toe, recovered_q);
        failure.Compute(30, recovered_q, zero, zero, recovered_jacobian, current_toe, torque);
        Require(!failure.ik_failed(), "explicit Tracking restart did not clear the hold");
        std::cout << "PASS: IK divergence -> measured-pose hold, zero FF, no auto-resume, MIT hold/settling, restart\n";

        auto ctc_failure_config = YAML::Clone(failure_config);
        ctc_failure_config["tracking"]["mode"] = "ctc";
        const auto ctc_failure_path = temporary_directory + "/ctc-failure.yaml";
        std::ofstream(ctc_failure_path) << ctc_failure_config;
        qhrr05::TrackingController ctc_failure(ctc_failure_path);
        prepare(ctc_failure, 0, current_toe, recovered_q);
        ctc_failure.Compute(0, recovered_q, zero, zero, recovered_jacobian, current_toe, torque);
        Require(!ctc_failure.ik_failed(), "CTC could not initialize at the measured pose");
        ctc_failure.Compute(1.137, stopped_q, zero, zero, zero_jacobian, current_toe, torque);
        Require(ctc_failure.ik_failed() && ctc_failure.joint_reference() == stopped_q &&
                    ctc_failure.ctc_torque() == zero && ctc_failure.inverse_dynamics_torque() == zero,
            "CTC IK failure did not latch the measured pose and clear its dynamic torque");
        ctc_failure.Compute(20, displaced_q, damping_velocity, hold_gravity, zero_jacobian, current_toe, torque);

        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - (hold_gravity[i] + ctc_failure.kp_motor()[i] *
                        (stopped_q[i] - displaced_q[i]) - ctc_failure.kd_motor()[i] * damping_velocity[i])) < 1e-10,
                "CTC failure did not use the same latched gravity/PD hold");

        auto ctc_limited_config = YAML::Clone(ctc_config);
        ctc_limited_config["tracking"]["output_torque_limit_nm"] = YAML::Load("[.001,.001,.001]");
        const auto ctc_limited_path = temporary_directory + "/ctc-limited.yaml";
        std::ofstream(ctc_limited_path) << ctc_limited_config;
        qhrr05::TrackingController ctc_limited(ctc_limited_path);
        prepare(ctc_limited, 0, current_toe, recovered_q);
        ctc_limited.Compute(0, recovered_q, zero, zero, recovered_jacobian, current_toe, torque);
        Require(ctc_limited.torque_saturated(), "CTC torque limit did not report saturation");

        for (int i = 0; i < 3; ++i)
            Require(std::abs(torque[i] - std::clamp(ctc_limited.ctc_torque()[i], -.001, .001)) < 1e-12,
                "CTC bypassed the configured torque limit");

        std::cout << "PASS: CTC measured-state dynamics, no duplicate impedance, torque limit and IK hold\n";
        std::filesystem::remove_all(temporary_directory);
        std::cout << "PASS: periodic continuous reference, derivative, MIT ZOH closed loop, restart\n";
        return 0;
    } catch (const std::exception& error) {
        if (!temporary_directory.empty())
            std::filesystem::remove_all(temporary_directory);

        std::cerr << error.what() << '\n';
        return 1;
    }
}
