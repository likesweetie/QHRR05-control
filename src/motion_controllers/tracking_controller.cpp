#include "motion_controllers/tracking_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <filesystem>
#include <iostream>
#include <limits>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
TrackingController::TrackingController(const std::string& config_path) : ilc_(config_path) {
    const auto config = YAML::LoadFile(config_path);

    if (config["robot"].as<std::string>() != "qhrr-05")
        throw std::runtime_error("only qhrr-05 is supported");

    const auto settings = config["tracking"];
    const auto trajectory = settings["trajectory"];
    const auto motor = settings["motor"];
    const auto id = settings["inverse_dynamics"];

    if (id && id["enabled"].IsDefined())
        throw std::runtime_error("tracking.inverse_dynamics.enabled was removed; select mode=impedance to disable IDFF");

    for (const char* key : {"period_sec", "ramp_sec", "arc_length_samples", "amplitude_x_m",
             "amplitude_y_m", "amplitude_z_m", "kp_motor", "kd_motor", "torque_limit_nm",
             "stiffness_n_m", "damping_ns_m"})
        if (settings[key].IsDefined())
            throw std::runtime_error(std::string("retired tracking key: ") + key +
                "; use trajectory/motor/impedance/output_torque_limit_nm");

    for (const char* key : {"jacobian_regularization_m", "maximum_jacobian_regularization_m",
             "singularity_threshold_m", "ik_tolerance_m", "ik_max_iterations", "ik_step_limit_rad"})
        if (id && id[key].IsDefined())
            throw std::runtime_error(std::string("common IK parameter belongs under tracking.ik: ") + key);

    const auto ctc = settings["ctc"];

    if (ctc && (ctc["kp"].IsDefined() || ctc["kd"].IsDefined()))
        throw std::runtime_error("tracking.ctc kp/kd were renamed to kp_per_s2/kd_per_s");

    const auto mode = settings["mode"].as<std::string>();

    if (mode == "ctc")
        method_ = TrackingMethod::CTC;
    else if (mode == "impedance")
        method_ = TrackingMethod::Impedance;
    else if (mode != "inverse_dynamics")
        throw std::runtime_error("tracking.mode must be inverse_dynamics, ctc or impedance");

    const auto positive = [&](const YAML::Node& node, const char* key) {
        const double value = node[key].as<double>();

        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error(std::string("tracking parameter must be positive: ") + key);

        return value;
    };
    const auto axes = [&](const YAML::Node& node, const char* key) {
        const auto values = node[key].as<std::vector<double>>();

        if (values.size() != 3)
            throw std::runtime_error(std::string("expected three tracking values: ") + key);

        std::array<double, 3> result;
        std::copy(values.begin(), values.end(), result.begin());

        for (double value : result)
            if (!std::isfinite(value) || value < 0)
                throw std::runtime_error(std::string("invalid tracking value: ") + key);

        return result;
    };

    period_ = positive(trajectory, "period_sec");
    ramp_time_ = positive(trajectory, "ramp_sec");
    amplitude_ = axes(trajectory, "amplitude_m");
    arc_length_samples_ = trajectory["arc_length_samples"].as<int>();
    const auto timing = trajectory["timing"].as<std::string>();

    if (timing == "uniform_acceleration")
        uniform_acceleration_ = true;
    else if (timing != "constant_speed")
        throw std::runtime_error("tracking.trajectory.timing must be uniform_acceleration or constant_speed");

    if (arc_length_samples_ < 64 || arc_length_samples_ > 65536 ||
        std::any_of(amplitude_.begin(), amplitude_.end(), [](double value) { return value <= 0; }))
        throw std::runtime_error("tracking trajectory needs positive amplitudes and 64..65536 arc-length samples");

    if (uniform_acceleration_ && arc_length_samples_ < 256)
        throw std::runtime_error("uniform_acceleration needs at least 256 arc_length_samples for its smooth timing profile");

    kp_motor_ = axes(motor, "kp_nm_rad");
    kd_motor_ = axes(motor, "kd_nm_s_rad");
    torque_limit_ = axes(settings, "output_torque_limit_nm");

    for (int i = 0; i < 3; ++i)
        if (kp_motor_[i] <= 0 || kp_motor_[i] > 500 || kd_motor_[i] > 5 || torque_limit_[i] <= 0)
            throw std::runtime_error("invalid tracking MIT gains or output torque limit");

    if (method_ == TrackingMethod::CTC) {
        ctc_kp_ = axes(settings["ctc"], "kp_per_s2");
        ctc_kd_ = axes(settings["ctc"], "kd_per_s");

        for (int i = 0; i < 3; ++i)
            if (ctc_kp_[i] <= 0 || ctc_kd_[i] <= 0)
                throw std::runtime_error("tracking CTC gains must be positive");
    } else {
        stiffness_ = axes(settings["impedance"], "stiffness_n_m");
        damping_ = axes(settings["impedance"], "damping_ns_m");

        for (int i = 0; i < 3; ++i)
            if (stiffness_[i] <= 0 || damping_[i] <= 0)
                throw std::runtime_error("tracking task impedance gains must be positive");
    }

    if (inverse_dynamics_enabled()) {
        id_alpha_ = id["alpha"].as<double>();

        if (!std::isfinite(id_alpha_) || id_alpha_ < 0 || id_alpha_ > 1)
            throw std::runtime_error("tracking inverse dynamics alpha must be in [0, 1]");
    }

    {
        const auto ik = settings["ik"];
        regularization_ = positive(ik, "damping_min_m");
        maximum_regularization_ = positive(ik, "damping_max_m");
        singularity_threshold_ = positive(ik, "singularity_threshold_m");
        ik_tolerance_ = positive(ik, "tolerance_m");
        ik_step_limit_ = positive(ik, "step_limit_rad");
        ik_iterations_ = ik["max_iterations"].as<int>();
        trajectory_samples_ = ik["trajectory_samples"].as<int>();
        condition_limit_ = config["workspace"]["maximum_condition_number"].as<double>();

        if (ik_iterations_ <= 0 || ik_iterations_ > 100 || maximum_regularization_ < regularization_)
            throw std::runtime_error("invalid Tracking IK iteration count or damping bounds");

        if (trajectory_samples_ < 64 || trajectory_samples_ > 16384)
            throw std::runtime_error("tracking.ik.trajectory_samples must be in 64..16384");

        if (!std::isfinite(condition_limit_) || condition_limit_ < 1)
            throw std::runtime_error("workspace.maximum_condition_number must be finite and at least 1");
    }

    BuildArcLengthTables();

    if (uniform_acceleration_)
        BuildAccelerationProfiles();

    {
        const auto path = std::filesystem::path(config_path).parent_path() /
                          config["model_path"].as<std::string>();
        char error[1024]{};
        id_model_.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));

        if (!id_model_)
            throw std::runtime_error("tracking model load failed: " + std::string(error));

        if (id_model_->nq != 3 || id_model_->nv != 3 || id_model_->nu != 3)
            throw std::runtime_error("tracking model requires three joints and actuators");

        toe_site_ = mj_name2id(id_model_.get(), mjOBJ_SITE, "FL_foot");
        id_data_.reset(mj_makeData(id_model_.get()));

        if (toe_site_ < 0 || !id_data_)
            throw std::runtime_error("tracking model needs FL_foot and allocated mjData");
    }
}

void TrackingController::Start(double time, const std::array<double, 3>& toe_position,
    const std::array<double, 3>& joint_position,
    TrackingPlane plane) {
    TrackingPlaneAxes(plane);  // Reject unknown planes before changing the active trajectory.

    if (!std::isfinite(time))
        throw std::runtime_error("invalid tracking start time");

    for (const auto& values : {toe_position, joint_position})
        for (double value : values)
            if (!std::isfinite(value))
                throw std::runtime_error("invalid tracking start position");

    started_ = time;
    plane_ = plane;
    center_ = toe_reference_ = toe_position;
    toe_velocity_reference_.fill(0);
    toe_acceleration_reference_.fill(0);
    joint_reference_ = joint_position;
    joint_velocity_reference_.fill(0);
    joint_acceleration_reference_.fill(0);
    inverse_dynamics_torque_.fill(0);
    ctc_torque_.fill(0);
    ik_error_ = 0;
    torque_saturated_ = false;
    ik_failed_ = false;
    ik_failure_reason_ = "";
    minimum_singular_value_ = 0;
    jacobian_damping_ = regularization_;
    ilc_.Reset();

    checking_ = true;
    checked_samples_ = 0;
    peak_path_condition_ = 0;
    validation_joint_ = joint_position;
    std::cout << "[tracking] CHECKING trajectory " << TrackingPlaneName(plane_)
              << ": holding entry pose, condition limit=" << condition_limit_ << '\n';
}

bool TrackingController::CheckCondition(const std::array<double, 9>& jacobian) {
    double jjt[9]{};

    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            for (int joint = 0; joint < 3; ++joint)
                jjt[3 * row + column] += jacobian[3 * row + joint] * jacobian[3 * column + joint];

    double eigenvalues[3], eigenvectors[9], quaternion[4];
    mju_eig3(eigenvalues, eigenvectors, quaternion, jjt);
    const double minimum = *std::min_element(eigenvalues, eigenvalues + 3);
    const double maximum = *std::max_element(eigenvalues, eigenvalues + 3);
    condition_number_ = minimum > 1e-24 && std::isfinite(minimum) && std::isfinite(maximum)
        ? std::sqrt(maximum / minimum) : std::numeric_limits<double>::infinity();

    if (condition_number_ > condition_limit_) {
        ik_failure_reason_ = "Jacobian condition limit exceeded";
        return false;
    }

    return true;
}

bool TrackingController::CheckTrajectory() {
    const auto axes = TrackingPlaneAxes(plane_);
    std::array<double, 9> jacobian;
    const auto held_joint = joint_reference_;
    joint_reference_ = validation_joint_;
    std::copy(validation_joint_.begin(), validation_joint_.end(), id_data_->qpos);
    mj_fwdPosition(id_model_.get(), id_data_.get());
    mj_jacSite(id_model_.get(), id_data_.get(), jacobian.data(), nullptr, toe_site_);
    // Bound work per control tick. Physics/CAN never has to advance for this check.
    const int end = std::min(checked_samples_ + 4, trajectory_samples_ + 1);

    for (int sample = checked_samples_; sample < end; ++sample) {
        const double phase = 2 * std::acos(-1.0) * sample / trajectory_samples_;
        toe_reference_ = center_;
        toe_reference_[axes[0]] += amplitude_[axes[0]] * std::sin(phase);
        toe_reference_[axes[1]] += amplitude_[axes[1]] * std::sin(2 * phase);

        if (!CheckCondition(jacobian) || !SolvePosition(jacobian) || !CheckCondition(jacobian)) {
            std::cerr << "[tracking] TRAJECTORY REJECTED -> HOLD (" << TrackingPlaneName(plane_)
                      << "): " << ik_failure_reason_ << ", phase=" << phase
                      << ", condition=" << condition_number_ << ", limit=" << condition_limit_
                      << ", IK error=" << ik_error_ << " m\n";
            joint_reference_ = held_joint;
            return false;
        }

        peak_path_condition_ = std::max(peak_path_condition_, condition_number_);
        ++checked_samples_;
    }

    validation_joint_ = joint_reference_;
    joint_reference_ = held_joint;
    toe_reference_ = center_;

    if (checked_samples_ == trajectory_samples_ + 1) {
        checking_ = false;
        ik_error_ = 0;
        std::cout << "[tracking] trajectory checked " << TrackingPlaneName(plane_)
                  << ": peak condition=" << peak_path_condition_ << "/" << condition_limit_
                  << ", samples=" << trajectory_samples_ << '\n';
    }

    return true;
}

void TrackingController::Hold(const std::array<double, 3>& joint_position) {
    ik_failed_ = true;
    checking_ = false;
    ilc_.Hold();
    joint_reference_ = joint_position;
    joint_velocity_reference_.fill(0);
    joint_acceleration_reference_.fill(0);
    toe_velocity_reference_.fill(0);
    toe_acceleration_reference_.fill(0);
    inverse_dynamics_torque_.fill(0);
    ctc_torque_.fill(0);
    std::copy(joint_position.begin(), joint_position.end(), id_data_->qpos);
    mju_zero(id_data_->qvel, 3);
    mju_zero(id_data_->qacc, 3);
    mj_fwdPosition(id_model_.get(), id_data_.get());
    std::copy_n(id_data_->site_xpos + 3 * toe_site_, 3, toe_reference_.begin());
}

void TrackingController::BuildArcLengthTables() {
    const double phase_step = 2 * std::acos(-1.0) / arc_length_samples_;

    for (const auto plane : {TrackingPlane::XY, TrackingPlane::XZ, TrackingPlane::YZ}) {
        const auto axes = TrackingPlaneAxes(plane);
        auto& lengths = arc_lengths_[static_cast<int>(plane)];
        lengths.resize(arc_length_samples_ + 1);
        lengths[0] = 0;
        const auto speed = [&](double phase) {
            return std::hypot(amplitude_[axes[0]] * std::cos(phase),
                2 * amplitude_[axes[1]] * std::cos(2 * phase));
        };

        // Simpson integration of |dx/dphase|; tables are built only at initialization.
        for (int i = 0; i < arc_length_samples_; ++i) {
            const double lo = i * phase_step, hi = (i + 1) * phase_step;
            lengths[i + 1] = lengths[i] + phase_step / 6 *
                (speed(lo) + 4 * speed((lo + hi) / 2) + speed(hi));

            if (!std::isfinite(lengths[i + 1]) || lengths[i + 1] <= lengths[i])
                throw std::runtime_error("tracking arc-length table must be finite and increasing");
        }
    }
}

double TrackingController::PhaseTimeDensity(TrackingPlane plane, double phase) const {
    const auto& coefficients = timing_coefficients_[static_cast<int>(plane)];
    double log_speed = 0;

    for (int harmonic = 0; harmonic < kTimingHarmonics; ++harmonic)
        log_speed += coefficients[harmonic] * std::cos(2 * (harmonic + 1) * phase);

    return std::exp(-log_speed);
}

void TrackingController::BuildAccelerationProfiles() {
    constexpr int samples = 2048;
    constexpr double smoothing_width = .02;  // Phase radians; rounds acceleration switches.
    const double phase_step = 2 * std::acos(-1.0) / samples;

    for (const auto plane : {TrackingPlane::XY, TrackingPlane::XZ, TrackingPlane::YZ}) {
        const auto axes = TrackingPlaneAxes(plane);
        const double a = amplitude_[axes[0]], b = amplitude_[axes[1]];
        std::array<double, samples> tangent_norm, curvature, midpoint_curvature, distance, speed_squared;
        const auto geometry = [&](double phase, double& norm, double& kappa) {
            const double x = a * std::cos(phase), y = 2 * b * std::cos(2 * phase);
            const double xx = -a * std::sin(phase), yy = -4 * b * std::sin(2 * phase);
            norm = std::hypot(x, y);
            kappa = std::abs(x * yy - y * xx) / (norm * norm * norm);
        };

        for (int i = 0; i < samples; ++i) {
            double midpoint_norm;
            geometry(i * phase_step, tangent_norm[i], curvature[i]);
            geometry((i + .5) * phase_step, midpoint_norm, midpoint_curvature[i]);
        }

        // Unit acceleration envelope: a_t = du/(2 ds), a_n = kappa*u,
        // where u = Cartesian speed squared. Both directions constrain each edge.
        for (int i = 0; i < samples; ++i) {
            double midpoint_norm, unused;
            geometry((i + .5) * phase_step, midpoint_norm, unused);
            distance[i] = phase_step / 6 *
                (tangent_norm[i] + 4 * midpoint_norm + tangent_norm[(i + 1) % samples]);
            speed_squared[i] = 1 / std::max({curvature[i], midpoint_curvature[i],
                midpoint_curvature[(i + samples - 1) % samples]});

            if (!std::isfinite(speed_squared[i]) || speed_squared[i] <= 0 ||
                !std::isfinite(distance[i]) || distance[i] <= 0)
                throw std::runtime_error("invalid tracking acceleration envelope geometry");
        }

        const auto reachable = [&](double u, int edge) {
            const double k = midpoint_curvature[edge], ds = distance[edge];
            const double kd = k * ds, ku = k * u;
            const double radicand = 1 + kd * kd - ku * ku;

            if (!std::isfinite(radicand) || radicand < -1e-10)
                throw std::runtime_error("tracking acceleration envelope has an infeasible edge");

            return ((1 - kd * kd) * u + 2 * ds * std::sqrt(std::max(0.0, radicand))) /
                (1 + kd * kd);
        };
        bool converged = false;

        for (int pass = 0; pass < 32; ++pass) {
            const auto previous = speed_squared;

            for (int i = 0; i < samples; ++i) {
                const int next = (i + 1) % samples;
                speed_squared[next] = std::min(speed_squared[next], reachable(speed_squared[i], i));
            }

            for (int i = samples - 1; i >= 0; --i)
                speed_squared[i] = std::min(speed_squared[i], reachable(speed_squared[(i + 1) % samples], i));

            double change = 0, scale = 0;

            for (int i = 0; i < samples; ++i) {
                change = std::max(change, std::abs(speed_squared[i] - previous[i]));
                scale = std::max(scale, speed_squared[i]);
            }

            if (change <= 1e-10 * scale) {
                converged = true;
                break;
            }
        }

        if (!converged)
            throw std::runtime_error("tracking acceleration envelope did not converge");

        // Fit log phase speed, so speed stays positive. Even harmonics preserve
        // both identical half-laps; Gaussian spectral smoothing makes it periodic C-infinity.
        auto& coefficients = timing_coefficients_[static_cast<int>(plane)];

        for (int harmonic = 0; harmonic < kTimingHarmonics; ++harmonic) {
            const double frequency = 2 * (harmonic + 1);
            double sum = 0;

            for (int i = 0; i < samples; ++i)
                sum += (.5 * std::log(speed_squared[i]) - std::log(tangent_norm[i])) *
                    std::cos(frequency * i * phase_step);

            coefficients[harmonic] = 2 * sum / samples *
                std::exp(-.5 * std::pow(frequency * smoothing_width, 2));

            if (!std::isfinite(coefficients[harmonic]))
                throw std::runtime_error("non-finite tracking acceleration timing coefficient");
        }

        auto& times = phase_times_[static_cast<int>(plane)];
        times.resize(arc_length_samples_ + 1);
        times[0] = 0;
        const double step = 2 * std::acos(-1.0) / arc_length_samples_;

        for (int i = 0; i < arc_length_samples_; ++i) {
            times[i + 1] = times[i] + step / 6 * (PhaseTimeDensity(plane, i * step) +
                4 * PhaseTimeDensity(plane, (i + .5) * step) + PhaseTimeDensity(plane, (i + 1) * step));

            if (!std::isfinite(times[i + 1]) || times[i + 1] <= times[i])
                throw std::runtime_error("tracking phase-time table must be finite and increasing");
        }
    }
}

void TrackingController::ComputeReference(double elapsed) {
    const auto axes = TrackingPlaneAxes(plane_);
    const auto& integral = uniform_acceleration_ ? phase_times_[static_cast<int>(plane_)]
                                               : arc_lengths_[static_cast<int>(plane_)];
    const double s = std::clamp(elapsed / ramp_time_, 0.0, 1.0);
    const double ramp = s * s * s * (10 + s * (-15 + 6 * s));
    const double ramp_rate = 30 * s * s * (1 - s) * (1 - s) / ramp_time_;

    // Ramp the profile clock, preserving position/velocity/acceleration continuity.
    const double travel_time = elapsed < ramp_time_
        ? ramp_time_ * s * s * s * s * (2.5 + s * (-3 + s))
        : elapsed - .5 * ramp_time_;
    const double progress = std::fmod(travel_time, period_) / period_ * integral.back();
    const auto upper = std::upper_bound(integral.begin(), integral.end(), progress);
    const int segment = std::min(static_cast<int>(upper - integral.begin()) - 1,
        arc_length_samples_ - 1);
    const double phase_step = 2 * std::acos(-1.0) / arc_length_samples_;
    const double lo = segment * phase_step, hi = (segment + 1) * phase_step;
    const auto density = [&](double phase) {
        return uniform_acceleration_ ? PhaseTimeDensity(plane_, phase)
            : std::hypot(amplitude_[axes[0]] * std::cos(phase),
            2 * amplitude_[axes[1]] * std::cos(2 * phase));
    };
    double phase = lo + phase_step *
        (progress - integral[segment]) / (integral[segment + 1] - integral[segment]);

    // Refine the inverse distance/time integral inside one cell. No accumulated phase
    // integration: skipped/irregular controller ticks still sample the same path.
    for (int iteration = 0; iteration < 3; ++iteration) {
        const double local_integral = (phase - lo) / 6 *
            (density(lo) + 4 * density((lo + phase) / 2) + density(phase));
        phase = std::clamp(phase - (integral[segment] + local_integral - progress) / density(phase),
            lo, hi);
    }

    const std::array<double, 2> tangent{
        amplitude_[axes[0]] * std::cos(phase), 2 * amplitude_[axes[1]] * std::cos(2 * phase)};
    const std::array<double, 2> curvature{
        -amplitude_[axes[0]] * std::sin(phase), -4 * amplitude_[axes[1]] * std::sin(2 * phase)};
    const double tangent_norm = std::hypot(tangent[0], tangent[1]);
    double log_rate_derivative = -(tangent[0] * curvature[0] + tangent[1] * curvature[1]) /
        (tangent_norm * tangent_norm);

    if (uniform_acceleration_) {
        log_rate_derivative = 0;
        const auto& coefficients = timing_coefficients_[static_cast<int>(plane_)];

        for (int harmonic = 0; harmonic < kTimingHarmonics; ++harmonic) {
            const double frequency = 2 * (harmonic + 1);
            log_rate_derivative -= frequency * coefficients[harmonic] * std::sin(frequency * phase);
        }
    }

    const double rate = integral.back() / period_ / density(phase);
    const double phase_rate = rate * ramp;
    const double phase_acceleration = rate * ramp_rate + log_rate_derivative * phase_rate * phase_rate;

    toe_reference_ = center_;
    toe_velocity_reference_.fill(0);
    toe_acceleration_reference_.fill(0);

    for (int component = 0; component < 2; ++component) {
        const int axis = axes[component];
        toe_reference_[axis] += amplitude_[axis] * std::sin((component + 1) * phase);
        toe_velocity_reference_[axis] = tangent[component] * phase_rate;
        toe_acceleration_reference_[axis] = curvature[component] * phase_rate * phase_rate +
            tangent[component] * phase_acceleration;
    }
}

bool TrackingController::SolveJacobian(const std::array<double, 9>& jacobian,
    const std::array<double, 3>& cartesian, std::array<double, 3>& result) {
    for (double value : jacobian)
        if (!std::isfinite(value))
            return false;

    for (double value : cartesian)
        if (!std::isfinite(value))
            return false;

    // Adaptive DLS: use the smallest singular value of J to increase damping
    // smoothly near singularities. The same inverse maps position, velocity and acceleration.
    double jjt[9]{};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                jjt[3 * i + j] += jacobian[3 * i + k] * jacobian[3 * j + k];

    double eigenvalues[3], eigenvectors[9], quaternion[4];
    mju_eig3(eigenvalues, eigenvectors, quaternion, jjt);
    const double minimum_eigenvalue = *std::min_element(eigenvalues, eigenvalues + 3);

    if (!std::isfinite(minimum_eigenvalue))
        return false;

    minimum_singular_value_ = std::sqrt(std::max(0.0, minimum_eigenvalue));
    const double ratio = std::min(1.0, minimum_singular_value_ / singularity_threshold_);
    const double weight = 1 - ratio * ratio;
    const double damping_squared = regularization_ * regularization_ +
        (maximum_regularization_ * maximum_regularization_ - regularization_ * regularization_) *
            weight * weight;
    jacobian_damping_ = std::sqrt(damping_squared);

    for (int i = 0; i < 3; ++i)
        jjt[3 * i + i] += damping_squared;

    if (mju_cholFactor(jjt, 3, 0) != 3)
        return false;

    double solved[3];
    mju_cholSolve(solved, jjt, cartesian.data(), 3);
    result.fill(0);

    for (int joint = 0; joint < 3; ++joint)
        for (int axis = 0; axis < 3; ++axis)
            result[joint] += jacobian[3 * axis + joint] * solved[axis];

    return std::all_of(result.begin(), result.end(), [](double value) { return std::isfinite(value); });
}

bool TrackingController::SolvePosition(std::array<double, 9>& jacobian) {
    auto* model = id_model_.get();
    auto* data = id_data_.get();
    std::copy(joint_reference_.begin(), joint_reference_.end(), data->qpos);

    // Warm-start geometric IK from the preceding desired pose. Do not silently
    // project the user's trajectory onto the physical joint-limit workspace.
    for (int iteration = 0; iteration <= ik_iterations_; ++iteration) {
        mj_fwdPosition(model, data);
        mj_jacSite(model, data, jacobian.data(), nullptr, toe_site_);
        std::array<double, 3> error;

        for (int axis = 0; axis < 3; ++axis)
            error[axis] = toe_reference_[axis] - data->site_xpos[3 * toe_site_ + axis];

        ik_error_ = mju_norm3(error.data());

        if (ik_error_ <= ik_tolerance_)
            break;

        if (iteration == ik_iterations_ || !std::isfinite(ik_error_)) {
            ik_failure_reason_ = "IK did not converge";
            return false;
        }

        std::array<double, 3> step;

        if (!SolveJacobian(jacobian, error, step)) {
            ik_failure_reason_ = "IK Jacobian solve failed";
            return false;
        }

        const double scale = std::min(1.0, ik_step_limit_ / std::max(mju_norm3(step.data()), 1e-12));

        for (int joint = 0; joint < 3; ++joint)
            data->qpos[joint] += scale * step[joint];
    }

    std::copy_n(data->qpos, 3, joint_reference_.begin());
    return true;
}

bool TrackingController::ComputeJointReference() {
    auto* model = id_model_.get();
    auto* data = id_data_.get();
    std::array<double, 9> jacobian{};

    if (!SolvePosition(jacobian) || !CheckCondition(jacobian))
        return false;

    if (!SolveJacobian(jacobian, toe_velocity_reference_, joint_velocity_reference_)) {
        ik_failure_reason_ = "desired velocity solve failed";
        return false;
    }

    std::copy(joint_velocity_reference_.begin(), joint_velocity_reference_.end(), data->qvel);
    mj_forward(model, data);

    std::array<double, 9> jacobian_dot{};
    mj_jacDot(model, data, jacobian_dot.data(), nullptr,
        data->site_xpos + 3 * toe_site_, model->site_bodyid[toe_site_]);
    auto acceleration = toe_acceleration_reference_;

    for (int axis = 0; axis < 3; ++axis)
        for (int joint = 0; joint < 3; ++joint)
            acceleration[axis] -= jacobian_dot[3 * axis + joint] * joint_velocity_reference_[joint];

    if (!SolveJacobian(jacobian, acceleration, joint_acceleration_reference_)) {
        ik_failure_reason_ = "desired acceleration solve failed";
        return false;
    }

    return true;
}

bool TrackingController::ComputeDynamicsTorque(const std::array<double, 3>& joint_position,
    const std::array<double, 3>& joint_velocity) {
    auto* model = id_model_.get();
    auto* data = id_data_.get();

    if (method_ == TrackingMethod::CTC) {
        // Feedback is shaped in acceleration units. Evaluate M and bias at the
        // measured state; the IK state above remains only the desired reference.
        std::copy(joint_position.begin(), joint_position.end(), data->qpos);
        std::copy(joint_velocity.begin(), joint_velocity.end(), data->qvel);
        mj_forward(model, data);

        for (int joint = 0; joint < 3; ++joint)
            data->qacc[joint] = joint_acceleration_reference_[joint] +
                ctc_kp_[joint] * (joint_reference_[joint] - joint_position[joint]) +
                ctc_kd_[joint] * (joint_velocity_reference_[joint] - joint_velocity[joint]);
    } else {
        std::copy(joint_acceleration_reference_.begin(), joint_acceleration_reference_.end(), data->qacc);
    }

    auto& dynamics_torque = method_ == TrackingMethod::CTC ? ctc_torque_ : inverse_dynamics_torque_;
    mj_rne(model, data, 1, dynamics_torque.data());

    // RNE excludes armature. Include rotor inertia to match MuJoCo's full M(q),
    // while deliberately omitting passive damping/springs/friction and contacts.
    for (int joint = 0; joint < 3; ++joint) {
        dynamics_torque[joint] += model->dof_armature[joint] * data->qacc[joint];

        if (!std::isfinite(dynamics_torque[joint])) {
            ik_failure_reason_ = "non-finite inverse dynamics torque";
            return false;
        }
    }

    return true;
}

void TrackingController::Compute(double time,
    const std::array<double, 3>& joint_position,
    const std::array<double, 3>& joint_velocity,
    const std::array<double, 3>& gravity_torque,
    const std::array<double, 9>& jacobian,
    const std::array<double, 3>& toe_position,
    std::array<double, 3>& torque) {
    if (!std::isfinite(time) || time < started_)
        throw std::runtime_error("invalid tracking time");

    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(joint_position[i]) || !std::isfinite(joint_velocity[i]) || !std::isfinite(gravity_torque[i]) ||
            !std::isfinite(toe_position[i]))
            throw std::runtime_error("invalid tracking feedback");

    for (double value : jacobian)
        if (!std::isfinite(value))
            throw std::runtime_error("invalid tracking Jacobian");

    if (checking_) {
        if (!CheckCondition(jacobian)) {
            std::cerr << "[tracking] TRAJECTORY REJECTED -> HOLD: actual Jacobian condition="
                      << condition_number_ << "/" << condition_limit_ << '\n';
            Hold(joint_position);
        } else if (!CheckTrajectory()) {
            const auto entry_joint = joint_reference_;
            Hold(entry_joint);
        } else if (!checking_) {
            started_ = time;  // The trajectory ramp starts only after approval.
        }
    }

    if (!ik_failed_ && !checking_) {
        ComputeReference(time - started_);

        // Impedance + ILC needs joint references for learning, but no IDFF/CTC torque.
        bool reference_valid = CheckCondition(jacobian);

        if (reference_valid && joint_ik_enabled()) {
            reference_valid = ComputeJointReference();

            if (reference_valid && method_ != TrackingMethod::Impedance)
                reference_valid = ComputeDynamicsTorque(joint_position, joint_velocity);
        } else if (reference_valid) {
            joint_reference_ = joint_position;
        }

        if (!reference_valid) {
            // Runtime IK failure is latched. Hold the measured pose, never a failed
            // iterate or an old target that may still be far ahead of the leg.
            std::cerr << "[tracking] IK FAILED -> HOLD at t=" << time << " ("
                      << TrackingPlaneName(plane_) << "): " << ik_failure_reason_
                      << ", condition=" << condition_number_ << "/" << condition_limit_
                      << ", error=" << ik_error_ << " m, target=[" << toe_reference_[0] << ","
                      << toe_reference_[1] << "," << toe_reference_[2] << "]\n";
            Hold(joint_position);
        }
    }

    std::array<double, 3> toe_velocity{}, force{};

    if (method_ != TrackingMethod::CTC && !ik_failed_ && !checking_) {
        for (int axis = 0; axis < 3; ++axis) {
            for (int joint = 0; joint < 3; ++joint)
                toe_velocity[axis] += jacobian[3 * axis + joint] * joint_velocity[joint];

            force[axis] = stiffness_[axis] * (toe_reference_[axis] - toe_position[axis]) +
                          damping_[axis] * (toe_velocity_reference_[axis] - toe_velocity[axis]);
        }
    }

    torque_saturated_ = false;

    if (!ik_failed_ && !checking_)
        ilc_.Compute(time - started_);

    for (int joint = 0; joint < 3; ++joint) {
        double effort = gravity_torque[joint] - kd_motor_[joint] * joint_velocity[joint];

        if (ik_failed_ || checking_) {
            effort += kp_motor_[joint] * (joint_reference_[joint] - joint_position[joint]);
        } else if (method_ == TrackingMethod::CTC) {
            effort = ctc_torque_[joint];
        } else {
            if (inverse_dynamics_enabled())
                effort += id_alpha_ * (inverse_dynamics_torque_[joint] - gravity_torque[joint]);

            for (int axis = 0; axis < 3; ++axis)
                effort += jacobian[3 * axis + joint] * force[axis];
        }

        effort += ilc_.torque()[joint];

        if (!std::isfinite(effort))
            throw std::runtime_error("non-finite tracking torque");

        torque_saturated_ |= std::abs(effort) > torque_limit_[joint];
        torque[joint] = std::clamp(effort, -torque_limit_[joint], torque_limit_[joint]);
    }

    if (ilc_.enabled() && !ik_failed_ && !checking_) {
        std::array<double, 3> position_error, velocity_error, toe_error;

        for (int i = 0; i < 3; ++i) {
            position_error[i] = joint_reference_[i] - joint_position[i];
            velocity_error[i] = joint_velocity_reference_[i] - joint_velocity[i];
            toe_error[i] = toe_reference_[i] - toe_position[i];
        }

        ilc_.Record(position_error, velocity_error, mju_norm3(toe_error.data()), torque_saturated_);
    }
}
}  // namespace qhrr05
