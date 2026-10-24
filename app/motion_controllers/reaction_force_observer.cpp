#include "reaction_force_observer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
double MinimumEigenvalue(std::array<double, 9> a) {
    // Jacobi diagonalization of the symmetric 3x3 J*J^T.
    for (int sweep = 0; sweep < 12; ++sweep) {
        int p = 0, q = 1;
        if (std::abs(a[2]) > std::abs(a[1])) q = 2;
        if (std::abs(a[5]) > std::abs(a[3*p+q])) { p = 1; q = 2; }
        if (std::abs(a[3*p+q]) < 1e-14) break;
        const double angle = .5*std::atan2(2*a[3*p+q], a[3*q+q]-a[3*p+p]);
        const double c = std::cos(angle), s = std::sin(angle);
        const double pp = a[3*p+p], qq = a[3*q+q], pq = a[3*p+q];
        for (int k = 0; k < 3; ++k) if (k != p && k != q) {
            const double kp = a[3*k+p], kq = a[3*k+q];
            a[3*k+p] = a[3*p+k] = c*kp-s*kq;
            a[3*k+q] = a[3*q+k] = s*kp+c*kq;
        }
        a[3*p+p] = c*c*pp-2*c*s*pq+s*s*qq;
        a[3*q+q] = s*s*pp+2*c*s*pq+c*c*qq;
        a[3*p+q] = a[3*q+p] = 0;
    }
    return std::min({a[0], a[4], a[8]});
}
std::array<double, 3> Solve(std::array<double, 9> a, std::array<double, 3> b) {
    // Positive definite due to damped least squares; small Cholesky solve.
    double l[9] = {};
    for (int i = 0; i < 3; ++i) for (int j = 0; j <= i; ++j) {
        double v = a[3*i+j];
        for (int k = 0; k < j; ++k) v -= l[3*i+k]*l[3*j+k];
        if (i == j) {
            if (v <= 0) throw std::runtime_error("RFOB matrix is not positive definite");
            l[3*i+j] = std::sqrt(v);
        } else l[3*i+j] = v/l[3*j+j];
    }
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < i; ++k) b[i] -= l[3*i+k]*b[k];
        b[i] /= l[3*i+i];
    }
    for (int i = 2; i >= 0; --i) {
        for (int k = i+1; k < 3; ++k) b[i] -= l[3*k+i]*b[k];
        b[i] /= l[3*i+i];
    }
    return b;
}
}
ReactionForceObserver::ReactionForceObserver(const std::string& path) {
    const auto node = YAML::LoadFile(path)["magnetic_touch"]["observer"];
    bandwidth_ = node["bandwidth_hz"].as<double>();
    regularization_ = node["regularization_m"].as<double>();
    singular_limit_ = node["minimum_singular_value_m"].as<double>();
    force_limit_ = node["maximum_estimate_n"].as<double>();
    friction_velocity_ = node["friction_velocity_rad_s"].as<double>();
    for (double value : {bandwidth_, regularization_, singular_limit_, force_limit_, friction_velocity_})
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("RFOB parameters must be positive and finite");
}
void ReactionForceObserver::Reset() {
    measurement_ = 0; valid_ = false; force_.fill(0); filtered_torque_.fill(0);
}
void ReactionForceObserver::Update(const ShmState& f) {
    for (const auto& values : {f.mass, f.jacobian})
        for (double value : values) if (!std::isfinite(value)) { valid_ = false; return; }
    for (const auto& values : {f.acceleration, f.motor_torque, f.bias_torque, f.passive_torque, f.friction_limit, f.velocity})
        for (double value : values) if (!std::isfinite(value)) { valid_ = false; return; }
    std::array<double, 9> jjt{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) jjt[3*i+j] += f.jacobian[3*i+k]*f.jacobian[3*j+k];
    minimum_singular_value_ = std::sqrt(std::max(0.0, MinimumEigenvalue(jjt)));
    if (minimum_singular_value_ < singular_limit_) { valid_ = false; return; }
    if (!f.measurement_sequence || !std::isfinite(f.measurement_dt) || f.measurement_dt <= 0) {
        valid_ = false; return;
    }
    if (f.measurement_sequence == measurement_) return;  // No duplicate differentiation/filter update.
    if (f.measurement_sequence < measurement_) Reset();
    const auto samples = measurement_ ? f.measurement_sequence-measurement_ : 1;
    const double dt = f.measurement_dt*samples;
    const double alpha = -std::expm1(-2*3.141592653589793*bandwidth_*dt);
    for (int i = 0; i < 3; ++i) {
        double residual = f.bias_torque[i]-f.motor_torque[i]-f.passive_torque[i];
        for (int j = 0; j < 3; ++j) residual += f.mass[3*i+j]*f.acceleration[j];
        // Explicit smooth dry-friction MODEL, never actual constraint force.
        residual += f.friction_limit[i]*std::tanh(f.velocity[i]/friction_velocity_);
        filtered_torque_[i] += alpha*(residual-filtered_torque_[i]);
    }
    measurement_ = f.measurement_sequence;
    std::array<double, 3> rhs{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) rhs[i] += f.jacobian[3*i+j]*filtered_torque_[j];
        jjt[3*i+i] += regularization_*regularization_;
    }
    force_ = Solve(jjt, rhs);
    double squared = 0;
    for (double value : force_) squared += value*value;
    valid_ = std::isfinite(squared) && squared <= force_limit_*force_limit_;
}
}  // namespace qhrr05
