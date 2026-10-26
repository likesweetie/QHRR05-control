#include "simulation/virtual_iq_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
VirtualIqSensor::VirtualIqSensor(const std::string& config_path, int axis) {
    if (axis < 0 || axis >= 3)
        throw std::runtime_error("Iq sensor axis must be HR/HP/KP (0..2)");

    const auto settings = YAML::LoadFile(config_path)["iq_sensor"];
    enabled_ = settings["enabled"].as<bool>();
    const auto value = [&](const char* key) {
        const auto node = settings[key];

        if (node.IsSequence()) {
            if (node.size() != 3)
                throw std::runtime_error(std::string("expected three iq_sensor values: ") + key);

            return node[axis];
        }

        return node;
    };
    const auto scalar = [&](const char* key) {
        const double result = value(key).as<double>();

        if (!std::isfinite(result))
            throw std::runtime_error(std::string("iq_sensor value must be finite: ") + key);

        return result;
    };

    gain_ = scalar("gain");
    offset_ = scalar("offset_a");
    noise_std_ = scalar("noise_std_a");
    ripple_amplitude_ = scalar("ripple_amplitude_a");
    ripple_harmonic_ = scalar("ripple_harmonic");
    ripple_phase_ = scalar("ripple_phase_rad");
    bandwidth_ = scalar("bandwidth_hz");
    quantization_ = scalar("quantization_a");
    current_limit_ = scalar("current_limit_a");
    seed_ = value("random_seed").as<std::uint32_t>();

    if (noise_std_ < 0 || ripple_amplitude_ < 0 || ripple_harmonic_ < 1 ||
        ripple_harmonic_ != std::floor(ripple_harmonic_) || bandwidth_ < 0 || quantization_ < 0 ||
        current_limit_ <= 0 || current_limit_ > 32767.0 * 33 / 2048)
        throw std::runtime_error(
            "invalid iq_sensor noise/ripple/filter/quantization/current limit");

    Reset();
}

void VirtualIqSensor::Reset() {
    generator_.seed(seed_);
    normal_.reset();
    filtered_a_ = 0;
    initialized_ = false;
}

double VirtualIqSensor::Sample(double iq_true_a, double electrical_angle_rad, double dt_sec) {
    if (!std::isfinite(iq_true_a) || !std::isfinite(electrical_angle_rad) ||
        !std::isfinite(dt_sec) || dt_sec <= 0)
        throw std::runtime_error("Iq sensor requires finite current/angle and positive sample dt");

    if (!enabled_)
        return iq_true_a;

    const double noisy =
        gain_ * iq_true_a + offset_ +
        ripple_amplitude_ * std::sin(ripple_harmonic_ * electrical_angle_rad + ripple_phase_) +
        noise_std_ * normal_(generator_);

    if (!std::isfinite(noisy))
        throw std::runtime_error("Iq sensor output overflow");

    if (!initialized_ || bandwidth_ == 0)
        filtered_a_ = noisy;
    else
        filtered_a_ +=
            -std::expm1(-2 * 3.141592653589793 * bandwidth_ * dt_sec) * (noisy - filtered_a_);

    initialized_ = true;
    const double reported =
        quantization_ > 0 ? std::round(filtered_a_ / quantization_) * quantization_ : filtered_a_;

    if (!std::isfinite(reported))
        throw std::runtime_error("Iq sensor quantization overflow");

    return std::clamp(reported, -current_limit_, current_limit_);
}
}  // namespace qhrr05
