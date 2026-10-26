#include "mujoco_simulate/virtual_iq_sensor.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    char directory[] = "/tmp/qhrr05-iq-unit-XXXXXX";

    try {
        Require(argc == 2 && ::mkdtemp(directory),
            "expected simulation config and temporary directory");
        const auto path = std::string(directory) + "/sensor.yaml";
        auto config = YAML::LoadFile(argv[1]);
        const auto save = [&] {
            std::ofstream(path) << config;
        };
        auto s = config["iq_sensor"];
        s["enabled"] = false;
        s["gain"] = 3;
        s["offset_a"] = 5;
        save();
        qhrr05::VirtualIqSensor disabled(path, 0);
        Require(disabled.Sample(2, .5, .001) == 2, "disabled sensor changed ideal current");

        s["enabled"] = true;
        s["gain"] = 1.1;
        s["offset_a"] = .2;
        s["noise_std_a"] = 0;
        s["ripple_amplitude_a"] = .3;
        s["ripple_harmonic"] = 6;
        s["ripple_phase_rad"] = 0;
        s["bandwidth_hz"] = 0;
        s["quantization_a"] = 0;
        save();
        qhrr05::VirtualIqSensor affine(path, 0);
        const double measured = affine.Sample(2, 3.141592653589793 / 12, .001);
        Require(std::abs(measured - 2.7) < 1e-12, "gain/offset/electrical ripple mismatch");
        Require(
            std::abs(9 * .08 * 2 - 1.44) < 1e-12 && std::abs(9 * .08 * measured - 1.944) < 1e-12,
            "actual and measured torque conversion mismatch");

        s["gain"] = 1;
        s["offset_a"] = 0;
        s["ripple_amplitude_a"] = 0;
        s["bandwidth_hz"] = 20;
        save();
        qhrr05::VirtualIqSensor filtered(path, 0);
        Require(filtered.Sample(0, 0, .001) == 0, "filter initialization mismatch");
        const double response = filtered.Sample(2, 0, .001);
        Require(std::abs(response - 2 * -std::expm1(-2 * 3.141592653589793 * 20 * .001)) < 1e-12,
            "filter used the wrong sample interval");

        s["bandwidth_hz"] = 0;
        s["quantization_a"] = .1;
        s["current_limit_a"] = 1.25;
        save();
        qhrr05::VirtualIqSensor adc(path, 0);
        Require(std::abs(adc.Sample(.26, 0, .001) - .3) < 1e-12 && adc.Sample(3, 0, .001) == 1.25 &&
                    adc.Sample(-3, 0, .001) == -1.25,
            "ADC quantization or sensor clipping mismatch");

        s["quantization_a"] = 0;
        s["current_limit_a"] = 40;
        s["noise_std_a"] = .1;
        save();
        qhrr05::VirtualIqSensor noise(path, 0), repeat(path, 0), other_axis(path, 1);
        double sum = 0, squared = 0;
        bool different_axis = false;
        const double first = noise.Sample(2, 0, .001);
        Require(first == repeat.Sample(2, 0, .001), "same seed did not reproduce noise");
        noise.Reset();
        Require(
            noise.Sample(2, 0, .001) == first, "reset did not reproduce the initial sensor state");

        noise.Reset();
        other_axis.Reset();

        for (int i = 0; i < 20000; ++i) {
            const double reading = noise.Sample(2, 0, .001);
            const double error = reading - 2;
            sum += error;
            squared += error * error;
            different_axis |= reading != other_axis.Sample(2, 0, .001);
        }

        Require(std::abs(sum / 20000) < .005 && std::abs(std::sqrt(squared / 20000) - .1) < .005,
            "configured white-noise statistics were not applied");
        Require(different_axis, "axis seeds did not produce independent streams");

        for (const char* key :
            {"noise_std_a", "bandwidth_hz", "quantization_a", "current_limit_a"}) {
            auto invalid = YAML::Clone(config);
            invalid["iq_sensor"][key] = -1;
            std::ofstream(path) << invalid;
            bool rejected = false;

            try {
                qhrr05::VirtualIqSensor bad(path, 0);
            } catch (const std::exception&) {
                rejected = true;
            }

            Require(rejected, "invalid sensor setting was silently accepted");
        }

        s["noise_std_a"] = 0;
        s["gain"] = YAML::Load("[1,2,3]");
        save();
        qhrr05::VirtualIqSensor third_axis(path, 2);
        Require(third_axis.Sample(2, 0, .001) == 6, "per-axis sensor gain was not selected");
        config["iq_sensor"].remove("enabled");
        save();
        bool missing_rejected = false;

        try {
            qhrr05::VirtualIqSensor missing(path, 0);
        } catch (const std::exception&) {
            missing_rejected = true;
        }

        Require(missing_rejected, "missing enable setting silently disabled the sensor");

        bool rejected = false;

        try {
            noise.Sample(2, 0, 0);
        } catch (const std::exception&) {
            rejected = true;
        }

        Require(rejected, "zero sample period was accepted");
        std::filesystem::remove_all(directory);
        std::cout
            << "PASS: Iq bypass, affine/ripple, filter, ADC, clipping, seeded noise/reset, validation\n";
        return 0;
    } catch (const std::exception& error) {
        std::filesystem::remove_all(directory);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
