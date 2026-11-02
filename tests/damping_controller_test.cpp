#include "motion_controllers/damping_controller.hpp"
#include "motor_can_protocol/openrobot_protocol.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unistd.h>

namespace {
void Require(bool condition, const char* reason) {
    if (!condition)
        throw std::runtime_error(reason);
}
void Near(double actual, double expected) {
    Require(std::abs(actual - expected) < 1e-10, "incorrect differentiated damping torque");
}
}

int main() {
    const auto path = std::filesystem::temp_directory_path() /
                      ("qhrr-damping-" + std::to_string(getpid()) + ".yaml");
    try {
        std::ofstream(path) << "damping: {gain_nm_s_rad: [1, 2, 0]}\n"
                               "limits: {torque_nm: [3, 4, 5]}\n";
        qhrr05::DampingController controller(path.string());
        std::array<double, 3> position_reference{}, torque{};
        std::array<double, 3> q{1, 2, 3};
        std::array<std::uint64_t, 3> seq{1, 1, 1}, ns{1000000000, 1000000000, 1000000000};
        controller.Compute(q, seq, ns, position_reference, torque);
        for (double value : torque)
            Near(value, 0);  // First sample seeds position, never differentiates from zero.

        q = {1.01, 1.99, 3.50};
        seq.fill(2);
        ns.fill(1010000000);
        controller.Compute(q, seq, ns, position_reference, torque);
        const double decay = std::exp(-2 * 3.141592653589793 * 30 * .01);
        const double first_step = 1 - decay;
        Near(torque[0], -first_step);
        Near(torque[1], 2 * first_step);
        Near(torque[2], 0);
        for (int repeat = 0; repeat < 100; ++repeat) {
            controller.Compute(q, seq, ns, position_reference, torque);
            Near(torque[0], -first_step);
            Near(torque[1], 2 * first_step);
        }
        q[0] += .02;
        seq[0] = 3;
        ns[0] += 10000000;
        controller.Compute(q, seq, ns, position_reference, torque);
        Near(torque[0], -(2 - (2 - first_step) * decay));
        Near(torque[1], 2 * first_step);  // Independent sample/filter histories.

        q[0] += 1;
        seq[0] = 4;
        ns[0] += 1000;
        controller.Compute(q, seq, ns, position_reference, torque);
        Near(torque[0], -3);  // Common output torque limit.
        ns.fill(100);
        controller.Compute(q, seq, ns, position_reference, torque);
        for (double value : torque)
            Near(value, 0);  // Clock reset also works when the sequence is repeated.
        controller.Reset();
        q.fill(7);
        controller.Compute(q, seq, ns, position_reference, torque);
        for (double value : torque)
            Near(value, 0);  // Mode reentry does not create an impulse.

        // The pole follows elapsed sample time even when encoder cadence varies.
        for (const auto step_ns : {1000000ULL, 7000000ULL, 2000000ULL, 11000000ULL}) {
            q[0] += step_ns / 1e9;  // Constant 1 rad/s.
            ns[0] += step_ns;
            ++seq[0];
            controller.Compute(q, seq, ns, position_reference, torque);
        }
        Near(controller.velocity()[0], 1 - std::exp(-2 * 3.141592653589793 * 30 * .021));
        Near(torque[0], -controller.velocity()[0]);

        // MIT positions retain complete turns; crossing 2*pi must not wrap the input.
        constexpr double turn = 2 * 3.141592653589793;
        for (double direction : {1.0, -1.0}) {
            controller.Reset();
            q.fill(direction * (turn - .001));
            controller.Compute(q, seq, ns, position_reference, torque);
            q[0] = direction * (turn + .001);
            ++seq[0];
            ns[0] += 2000000;
            controller.Compute(q, seq, ns, position_reference, torque);
            Near(controller.velocity()[0],
                direction * (1 - std::exp(-2 * 3.141592653589793 * 30 * .002)));
        }

        controller.Reset();
        q.fill(0);
        controller.Compute(q, seq, ns, position_reference, torque);
        q[0] = turn + .2;
        ++seq[0];
        ns[0] += 4000000000ULL;
        controller.Compute(q, seq, ns, position_reference, torque);
        Near(controller.velocity()[0], (turn + .2) / 4);
        Near(position_reference[0], turn + .2);

        // Measure frequency response at 2 kHz without assuming a particular recurrence.
        const auto measured_gain = [&](double frequency) {
            controller.Reset();
            q.fill(0);
            ns.fill(0);
            seq.fill(0);
            controller.Compute(q, seq, ns, position_reference, torque);
            double in_phase = 0, quadrature = 0;
            for (int sample = 1; sample <= 5000; ++sample) {
                const double phase = 2 * 3.141592653589793 * frequency * sample * .0005;
                q[0] += .0005 * std::cos(phase);
                ns[0] += 500000;
                ++seq[0];
                controller.Compute(q, seq, ns, position_reference, torque);
                if (sample > 1000) {
                    in_phase += controller.velocity()[0] * std::cos(phase);
                    quadrature += controller.velocity()[0] * std::sin(phase);
                }
            }
            return 2 * std::hypot(in_phase, quadrature) / 4000;
        };
        Require(std::abs(measured_gain(3) - 1) < .01, "LPF attenuates low-frequency velocity");
        Require(std::abs(measured_gain(30) - std::sqrt(.5)) < .002,
            "LPF cutoff is not 30 Hz (-3 dB)");
        Require(measured_gain(300) < .12, "LPF does not reject high-frequency velocity");

        for (double torque : {-33., -5., -1., 0., 1., 5., 33.}) {
            const auto bytes = openrobot::EncodeMitCommand({0, 0, 0, 0, torque}, 45, 1, 33);
            const auto decoded = openrobot::DecodeMitCommand(bytes, 45, 1, 33);
            Require(decoded.kp == 0 && decoded.kd == 0, "pure torque packet contains motor gains");
            Require(std::abs(decoded.tau_ff - torque) <= 33. / 255 + 1e-12,
                "CAN torque exceeds half-LSB quantization error");
        }
        Near(openrobot::DecodeMitCommand({0xC0, 0x80, 0, 0x80, 0, 0, 0, 0}, 45, 1, 33).tau_ff, -33);
        Near(openrobot::DecodeMitCommand({0xC0, 0x80, 0, 0x80, 0, 0, 0, 255}, 45, 1, 33).tau_ff, 33);
        std::filesystem::remove(path);
        std::cout << "PASS: 30 Hz velocity LPF, cumulative MIT position, sample hold/reset, limits and CAN torque\n";
        return 0;
    } catch (const std::exception& error) {
        std::filesystem::remove(path);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
