#include "interfaces/observations.hpp"
#include <iostream>
#include <limits>

void Check(bool value, const char* reason) {
    if (!value)
        throw std::runtime_error(reason);
}

int main() {
    try {
        MotorData data;
        Check(!qhrr05::ReadObservation(data).position_velocity, "unreceived zeros appeared valid");
        data.supported.fill(kCanSignals);
        Check(!qhrr05::HasSignals(data, 0, MotorCurrent), "supported alone appeared received");
        data.valid.fill(kCanSignals);
        data.received_sequence.fill(1);
        data.received_ns.fill(100);
        data.measurement_ns.fill(100);
        data.enabled.fill(true);
        auto observed = qhrr05::ReadObservation(data);
        Check(
            observed.position_velocity && observed.torque_valid, "real numeric zero was rejected");
        Check(!qhrr05::CoordinatesValid(observed.coordinate_valid),
            "raw encoder implied calibrated joint zero");
        data.coordinate_valid.fill(true);
        data.position_offset = {1, 2, 3};
        Check(qhrr05::ReadObservation(data).position[1] == 2, "joint frame conversion lost");
        data.valid[1] = 0;
        Check(!qhrr05::ReadObservation(data).position_velocity,
            "invalid axis was hidden by other valid axes");
        data.valid.fill(kDebugSignals);
        data.supported.fill(kDebugSignals);
        Check(!qhrr05::HasSignals(data, 0, MotorCurrent), "direct debug invented current feedback");
        Check(
            qhrr05::ReadObservation(data).torque_valid, "explicit effort observation disappeared");
        data.valid[0] = JointVelocity;
        Check(qhrr05::ReadObservation(data).velocity_valid,
            "velocity channel depends on unrelated position availability");
        data.feedback[0].velocity = std::numeric_limits<double>::quiet_NaN();
        Check(
            !qhrr05::ReadObservation(data).position_velocity, "nonfinite feedback appeared valid");
        std::cout
            << "PASS: capability, sample validity, calibrated coordinates, derived torque and unavailable current\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
