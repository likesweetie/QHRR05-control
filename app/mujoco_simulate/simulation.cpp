#include <array>
#include "simulation.hpp"
#include <poll.h>
#include <cerrno>
#include <sys/socket.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qhrr05 {
std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model) {
    const auto values = value.as<std::vector<double>>();
    if (values.size() != 3) throw std::runtime_error("expected three joint positions");
    std::array<double, 3> result;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(values[i]) || values[i] < model->jnt_range[2*i] || values[i] > model->jnt_range[2*i+1])
            throw std::runtime_error("joint position is outside model limits");
        result[i] = values[i];
    }
    return result;
}

void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data) {
    mj_resetData(model, data);
    std::copy(initial.begin(), initial.end(), data->qpos);
    mj_forward(model, data);
}

void ExchangeControl(int fd, const mjModel* model, mjData* data, std::array<double, 3>& reference) {
    mj_forward(model, data);
    // Directly exchange the three joints: time, q[3], dq[3], bias[3] -> torque[3], reference[3].
    double input[10] = {data->time};
    for (int i = 0; i < 3; ++i) {
        input[1+i] = data->qpos[i];
        input[4+i] = data->qvel[i];
        input[7+i] = data->qfrc_bias[i];
    }
    ssize_t size;
    do { size = ::send(fd, input, sizeof(input), MSG_NOSIGNAL); } while (size < 0 && errno == EINTR);
    if (size != sizeof(input)) throw std::runtime_error("joint input write failed");
    pollfd descriptor{fd, POLLIN, 0};
    int ready;
    do { ready = ::poll(&descriptor, 1, 1000); } while (ready < 0 && errno == EINTR);
    if (ready <= 0) throw std::runtime_error("control response timed out or poll failed");
    double output[6];
    do { size = ::recv(fd, output, sizeof(output), MSG_TRUNC); } while (size < 0 && errno == EINTR);
    if (size != sizeof(output)) throw std::runtime_error("joint output read failed or has the wrong size");
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(output[i]) || !std::isfinite(output[3+i])) throw std::runtime_error("non-finite joint output");
        data->ctrl[i] = std::clamp(output[i], model->actuator_ctrlrange[2*i], model->actuator_ctrlrange[2*i+1]);
        reference[i] = output[3+i];
    }
}
}  // namespace qhrr05
