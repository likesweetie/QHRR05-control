#include <array>
#include "simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qhrr05 {
std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model) {
    const auto values = value.as<std::vector<double>>();

    if (values.size() != 3)
        throw std::runtime_error("expected three joint positions");

    std::array<double, 3> result;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(values[i]) || values[i] < model->jnt_range[2 * i] ||
            values[i] > model->jnt_range[2 * i + 1])
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

ShmCommand UpdateControlIO(ShmData& shm,
    const mjModel* model,
    mjData* data,
    mjData* gravity_data,
    std::array<double, 3>& reference,
    const std::array<double, 3>& acceleration,
    std::uint64_t measurement_sequence) {
    mj_forward(model, data);

    // Separate zero-velocity state: qfrc_bias contains gravity without Coriolis.
    mju_copy(gravity_data->qpos, data->qpos, model->nq);
    mju_zero(gravity_data->qvel, model->nv);
    mj_forward(model, gravity_data);

    ShmCommand command;

    {
        shm_utils::Lock lock(shm);

        if (shm.stopping)
            throw std::runtime_error("SHM stopped during control");

        const auto sequence = shm.state_sequence + 1;
        auto& state = shm.states[sequence % 2];
        state.sequence = sequence;

        std::copy_n(data->qpos, 3, state.position.begin());
        std::copy_n(data->qvel, 3, state.velocity.begin());
        std::copy_n(gravity_data->qfrc_bias, 3, state.gravity_torque.begin());

        state.measurement_sequence = measurement_sequence;
        state.measurement_dt = model->opt.timestep;
        state.acceleration = acceleration;

        std::copy_n(data->qfrc_actuator, 3, state.motor_torque.begin());
        std::copy_n(data->qfrc_bias, 3, state.bias_torque.begin());
        std::copy_n(data->qfrc_passive, 3, state.passive_torque.begin());
        std::copy_n(model->dof_frictionloss, 3, state.friction_limit.begin());

        mj_fullM(model, state.mass.data(), data->qM);
        const int site = mj_name2id(model, mjOBJ_SITE, "FL_foot");

        if (site < 0)
            throw std::runtime_error("missing FL_foot site");

        mj_jacSite(model, data, state.jacobian.data(), nullptr, site);
        std::copy_n(data->site_xpos + 3 * site, 3, state.toe_position.begin());

        for (int i = 0; i < 3; ++i) {
            state.joint_min[i] = model->jnt_range[2 * i];
            state.joint_max[i] = model->jnt_range[2 * i + 1];
        }

        state.published_ns = shm_utils::MonotonicNs();
        shm.state_sequence = sequence;
        shm_utils::Notify(shm);

        // Both loops are independent: apply the latest published command,
        // including repeated commands when the controller runs more slowly.
        command = shm.commands[shm.command_sequence % 2];
    }

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(command.torque[i]) || !std::isfinite(command.reference[i]))
            throw std::runtime_error("non-finite joint output");

        data->ctrl[i] = std::clamp(command.torque[i],
            model->actuator_ctrlrange[2 * i],
            model->actuator_ctrlrange[2 * i + 1]);
        reference[i] = command.reference[i];
    }

    return command;
}

std::array<double, 6> ReadContactGroundTruth(const mjModel* model, const mjData* data) {
    const int foot = mj_name2id(model, mjOBJ_GEOM, "FL");
    const int object = mj_name2id(model, mjOBJ_GEOM, "touch_object");
    const int site = mj_name2id(model, mjOBJ_SITE, "FL_foot");

    if (foot < 0 || object < 0 || site < 0)
        throw std::runtime_error("missing Magnetic Touch scene geometry");

    std::array<double, 6> total{};

    for (int i = 0; i < data->ncon; ++i) {
        const auto& contact = data->contact[i];

        if (!((contact.geom[0] == foot && contact.geom[1] == object) ||
                (contact.geom[1] == foot && contact.geom[0] == object)))
            continue;

        mjtNum wrench[6], force[3], moment[3], offset[3], lever[3];
        mj_contactForce(model, data, i, wrench);
        mju_mulMatTVec(force, contact.frame, wrench, 3, 3);
        mju_mulMatTVec(moment, contact.frame, wrench + 3, 3, 3);
        mju_sub3(offset, contact.pos, data->site_xpos + 3 * site);
        mju_cross(lever, offset, force);
        const double sign = contact.geom[1] == foot ? 1 : -1;

        for (int axis = 0; axis < 3; ++axis) {
            total[axis] += sign * force[axis];
            total[3 + axis] += sign * (moment[axis] + lever[axis]);
        }
    }

    return total;
}
}  // namespace qhrr05
