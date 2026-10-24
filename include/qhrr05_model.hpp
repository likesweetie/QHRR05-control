#pragma once

#include "qhrr05_platform.hpp"

#include <mujoco/mujoco.h>

#include <stdexcept>

namespace qhrr05 {

inline void ValidateModel(const mjModel* model) {
    if (!model || !HasDimensions(model->nq, model->nv, model->nu) ||
        model->njnt != kJointCount || model->nbody != kJointCount + 2 ||
        model->nsensor != 0) {
        throw std::runtime_error("qhrr-05 requires a fixed torso and exactly 3 FL joints without sensors");
    }
    const int base = mj_name2id(model, mjOBJ_BODY, "base");
    if (base <= 0 || model->body_parentid[base] != 0 || model->body_jntnum[base] != 0) {
        throw std::runtime_error("qhrr-05 torso must be fixed directly to world");
    }
    int parent = base;
    for (int i = 0; i < kJointCount; ++i) {
        const int body = mj_name2id(model, mjOBJ_BODY, kLinkNames[i]);
        const int joint = mj_name2id(model, mjOBJ_JOINT, kJointNames[i]);
        const int actuator = mj_name2id(model, mjOBJ_ACTUATOR, kActuatorNames[i]);
        if (body < 0 || joint != i || actuator != i ||
            model->body_parentid[body] != parent || model->jnt_bodyid[joint] != body ||
            model->jnt_type[joint] != mjJNT_HINGE || model->jnt_qposadr[joint] != i ||
            model->jnt_dofadr[joint] != i || model->actuator_trntype[i] != mjTRN_JOINT ||
            model->actuator_trnid[2*i] != joint) {
            throw std::runtime_error("qhrr-05 FL joint/link/actuator names, order or mapping do not match");
        }
        parent = body;
    }
}

}  // namespace qhrr05
