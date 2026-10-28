#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
void Require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "Expected one model path");
        char error[2048] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, std::string("Model load failed: ") + error);
        const mjModel* m = model.get();
        Require(m->nq == 3 && m->nv == 3 && m->nu == 3, "Expected nq=nv=nu=3");
        Require(m->njnt == 3 && m->nbody == 5 + m->nmocap, "Expected torso and FL links only");
        Require(m->nsensor == 0, "Unexpected sensor hardware");
        const int base = mj_name2id(m, mjOBJ_BODY, "base");
        Require(base > 0 && m->body_parentid[base] == 0 && m->body_jntnum[base] == 0,
            "Torso must be fixed directly to world");
        const std::array<const char*, 3> joints = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};

        for (int i = 0; i < 3; ++i) {
            const int j = mj_name2id(m, mjOBJ_JOINT, joints[i]);
            Require(j == i && m->jnt_type[j] == mjJNT_HINGE, "Unexpected joint order/type");
            Require(m->jnt_qposadr[j] == i && m->jnt_dofadr[j] == i,
                "Fixed-base joint addresses must start at zero");
            Require(m->actuator_trntype[i] == mjTRN_JOINT && m->actuator_trnid[2 * i] == j,
                "Actuator is mapped to the wrong joint");
            Require(m->actuator_ctrllimited[i] &&
                        std::isfinite(m->actuator_ctrlrange[2 * i]) &&
                        std::isfinite(m->actuator_ctrlrange[2 * i + 1]) &&
                        m->actuator_ctrlrange[2 * i] < 0 &&
                        m->actuator_ctrlrange[2 * i + 1] > 0 &&
                        std::abs(m->actuator_ctrlrange[2 * i] +
                                 m->actuator_ctrlrange[2 * i + 1]) < 1e-10,
                "Motor torque limits must be finite, symmetric and span zero");
        }

        const int home = mj_name2id(m, mjOBJ_KEY, "home");
        Require(home >= 0, "Missing home keyframe");
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(m), mj_deleteData);
        Require(data != nullptr, "Could not allocate simulation data");
        mjData* d = data.get();
        mj_resetDataKeyframe(m, d, home);

        for (int i = 0; i < 3; ++i) {
            Require(d->ctrl[i] == 0, "Home keyframe must not command motor torque");
            Require(d->qpos[i] >= m->jnt_range[2 * i] && d->qpos[i] <= m->jnt_range[2 * i + 1],
                "Home pose is outside joint limits");
        }

        mj_forward(m, d);
        const std::array<double, 3> base_position = {
            d->xpos[3 * base], d->xpos[3 * base + 1], d->xpos[3 * base + 2]};
        const std::array<double, 4> base_quaternion = {d->xquat[4 * base],
            d->xquat[4 * base + 1],
            d->xquat[4 * base + 2],
            d->xquat[4 * base + 3]};

        for (int step = 0; step < 2000; ++step) {
            for (int i = 0; i < 3; ++i)
                d->ctrl[i] = 0.2 * std::sin(d->time + i);

            d->xfrc_applied[6 * base] = 100;
            mj_step(m, d);

            for (int i = 0; i < 3; ++i) {
                Require(std::isfinite(d->qpos[i]) && std::isfinite(d->qvel[i]),
                    "Non-finite state during stepping");
                Require(std::abs(d->xpos[3 * base + i] - base_position[i]) < 1e-12,
                    "Fixed torso translated");
            }

            for (int i = 0; i < 4; ++i) {
                Require(std::abs(d->xquat[4 * base + i] - base_quaternion[i]) < 1e-12,
                    "Fixed torso rotated");
            }
        }

        Require(std::abs(d->time - 2.0) < 1e-9, "Simulation time reset or timestep changed");

        for (int i = 0; i < mjNWARNING; ++i) {
            Require(d->warning[i].number == 0, "MuJoCo reported a dynamics warning");
        }

        std::cout << "PASS: " << argv[1]
                  << " (3 FL joints, fixed torso, motor limits, 2000 steps)\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
