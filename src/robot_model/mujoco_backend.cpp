#include "robot_model/backend.hpp"
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <set>
#include <stdexcept>
#include <vector>

namespace qhrr05 {
namespace {
using Vec3 = std::array<double, 3>;

template <std::size_t N> void Finite(const std::array<double, N>& v) {
    for (double x : v)
        if (!std::isfinite(x))
            throw std::runtime_error("non-finite robot backend state");
}

void Remove(mjSpec* spec, mjsElement* element) {
#if mjVERSION_HEADER >= 3016000
    const int error = mjs_delete(spec, element);
#else
    const int error = mjs_delete(element);
#endif
    if (error)
        throw std::runtime_error("cannot remove render-only model asset");
}

class MujocoBackend final : public ModelBackend {
public:
    explicit MujocoBackend(const std::string& config_path) {
        const auto config = YAML::LoadFile(config_path);
        const auto path = std::filesystem::path(config_path).parent_path() /
                          config["model_path"].as<std::string>();
        char error[2048]{};
        std::unique_ptr<mjSpec, decltype(&mj_deleteSpec)> spec(
            mj_parseXML(path.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
        if (!spec)
            throw std::runtime_error("robot backend parse failed: " + std::string(error));
        // Only discard visuals whose body's mass/inertia are explicitly specified.
        // Never derive or alter a physical body's inertia to save a mesh load.
        std::vector<mjsElement*> visuals;
        for (auto* e = mjs_firstElement(spec.get(), mjOBJ_GEOM); e;
             e = mjs_nextElement(spec.get(), e)) {
            auto* geom = mjs_asGeom(e);
            auto* body = mjs_getParent(e);
            if (geom->type == mjGEOM_MESH && !geom->contype && !geom->conaffinity &&
                body->explicitinertial && spec->compiler.inertiafromgeom != mjINERTIAFROMGEOM_TRUE)
                visuals.push_back(e);
        }
        for (auto* e : visuals)
            Remove(spec.get(), e);
        std::set<std::string> used;
        for (auto* e = mjs_firstElement(spec.get(), mjOBJ_GEOM); e;
             e = mjs_nextElement(spec.get(), e)) {
            auto* g = mjs_asGeom(e);
            if (g->type == mjGEOM_MESH)
                used.insert(mjs_getString(g->meshname));
        }
        std::vector<mjsElement*> unused;
        for (auto* e = mjs_firstElement(spec.get(), mjOBJ_MESH); e;
             e = mjs_nextElement(spec.get(), e)) {
#if mjVERSION_HEADER >= 3016000
            const auto* name = mjs_getString(mjs_getName(e));
#else
            const auto* name = mjs_getString(mjs_asMesh(e)->name);
#endif
            if (!used.count(name))
                unused.push_back(e);
        }
        for (auto* e : unused)
            Remove(spec.get(), e);
        model_.reset(mj_compile(spec.get(), nullptr));
        if (!model_)
            throw std::runtime_error(
                "robot backend compile failed: " + std::string(mjs_getError(spec.get())));
        auto* m = model_.get();
        if (m->nq != 3 || m->nv != 3 || m->nu != 3)
            throw std::runtime_error("robot backend requires three fixed-base joints/actuators");
        const char* joints[] = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
        for (int i = 0; i < 3; ++i) {
            const int j = mj_name2id(m, mjOBJ_JOINT, joints[i]);
            if (j < 0 || m->jnt_type[j] != mjJNT_HINGE || m->jnt_qposadr[j] != i ||
                m->jnt_dofadr[j] != i)
                throw std::runtime_error("robot backend joint ordering mismatch");
            state_.joint_min[i] = m->jnt_range[2 * j];
            state_.joint_max[i] = m->jnt_range[2 * j + 1];
            state_.friction[i] = m->dof_frictionloss[i];
            state_.damping[i] = m->dof_damping[i];
        }
        site_ = mj_name2id(m, mjOBJ_SITE, "FL_foot");
        if (site_ < 0)
            throw std::runtime_error("robot backend requires FL_foot site");
        actual_.reset(mj_makeData(m));
        scratch_.reset(mj_makeData(m));
        gravity_.reset(mj_makeData(m));
        if (!actual_ || !scratch_ || !gravity_)
            throw std::runtime_error("robot backend data allocation failed");
        std::cout << "[model-backend] MuJoCo " << mj_versionString() << " model=" << path.string()
                  << " render-only meshes removed=" << unused.size() << '\n';
    }

    const RobotModelState& state() const override {
        return state_;
    }

    std::uint64_t updates() const override {
        return updates_;
    }

    const RobotModelState& Update(const Vec3& q, const Vec3& dq) override {
        Finite(q);
        Finite(dq);
        const bool pose = !initialized_ || q != state_.joint_position;
        if (!pose && dq == state_.joint_velocity)
            return state_;
        auto* m = model_.get();
        auto* d = actual_.get();
        if (pose) {
            Position(d, q);
            mj_crb(m, d);
#if mjVERSION_HEADER >= 3016000
            mj_fullM(m, d, state_.mass.data());
#else
            mj_fullM(m, state_.mass.data(), d->qM);
#endif
            mj_jacSite(m, d, state_.jacobian.data(), nullptr, site_);
            std::copy_n(d->site_xpos + 3 * site_, 3, state_.toe_position.begin());
            Position(gravity_.get(), q);
            mju_zero(gravity_->qvel, 3);
            mj_comVel(m, gravity_.get());
            mj_rne(m, gravity_.get(), 0, state_.gravity.data());
        }
        std::copy(dq.begin(), dq.end(), d->qvel);
        mj_fwdVelocity(m, d);
        std::copy_n(d->qfrc_bias, 3, state_.bias.begin());
        std::copy_n(d->qfrc_passive, 3, state_.passive.begin());
        Finite(state_.mass);
        Finite(state_.jacobian);
        Finite(state_.toe_position);
        Finite(state_.gravity);
        Finite(state_.bias);
        Finite(state_.passive);
        state_.joint_position = q;
        state_.joint_velocity = dq;
        initialized_ = true;
        ++updates_;
        return state_;
    }

    KinematicState Evaluate(const Vec3& q, bool jacobian) override {
        Prepare(q, {}, false);
        KinematicState result;
        std::copy_n(scratch_->site_xpos + 3 * site_, 3, result.position.begin());
        if (jacobian)
            mj_jacSite(model_.get(), scratch_.get(), result.jacobian.data(), nullptr, site_);
        Finite(result.position);
        if (jacobian)
            Finite(result.jacobian);
        return result;
    }

    Vec3 ConvectiveAcceleration(const Vec3& q, const Vec3& dq) override {
        Prepare(q, dq, true);
        std::array<double, 9> dot{};
        mj_jacDot(model_.get(),
            scratch_.get(),
            dot.data(),
            nullptr,
            scratch_->site_xpos + 3 * site_,
            model_->site_bodyid[site_]);
        Vec3 result;
        mju_mulMatVec(result.data(), dot.data(), dq.data(), 3, 3);
        Finite(result);
        return result;
    }

    Vec3 InverseDynamics(const Vec3& q, const Vec3& dq, const Vec3& ddq) override {
        Finite(ddq);
        if (initialized_ && q == state_.joint_position && dq == state_.joint_velocity) {
            Vec3 result;
            mju_mulMatVec(result.data(), state_.mass.data(), ddq.data(), 3, 3);
            for (int i = 0; i < 3; ++i)
                result[i] += state_.bias[i];
            Finite(result);
            return result;
        }
        Prepare(q, dq, true);
        std::copy(ddq.begin(), ddq.end(), scratch_->qacc);
        Vec3 torque;
        mj_rne(model_.get(), scratch_.get(), 1, torque.data());
        for (int i = 0; i < 3; ++i)
            torque[i] += model_->dof_armature[i] * ddq[i];
        Finite(torque);
        return torque;
    }

private:
    void Position(mjData* d, const Vec3& q) {
        std::copy(q.begin(), q.end(), d->qpos);
        mj_kinematics(model_.get(), d);
        mj_comPos(model_.get(), d);
        mj_tendon(model_.get(), d);
        mj_transmission(model_.get(), d);
    }

    void Prepare(const Vec3& q, const Vec3& dq, bool velocity) {
        Finite(q);
        if (velocity)
            Finite(dq);
        if (!scratch_valid_ || q != scratch_q_) {
            Position(scratch_.get(), q);
            scratch_q_ = q;
            scratch_valid_ = true;
            velocity_valid_ = false;
        }
        if (velocity && (!velocity_valid_ || dq != scratch_dq_)) {
            std::copy(dq.begin(), dq.end(), scratch_->qvel);
            mj_comVel(model_.get(), scratch_.get());
            scratch_dq_ = dq;
            velocity_valid_ = true;
        }
    }

    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> actual_{nullptr, mj_deleteData},
        scratch_{nullptr, mj_deleteData}, gravity_{nullptr, mj_deleteData};
    RobotModelState state_{};
    Vec3 scratch_q_{}, scratch_dq_{};
    int site_ = -1;
    bool initialized_ = false, scratch_valid_ = false, velocity_valid_ = false;
    std::uint64_t updates_ = 0;
};
}

std::unique_ptr<ModelBackend> MakeMujocoBackend(const std::string& config) {
    return std::make_unique<MujocoBackend>(config);
}
}  // namespace qhrr05
