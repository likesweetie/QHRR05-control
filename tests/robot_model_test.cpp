#include "robot_model/backend.hpp"
#include <mujoco/mujoco.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

using namespace qhrr05;
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;

Vec3 ModelProduct(const Mat3& matrix, const Vec3& vector) {
    Vec3 result;
    mju_mulMatVec(result.data(), matrix.data(), vector.data(), 3, 3);
    return result;
}

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

template <std::size_t N>
void Near(const std::array<double, N>& a, const double* b, double tolerance, const char* message) {
    for (std::size_t i = 0; i < N; ++i)
        if (std::abs(a[i] - b[i]) > tolerance || !std::isfinite(a[i])) {
            std::cerr << message << " index=" << i << " actual=" << a[i] << " expected=" << b[i]
                      << '\n';
            throw std::runtime_error(message);
        }
}

template <class F> double Benchmark(F fn, int iterations) {
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i)
        fn(i);
    return std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start)
               .count() /
           iterations;
}

void DenseMass(const mjModel* model, const mjData* data, double* mass) {
#if mjVERSION_HEADER >= 3016000
    mj_fullM(model, data, mass);
#else
    mj_fullM(model, mass, data->qM);
#endif
}

void CheckInertiaCompilation() {
    const auto dir =
        std::filesystem::temp_directory_path() / ("qhrr-inertia-" + std::to_string(::getpid()));
    std::filesystem::create_directory(dir);
    try {
        for (const std::string mode : {"auto", "true"}) {
            const auto xml = dir / "model.xml";
            std::ofstream out(xml);
            out << "<mujoco><compiler angle='radian' inertiafromgeom='" << mode
                << "'/><asset><mesh name='m' vertex='0 0 0  .1 0 0  0 .1 0  0 0 .1'/></asset><worldbody>";
            const char* names[] = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
            for (int i = 0; i < 3; ++i) {
                out << "<body pos='0 0 .2'><joint name='" << names[i]
                    << "' type='hinge' range='-3 3'/>";
                // First body needs inferred mesh inertia, others have explicit inertia.
                if (i)
                    out << "<inertial pos='0 0 0' mass='2' diaginertia='.01 .02 .03'/>";
                out << "<geom type='mesh' mesh='m' contype='0' conaffinity='0'/>";
            }
            out << "<site name='FL_foot' pos='0 0 .1'/></body></body></body></worldbody><actuator>";
            for (const auto* name : names)
                out << "<motor joint='" << name << "'/>";
            out << "</actuator></mujoco>";
            out.close();
            const auto yaml = dir / "control.yaml";
            std::ofstream(yaml) << "model_path: model.xml\n";
            auto backend = MakeMujocoBackend(yaml.string());
            char error[1024]{};
            std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                mj_loadXML(xml.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
            Require(bool(model), error);
            std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
                mj_makeData(model.get()), mj_deleteData);
            mj_forward(model.get(), data.get());
            Mat3 mass{};
            DenseMass(model.get(), data.get(), mass.data());
            Near(backend->Update({}, {}).mass,
                mass.data(),
                1e-12,
                "inertia stripping changed physical model");
        }
        std::filesystem::remove_all(dir);
    } catch (...) {
        std::filesystem::remove_all(dir);
        throw;
    }
}

int main(int argc, char** argv) {
    try {
        Require(argc == 3, "expected control config and full model");
        CheckInertiaCompilation();
        const auto start = std::chrono::steady_clock::now();
        auto backend = MakeMujocoBackend(argv[1]);
        const double backend_load =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        char error[2048]{};
        const auto full_start = std::chrono::steady_clock::now();
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(bool(model), error);
        const double full_load =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - full_start)
                .count();
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        const int site = mj_name2id(model.get(), mjOBJ_SITE, "FL_foot");
        std::mt19937 random(137);
        std::uniform_real_distribution<double> sample(-1, 1);
        for (int n = 0; n < 200; ++n) {
            Vec3 q{sample(random) * .5, .9 + sample(random) * .6, -1.8 + sample(random) * .5};
            Vec3 dq{sample(random) * 2, sample(random) * 2, sample(random) * 2},
                ddq{sample(random) * 5, sample(random) * 5, sample(random) * 5};
            std::copy(q.begin(), q.end(), data->qpos);
            std::copy(dq.begin(), dq.end(), data->qvel);
            mj_forward(model.get(), data.get());
            const auto state = backend->Update(q, dq);
            Mat3 mass{}, jac{}, jdot{};
            DenseMass(model.get(), data.get(), mass.data());
            mj_jacSite(model.get(), data.get(), jac.data(), nullptr, site);
            Near(state.mass, mass.data(), 1e-11, "mass");
            Near(state.jacobian, jac.data(), 1e-12, "J");
            Near(state.toe_position, data->site_xpos + 3 * site, 1e-12, "toe");
            Near(state.bias, data->qfrc_bias, 1e-11, "bias");
            Near(state.passive, data->qfrc_passive, 1e-11, "passive");
            Near(state.damping, model->dof_damping, 1e-12, "damping");
            Near(state.friction, model->dof_frictionloss, 1e-12, "friction");
            mj_jacDot(model.get(),
                data.get(),
                jdot.data(),
                nullptr,
                data->site_xpos + 3 * site,
                model->site_bodyid[site]);
            const auto expected_convective = ModelProduct(jdot, dq);
            Near(backend->ConvectiveAcceleration(q, dq),
                expected_convective.data(),
                1e-11,
                "Jdot dq");
            // Independent directional derivative of J, not just the same MuJoCo call.
            Vec3 plus = q, minus = q;
            for (int i = 0; i < 3; ++i) {
                plus[i] += 1e-6 * dq[i];
                minus[i] -= 1e-6 * dq[i];
            }
            auto jp = backend->Evaluate(plus).jacobian, jm = backend->Evaluate(minus).jacobian;
            for (int i = 0; i < 9; ++i)
                jp[i] = (jp[i] - jm[i]) / 2e-6;
            auto derivative = ModelProduct(jp, dq);
            Near(expected_convective, derivative.data(), 1e-8, "finite difference Jdot");
            auto expected = ModelProduct(mass, ddq);
            for (int i = 0; i < 3; ++i)
                expected[i] += data->qfrc_bias[i];
            Near(backend->InverseDynamics(q, dq, ddq),
                expected.data(),
                1e-10,
                "measured inverse dynamics");
            // Force scratch/RNE path while keeping the same expected physics.
            backend->Update(plus, dq);
            Near(backend->InverseDynamics(q, dq, ddq),
                expected.data(),
                1e-10,
                "target inverse dynamics");
            backend->Update(q, dq);
            const auto updates = backend->updates();
            backend->Update(q, dq);
            Require(backend->updates() == updates, "unchanged input recomputed");
            backend->Evaluate(plus);
            backend->ConvectiveAcceleration(minus, dq);
            Require(backend->state().joint_position == q && backend->state().joint_velocity == dq,
                "scratch evaluation corrupted measured state");
            mju_zero(data->qvel, 3);
            mj_forward(model.get(), data.get());
            Near(state.gravity, data->qfrc_bias, 1e-11, "gravity");
        }
        const Vec3 q{.1, .9, -1.8}, dq{.2, -.3, .4};
        volatile double sink = 0;
        const auto fast = Benchmark(
            [&](int i) {
                auto a = q;
                a[0] += i * 1e-7;
                sink = backend->Update(a, dq).bias[0];
            },
            10000);
        const auto full = Benchmark(
            [&](int i) {
                std::copy(q.begin(), q.end(), data->qpos);
                data->qpos[0] += i * 1e-7;
                std::copy(dq.begin(), dq.end(), data->qvel);
                mj_forward(model.get(), data.get());
                sink = data->qfrc_bias[0];
            },
            10000);
        const auto fk = Benchmark(
            [&](int i) {
                auto a = q;
                a[0] += i * 1e-7;
                sink = backend->Evaluate(a).jacobian[0];
            },
            10000);
        std::cout << "BENCH model load: backend=" << backend_load
                  << " ms, full visual model=" << full_load << " ms\n"
                  << "BENCH measured update (M/J/bias/gravity/passive)=" << fast
                  << " us, full mj_forward=" << full << " us, IK FK/J=" << fk << " us\n";
        (void)sink;
        std::cout
            << "PASS: 200 pose dynamics/Jdot comparisons, inertia preservation, scratch isolation and cache\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
