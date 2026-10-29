#include "motion_controllers/numerics/math.hpp"
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
using namespace qhrr05::numeric;

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

void CheckMath() {
    const Mat3 j{.3, .02, .04, -.03, .25, .01, .02, -.01, .2};
    const Vec3 x{.2, -.3, .7};
    JacobianSolver solver;
    Require(solver.Factor(j, {}) && solver.method() == SolveMethod::DirectInverse,
        "well conditioned J must use actual inverse");
    Vec3 result;
    Require(solver.Solve(Multiply(j, x), result), "inverse solve failed");
    Near(result, x.data(), 1e-13, "inverse residual");
    Require(solver.SolveTranspose(TransposeMultiply(j, x), result), "transpose inverse failed");
    Near(result, x.data(), 1e-13, "transpose residual");
    const Mat3 rank_deficient{.2, 0, 0, 0, .1, 0, 0, 0, 0};
    Require(solver.Factor(rank_deficient, {}) && solver.method() == SolveMethod::AdaptiveDls,
        "rank deficient must use DLS");
    Require(std::abs(solver.damping() - .03) < 1e-15, "rank deficient damping");
    Require(solver.Solve(x, result) && Norm(result) < 20, "DLS finite bounded solve");
    Require(solver.SolveTranspose(x, result) && Norm(result) < 20, "DLS finite bounded transpose");
    Mat3 inverse{};
    Require(!Inverse(rank_deficient, inverse), "singular inverse must fail");
    Require(solver.Factor(Mat3{1, 0, 0, 0, .5, 0, 0, 0, .025}, {}) &&
                solver.method() == SolveMethod::AdaptiveDls,
        "bad condition must use DLS even with sufficient sigma_min");
    const double bad_damping = solver.damping();
    Require(solver.Factor(Mat3{1, 0, 0, 0, .5, 0, 0, 0, .07}, {}) && solver.damping() < bad_damping,
        "adaptive damping must increase toward singularity");
    Mat3 invalid = j;
    invalid[0] = std::numeric_limits<double>::quiet_NaN();
    Require(!solver.Factor(invalid, {}) && !solver.Solve(x, result),
        "invalid input must fail explicitly");
    std::mt19937 random(716);
    std::uniform_real_distribution<double> sample(-1, 1);
    for (int n = 0; n < 1000; ++n) {
        Mat3 a;
        for (auto& v : a)
            v = sample(random);
        auto spectrum = SingularValues(a);
        Mat3 gram{};
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                for (int l = 0; l < 3; ++l)
                    gram[3 * i + k] += a[3 * i + l] * a[3 * k + l];
        double eigen[3], vectors[9], quat[4];
        mju_eig3(eigen, vectors, quat, gram.data());
        Require(spectrum.valid &&
                    std::abs(spectrum.maximum - std::sqrt(*std::max_element(eigen, eigen + 3))) <
                        1e-10 &&
                    std::abs(spectrum.minimum -
                             std::sqrt(std::max(0., *std::min_element(eigen, eigen + 3)))) < 1e-9,
            "spectrum disagrees with MuJoCo");
    }
    std::array<double, 24> coefficients;
    for (auto& c : coefficients)
        c = sample(random) * .1;
    for (int i = 0; i < 1000; ++i) {
        const double phase = i * .031;
        const auto series = EvaluateCosineSeries(coefficients.data(), coefficients.size(), phase);
        double value = 0, derivative = 0;
        for (int k = 0; k < 24; ++k) {
            value += coefficients[k] * std::cos((k + 1) * phase);
            derivative -= (k + 1) * coefficients[k] * std::sin((k + 1) * phase);
        }
        Require(std::abs(series.value - value) < 1e-12 &&
                    std::abs(series.derivative - derivative) < 1e-10,
            "harmonic recurrence changed trajectory");
    }
    volatile double sink = 0;
    const auto direct = Benchmark(
        [&](int i) {
            auto a = j;
            a[0] += i * 1e-10;
            solver.Factor(a, {});
            solver.Solve(x, result);
            solver.Solve(x, result);
            sink = result[0];
        },
        50000);
    const auto legacy = Benchmark(
        [&](int i) {
            auto a = j;
            a[0] += i * 1e-10;
            Mat3 gram{};
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    for (int k = 0; k < 3; ++k)
                        gram[3 * r + c] += a[3 * r + k] * a[3 * c + k];
            double ev[3], v[9], q[4];
            mju_eig3(ev, v, q, gram.data());
            for (int solve = 0; solve < 2; ++solve) {
                auto h = gram;
                for (int k = 0; k < 3; ++k)
                    h[3 * k + k] += 1e-6;
                mju_cholFactor(h.data(), 3, 1e-15);
                Vec3 z{};
                mju_cholSolve(z.data(), h.data(), x.data(), 3);
                result = TransposeMultiply(a, z);
            }
            sink = result[0];
        },
        50000);
    std::cout << "BENCH Jacobian factor + 2 solves: direct=" << direct
              << " us, eigenvectors + repeated DLS=" << legacy << " us\n";
    (void)sink;
}

int main() {
    try {
        CheckMath();
        std::cout << "PASS: motion-controller numerics\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
