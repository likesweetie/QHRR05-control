#include "motion_controllers/numerics/math.hpp"
#include <algorithm>
#include <cmath>

namespace qhrr05::numeric {
namespace {
template <std::size_t N> bool Finite(const std::array<double, N>& a) {
    return std::all_of(a.begin(), a.end(), [](double x) {
        return std::isfinite(x);
    });
}
}

Vec3 Subtract(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 Lerp(const Vec3& a, const Vec3& b, double t) {
    return {a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])};
}

double Dot(const Vec3& a, const Vec3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double Norm(const Vec3& a) {
    return std::hypot(a[0], a[1], a[2]);
}

double Distance(const Vec3& a, const Vec3& b) {
    return Norm(Subtract(a, b));
}

Vec3 Multiply(const Mat3& a, const Vec3& b) {
    return {a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
        a[3] * b[0] + a[4] * b[1] + a[5] * b[2],
        a[6] * b[0] + a[7] * b[1] + a[8] * b[2]};
}

Vec3 TransposeMultiply(const Mat3& a, const Vec3& b) {
    return {a[0] * b[0] + a[3] * b[1] + a[6] * b[2],
        a[1] * b[0] + a[4] * b[1] + a[7] * b[2],
        a[2] * b[0] + a[5] * b[1] + a[8] * b[2]};
}

double Determinant(const Mat3& a) {
    return a[0] * (a[4] * a[8] - a[5] * a[7]) - a[1] * (a[3] * a[8] - a[5] * a[6]) +
           a[2] * (a[3] * a[7] - a[4] * a[6]);
}

bool Inverse(const Mat3& a, Mat3& inverse) {
    if (!Finite(a))
        return false;
    double scale = 0;
    for (double x : a)
        scale = std::max(scale, std::abs(x));
    if (!scale)
        return false;
    Mat3 b;
    for (int i = 0; i < 9; ++i)
        b[i] = a[i] / scale;
    Mat3 adj{b[4] * b[8] - b[5] * b[7],
        b[2] * b[7] - b[1] * b[8],
        b[1] * b[5] - b[2] * b[4],
        b[5] * b[6] - b[3] * b[8],
        b[0] * b[8] - b[2] * b[6],
        b[2] * b[3] - b[0] * b[5],
        b[3] * b[7] - b[4] * b[6],
        b[1] * b[6] - b[0] * b[7],
        b[0] * b[4] - b[1] * b[3]};
    const double det = b[0] * adj[0] + b[1] * adj[3] + b[2] * adj[6];
    if (!std::isfinite(det) || std::abs(det) <= 32 * std::numeric_limits<double>::epsilon())
        return false;
    for (int i = 0; i < 9; ++i)
        inverse[i] = (adj[i] / det) / scale;
    return Finite(inverse);
}

HarmonicSeries EvaluateCosineSeries(const double* coefficients, std::size_t count, double phase) {
    const double c = std::cos(phase), s = std::sin(phase);
    double ck = c, sk = s;
    HarmonicSeries result;
    for (std::size_t k = 0; k < count; ++k) {
        result.value += coefficients[k] * ck;
        result.derivative -= (k + 1) * coefficients[k] * sk;
        const double next = ck * c - sk * s;
        sk = sk * c + ck * s;
        ck = next;
    }
    return result;
}

Spectrum SingularValues(const Mat3& j) {
    Spectrum result;
    if (!Finite(j))
        return result;
    double scale = 0;
    for (double x : j)
        scale = std::max(scale, std::abs(x));
    if (!scale) {
        result.valid = true;
        return result;
    }
    Mat3 a{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            for (int n = 0; n < 3; ++n)
                a[3 * i + k] += (j[3 * i + n] / scale) * (j[3 * k + n] / scale);
    // Only eigenvalues are needed; no eigenvector/quaternion construction.
    for (int sweep = 0; sweep < 16; ++sweep) {
        int p = 0, q = 1;
        if (std::abs(a[2]) > std::abs(a[1]))
            q = 2;
        if (std::abs(a[5]) > std::abs(a[3 * p + q])) {
            p = 1;
            q = 2;
        }
        if (std::abs(a[3 * p + q]) <= 4 * std::numeric_limits<double>::epsilon())
            break;
        const double tau = (a[3 * q + q] - a[3 * p + p]) / (2 * a[3 * p + q]);
        const double t = std::copysign(1.0, tau) / (std::abs(tau) + std::hypot(1.0, tau));
        const double c = 1 / std::hypot(1.0, t), s = t * c;
        const double pp = a[3 * p + p], qq = a[3 * q + q], pq = a[3 * p + q];
        for (int k = 0; k < 3; ++k)
            if (k != p && k != q) {
                const double kp = a[3 * k + p], kq = a[3 * k + q];
                a[3 * k + p] = a[3 * p + k] = c * kp - s * kq;
                a[3 * k + q] = a[3 * q + k] = s * kp + c * kq;
            }
        a[3 * p + p] = pp - t * pq;
        a[3 * q + q] = qq + t * pq;
        a[3 * p + q] = a[3 * q + p] = 0;
    }
    result.minimum = scale * std::sqrt(std::max(0.0, std::min({a[0], a[4], a[8]})));
    result.maximum = scale * std::sqrt(std::max(0.0, std::max({a[0], a[4], a[8]})));
    if (result.minimum > 0)
        result.condition = result.maximum / result.minimum;
    result.valid = std::isfinite(result.minimum) && std::isfinite(result.maximum);
    return result;
}

bool JacobianSolver::Factor(const Mat3& j, const DampingPolicy& p) {
    method_ = SolveMethod::Invalid;
    damping_ = 0;
    spectrum_ = SingularValues(j);
    if (!spectrum_.valid || !std::isfinite(p.threshold) || p.threshold <= 0 ||
        !std::isfinite(p.minimum) || p.minimum <= 0 || !std::isfinite(p.maximum) ||
        p.maximum < p.minimum || !std::isfinite(p.direct_condition_limit) ||
        p.direct_condition_limit < 1)
        return false;
    jacobian_ = j;
    if (spectrum_.minimum >= p.threshold && spectrum_.condition <= p.direct_condition_limit) {
        if (!Inverse(j, inverse_))
            return false;
        method_ = SolveMethod::DirectInverse;
        return true;
    }
    const double threshold = std::max(p.threshold, spectrum_.maximum / p.direct_condition_limit);
    const double ratio = std::min(1.0, spectrum_.minimum / threshold), weight = 1 - ratio * ratio;
    const double damping2 =
        p.minimum * p.minimum + (p.maximum * p.maximum - p.minimum * p.minimum) * weight * weight;
    damping_ = std::sqrt(damping2);
    Mat3 gram{};
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k < 3; ++k)
            for (int n = 0; n < 3; ++n)
                gram[3 * i + k] += j[3 * i + n] * j[3 * k + n];
    for (int i = 0; i < 3; ++i)
        gram[3 * i + i] += damping2;
    cholesky_.fill(0);
    for (int i = 0; i < 3; ++i)
        for (int k = 0; k <= i; ++k) {
            double value = gram[3 * i + k];
            for (int n = 0; n < k; ++n)
                value -= cholesky_[3 * i + n] * cholesky_[3 * k + n];
            if (i == k) {
                if (!(value > 0) || !std::isfinite(value))
                    return false;
                cholesky_[3 * i + k] = std::sqrt(value);
            } else
                cholesky_[3 * i + k] = value / cholesky_[3 * k + k];
        }
    method_ = SolveMethod::AdaptiveDls;
    return true;
}

bool JacobianSolver::SolvePositiveDefinite(const Vec3& rhs, Vec3& out) const {
    out = rhs;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < i; ++j)
            out[i] -= cholesky_[3 * i + j] * out[j];
        out[i] /= cholesky_[3 * i + i];
    }
    for (int i = 2; i >= 0; --i) {
        for (int j = i + 1; j < 3; ++j)
            out[i] -= cholesky_[3 * j + i] * out[j];
        out[i] /= cholesky_[3 * i + i];
    }
    return Finite(out);
}

bool JacobianSolver::Solve(const Vec3& rhs, Vec3& out) const {
    if (!Finite(rhs) || method_ == SolveMethod::Invalid)
        return false;
    if (method_ == SolveMethod::DirectInverse)
        out = Multiply(inverse_, rhs);
    else {
        Vec3 solved;
        if (!SolvePositiveDefinite(rhs, solved))
            return false;
        out = TransposeMultiply(jacobian_, solved);
    }
    return Finite(out);
}

bool JacobianSolver::SolveTranspose(const Vec3& rhs, Vec3& out) const {
    if (!Finite(rhs) || method_ == SolveMethod::Invalid)
        return false;
    if (method_ == SolveMethod::DirectInverse)
        out = TransposeMultiply(inverse_, rhs);
    else
        return SolvePositiveDefinite(Multiply(jacobian_, rhs), out);
    return Finite(out);
}

double TriangleDistanceSquared(const Vec3& p, const Mat3& t) {
    const Vec3 a{t[0], t[1], t[2]}, b{t[3], t[4], t[5]}, c{t[6], t[7], t[8]};
    const auto u = Subtract(b, a), v = Subtract(c, a), w = Subtract(p, a);
    const double uu = Dot(u, u), uv = Dot(u, v), vv = Dot(v, v), denominator = uu * vv - uv * uv;
    if (denominator > 1e-24) {
        const double s = (vv * Dot(w, u) - uv * Dot(w, v)) / denominator,
                     r = (uu * Dot(w, v) - uv * Dot(w, u)) / denominator;
        if (s >= 0 && r >= 0 && s + r <= 1) {
            const Vec3 e{
                w[0] - s * u[0] - r * v[0], w[1] - s * u[1] - r * v[1], w[2] - s * u[2] - r * v[2]};
            return Dot(e, e);
        }
    }
    double best = std::numeric_limits<double>::infinity();
    for (const auto& edge :
        {std::array<Vec3, 2>{a, b}, std::array<Vec3, 2>{b, c}, std::array<Vec3, 2>{c, a}}) {
        const auto d = Subtract(edge[1], edge[0]);
        const double f =
            std::clamp(Dot(Subtract(p, edge[0]), d) / std::max(Dot(d, d), 1e-24), 0., 1.);
        const auto delta = Subtract(p, Lerp(edge[0], edge[1], f));
        best = std::min(best, Dot(delta, delta));
    }
    return best;
}
}  // namespace qhrr05::numeric
