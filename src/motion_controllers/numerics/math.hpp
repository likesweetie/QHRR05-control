#pragma once
#include <array>
#include <cstddef>
#include <limits>

namespace qhrr05::numeric {
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;  // row-major
Vec3 Subtract(const Vec3& a, const Vec3& b);
Vec3 Lerp(const Vec3& a, const Vec3& b, double t);
double Dot(const Vec3& a, const Vec3& b);
double Norm(const Vec3& a);
double Distance(const Vec3& a, const Vec3& b);
double TriangleDistanceSquared(const Vec3& p, const Mat3& triangle);
Vec3 Multiply(const Mat3& a, const Vec3& b);
Vec3 TransposeMultiply(const Mat3& a, const Vec3& b);
double Determinant(const Mat3& a);
bool Inverse(const Mat3& a, Mat3& inverse);

// Sum c[k]*cos((k+1)*phase), and its derivative, using one sin/cos pair.
struct HarmonicSeries {
    double value = 0, derivative = 0;
};

HarmonicSeries EvaluateCosineSeries(const double* coefficients, std::size_t count, double phase);

struct Spectrum {
    double minimum = 0, maximum = 0;
    double condition = std::numeric_limits<double>::infinity();
    bool valid = false;
};

Spectrum SingularValues(const Mat3& jacobian);

struct DampingPolicy {
    double threshold = .02, minimum = .001, maximum = .03, direct_condition_limit = 10;
};
enum class SolveMethod { Invalid, DirectInverse, AdaptiveDls };

// Factor once, reuse for position/velocity/acceleration or the transpose force map.
// DLS is a deliberate, reported numerical path, never a fallback from failed inversion.
class JacobianSolver {
public:
    bool Factor(const Mat3& jacobian, const DampingPolicy& policy);
    bool Solve(const Vec3& cartesian, Vec3& joint) const;
    bool SolveTranspose(const Vec3& joint_torque, Vec3& force) const;

    const Spectrum& spectrum() const {
        return spectrum_;
    }

    SolveMethod method() const {
        return method_;
    }

    double damping() const {
        return damping_;
    }

private:
    bool SolvePositiveDefinite(const Vec3& rhs, Vec3& result) const;
    Mat3 jacobian_{}, inverse_{}, cholesky_{};
    Spectrum spectrum_{};
    SolveMethod method_ = SolveMethod::Invalid;
    double damping_ = 0;
};
}  // namespace qhrr05::numeric
