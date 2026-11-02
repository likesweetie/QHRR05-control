#include "motion_controllers/ball_following_controller.hpp"
#include "control/control.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <queue>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace qhrr05 {
namespace {
using Vec = numeric::Vec3;
using numeric::Distance;
using numeric::Dot;
using numeric::Lerp;
using numeric::TriangleDistanceSquared;
const auto Sub = numeric::Subtract;
Vec Cross(const Vec& a, const Vec& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]};
}
}

BallFollowingController::BallFollowingController(
    const std::string& config_path, const std::atomic<bool>* cancellation)
    : BallFollowingController(config_path, nullptr, cancellation, 0) {
}

BallFollowingController::BallFollowingController(
    const std::string& config_path, ModelBackend& backend, const std::atomic<bool>* cancellation)
    : BallFollowingController(config_path, &backend, cancellation, 0) {
}

BallFollowingController::BallFollowingController(const std::string& config_path,
    ModelBackend* backend,
    const std::atomic<bool>* cancellation,
    int)
    : backend_(backend), cancellation_(cancellation) {
    const auto config = YAML::LoadFile(config_path);
    const bool torque_ref_as_position = ReadTorqueRefAsPosition(config);
    const auto s = config["ball_following"];
    enabled_ = s["enabled"].as<bool>();
    const auto scalar = [&](const char* key) {
        const double value = s[key].as<double>();

        if (!std::isfinite(value) || value <= 0)
            throw std::runtime_error(std::string("invalid ball following parameter: ") + key);

        return value;
    };
    const auto values = [&](const char* key) {
        const auto v = s[key].as<std::vector<double>>();

        if (v.size() != 3)
            throw std::runtime_error(std::string("expected three ball following values: ") + key);

        Vec result{v[0], v[1], v[2]};

        for (double value : result)
            if (!std::isfinite(value) || value < 0)
                throw std::runtime_error("invalid ball following joint values");

        return result;
    };

    margin_ = scalar("workspace_margin_m");
    padding_ = scalar("workspace_sampling_padding_m");
    if (s["minimum_singular_value_m"].IsDefined())
        throw std::runtime_error(
            "ball_following.minimum_singular_value_m was replaced by maximum_condition_number");

    if (s["maximum_condition_number"].IsDefined())
        throw std::runtime_error(
            "maximum_condition_number belongs under workspace, not ball_following");

    condition_limit_ = config["workspace"]["maximum_condition_number"].as<double>();

    if (!std::isfinite(condition_limit_) || condition_limit_ < 1)
        throw std::runtime_error(
            "workspace.maximum_condition_number must be finite and at least 1");
    foot_speed_ = scalar("foot_speed_m_s");
    smoothing_sec_ = s["smoothing_sec"] ? scalar("smoothing_sec") : .10;
    slow_speed_ = scalar("ball_slow_speed_m_s");
    normal_speed_ = scalar("ball_normal_speed_m_s");
    fast_speed_ = scalar("ball_fast_speed_m_s");
    trigger_threshold_ = scalar("trigger_threshold");
    grid_ = s["workspace_grid_points"].as<int>();
    kp_ = values("kp_motor");
    kd_ = values("kd_motor");
    torque_limit_ = ReadControlTorqueLimits(config);

    if (grid_ < 9 || grid_ > 65 || trigger_threshold_ > 1 || slow_speed_ >= normal_speed_ ||
        normal_speed_ >= fast_speed_)
        throw std::runtime_error("invalid ball following grid, trigger or speed order");

    for (int i = 0; i < 3; ++i) {
        if (kp_[i] < 0 || kp_[i] > 500 || kd_[i] > 5 || torque_limit_[i] <= 0)
            throw std::runtime_error("invalid ball following MIT gains/torque limit");
        if (torque_ref_as_position && kp_[i] == 0)
            throw std::runtime_error(
                "ball_following.kp_motor must be positive when torque_reference_mode=torque_ref_as_position");
    }

    if (!enabled_)
        return;  // Explicitly unavailable mode, never an automatic control fallback.

    if (!backend_) {
        owned_backend_ = MakeMujocoBackend(config_path);
        backend_ = owned_backend_.get();
    }

    const auto home = config["homing"]["position_rad"].as<std::vector<double>>();

    if (home.size() != 3)
        throw std::runtime_error("ball FK branch needs three homing angles");

    BuildWorkspace({home[0], home[1], home[2]});
    BuildInterpolation();
}

std::array<double, 3> BallFollowingController::Forward(const Vec& q, bool* regular) {
    if (cancellation_ && cancellation_->load(std::memory_order_relaxed))
        throw std::runtime_error("workspace generation cancelled");

    const auto kinematics = backend_->Evaluate(q, regular != nullptr);
    if (regular) {
        *regular = numeric::Determinant(kinematics.jacobian) * branch_sign_ > 0 &&
                   numeric::SingularValues(kinematics.jacobian).condition <= condition_limit_;
    }
    return kinematics.position;
}

int BallFollowingController::Neighbor(int node, int axis, int sign) const {
    const int strides[] = {grid_ * grid_, grid_, 1};
    const int coordinate = node / strides[axis] % grid_;

    if (coordinate + sign < 0 || coordinate + sign >= grid_)
        return -1;

    return node + sign * strides[axis];
}

bool BallFollowingController::ClearOfBoundary(const Vec& p) const {
    const std::array<int, 3> bin{int(std::floor(p[0] / margin_)),
        int(std::floor(p[1] / margin_)),
        int(std::floor(p[2] / margin_))};
    const auto found = boundary_bins_.find(bin);

    if (found == boundary_bins_.end())
        return true;  // No boundary triangle lies within the expanded spatial bin.

    const double clearance = margin_ + padding_;

    for (int triangle : found->second)
        if (TriangleDistanceSquared(p, boundary_[triangle]) < clearance * clearance)
            return false;

    return true;
}

bool BallFollowingController::SafeEdge(int from, int to) {
    const double length = Distance(positions_[from], positions_[to]);
    const int steps = std::max(2, int(std::ceil(length / (padding_ * .5))));

    for (int step = 1; step < steps; ++step) {
        bool regular;
        const auto p = Forward(Lerp(joints_[from], joints_[to], double(step) / steps), &regular);

        if (!regular || !ClearOfBoundary(p))
            return false;
    }

    return true;
}

bool BallFollowingController::SafePose(const Vec& q, bool entry) {
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(q[i]) || q[i] < backend_->state().joint_min[i] ||
            q[i] > backend_->state().joint_max[i]) return false;
    bool regular;
    const auto p = Forward(q, &regular);
    return regular && (entry || ClearOfBoundary(p));
}

bool BallFollowingController::SafeSegment(const Vec& from, const Vec& to, bool entry) {
    const double distance = Distance(Forward(from), Forward(to));
    const int steps = std::max(1, int(std::ceil(std::max(distance / (padding_ * .5),
        Distance(from, to) / .02))));
    for (int i = 1; i <= steps; ++i)
        if (!SafePose(Lerp(from, to, double(i) / steps), entry)) return false;
    return true;
}

void BallFollowingController::BuildWorkspace(const Vec& home) {
    // Select the same nonsingular kinematic branch as the configured homing pose.
    branch_sign_ = numeric::Determinant(backend_->Evaluate(home).jacobian);
    const int count = grid_ * grid_ * grid_, cells = grid_ - 1;
    joints_.resize(count);
    positions_.resize(count);
    valid_.resize(count);
    edges_.resize(count);
    plan_distance_.resize(count);
    plan_parent_.resize(count);
    plan_open_.reserve(6 * count);
    path_.reserve(count);

    for (int node = 0; node < count; ++node) {
        const int coordinate[] = {node / (grid_ * grid_), node / grid_ % grid_, node % grid_};

        for (int axis = 0; axis < 3; ++axis)
            joints_[node][axis] =
                backend_->state().joint_min[axis] +
                double(coordinate[axis]) / cells *
                    (backend_->state().joint_max[axis] - backend_->state().joint_min[axis]);

        bool regular;
        positions_[node] = Forward(joints_[node], &regular);
        valid_[node] = regular;
    }

    const auto vertex = [&](int x, int y, int z) {
        return x * grid_ * grid_ + y * grid_ + z;
    };
    const auto cell = [&](int x, int y, int z) {
        return x * cells * cells + y * cells + z;
    };
    std::vector<unsigned char> volume(cells * cells * cells), incident(count);

    for (int x = 0; x < cells; ++x)
        for (int y = 0; y < cells; ++y)
            for (int z = 0; z < cells; ++z) {
                bool valid = true;

                for (int dx = 0; dx < 2; ++dx)
                    for (int dy = 0; dy < 2; ++dy)
                        for (int dz = 0; dz < 2; ++dz)
                            valid &= valid_[vertex(x + dx, y + dy, z + dz)];

                volume[cell(x, y, z)] = valid;
            }

    // Rough FK cell boundary, including joint limits and poorly conditioned poses.
    for (int x = 0; x < cells; ++x)
        for (int y = 0; y < cells; ++y)
            for (int z = 0; z < cells; ++z) {
                if (!volume[cell(x, y, z)])
                    continue;

                for (int dx = 0; dx < 2; ++dx)
                    for (int dy = 0; dy < 2; ++dy)
                        for (int dz = 0; dz < 2; ++dz)
                            incident[vertex(x + dx, y + dy, z + dz)] = 1;

                for (int axis = 0; axis < 3; ++axis)
                    for (int side = 0; side < 2; ++side) {
                        int neighbor[] = {x, y, z};
                        neighbor[axis] += side ? 1 : -1;

                        if (neighbor[axis] >= 0 && neighbor[axis] < cells &&
                            volume[cell(neighbor[0], neighbor[1], neighbor[2])])
                            continue;

                        Vec corners[4];
                        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                        const int offsets[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

                        for (int k = 0; k < 4; ++k) {
                            int coordinate[] = {x, y, z};
                            coordinate[axis] += side;
                            coordinate[u] += offsets[k][0];
                            coordinate[v] += offsets[k][1];
                            corners[k] =
                                positions_[vertex(coordinate[0], coordinate[1], coordinate[2])];
                        }

                        for (int k : {1, 2}) {
                            std::array<double, 9> triangle;
                            std::copy(corners[0].begin(), corners[0].end(), triangle.begin());
                            std::copy(corners[k].begin(), corners[k].end(), triangle.begin() + 3);
                            std::copy(
                                corners[k + 1].begin(), corners[k + 1].end(), triangle.begin() + 6);
                            boundary_.push_back(triangle);
                        }
                    }
            }

    const double clearance = margin_ + padding_;

    for (std::size_t index = 0; index < boundary_.size(); ++index) {
        const auto& t = boundary_[index];
        int lo[3], hi[3];

        for (int axis = 0; axis < 3; ++axis) {
            lo[axis] = int(
                std::floor((std::min({t[axis], t[3 + axis], t[6 + axis]}) - clearance) / margin_));
            hi[axis] = int(
                std::floor((std::max({t[axis], t[3 + axis], t[6 + axis]}) + clearance) / margin_));
        }

        for (int x = lo[0]; x <= hi[0]; ++x)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int z = lo[2]; z <= hi[2]; ++z)
                    boundary_bins_[{x, y, z}].push_back(int(index));
    }

    int seed = -1;
    double best = std::numeric_limits<double>::infinity();

    for (int node = 0; node < count; ++node) {
        valid_[node] = incident[node] && ClearOfBoundary(positions_[node]);

        if (valid_[node] && Distance(joints_[node], home) < best) {
            seed = node;
            best = Distance(joints_[node], home);
        }
    }

    if (seed < 0)
        throw std::runtime_error(
            "ball FK workspace is empty after condition-number filtering and clearance erosion");

    // Build only the connected safe component. Every interpolated edge is checked in FK.
    std::queue<int> pending;
    std::vector<unsigned char> visited(count);
    pending.push(seed);
    visited[seed] = 1;

    while (!pending.empty()) {
        const int from = pending.front();
        pending.pop();
        valid_indices_.push_back(from);

        for (int axis = 0; axis < 3; ++axis)
            for (int sign : {-1, 1}) {
                const int to = Neighbor(from, axis, sign);

                if (to < 0 || !valid_[to])
                    continue;

                const int bit = 2 * axis + (sign > 0);

                if (!(edges_[from] & (1 << bit)) && !SafeEdge(from, to))
                    continue;

                edges_[from] |= 1 << bit;
                edges_[to] |= 1 << (2 * axis + (sign < 0));

                if (!visited[to]) {
                    visited[to] = 1;
                    pending.push(to);
                }
            }
    }

    valid_ = std::move(visited);
    goal_ = seed;
}

void BallFollowingController::BuildInterpolation() {
    constexpr int orders[6][3]{{0, 1, 2}, {0, 2, 1}, {1, 0, 2},
        {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    const int stride[3]{grid_ * grid_, grid_, 1};
    const double bin_size = 2 * margin_;
    for (int x = 0; x + 1 < grid_; ++x)
        for (int y = 0; y + 1 < grid_; ++y)
            for (int z = 0; z + 1 < grid_; ++z) {
                const int origin = x * stride[0] + y * stride[1] + z;
                bool connected = true;
                for (int corner = 0; corner < 8; ++corner) {
                    int node = origin;
                    for (int axis = 0; axis < 3; ++axis)
                        if (corner & (1 << axis)) node += stride[axis];
                    connected &= bool(valid_[node]);
                    for (int axis = 0; axis < 3; ++axis)
                        if (!(corner & (1 << axis)))
                            connected &= bool(edges_[node] & (1 << (2 * axis + 1)));
                }
                if (!connected) continue;
                for (const auto& order : orders) {
                    const std::array<int, 4> nodes{origin, origin + stride[order[0]],
                        origin + stride[order[0]] + stride[order[1]],
                        origin + stride[0] + stride[1] + stride[2]};
                    const auto& p = positions_[nodes[0]];
                    const auto a = Sub(positions_[nodes[1]], p);
                    const auto b = Sub(positions_[nodes[2]], p);
                    const auto c = Sub(positions_[nodes[3]], p);
                    const double determinant = Dot(a, Cross(b, c));
                    const double joint_determinant = Dot(Sub(joints_[nodes[1]], joints_[nodes[0]]),
                        Cross(Sub(joints_[nodes[2]], joints_[nodes[0]]),
                            Sub(joints_[nodes[3]], joints_[nodes[0]])));
                    if (std::abs(determinant) < 1e-15 ||
                        determinant * joint_determinant * branch_sign_ <= 0) continue;
                    bool safe = true;
                    // Check the interior and face/edge midpoints in actual FK as
                    // well as the prevalidated grid vertices and axis edges.
                    Vec center{};
                    for (int node : nodes)
                        for (int axis = 0; axis < 3; ++axis) center[axis] += .25 * joints_[node][axis];
                    safe &= SafePose(center);
                    for (int excluded = 0; excluded < 4 && safe; ++excluded) {
                        Vec face{};
                        for (int j = 0; j < 4; ++j) if (j != excluded)
                            for (int axis = 0; axis < 3; ++axis) face[axis] += joints_[nodes[j]][axis] / 3;
                        safe &= SafePose(face);
                    }
                    for (int i = 0; i < 4 && safe; ++i)
                        for (int j = i + 1; j < 4 && safe; ++j)
                            safe &= SafePose(Lerp(joints_[nodes[i]], joints_[nodes[j]], .5));
                    if (!safe) continue;
                    std::array<Vec, 4> basis{p, Cross(b, c), Cross(c, a), Cross(a, b)};
                    for (int row = 1; row < 4; ++row)
                        for (double& value : basis[row]) value /= determinant;
                    const int index = int(interpolation_nodes_.size());
                    interpolation_nodes_.push_back(nodes);
                    interpolation_basis_.push_back(basis);
                    int lo[3], hi[3];
                    for (int axis = 0; axis < 3; ++axis) {
                        double minimum = p[axis], maximum = p[axis];
                        for (int node : nodes) {
                            minimum = std::min(minimum, positions_[node][axis]);
                            maximum = std::max(maximum, positions_[node][axis]);
                        }
                        lo[axis] = int(std::floor(minimum / bin_size));
                        hi[axis] = int(std::floor(maximum / bin_size));
                    }
                    for (int i = lo[0]; i <= hi[0]; ++i)
                        for (int j = lo[1]; j <= hi[1]; ++j)
                            for (int k = lo[2]; k <= hi[2]; ++k)
                                interpolation_bins_[{i, j, k}].push_back(index);
                }
            }
}

bool BallFollowingController::Interpolate(const Vec& position, Vec& q, int& anchor) {
    const auto sample = [&](int index) {
        if (index < 0) return false;
        const auto& basis = interpolation_basis_[index];
        const auto delta = Sub(position, basis[0]);
        std::array<double, 4> weights{0, Dot(basis[1], delta), Dot(basis[2], delta), Dot(basis[3], delta)};
        weights[0] = 1 - weights[1] - weights[2] - weights[3];
        for (double value : weights) if (value < -1e-10 || value > 1 + 1e-10) return false;
        double sum = 0;
        for (double& value : weights) { value = std::clamp(value, 0.0, 1.0); sum += value; }
        Vec candidate{};
        for (int i = 0; i < 4; ++i)
            for (int axis = 0; axis < 3; ++axis)
                candidate[axis] += weights[i] / sum * joints_[interpolation_nodes_[index][i]][axis];
        if (!SafePose(candidate)) return false;
        q = candidate;
        anchor = interpolation_nodes_[index][std::max_element(weights.begin(), weights.end()) - weights.begin()];
        interpolation_cell_ = index;
        return true;
    };
    if (sample(interpolation_cell_)) return true;
    for (double value : position) if (!std::isfinite(value) || std::abs(value) > 1000) return false;
    const double bin_size = 2 * margin_;
    const auto found = interpolation_bins_.find({int(std::floor(position[0] / bin_size)),
        int(std::floor(position[1] / bin_size)), int(std::floor(position[2] / bin_size))});
    if (found != interpolation_bins_.end())
        for (int index : found->second) if (sample(index)) return true;
    return false;
}

std::vector<std::array<double, 9>> BallFollowingController::WorkspaceSurface() const {
    if (!enabled_ || valid_indices_.empty())
        throw std::runtime_error(
            "workspace overlay requires enabled ball_following with a valid FK workspace");

    const int cells = grid_ - 1;
    const auto vertex = [&](int x, int y, int z) {
        return x * grid_ * grid_ + y * grid_ + z;
    };
    const auto cell = [&](int x, int y, int z) {
        return x * cells * cells + y * cells + z;
    };
    std::vector<unsigned char> volume(cells * cells * cells);

    // Show only complete cells of the eroded, connected FK workspace. A convex
    // hull would incorrectly fill unreachable cavities and singularity exclusions.
    for (int x = 0; x < cells; ++x)
        for (int y = 0; y < cells; ++y)
            for (int z = 0; z < cells; ++z) {
                bool valid = true;

                for (int dx = 0; dx < 2; ++dx)
                    for (int dy = 0; dy < 2; ++dy)
                        for (int dz = 0; dz < 2; ++dz) {
                            const int node = vertex(x + dx, y + dy, z + dz);
                            const int offset[] = {dx, dy, dz};
                            valid &= valid_[node];

                            for (int axis = 0; axis < 3; ++axis)
                                if (!offset[axis])
                                    valid &= bool(edges_[node] & (1 << (2 * axis + 1)));
                        }

                volume[cell(x, y, z)] = valid;
            }

    std::vector<std::array<double, 9>> surface;

    for (int x = 0; x < cells; ++x)
        for (int y = 0; y < cells; ++y)
            for (int z = 0; z < cells; ++z) {
                if (!volume[cell(x, y, z)])
                    continue;

                for (int axis = 0; axis < 3; ++axis)
                    for (int side = 0; side < 2; ++side) {
                        int neighbor[] = {x, y, z};
                        neighbor[axis] += side ? 1 : -1;

                        if (neighbor[axis] >= 0 && neighbor[axis] < cells &&
                            volume[cell(neighbor[0], neighbor[1], neighbor[2])])
                            continue;

                        Vec corners[4];
                        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
                        const int offsets[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};

                        for (int k = 0; k < 4; ++k) {
                            int coordinate[] = {x, y, z};
                            coordinate[axis] += side;
                            coordinate[u] += offsets[k][0];
                            coordinate[v] += offsets[k][1];
                            corners[k] =
                                positions_[vertex(coordinate[0], coordinate[1], coordinate[2])];
                        }

                        for (int k : {1, 2}) {
                            const bool reverse = (side ? 1 : -1) * branch_sign_ < 0;
                            const auto& a = corners[0];
                            const auto& b = corners[reverse ? k + 1 : k];
                            const auto& c = corners[reverse ? k : k + 1];
                            surface.push_back(
                                {a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]});
                        }
                    }
            }

    if (surface.empty())
        throw std::runtime_error(
            "FK workspace has no complete volume cells to display; increase workspace_grid_points");

    return surface;
}

int BallFollowingController::Nearest(const Vec& position) const {
    int result = -1;
    double best = std::numeric_limits<double>::infinity();

    for (int node : valid_indices_) {
        const auto difference = Sub(positions_[node], position);
        const double distance = Dot(difference, difference);

        if (result < 0 || distance < best) {
            best = distance;
            result = node;
        }
    }

    return result;
}

void BallFollowingController::Plan(int from, int to) {
    path_.clear();
    path_next_ = 0;
    check_route_ = true;
    auto& distance = plan_distance_;
    auto& parent = plan_parent_;
    auto& open = plan_open_;
    std::fill(distance.begin(), distance.end(), std::numeric_limits<double>::infinity());
    std::fill(parent.begin(), parent.end(), -1);
    open.clear();
    const auto push = [&](std::pair<double, int> entry) {
        open.push_back(entry);
        std::push_heap(open.begin(), open.end(), std::greater<std::pair<double, int>>{});
    };
    distance[from] = 0;
    push({Distance(positions_[from], positions_[to]), from});

    while (!open.empty()) {
        std::pop_heap(open.begin(), open.end(), std::greater<std::pair<double, int>>{});
        const auto [cost, node] = open.back();
        open.pop_back();

        if (node == to)
            break;

        if (cost > distance[node] + Distance(positions_[node], positions_[to]) + 1e-10)
            continue;

        for (int axis = 0; axis < 3; ++axis)
            for (int sign : {-1, 1}) {
                if (!(edges_[node] & (1 << (2 * axis + (sign > 0)))))
                    continue;

                const int next = Neighbor(node, axis, sign);
                const double next_cost =
                    distance[node] + Distance(positions_[node], positions_[next]);

                if (next_cost < distance[next]) {
                    distance[next] = next_cost;
                    parent[next] = node;
                    push({next_cost + Distance(positions_[next], positions_[to]), next});
                }
            }
    }

    if (from != to && parent[to] < 0)
        throw std::runtime_error("ball FK workspace graph has no connected path");

    for (int node = to; node != from; node = parent[node])
        path_.push_back(node);

    std::reverse(path_.begin(), path_.end());
}

void BallFollowingController::PlanTarget() {
    planned_target_ = target_position_;
    replan_ = false;
    direct_path_ = SafeSegment(reference_, target_joint_, entering_);
    if (direct_path_) { path_.clear(); return; }
    int anchor = Nearest(toe_reference_);
    if (!SafeSegment(reference_, joints_[anchor], entering_)) {
        anchor = -1;
        double best = std::numeric_limits<double>::infinity();
        for (int node : valid_indices_) {
            const double distance = Distance(reference_, joints_[node]);
            if (distance < best && SafeSegment(reference_, joints_[node], entering_)) {
                anchor = node;
                best = distance;
            }
        }
    }
    if (anchor < 0)
        throw std::runtime_error("ball cannot connect the entry pose to its validated FK workspace");
    Plan(anchor, goal_);
    path_.insert(path_.begin(), anchor);
}

void BallFollowingController::Start(const Vec& joint_position) {
    if (!enabled_)
        throw std::runtime_error("ball following is disabled in control configuration");
    bool regular;
    ball_ = toe_reference_ = Forward(joint_position, &regular);
    if (!regular)
        throw std::runtime_error(
            "ball following requires the nonsingular homing branch; select H first");
    reference_ = target_joint_ = previous_target_joint_ = joint_position;
    target_position_ = previous_ball_ = ball_;
    reference_velocity_.fill(0);
    velocity_stage_.fill(0);
    filtered_velocity_.fill(0);
    previous_interpolated_ = false;
    interpolation_cell_ = -1;
    goal_ = Nearest(ball_);
    path_.clear();
    entering_ = !SafePose(reference_);
    if (entering_) {
        target_joint_ = joints_[goal_];
        target_position_ = positions_[goal_];
    }
    limited_ = false;
    replan_ = true;
}

void BallFollowingController::SetBallPosition(const Vec& position) {
    for (double value : position)
        if (!std::isfinite(value))
            throw std::runtime_error("invalid ball position");

    ball_ = position;
}

void BallFollowingController::Compute(double dt,
    const Vec& direction,
    double l2,
    double r2,
    bool connected,
    const Vec& joint_position,
    const Vec& joint_velocity,
    const Vec& gravity,
    std::array<double, 3>& position_reference,
    std::array<double, 3>& torque) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(l2) || !std::isfinite(r2))
        throw std::runtime_error("invalid ball following dt or triggers");
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(joint_position[i]) || !std::isfinite(joint_velocity[i]))
            throw std::runtime_error("invalid ball joint feedback");

    Vec input = direction;

    for (double value : input)
        if (!std::isfinite(value))
            throw std::runtime_error("invalid ball joystick direction");

    const double magnitude = std::sqrt(Dot(input, input));
    ball_speed_ = l2 > trigger_threshold_   ? slow_speed_
                  : r2 > trigger_threshold_ ? fast_speed_
                                            : normal_speed_;

    if (connected)
        for (int axis = 0; axis < 3; ++axis)
            ball_[axis] += dt * ball_speed_ * input[axis] / std::max(1.0, magnitude);

    const bool interpolated = !entering_ && Interpolate(ball_, target_joint_, goal_);
    limited_ = !interpolated;
    if (!interpolated && !entering_) {
        goal_ = Nearest(ball_);
        target_joint_ = joints_[goal_];
    }
    target_position_ = Forward(target_joint_);
    if (replan_ || Distance(target_position_, planned_target_) > padding_ ||
        (!direct_path_ && path_next_ >= path_.size()))
        PlanTarget();

    Vec waypoint = target_joint_;
    bool routing = false;
    if (!direct_path_) {
        const auto previous = path_next_;
        while (path_next_ < path_.size() &&
            Distance(toe_reference_, positions_[path_[path_next_]]) < padding_)
            ++path_next_;
        if (path_next_ < path_.size()) {
            // Look ahead only along FK-checked shortcuts of the existing graph.
            // Intermediate grid vertices are guides, not mandatory stops.
            if (check_route_ || previous != path_next_ ||
                Distance(toe_reference_, route_check_position_) > padding_) {
                const auto end = std::min(path_.size(), path_next_ + 8);
                for (std::size_t next = end; --next > path_next_;)
                    if (SafeSegment(reference_, joints_[path_[next]], entering_)) {
                        path_next_ = next;
                        break;
                    }
                route_check_position_ = toe_reference_;
                check_route_ = false;
            }
            waypoint = joints_[path_[path_next_]];
            routing = path_next_ + 1 < path_.size();
        }
    }
    Vec desired_velocity = Sub(waypoint, reference_);
    for (double& value : desired_velocity) value /= 8 * smoothing_sec_;
    // This is the difference of consecutive FK-table interpolants, not a
    // Jacobian inversion. It only moves the position trajectory; MIT v_des stays 0.
    if (direct_path_ && interpolated && previous_interpolated_ &&
        Distance(ball_, previous_ball_) <= 2 * fast_speed_ * dt)
        for (int i = 0; i < 3; ++i)
            desired_velocity[i] += (target_joint_[i] - previous_target_joint_[i]) / dt;
    previous_target_joint_ = target_joint_;
    previous_ball_ = ball_;
    previous_interpolated_ = interpolated;

    Vec preview = reference_;
    for (int i = 0; i < 3; ++i) preview[i] += dt * desired_velocity[i];
    const double speed = Distance(Forward(preview), toe_reference_) / dt;
    for (double& value : desired_velocity)
        value *= routing ? .98 * foot_speed_ / std::max(speed, 1e-12)
                         : std::min(1.0, .98 * foot_speed_ / std::max(speed, 1e-12));

    const auto old_reference = reference_;
    const double decay = std::exp(-dt / smoothing_sec_);
    // Two cascaded velocity filters, analytically integrated. Only forward
    // kinematics checks their motion; no iterative position solver is involved.
    const auto advance = [&](const Vec& desired, Vec& candidate, Vec& first, Vec& second) {
        for (int i = 0; i < 3; ++i) {
            const double a = velocity_stage_[i] - desired[i];
            const double b = filtered_velocity_[i] - desired[i];
            first[i] = desired[i] + a * decay;
            second[i] = desired[i] + (b + a * dt / smoothing_sec_) * decay;
            candidate[i] = reference_[i] + desired[i] * dt +
                b * smoothing_sec_ * (1 - decay) +
                a * smoothing_sec_ * (1 - (1 + dt / smoothing_sec_) * decay);
        }
    };
    Vec candidate{}, first{}, second{}, stop{};
    advance(desired_velocity, candidate, first, second);
    const auto safe = [&]() {
        for (int i = 0; i < 3; ++i)
            stop[i] = candidate[i] + smoothing_sec_ * (first[i] + second[i]);
        return SafeSegment(reference_, candidate, entering_) && SafeSegment(candidate, stop, entering_);
    };
    if (!safe()) {
        advance({}, candidate, first, second);
        replan_ = true;
        if (!safe()) { candidate = reference_; first.fill(0); second.fill(0); }
    }
    const double travel = Distance(toe_reference_, Forward(candidate));
    if (travel > foot_speed_ * dt) {
        const double scale = foot_speed_ * dt / travel;
        candidate = Lerp(reference_, candidate, scale);
        for (int i = 0; i < 3; ++i) { first[i] *= scale; second[i] *= scale; }
    }
    reference_ = candidate;
    velocity_stage_ = first;
    filtered_velocity_ = second;
    if (entering_ && SafePose(reference_)) { entering_ = false; replan_ = true; }

    toe_reference_ = Forward(reference_);
    position_reference = reference_;
    saturated_ = false;

    for (int joint = 0; joint < 3; ++joint) {
        reference_velocity_[joint] = (reference_[joint] - old_reference[joint]) / dt;
        const double effort = gravity[joint];

        if (!std::isfinite(effort))
            throw std::runtime_error("invalid ball following joint input/output");

        saturated_ |= std::abs(effort) > torque_limit_[joint];
        torque[joint] = std::clamp(effort, -torque_limit_[joint], torque_limit_[joint]);
    }
}
}  // namespace qhrr05
