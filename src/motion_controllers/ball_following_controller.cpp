#include "motion_controllers/ball_following_controller.hpp"
#include "control/limits.hpp"

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

    for (int i = 0; i < 3; ++i)
        if (kp_[i] <= 0 || kp_[i] > 500 || kd_[i] > 5 || torque_limit_[i] <= 0)
            throw std::runtime_error("invalid ball following MIT gains/torque limit");

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
    planned_goal_ = to;
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

void BallFollowingController::BeginSegment(const Vec& target, int node) {
    segment_start_ = reference_;
    segment_end_ = target;
    segment_node_ = node;
    segment_progress_ = 0;
    segment_arc_[0] = 0;
    auto previous = Forward(segment_start_);

    for (int i = 1; i <= 32; ++i) {
        const auto point = Forward(Lerp(segment_start_, segment_end_, double(i) / 32));
        segment_arc_[i] = segment_arc_[i - 1] + Distance(point, previous);
        previous = point;
    }
}

void BallFollowingController::Start(const Vec& joint_position) {
    if (!enabled_)
        throw std::runtime_error("ball following is disabled in control configuration");

    bool regular;
    ball_ = toe_reference_ = Forward(joint_position, &regular);

    if (!regular)
        throw std::runtime_error(
            "ball following requires the nonsingular homing branch; select H first");

    reference_ = joint_position;
    reference_velocity_.fill(0);
    goal_ = Nearest(ball_);
    previous_ball_ = ball_;
    int seed = -1;
    double best = std::numeric_limits<double>::infinity();

    for (int node : valid_indices_)
        if (Distance(joint_position, joints_[node]) < best) {
            best = Distance(joint_position, joints_[node]);
            seed = node;
        }

    // Entry may start outside the eroded workspace: enter continuously, never teleport.
    for (int i = 1; i <= 64; ++i) {
        Forward(Lerp(reference_, joints_[seed], double(i) / 64), &regular);

        if (!regular)
            throw std::runtime_error(
                "ball following entry crosses a singular branch; select H first");
    }

    current_node_ = -1;
    planned_goal_ = -1;
    path_.clear();
    entering_ = true;
    limited_ = false;
    BeginSegment(joints_[seed], seed);
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
    Vec& torque) {
    if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(l2) || !std::isfinite(r2))
        throw std::runtime_error("invalid ball following dt or triggers");

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

    if (Distance(ball_, previous_ball_) > 1e-8) {
        goal_ = Nearest(ball_);
        previous_ball_ = ball_;
    }

    limited_ = Distance(ball_, positions_[goal_]) > margin_;
    const auto old_reference = reference_;
    double travel = foot_speed_ * dt;

    while (travel > 1e-12) {
        if (segment_node_ < 0) {
            if (current_node_ == goal_)
                break;

            if (planned_goal_ != goal_ || path_next_ == path_.size())
                Plan(current_node_, goal_);

            const int next = path_[path_next_++];
            BeginSegment(joints_[next], next);
        }

        const double remaining = segment_arc_[32] - segment_progress_;
        const double advance = std::min(travel, remaining);
        segment_progress_ += advance;
        travel -= advance;
        const auto upper =
            std::upper_bound(segment_arc_.begin(), segment_arc_.end(), segment_progress_);
        const int i = std::clamp(int(upper - segment_arc_.begin()), 1, 32);
        const double fraction = (segment_progress_ - segment_arc_[i - 1]) /
                                std::max(segment_arc_[i] - segment_arc_[i - 1], 1e-12);
        reference_ = Lerp(segment_start_, segment_end_, (i - 1 + fraction) / 32);

        if (remaining - advance < 1e-10) {
            reference_ = segment_end_;
            current_node_ = segment_node_;
            segment_node_ = -1;
            entering_ = false;
        } else
            break;
    }

    toe_reference_ = Forward(reference_);
    saturated_ = false;

    for (int joint = 0; joint < 3; ++joint) {
        reference_velocity_[joint] = (reference_[joint] - old_reference[joint]) / dt;
        const double effort = gravity[joint] +
                              kp_[joint] * (reference_[joint] - joint_position[joint]) +
                              kd_[joint] * (reference_velocity_[joint] - joint_velocity[joint]);

        if (!std::isfinite(effort))
            throw std::runtime_error("invalid ball following joint input/output");

        saturated_ |= std::abs(effort) > torque_limit_[joint];
        torque[joint] = std::clamp(effort, -torque_limit_[joint], torque_limit_[joint]);
    }
}
}  // namespace qhrr05
