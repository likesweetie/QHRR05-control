#pragma once

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mujoco/mujoco.h>

namespace qhrr05 {
class BallFollowingController {
public:
    explicit BallFollowingController(const std::string& config_path,
        const std::atomic<bool>* cancellation = nullptr);
    std::vector<std::array<double, 9>> WorkspaceSurface() const;
    void Start(const std::array<double, 3>& joint_position);
    void SetBallPosition(const std::array<double, 3>& position);
    void Compute(double dt, const std::array<double, 3>& joystick_direction,
        double l2, double r2, bool connected,
        const std::array<double, 3>& joint_position,
        const std::array<double, 3>& joint_velocity,
        const std::array<double, 3>& gravity, std::array<double, 3>& torque);

    const std::array<double, 3>& kp_motor() const {
        return kp_;
    }

    const std::array<double, 3>& kd_motor() const {
        return kd_;
    }

    const std::array<double, 3>& ball_position() const {
        return ball_;
    }

    const std::array<double, 3>& toe_reference() const {
        return toe_reference_;
    }

    const std::array<double, 3>& nearest_position() const {
        return positions_[goal_];
    }

    const std::array<double, 3>& joint_reference() const {
        return reference_;
    }

    const std::array<double, 3>& joint_velocity_reference() const {
        return reference_velocity_;
    }

    bool workspace_limited() const {
        return limited_;
    }

    bool entering_workspace() const {
        return entering_;
    }

    bool torque_saturated() const {
        return saturated_;
    }

    double ball_speed() const {
        return ball_speed_;
    }

    double foot_speed() const {
        return foot_speed_;
    }

    double margin() const {
        return margin_;
    }

    std::size_t workspace_points() const {
        return valid_indices_.size();
    }

private:
    void BuildWorkspace(const std::array<double, 3>& home);
    std::array<double, 3> Forward(const std::array<double, 3>& q, bool* regular = nullptr);
    bool ClearOfBoundary(const std::array<double, 3>& position) const;
    bool SafeEdge(int from, int to);
    void Plan(int from, int to);
    void BeginSegment(const std::array<double, 3>& target, int node);
    int Nearest(const std::array<double, 3>& position) const;
    int Neighbor(int node, int axis, int sign) const;

    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    int site_ = -1, grid_ = 0, current_node_ = -1, segment_node_ = -1, goal_ = -1;
    int planned_goal_ = -1;
    double margin_, padding_, condition_limit_, branch_sign_ = 1;
    double foot_speed_, slow_speed_, normal_speed_, fast_speed_, trigger_threshold_;
    double ball_speed_ = 0, segment_progress_ = 0;
    bool limited_ = false, entering_ = false, saturated_ = false;
    bool enabled_ = true;
    const std::atomic<bool>* cancellation_ = nullptr;
    std::array<double, 3> kp_, kd_, torque_limit_, ball_{}, reference_{}, reference_velocity_{};
    std::array<double, 3> toe_reference_{}, previous_ball_{}, segment_start_{}, segment_end_{};
    std::array<double, 33> segment_arc_{};
    std::vector<std::array<double, 3>> joints_, positions_;
    std::vector<unsigned char> valid_, edges_;
    std::vector<int> valid_indices_, path_;
    std::size_t path_next_ = 0;
    std::vector<std::array<double, 9>> boundary_;
    std::map<std::array<int, 3>, std::vector<int>> boundary_bins_;
};
}  // namespace qhrr05
