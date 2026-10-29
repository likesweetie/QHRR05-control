#pragma once

#include <mujoco/mujoco.h>
#include "ipc/control_types.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>
#include <vector>

namespace qhrr05::viewer {
// Viewer-local plot sample. Shared memory keeps only the canonical domains.
struct Sample {
    double time = 0, control_time = 0;
    std::uint64_t run_id = 0;
    int mode = 0;
    std::array<double, 3> position{}, reference{}, measured_torque{};
    MagneticTelemetry magnetic{};
    std::array<double, 3> tracking_reference{};
    TrackingPlane tracking_plane = TrackingPlane::XY;
    TrackingMethod tracking_method = TrackingMethod::InverseDynamics;
    std::uint64_t motion_sequence = 0;
    bool dynamics_enabled = false;
    std::array<double, 3> dynamics_torque{}, commanded_torque{};
    std::array<double, 3> ball_position{}, ball_reference{};
    std::array<double, 3> toe_position{}, contact_force{}, contact_torque{}, object_position{};
};

class LivePlots {
public:
    explicit LivePlots(double history_sec) : history_sec_(history_sec) {
        for (auto& figure : figures_) {
            mjv_defaultFigure(&figure);
            figure.flg_legend = 1;
            figure.flg_extend = 0;
            figure.gridsize[0] = figure.gridsize[1] = 5;
            figure.linewidth = 2;
            const float background[] = {.06f, .08f, .10f, 1.f};
            std::copy(background, background + 4, figure.figurergba);
            std::snprintf(figure.xlabel, sizeof(figure.xlabel), "Simulation time [s]");
            std::snprintf(figure.xformat, sizeof(figure.xformat), "%%.1f");
            std::snprintf(figure.yformat, sizeof(figure.yformat), "%%.3f");
        }

        std::snprintf(
            figures_[0].title, sizeof(figures_[0].title), "Joint angle [rad] - measured/reference");
        std::snprintf(figures_[1].title,
            sizeof(figures_[1].title),
            "Actuator torque [N m] - measured feedback");
        std::snprintf(figures_[2].title,
            sizeof(figures_[2].title),
            "Contact force [N] - equivalent estimate/GT");
        std::snprintf(
            figures_[3].title, sizeof(figures_[3].title), "Toe position [m] - actual/reference");
        std::snprintf(figures_[4].title,
            sizeof(figures_[4].title),
            "Tracking [mm] - error and signed travel");
        std::snprintf(figures_[5].title,
            sizeof(figures_[5].title),
            "IDFF raw / torque limit [%%] - before alpha");
        const char* joints[] = {"hip", "thigh", "calf"};
        const char* axes[] = {"x", "y", "z"};
        const float colors[3][3] = {{.25f, .8f, 1.f}, {1.f, .55f, .2f}, {.4f, 1.f, .4f}};

        for (int i = 0; i < 3; ++i) {
            for (auto& figure : figures_)
                std::copy(colors[i], colors[i] + 3, figure.linergb[i]);

            std::snprintf(
                figures_[0].linename[i], sizeof(figures_[0].linename[i]), "%s", joints[i]);
            std::snprintf(
                figures_[1].linename[i], sizeof(figures_[1].linename[i]), "%s", joints[i]);
            std::snprintf(
                figures_[5].linename[i], sizeof(figures_[5].linename[i]), "%s", joints[i]);
            std::snprintf(figures_[0].linename[i + 3],
                sizeof(figures_[0].linename[i + 3]),
                "%s ref",
                joints[i]);
            std::snprintf(figures_[3].linename[i], sizeof(figures_[3].linename[i]), "%s", axes[i]);
            std::snprintf(figures_[3].linename[i + 3],
                sizeof(figures_[3].linename[i + 3]),
                "%s ref",
                axes[i]);
            std::snprintf(figures_[3].linename[i + 6],
                sizeof(figures_[3].linename[i + 6]),
                "%s ball",
                axes[i]);

            for (int c = 0; c < 3; ++c) {
                figures_[0].linergb[i + 3][c] = .45f + .5f * colors[i][c];
                figures_[3].linergb[i + 3][c] = .45f + .5f * colors[i][c];
            }
        }

        const char* forces[] = {"RFOB", "GT", "target", "command"};

        for (int i = 0; i < 4; ++i)
            std::snprintf(
                figures_[2].linename[i], sizeof(figures_[2].linename[i]), "%s", forces[i]);

        figures_[2].linergb[3][0] = 1;
        figures_[2].linergb[3][1] = .4;
        figures_[2].linergb[3][2] = 1;
        std::snprintf(figures_[4].linename[0], sizeof(figures_[4].linename[0]), "position error");
        std::snprintf(figures_[4].linename[1], sizeof(figures_[4].linename[1]), "reference travel");

        for (int i = 3; i < 5; ++i) {
            std::snprintf(figures_[5].linename[i],
                sizeof(figures_[5].linename[i]),
                "%s100%%",
                i == 3 ? "+" : "-");
            std::fill_n(figures_[5].linergb[i], 3, .6f);
        }

        mjv_defaultFigure(&path_);
        path_.flg_legend = 1;
        path_.flg_extend = 0;
        path_.linewidth = 2;
        path_.gridsize[0] = path_.gridsize[1] = 5;
        std::snprintf(path_.title, sizeof(path_.title), "Toe path X-Y [m]");
        std::snprintf(path_.xlabel, sizeof(path_.xlabel), "World X [m]");
        std::snprintf(path_.linename[0], sizeof(path_.linename[0]), "actual");
        std::snprintf(path_.linename[1], sizeof(path_.linename[1]), "reference");
    }

    void Clear() {
        size_ = next_ = 0;
    }

    void Push(const Sample& sample) {
        if (sample.mode == 3)
            for (int i = 0; i < 3; ++i)
                if ((sample.dynamics_enabled && !std::isfinite(sample.dynamics_torque[i])) ||
                    !std::isfinite(sample.commanded_torque[i]))
                    throw std::runtime_error("non-finite tracking torque plot sample");

        if (size_) {
            const auto& previous = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
            if (sample.run_id != previous.run_id || sample.time < previous.time ||
                (sample.mode >= 2 && sample.mode <= 4 &&
                    (sample.mode != previous.mode ||
                        sample.motion_sequence != previous.motion_sequence)))
                Clear();
        }

        samples_[next_] = sample;
        next_ = (next_ + 1) % mjMAXLINEPNT;
        size_ = std::min(size_ + 1, mjMAXLINEPNT);
    }

    bool DynamicsPeak(std::array<double, 3>& peak) const {
        peak.fill(0);

        if (!size_)
            return false;

        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
        bool found = false;

        for (int p = 0; p < size_; ++p) {
            const auto& sample = samples_[(next_ + mjMAXLINEPNT - size_ + p) % mjMAXLINEPNT];

            if (sample.mode != 3 || !sample.dynamics_enabled || sample.run_id != last.run_id ||
                sample.motion_sequence != last.motion_sequence ||
                sample.tracking_plane != last.tracking_plane ||
                sample.tracking_method != last.tracking_method ||
                sample.time < last.time - history_sec_)
                continue;

            found = true;

            for (int i = 0; i < 3; ++i)
                peak[i] = std::max(peak[i], std::abs(sample.dynamics_torque[i]));
        }

        return found && last.mode == 3;
    }

    std::string TorqueHeadroom(const std::array<double, 3>& limits, double sample_hz) const {
        if (!size_)
            return "\nTracking torque: awaiting plot samples";

        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];

        if (last.mode != 3)
            return "\nTracking torque: awaiting Tracking samples";

        std::array<double, 3> peak;
        const bool have_peak = DynamicsPeak(peak);
        std::ostringstream text;
        text << (last.tracking_method == TrackingMethod::CTC ? "\nCTC raw (before clipping): "
                    : last.tracking_method == TrackingMethod::Impedance
                        ? "\nImpedance mode (no IDFF): "
                        : "\nIDFF raw (before alpha): ")
             << (last.dynamics_enabled ? "ON" : "OFF - current N/A") << "\nSampled " << std::fixed
             << std::setprecision(0) << sample_hz << " Hz / last " << std::setprecision(1)
             << history_sec_ << " sim s"
             << "\n     limit / now / |peak| / left Nm (%)";
        const char* joints[] = {"HR", "HP", "KP"};

        for (int i = 0; i < 3; ++i) {
            text << '\n' << joints[i] << "  " << limits[i] << " / ";

            if (last.dynamics_enabled)
                text << std::showpos << last.dynamics_torque[i] << std::noshowpos;
            else
                text << "N/A";

            if (have_peak) {
                const double margin = limits[i] - peak[i];
                text << " / " << peak[i] << " / " << std::showpos << margin << " ("
                     << std::setprecision(0) << 100 * margin / limits[i] << "%)" << std::noshowpos
                     << std::setprecision(1) << (margin < 0 ? " OVER" : "");
            } else {
                text << " / N/A / N/A";
            }
        }

        text << "\nCtrl cmd left now [Nm]: " << std::showpos
             << limits[0] - std::abs(last.commanded_torque[0]) << " / "
             << limits[1] - std::abs(last.commanded_torque[1]) << " / "
             << limits[2] - std::abs(last.commanded_torque[2]);
        return text.str();
    }

    void AddToeTrail(mjvScene& scene) const {
        if (size_ < 2)
            return;

        constexpr double duration = 2.0;  // Simulation seconds; pauses freeze the trail.
        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];

        for (int p = 1; p < size_; ++p) {
            const int index = (next_ + mjMAXLINEPNT - size_ + p) % mjMAXLINEPNT;
            const auto& from = samples_[(index + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
            const auto& to = samples_[index];
            const double age = last.time - from.time;

            if (from.run_id != last.run_id || to.run_id != last.run_id || age < 0 ||
                age >= duration || to.time <= from.time)
                continue;

            if (mju_dist3(from.toe_position.data(), to.toe_position.data()) < 1e-6)
                continue;

            if (scene.ngeom == scene.maxgeom)
                throw std::runtime_error("MuJoCo scene has no room for the toe trail");

            const float fade = static_cast<float>(1 - age / duration);
            const float rgba[] = {1.f, .12f, .08f, .5f * fade * fade};
            auto& geom = scene.geoms[scene.ngeom++];
            mjv_initGeom(&geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rgba);
            mjv_connector(
                &geom, mjGEOM_CAPSULE, .001, from.toe_position.data(), to.toe_position.data());
            geom.category = mjCAT_DECOR;
            geom.objtype = mjOBJ_UNKNOWN;
            geom.objid = geom.segid = -1;
            geom.specular = geom.reflectance = 0;
            geom.emission = .4f;
        }
    }

    bool Prepare(const std::array<double, 3>& torque_limits) {
        if (!size_)
            return false;

        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
        std::fill(std::begin(path_.linepnt), std::end(path_.linepnt), 0);
        const auto path_axes = TrackingPlaneAxes(last.tracking_plane);
        // MuJoCo supports multiple subplot titles in this buffer. Clear old
        // suffixes as well when switching between differently sized labels.
        std::fill(std::begin(figures_[5].title), std::end(figures_[5].title), '\0');
        if (last.tracking_method == TrackingMethod::Impedance)
            std::snprintf(figures_[5].title,
                sizeof(figures_[5].title),
                "Impedance: no inverse dynamics torque");
        else
            std::snprintf(figures_[5].title,
                sizeof(figures_[5].title),
                "%s / torque limit [%%] - %s",
                last.tracking_method == TrackingMethod::CTC ? "CTC" : "IDFF raw",
                last.tracking_method == TrackingMethod::CTC ? "before clipping" : "before alpha");
        const char* axis_names[] = {"X", "Y", "Z"};
        std::snprintf(path_.title,
            sizeof(path_.title),
            "Toe path %s [m]",
            TrackingPlaneName(last.tracking_plane));
        std::snprintf(path_.xlabel, sizeof(path_.xlabel), "World %s [m]", axis_names[path_axes[0]]);
        path_.range[0][0] = path_.range[0][1] = 0;
        path_.range[1][0] = path_.range[1][1] = 0;

        for (auto& figure : figures_) {
            std::fill(std::begin(figure.linepnt), std::end(figure.linepnt), 0);
            figure.range[0][0] = std::max(0.0, last.time - history_sec_);
            figure.range[0][1] = std::max(history_sec_, last.time);
            figure.range[1][0] = figure.range[1][1] = 0;
        }

        for (int p = 0; p < size_; ++p) {
            const auto& sample = samples_[(next_ + mjMAXLINEPNT - size_ + p) % mjMAXLINEPNT];

            if (sample.time < last.time - history_sec_)
                continue;

            if ((last.mode == 2 || last.mode == 3 || last.mode == 4) && sample.mode != last.mode)
                continue;

            if (last.mode == 3 && sample.tracking_plane != last.tracking_plane)
                continue;

            if (last.mode == 3 &&
                (sample.tracking_method != last.tracking_method || sample.run_id != last.run_id ||
                    sample.motion_sequence != last.motion_sequence))
                continue;

            double error = 0, normal_gt = 0, gt_norm = 0;
            const auto& m = sample.magnetic;
            const auto& toe_reference = sample.mode == 4   ? sample.ball_reference
                                        : sample.mode == 3 ? sample.tracking_reference
                                                           : m.toe_reference;

            for (int i = 0; i < 3; ++i) {
                Add(figures_[0], i, sample.time, sample.position[i]);
                Add(figures_[0], i + 3, sample.time, sample.reference[i]);
                Add(figures_[1], i, sample.time, sample.measured_torque[i]);
                Add(figures_[3], i, sample.time, sample.toe_position[i]);
                Add(figures_[3], i + 3, sample.time, toe_reference[i]);
                error += std::pow(sample.toe_position[i] - toe_reference[i], 2);
                normal_gt -= m.normal[i] * sample.contact_force[i];
                gt_norm += sample.contact_force[i] * sample.contact_force[i];

                if (sample.mode == 4)
                    Add(figures_[3], i + 6, sample.time, sample.ball_position[i]);
            }

            const bool following =
                m.phase == MagneticPhase::Following || m.phase == MagneticPhase::Releasing;
            Add(figures_[2], 0, sample.time, m.estimated_normal_force);
            Add(figures_[2], 1, sample.time, following ? normal_gt : std::sqrt(gt_norm));
            Add(figures_[2], 2, sample.time, m.target_force);
            Add(figures_[2], 3, sample.time, m.force_command);
            Add(figures_[4], 0, sample.time, 1000 * std::sqrt(error));
            Add(figures_[4], 1, sample.time, 1000 * m.displacement);

            if (sample.mode == 3) {
                Add(path_, 0, sample.toe_position[path_axes[0]], sample.toe_position[path_axes[1]]);
                Add(path_,
                    1,
                    sample.tracking_reference[path_axes[0]],
                    sample.tracking_reference[path_axes[1]]);

                if (sample.dynamics_enabled)
                    for (int i = 0; i < 3; ++i)
                        Add(figures_[5],
                            i,
                            sample.time,
                            100 * sample.dynamics_torque[i] / torque_limits[i]);

                Add(figures_[5], 3, sample.time, 100);
                Add(figures_[5], 4, sample.time, -100);
            }
        }

        return true;
    }

    std::vector<const mjvFigure*> Panels(const std::array<double, 3>& torque_limits) {
        if (!Prepare(torque_limits))
            return {};
        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
        if (last.mode == 4)
            return {&figures_[3], &figures_[1]};
        if (last.mode == 3) {
            figures_[4].linepnt[1] = 0;
            return {&path_, &figures_[3], &figures_[4], &figures_[5]};
        }
        if (last.mode == 2)
            return {&figures_[2], &figures_[3], &figures_[4]};
        return {&figures_[0], &figures_[1]};
    }

    void Draw(mjrRect rect, mjrContext* context, const std::array<double, 3>& torque_limits) {
        if (!Prepare(torque_limits))
            return;
        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
        if (last.mode == 4) {
            const int half = rect.height / 2;
            mjr_figure({rect.left, rect.bottom + half, rect.width, rect.height - half},
                &figures_[3],
                context);
            mjr_figure({rect.left, rect.bottom, rect.width, half}, &figures_[1], context);
        } else if (last.mode == 3) {
            const int height = rect.height / 4;
            mjr_figure({rect.left, rect.bottom + 3 * height, rect.width, height}, &path_, context);
            mjr_figure(
                {rect.left, rect.bottom + 2 * height, rect.width, height}, &figures_[3], context);
            figures_[4].linepnt[1] = 0;
            mjr_figure(
                {rect.left, rect.bottom + height, rect.width, height}, &figures_[4], context);
            mjr_figure({rect.left, rect.bottom, rect.width, height}, &figures_[5], context);
        } else if (last.mode == 2) {
            const int height = rect.height / 3;

            for (int i = 0; i < 3; ++i)
                mjr_figure({rect.left, rect.bottom + (2 - i) * height, rect.width, height},
                    &figures_[2 + i],
                    context);
        } else {
            const int half = rect.height / 2;
            mjr_figure({rect.left, rect.bottom + half, rect.width, rect.height - half},
                &figures_[0],
                context);
            mjr_figure({rect.left, rect.bottom, rect.width, half}, &figures_[1], context);
        }
    }

private:
    static void Add(mjvFigure& figure, int line, double time, double value) {
        const int point = figure.linepnt[line]++;
        figure.linedata[line][2 * point] = time;
        figure.linedata[line][2 * point + 1] = value;
    }

    double history_sec_;
    std::array<Sample, mjMAXLINEPNT> samples_{};
    int size_ = 0, next_ = 0;
    std::array<mjvFigure, 6> figures_{};
    mjvFigure path_{};
};
}  // namespace qhrr05::viewer
