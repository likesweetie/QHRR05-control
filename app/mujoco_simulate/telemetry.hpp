#pragma once

#include <mujoco/mujoco.h>
#include "shared_memory.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace qhrr05::viewer {
using Sample = ShmSample;
class LivePlots {
public:
    explicit LivePlots(double history_sec) : history_sec_(history_sec) {
        for (auto& figure : figures_) {
            mjv_defaultFigure(&figure);
            figure.flg_legend = 1; figure.flg_extend = 0;
            figure.gridsize[0] = figure.gridsize[1] = 5;
            figure.linewidth = 2;
            const float background[] = {.06f,.08f,.10f,1.f};
            std::copy(background, background+4, figure.figurergba);
            std::snprintf(figure.xlabel, sizeof(figure.xlabel), "Simulation time [s]");
            std::snprintf(figure.xformat, sizeof(figure.xformat), "%%.1f");
            std::snprintf(figure.yformat, sizeof(figure.yformat), "%%.3f");
        }
        std::snprintf(figures_[0].title, sizeof(figures_[0].title), "Joint angle [rad] - measured/reference");
        std::snprintf(figures_[1].title, sizeof(figures_[1].title), "Actuator torque [N m] - actual output");
        std::snprintf(figures_[2].title, sizeof(figures_[2].title), "Contact force [N] - equivalent estimate/GT");
        std::snprintf(figures_[3].title, sizeof(figures_[3].title), "Toe position [m] - actual/reference");
        std::snprintf(figures_[4].title, sizeof(figures_[4].title), "Tracking [mm] - error and signed travel");
        const char* joints[] = {"hip", "thigh", "calf"};
        const char* axes[] = {"x", "y", "z"};
        const float colors[3][3] = {{.25f,.8f,1.f},{1.f,.55f,.2f},{.4f,1.f,.4f}};
        for (int i = 0; i < 3; ++i) {
            for (auto& figure : figures_) std::copy(colors[i], colors[i]+3, figure.linergb[i]);
            std::snprintf(figures_[0].linename[i], sizeof(figures_[0].linename[i]), "%s", joints[i]);
            std::snprintf(figures_[1].linename[i], sizeof(figures_[1].linename[i]), "%s", joints[i]);
            std::snprintf(figures_[0].linename[i+3], sizeof(figures_[0].linename[i+3]), "%s ref", joints[i]);
            std::snprintf(figures_[3].linename[i], sizeof(figures_[3].linename[i]), "%s", axes[i]);
            std::snprintf(figures_[3].linename[i+3], sizeof(figures_[3].linename[i+3]), "%s ref", axes[i]);
            for (int c = 0; c < 3; ++c) {
                figures_[0].linergb[i+3][c] = .45f+.5f*colors[i][c];
                figures_[3].linergb[i+3][c] = .45f+.5f*colors[i][c];
            }
        }
        const char* forces[] = {"RFOB", "GT", "target", "command"};
        for (int i = 0; i < 4; ++i) std::snprintf(figures_[2].linename[i], sizeof(figures_[2].linename[i]), "%s", forces[i]);
        figures_[2].linergb[3][0] = 1; figures_[2].linergb[3][1] = .4; figures_[2].linergb[3][2] = 1;
        std::snprintf(figures_[4].linename[0], sizeof(figures_[4].linename[0]), "position error");
        std::snprintf(figures_[4].linename[1], sizeof(figures_[4].linename[1]), "reference travel");
    }
    void Clear() { size_ = next_ = 0; }
    void Push(const Sample& sample) {
        samples_[next_] = sample;
        next_ = (next_+1) % mjMAXLINEPNT;
        size_ = std::min(size_+1, mjMAXLINEPNT);
    }
    void Draw(mjrRect rect, mjrContext* context) {
        if (!size_) return;
        const auto& last = samples_[(next_+mjMAXLINEPNT-1) % mjMAXLINEPNT];
        for (auto& figure : figures_) {
            std::fill(std::begin(figure.linepnt), std::end(figure.linepnt), 0);
            figure.range[0][0] = std::max(0.0, last.time-history_sec_);
            figure.range[0][1] = std::max(history_sec_, last.time);
            figure.range[1][0] = figure.range[1][1] = 0;
        }
        for (int p = 0; p < size_; ++p) {
            const auto& sample = samples_[(next_+mjMAXLINEPNT-size_+p) % mjMAXLINEPNT];
            if (sample.time < last.time-history_sec_) continue;
            if (last.mode == 2 && sample.mode != 2) continue;
            double error = 0, normal_gt = 0, gt_norm = 0;
            const auto& m = sample.magnetic;
            for (int i = 0; i < 3; ++i) {
                Add(figures_[0], i, sample.time, sample.position[i]);
                Add(figures_[0], i+3, sample.time, sample.reference[i]);
                Add(figures_[1], i, sample.time, sample.torque[i]);
                Add(figures_[3], i, sample.time, sample.toe_position[i]);
                Add(figures_[3], i+3, sample.time, m.toe_reference[i]);
                error += std::pow(sample.toe_position[i]-m.toe_reference[i],2);
                normal_gt -= m.normal[i]*sample.contact_force[i];
                gt_norm += sample.contact_force[i]*sample.contact_force[i];
            }
            const bool following = m.phase == MagneticPhase::Following || m.phase == MagneticPhase::Releasing;
            Add(figures_[2], 0, sample.time, m.estimated_normal_force);
            Add(figures_[2], 1, sample.time, following ? normal_gt : std::sqrt(gt_norm));
            Add(figures_[2], 2, sample.time, m.target_force);
            Add(figures_[2], 3, sample.time, m.force_command);
            Add(figures_[4], 0, sample.time, 1000*std::sqrt(error));
            Add(figures_[4], 1, sample.time, 1000*m.displacement);
        }
        if (last.mode == 2) {
            const int height = rect.height/3;
            for (int i = 0; i < 3; ++i)
                mjr_figure({rect.left,rect.bottom+(2-i)*height,rect.width,height}, &figures_[2+i], context);
        } else {
            const int half = rect.height/2;
            mjr_figure({rect.left,rect.bottom+half,rect.width,rect.height-half}, &figures_[0], context);
            mjr_figure({rect.left,rect.bottom,rect.width,half}, &figures_[1], context);
        }
    }
private:
    static void Add(mjvFigure& figure, int line, double time, double value) {
        const int point = figure.linepnt[line]++;
        figure.linedata[line][2*point] = time;
        figure.linedata[line][2*point+1] = value;
    }
    double history_sec_;
    std::array<Sample,mjMAXLINEPNT> samples_{};
    int size_ = 0, next_ = 0;
    std::array<mjvFigure,5> figures_{};
};
}  // namespace qhrr05::viewer
