#pragma once

#include <mujoco/mujoco.h>
#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>

namespace qhrr05::viewer {
struct Sample {
    double time = 0;
    std::array<double, 3> position{};
    std::array<double, 3> reference{};
    std::array<double, 3> torque{};
};
class Recorder {
public:
    explicit Recorder(const std::string& path) {
        if (path.empty()) return;
        if (std::filesystem::exists(path)) throw std::runtime_error("CSV already exists: " + path);
        file_.open(path);
        if (!file_) throw std::runtime_error("cannot open CSV: " + path);
        file_ << "run_id,sim_time";
        for (const auto& field : {"q", "q_ref", "torque"})
            for (const char* joint : {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}) file_ << ',' << field << '_' << joint;
        file_ << '\n' << std::setprecision(12);
    }
    void Write(const Sample& sample) {
        if (!file_.is_open()) return;
        file_ << run_id_ << ',' << sample.time;
        for (const auto& values : {sample.position, sample.reference, sample.torque})
            for (double value : values) file_ << ',' << value;
        file_ << '\n';
        if (!file_) throw std::runtime_error("CSV write failed");
    }
    void Finish() {
        if (!file_.is_open()) return;
        file_.flush();
        if (!file_) throw std::runtime_error("CSV flush failed");
    }
    void Restart() { ++run_id_; }
private:
    std::ofstream file_;
    unsigned int run_id_ = 0;
};

class LivePlots {
public:
    explicit LivePlots(double history_sec) : history_sec_(history_sec) {
        for (auto& figure : figures_) {
            mjv_defaultFigure(&figure);
            figure.flg_legend = 1;
            figure.flg_extend = 0;
            figure.gridsize[0] = 5;
            figure.gridsize[1] = 5;
            figure.linewidth = 2;
            figure.figurergba[0] = .06f;
            figure.figurergba[1] = .08f;
            figure.figurergba[2] = .10f;
            figure.figurergba[3] = 1;
            std::snprintf(figure.xlabel, sizeof(figure.xlabel), "Simulation time [s]");
            std::snprintf(figure.xformat, sizeof(figure.xformat), "%%.1f");
            std::snprintf(figure.yformat, sizeof(figure.yformat), "%%.2f");
        }
        std::snprintf(figures_[0].title, sizeof(figures_[0].title), "Joint angle [rad] - measured and reference");
        std::snprintf(figures_[1].title, sizeof(figures_[1].title), "Actuator torque [N m] - actual motor output");
        const float colors[3][3] = {{.25f,.8f,1.f},{1.f,.55f,.2f},{.4f,1.f,.4f}};
        const char* names[3] = {"hip", "thigh", "calf"};
        for (int i = 0; i < 3; ++i) {
            for (auto& figure : figures_) {
                std::copy(colors[i], colors[i]+3, figure.linergb[i]);
                std::snprintf(figure.linename[i], sizeof(figure.linename[i]), "%s", names[i]);
            }
            for (int c = 0; c < 3; ++c) figures_[0].linergb[i+3][c] = .45f + .5f*colors[i][c];
            std::snprintf(figures_[0].linename[i+3], sizeof(figures_[0].linename[i+3]), "%s ref", names[i]);
        }
    }
    void Clear() { size_ = 0; next_ = 0; }
    void Push(const Sample& sample) {
        samples_[next_] = sample;
        next_ = (next_ + 1) % mjMAXLINEPNT;
        size_ = std::min(size_ + 1, mjMAXLINEPNT);
    }
    void Draw(mjrRect rect, mjrContext* context) {
        if (size_ == 0) return;
        const auto& last = samples_[(next_ + mjMAXLINEPNT - 1) % mjMAXLINEPNT];
        for (auto& figure : figures_) {
            std::fill(std::begin(figure.linepnt), std::end(figure.linepnt), 0);
            figure.range[0][0] = static_cast<float>(std::max(0.0, last.time-history_sec_));
            figure.range[0][1] = static_cast<float>(std::max(history_sec_, last.time));
            figure.range[1][0] = 0;
            figure.range[1][1] = 0;  // Auto-scale y, including negative motor torques.
        }
        for (int p = 0; p < size_; ++p) {
            const auto& sample = samples_[(next_ + mjMAXLINEPNT-size_+p) % mjMAXLINEPNT];
            if (sample.time < last.time-history_sec_) continue;
            for (int i = 0; i < 3; ++i) {
                Add(figures_[0], i, sample.time, sample.position[i]);
                Add(figures_[0], i+3, sample.time, sample.reference[i]);
                Add(figures_[1], i, sample.time, sample.torque[i]);
            }
        }
        const int half = rect.height/2;
        mjr_figure({rect.left, rect.bottom+half, rect.width, rect.height-half}, &figures_[0], context);
        mjr_figure({rect.left, rect.bottom, rect.width, half}, &figures_[1], context);
    }
private:
    static void Add(mjvFigure& figure, int line, double time, double value) {
        const int point = figure.linepnt[line]++;
        figure.linedata[line][2*point] = static_cast<float>(time);
        figure.linedata[line][2*point+1] = static_cast<float>(value);
    }
    double history_sec_;
    std::array<Sample, mjMAXLINEPNT> samples_{};
    int size_ = 0;
    int next_ = 0;
    std::array<mjvFigure, 2> figures_{};
};


}  // namespace qhrr05::viewer
