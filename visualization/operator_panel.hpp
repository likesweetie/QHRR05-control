#pragma once
#include "snapshot.hpp"
#include "interfaces/input/client.hpp"
#include "simulation/telemetry.hpp"
#include "simulation/workspace_overlay.hpp"
#include <deque>

namespace qhrr05::visualization {
class OperatorPanel {
public:
    OperatorPanel(const std::string& config, InputClient* input, bool visible);
    void Update(
        const Snapshot& snapshot, const std::vector<Snapshot>& samples, std::uint64_t dropped);
    void Draw(int width, int height);
    void AddGeometry(mjvScene& scene);
    bool Key(int key);
    void Send(const std::string& operation);
    void Mode(int mode, int plane = -1);
    void Error(const std::string& message);
    bool visible;
    bool trail = true;
    int CanvasWidth(int width) const;

private:
    void Figure(const mjvFigure& figure, float height);
    std::string config_;
    InputClient* input_;
    Snapshot snapshot_{};
    std::unique_ptr<viewer::LivePlots> plots_;
    viewer::WorkspaceOverlay workspace_;
    std::array<double, 3> limits_{};
    double sample_hz_ = 100, maximum_force_ = 0;
    float target_force_ = 0;
    bool force_editing_ = false;
    std::uint64_t dropped_ = 0;
    std::deque<InputDecision> decisions_;
};
}  // namespace qhrr05::visualization
