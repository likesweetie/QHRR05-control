#include "operator_panel.hpp"
#include "control/limits.hpp"
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace qhrr05::visualization {
namespace {
const char* ModeName(int mode) {
    static const char* names[] = {"HOMING", "GRAVITY", "MAGNETIC", "TRACKING", "BALL", "DAMPING"};
    return mode >= 0 && mode < 6 ? names[mode] : "UNKNOWN";
}

viewer::Sample PlotSample(const Snapshot& snapshot) {
    const auto& s = snapshot.simulation;
    const auto& c = snapshot.control;
    viewer::Sample result;
    result.time = s.time;
    result.run_id = s.run_id;
    result.control_time = c.control_time;
    result.mode = c.mode;
    result.position = s.position_gt;
    result.reference = snapshot.motor.reference;
    result.measured_torque = snapshot.motor.motor_torque;
    result.magnetic = c.magnetic;
    result.tracking_reference = c.tracking_position_reference;
    result.tracking_plane = c.tracking_plane;
    result.tracking_method = c.tracking_method;
    result.motion_sequence = c.active_motion_sequence;
    result.dynamics_enabled = c.mode == 3 && !c.tracking_ik_failed && !c.tracking_checking &&
                              (c.tracking_method == TrackingMethod::CTC || c.tracking_idff_enabled);
    result.dynamics_torque = c.tracking_method == TrackingMethod::CTC
                                 ? c.tracking_ctc_torque
                                 : c.tracking_inverse_dynamics_torque;
    result.commanded_torque = snapshot.motor.debug_torque;
    result.ball_position = c.ball_position;
    result.ball_reference = c.ball_toe_reference;
    result.toe_position = s.toe_position;
    result.contact_force = s.contact_force;
    result.contact_torque = s.contact_torque;
    result.object_position = s.object_position;
    return result;
}
}  // namespace

OperatorPanel::OperatorPanel(const std::string& config, InputClient* input, bool shown)
    : visible(shown), config_(config), input_(input) {
    const auto settings = YAML::LoadFile(config);
    const auto history = settings["plots"]["history_sec"].as<double>();
    sample_hz_ = settings["plots"]["sample_hz"].as<double>();
    if (!std::isfinite(history) || !std::isfinite(sample_hz_) || history <= 0 || sample_hz_ <= 0 ||
        history * sample_hz_ > mjMAXLINEPNT)
        throw std::runtime_error("invalid visualization plot history/sample rate");
    plots_ = std::make_unique<viewer::LivePlots>(history);
    maximum_force_ = settings["magnetic_touch"]["maximum_force_n"].as<double>();
    limits_ = ReadControlTorqueLimits(settings);
    if (!std::isfinite(maximum_force_) || maximum_force_ <= 0)
        throw std::runtime_error("invalid visualization force/torque limits");
}

void OperatorPanel::Error(const std::string& message) {
    std::cerr << "[operator] ERROR: " << message << std::endl;
    InputDecision decision;
    decision.message = message;
    decisions_.push_front(decision);
    if (decisions_.size() > 12)
        decisions_.pop_back();
}

void OperatorPanel::Send(const std::string& operation) {
    if (!input_) {
        Error("Static preview has no live Input connection");
        return;
    }
    try {
        const auto id = input_->Submit(operation);
        InputDecision pending;
        pending.id = id;
        pending.operation = operation;
        pending.message = "REQUESTED / awaiting safety decision";
        decisions_.push_front(pending);
        if (decisions_.size() > 12)
            decisions_.pop_back();
    } catch (const std::exception& error) {
        Error(error.what());
    }
}

void OperatorPanel::Mode(int mode, int plane) {
    if (plane < 0)
        plane = int(snapshot_.control.requested_tracking_plane);
    if (mode == snapshot_.control.requested_mode &&
        plane == int(snapshot_.control.requested_tracking_plane))
        return;
    Send("mode " + std::to_string(mode) + ' ' + std::to_string(plane));
}

bool OperatorPanel::Key(int key) {
    if (key == GLFW_KEY_TAB) {
        visible = !visible;
        return true;
    }
    if (key == GLFW_KEY_W) {
        workspace_.Toggle(config_);
        return true;
    }
    if (key == GLFW_KEY_L) {
        trail = !trail;
        return true;
    }
    if (key == GLFW_KEY_D)
        Mode(5);
    else if (key == GLFW_KEY_H)
        Mode(0);
    else if (key == GLFW_KEY_G)
        Mode(1);
    else if (key == GLFW_KEY_M)
        Mode(2);
    else if (key == GLFW_KEY_T)
        Mode(3);
    else if (key == GLFW_KEY_B)
        Mode(4);
    else if (key >= GLFW_KEY_1 && key <= GLFW_KEY_3)
        Mode(3, key - GLFW_KEY_1);
    else if (key == GLFW_KEY_SPACE)
        Send("pause 2");
    else if (key == GLFW_KEY_RIGHT)
        Send("step");
    else if (key == GLFW_KEY_R)
        Send("reset");
    else
        return false;
    return true;
}

int OperatorPanel::CanvasWidth(int width) const {
    return visible
               ? std::clamp(width - std::clamp(int(width * .40), 180, 700), 1, std::max(1, width))
               : width;
}

void OperatorPanel::Update(
    const Snapshot& snapshot, const std::vector<Snapshot>& samples, std::uint64_t dropped) {
    snapshot_ = snapshot;
    dropped_ = dropped;
    if (!force_editing_)
        target_force_ = snapshot.control.target_force;
    for (const auto& sample : samples)
        plots_->Push(PlotSample(sample));
    if (input_)
        for (auto& decision : input_->Decisions()) {
            auto found = std::find_if(decisions_.begin(), decisions_.end(), [&](const auto& old) {
                return old.id == decision.id;
            });
            if (found != decisions_.end())
                *found = decision;
            else
                decisions_.push_front(decision);
            if (decisions_.size() > 12)
                decisions_.pop_back();
        }
}

void OperatorPanel::Figure(const mjvFigure& figure, float height) {
    ImGui::PushID(&figure);
    ImGui::TextUnformatted(figure.title);
    const ImVec2 start = ImGui::GetCursorScreenPos();
    const ImVec2 size{std::max(100.f, ImGui::GetContentRegionAvail().x), height};
    ImGui::InvisibleButton("plot", size);
    auto* draw = ImGui::GetWindowDrawList();
    const ImVec2 a{start.x + 56, start.y + 8}, b{start.x + size.x - 10, start.y + size.y - 32};
    draw->AddRectFilled(start, {start.x + size.x, start.y + size.y}, IM_COL32(16, 23, 30, 255), 4);
    double xmin = figure.range[0][0], xmax = figure.range[0][1], ymin = 1e100, ymax = -1e100;
    bool have = false;
    if (xmax <= xmin) {
        xmin = 1e100;
        xmax = -1e100;
        for (int line = 0; line < mjMAXLINE; ++line)
            for (int point = 0; point < figure.linepnt[line]; ++point) {
                const auto x = figure.linedata[line][point * 2];
                if (std::isfinite(x)) {
                    xmin = std::min(xmin, double(x));
                    xmax = std::max(xmax, double(x));
                }
            }
    }
    for (int line = 0; line < mjMAXLINE; ++line)
        for (int point = 0; point < figure.linepnt[line]; ++point) {
            const auto x = figure.linedata[line][point * 2];
            const auto y = figure.linedata[line][point * 2 + 1];
            if (std::isfinite(y) && x >= xmin && x <= xmax) {
                ymin = std::min(ymin, double(y));
                ymax = std::max(ymax, double(y));
                have = true;
            }
        }
    if (!have) {
        draw->AddText(a, IM_COL32(150, 160, 170, 255), "No samples");
        ImGui::PopID();
        return;
    }
    if (xmax <= xmin) {
        xmin -= .01;
        xmax += .01;
    }
    const auto padding = std::max(.001, (ymax - ymin) * .10);
    ymin -= padding;
    ymax += padding;
    for (int tick = 0; tick <= 4; ++tick) {
        const float f = tick / 4.f;
        const float x = a.x + (b.x - a.x) * f, y = b.y - (b.y - a.y) * f;
        draw->AddLine({x, a.y}, {x, b.y}, IM_COL32(46, 56, 66, 255));
        draw->AddLine({a.x, y}, {b.x, y}, IM_COL32(46, 56, 66, 255));
        char label[40];
        std::snprintf(label, sizeof(label), "%.3g", ymin + f * (ymax - ymin));
        draw->AddText({start.x + 2, y - 6}, IM_COL32(170, 180, 190, 255), label);
        std::snprintf(label, sizeof(label), "%.2f", xmin + f * (xmax - xmin));
        draw->AddText({x - 14, b.y + 3}, IM_COL32(170, 180, 190, 255), label);
    }
    const auto point = [&](double x, double y) {
        return ImVec2{a.x + float((x - xmin) / (xmax - xmin)) * (b.x - a.x),
            b.y - float((y - ymin) / (ymax - ymin)) * (b.y - a.y)};
    };
    draw->PushClipRect(a, b, true);
    for (int line = 0; line < mjMAXLINE; ++line) {
        const auto color = ImGui::ColorConvertFloat4ToU32(
            {figure.linergb[line][0], figure.linergb[line][1], figure.linergb[line][2], 1});
        const int stride = std::max(1, (figure.linepnt[line] + 299) / 300);
        bool previous_valid = false;
        ImVec2 previous{};
        for (int p = 0; p < figure.linepnt[line]; p += stride) {
            const auto x = figure.linedata[line][p * 2], y = figure.linedata[line][p * 2 + 1];
            if (!std::isfinite(x) || !std::isfinite(y)) {
                previous_valid = false;
                continue;
            }
            const auto current = point(x, y);
            if (previous_valid)
                draw->AddLine(previous, current, color, 1.5f);
            previous = current;
            previous_valid = true;
        }
    }
    draw->PopClipRect();
    for (int line = 0; line < mjMAXLINE; ++line) {
        if (!figure.linepnt[line])
            continue;
        if (line)
            ImGui::SameLine();
        ImGui::TextColored(
            {figure.linergb[line][0], figure.linergb[line][1], figure.linergb[line][2], 1},
            "%s",
            figure.linename[line]);
    }
    ImGui::PopID();
}

void OperatorPanel::Draw(int width, int height) {
    if (!visible)
        return;
    const int canvas = CanvasWidth(width);
    ImGui::SetNextWindowPos({float(canvas), 0});
    ImGui::SetNextWindowSize({float(width - canvas), float(height)});
    ImGui::Begin("QHRR05 Operations",
        nullptr,
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoSavedSettings);
    const auto& c = snapshot_.control;
    const auto& s = snapshot_.simulation;
    ImGui::Text("%s | %s | sim %.3f s / ctrl %.3f s",
        ModeName(c.mode),
        input_ ? (s.paused ? "PAUSED" : "LIVE") : "STATIC PREVIEW",
        s.time,
        c.control_time);
    ImGui::Text("Simulation IPC: %s", snapshot_.simulation_status.c_str());
    ImGui::BeginDisabled(!input_);
    const char* buttons[] = {
        "D Damping", "H Home", "G Gravity", "M Magnetic", "T Tracking", "B Ball"};
    const int modes[] = {5, 0, 1, 2, 3, 4};
    for (int i = 0; i < 6; ++i) {
        if (i % 3)
            ImGui::SameLine();
        if (ImGui::Button(buttons[i]))
            Mode(modes[i]);
    }
    if (ImGui::Button("1 XY"))
        Mode(3, 0);
    ImGui::SameLine();
    if (ImGui::Button("2 XZ"))
        Mode(3, 1);
    ImGui::SameLine();
    if (ImGui::Button("3 YZ"))
        Mode(3, 2);
    if (ImGui::Button(s.paused ? "Space Resume" : "Space Pause"))
        Send("pause 2");
    ImGui::SameLine();
    if (ImGui::Button("R Reset"))
        Send("reset");
    ImGui::SameLine();
    if (ImGui::Button("Right Step"))
        Send("step");
    if (ImGui::SliderFloat("Target force [N]", &target_force_, 0, float(maximum_force_), "%.2f")) {
        std::ostringstream command;
        command << std::setprecision(9) << "force " << target_force_;
        Send(command.str());
    }
    force_editing_ = ImGui::IsItemActive();
    ImGui::EndDisabled();
    ImGui::Checkbox("L Toe trail (2 sim s)", &trail);
    ImGui::SameLine();
    if (ImGui::Button("W Workspace"))
        workspace_.Toggle(config_);
    ImGui::TextWrapped("Workspace: %s", workspace_.Status().c_str());
    ImGui::TextWrapped(
        "Double-click: select | Ctrl+right drag: force / move object | Ctrl+left: rotate");
    ImGui::TextWrapped(
        "Tab: panel | F: camera tracking | Home: camera reset | Alt+0..5: geom groups");
    if (s.perturb_active)
        ImGui::Text("Applied perturbation: owner=%llu F=%.2f/%.2f/%.2f N",
            (unsigned long long)s.perturb_owner,
            s.perturb_wrench[0],
            s.perturb_wrench[1],
            s.perturb_wrench[2]);
    if (c.mode == 3) {
        const auto error = mju_dist3(s.toe_position.data(), c.tracking_position_reference.data());
        ImGui::Text("%s %s | error %.2f mm | check %.0f%% | torque %s",
            TrackingPlaneName(c.tracking_plane),
            c.tracking_ik_failed ? "IK HOLD" : "",
            error * 1000,
            c.tracking_check_progress * 100,
            c.tracking_torque_saturated ? "SATURATED" : "OK");
        ImGui::Text("ILC %s | updates %llu | skipped %llu | RMS %.2f mm",
            TrackingILCStateName(c.tracking_ilc_state),
            (unsigned long long)c.tracking_ilc_updates,
            (unsigned long long)c.tracking_ilc_skipped_cycles,
            c.tracking_ilc_last_rmse_m * 1000);
        if (ImGui::CollapsingHeader("Torque headroom"))
            ImGui::TextUnformatted(plots_->TorqueHeadroom(limits_, sample_hz_).c_str());
    } else if (c.mode == 2) {
        ImGui::Text("%s | %s | estimate %.2f N / target %.2f N / command %.2f N",
            MagneticPhaseName(c.magnetic.phase),
            MagneticReasonName(c.magnetic.reason),
            c.magnetic.estimated_normal_force,
            c.target_force,
            c.magnetic.force_command);
    } else if (c.mode == 4) {
        ImGui::Text("Joystick %s | speed %.2f m/s | workspace %s | torque %s",
            c.joystick_connected ? "CONNECTED" : "DISCONNECTED",
            c.ball_speed_m_s,
            c.ball_workspace_limited ? "LIMITED" : "OK",
            c.ball_torque_saturated ? "SATURATED" : "OK");
    } else {
        ImGui::Text("%s | toe %.3f / %.3f / %.3f m",
            c.holding ? "HOLD" : ModeName(c.mode),
            s.toe_position[0],
            s.toe_position[1],
            s.toe_position[2]);
    }
    if (!decisions_.empty()) {
        const auto& decision = decisions_.front();
        ImGui::TextColored(decision.approved ? ImVec4{.4f, 1, .6f, 1} : ImVec4{1, .65f, .3f, 1},
            "Input #%llu: %s",
            (unsigned long long)decision.id,
            decision.message.c_str());
    }
    if (ImGui::CollapsingHeader("Input decisions"))
        for (const auto& d : decisions_)
            ImGui::TextWrapped(
                "#%llu %s: %s", (unsigned long long)d.id, d.operation.c_str(), d.message.c_str());
    if (dropped_ || s.transport_drops)
        ImGui::Text("Skipped plot frames: local %llu / IPC %llu",
            (unsigned long long)dropped_,
            (unsigned long long)s.transport_drops);
    ImGui::Separator();
    const auto panels = plots_->Panels(limits_);
    if (panels.empty())
        ImGui::TextUnformatted("Waiting for live plot samples (no synthetic data)");
    const float graph_height =
        std::clamp((height - 400.f) / std::max(1, int(panels.size())), 120.f, 240.f);
    for (const auto* figure : panels)
        Figure(*figure, graph_height);
    ImGui::End();
}

void OperatorPanel::AddGeometry(mjvScene& scene) {
    if (trail)
        plots_->AddToeTrail(scene);
    workspace_.Draw(scene);
    if (snapshot_.control.mode == 4 && scene.ngeom < scene.maxgeom) {
        const double size[3]{.025, .025, .025};
        const float rgba[4]{.25f, .75f, 1, .35f};
        mjv_initGeom(&scene.geoms[scene.ngeom++],
            mjGEOM_SPHERE,
            size,
            snapshot_.control.ball_position.data(),
            nullptr,
            rgba);
    }
}
}  // namespace qhrr05::visualization
