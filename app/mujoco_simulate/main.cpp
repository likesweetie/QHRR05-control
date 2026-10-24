#include <array>
#include "simulation.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include "telemetry.hpp"
#include "simulate/simulate.h"
#include "simulate/glfw_adapter.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;
void OnSignal(int) { stopped = 1; }
struct Options {
    std::string model, config, csv;
    int fd = -1;
    bool headless = false;
    double duration = 0;
};
Options Parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") { options.headless = true; continue; }
        if (i+1 >= argc) throw std::runtime_error("missing value for " + arg);
        const std::string value = argv[++i];
        if (arg == "--model") options.model = value;
        else if (arg == "--control-config") options.config = value;
        else if (arg == "--csv") options.csv = value;
        else if (arg == "--io-fd") {
            std::size_t count = 0;
            options.fd = std::stoi(value, &count);
            if (count != value.size()) throw std::runtime_error("invalid --io-fd");
        } else if (arg == "--duration") {
            std::size_t count = 0;
            options.duration = std::stod(value, &count);
            if (count != value.size() || !std::isfinite(options.duration) || options.duration <= 0)
                throw std::runtime_error("duration must be positive and finite");
        } else throw std::runtime_error("unknown viewer argument: " + arg);
    }
    if (options.config.empty() || options.model.empty()) throw std::runtime_error("--model and --control-config are required");
    if (options.headless && options.duration <= 0) throw std::runtime_error("--headless requires --duration");
    return options;
}
}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    try {
        const auto options = Parse(argc, argv);
        const auto config = YAML::LoadFile(options.config);
        if (config["robot"].as<std::string>() != "qhrr-05") throw std::runtime_error("only qhrr-05 is supported");
        const double sample_hz = config["plots"]["sample_hz"].as<double>();
        const double history_sec = config["plots"]["history_sec"].as<double>();
        if (!std::isfinite(sample_hz) || !std::isfinite(history_sec) || sample_hz <= 0 ||
            history_sec <= 0 || sample_hz*history_sec > mjMAXLINEPNT)
            throw std::runtime_error("invalid plot sampling or history length");
        if (options.fd < 0 || ::fcntl(options.fd, F_GETFD) < 0) throw std::runtime_error("invalid control descriptor");
        char error[2048] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(options.model.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(std::string("model load failed: ") + error);
        if (model->nq != 3 || model->nv != 3 || model->nu != 3)
            throw std::runtime_error("expected three joint positions, velocities and motor inputs");
        const auto initial = qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        const auto home = qhrr05::ReadPosition(config["homing"]["position_rad"], model.get());
        if (sample_hz > 1.0/model->opt.timestep) throw std::runtime_error("plot rate exceeds simulation rate");
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("could not allocate model data");
        qhrr05::ResetSimulation(initial, model.get(), data.get());
        qhrr05::viewer::Recorder recorder(options.csv);
        auto plots = std::make_unique<qhrr05::viewer::LivePlots>(history_sec);
        std::array<double, 3> reference = initial;
        const double homing_duration = config["homing"]["duration_sec"].as<double>();
        double next_sample = 0;
        bool holding_reported = false;
        const auto reset = [&] {
            qhrr05::ResetSimulation(initial, model.get(), data.get());
            recorder.Restart(); plots->Clear();
            reference = initial;
            next_sample = 0;
            holding_reported = false;
        };
        const auto sample = [&] {
            qhrr05::viewer::Sample frame;
            frame.time = data->time;
            frame.reference = reference;
            for (int i = 0; i < 3; ++i) {
                frame.position[i] = data->qpos[i];
                frame.torque[i] = data->actuator_force[i];
            }
            plots->Push(frame);
            recorder.Write(frame);
        };
        const auto step = [&] {
            qhrr05::ExchangeControl(options.fd, model.get(), data.get(), reference);
            mj_step(model.get(), data.get());
            mj_forward(model.get(), data.get());
            for (int i = 0; i < mjNWARNING; ++i)
                if (data->warning[i].number) throw std::runtime_error("MuJoCo dynamics warning");
            if (data->time + 1e-9 >= next_sample) {
                sample();
                next_sample = data->time + 1.0/sample_hz;
            }
            if (!holding_reported && data->time >= homing_duration) {
                std::cout << "[homing] HOLD: maintaining configured homing pose" << std::endl;
                holding_reported = true;
            }
        };
        const auto finished = [&] { return options.duration > 0 && data->time+1e-9 >= options.duration; };
        std::cout << "[simulate] qhrr-05 viewer + separate control superloop" << std::endl;
        if (options.headless) {
            while (!stopped && !finished()) step();
        } else {
            mjvCamera camera;
            mjv_defaultCamera(&camera);
            mjvOption visual;
            mjv_defaultOption(&visual);
            mjvPerturb perturb;
            mjv_defaultPerturb(&perturb);
            auto sim = std::make_unique<mujoco::Simulate>(
                std::make_unique<mujoco::GlfwAdapter>(), &camera, &visual, &perturb, false);
            std::atomic<bool> failed{false};
            sim->user_panel_percent_ = 40;
            // Keyboard events and all model mutations use simulate's existing mutex.
            const auto guarded = [&](auto operation) {
                try { operation(); }
                catch (const std::exception& exception) {
                    std::cerr << "[simulate] ERROR: " << exception.what() << std::endl;
                    failed = true;
                    sim->exitrequest.store(1);
                }
            };
            sim->user_reset_callback_ = [&] { guarded(reset); };
            sim->user_step_callback_ = [&] { guarded(step); };
            sim->user_key_callback_ = [&](int key) {
                if (key == mjKEY_ESCAPE) { sim->exitrequest.store(1); return true; }
                if (key == 'R' || key == 'r') { sim->pending_.reset = true; return true; }
                return false;
            };
            sim->user_render_callback_ = [&] {
                const mujoco::MutexLock lock(sim->mtx);
                if (stopped) sim->exitrequest.store(1);
                const auto rect = sim->uistate.rect[3];
                const int width = rect.width*sim->user_panel_percent_/100;
                plots->Draw({rect.left+rect.width-width, rect.bottom, width, rect.height},
                            &sim->platform_ui->mjr_context());
                double max_error = 0;
                for (int i = 0; i < 3; ++i)
                    max_error = std::max(max_error, std::abs(home[i]-data->qpos[i]));
                char status[256];
                std::snprintf(status, sizeof(status), "%s | t=%.2f s | home error=%.4f rad",
                    !sim->run ? "PAUSED" : (holding_reported ? "HOLD" : "HOMING"), data->time, max_error);
                const mjrRect scene_rect{rect.left, rect.bottom, rect.width-width, rect.height};
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, scene_rect, "qhrr-05", status,
                            &sim->platform_ui->mjr_context());
                mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, scene_rect,
                    "SPACE: pause/resume   R: rehome   ESC: exit", nullptr, &sim->platform_ui->mjr_context());
            };
            std::thread physics([&] {
                sim->Load(model.get(), data.get(), options.model.c_str());
                auto next_tick = std::chrono::steady_clock::now();
                while (!sim->exitrequest.load() && !stopped) {
                    double tick_seconds = 0;
                    {
                        const mujoco::MutexLock lock(sim->mtx);
                        if (finished()) break;
                        const bool dropped = sim->droploadrequest.exchange(false);
                        const bool reloaded = sim->uiloadrequest.exchange(0) != 0;
                        if (dropped || reloaded) {
                            std::snprintf(sim->load_error, sizeof(sim->load_error),
                                "Only the configured qhrr-05 model is supported; restart daemon to reload.");
                            std::cerr << "[simulate] " << sim->load_error << std::endl;
                        }
                        guarded([&] {
                            if (sim->run) {
                                step();
                                sim->AddToHistory();
                            } else {
                                mj_forward(model.get(), data.get());
                            }
                        });
                        const double slowdown = 100.0/sim->percentRealTime[sim->real_time_index];
                        tick_seconds = model->opt.timestep*slowdown;
                    }
                    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(tick_seconds));
                    const auto now = std::chrono::steady_clock::now();
                    if (next_tick < now-std::chrono::milliseconds(100)) {
                        std::cerr << "[simulate] physics is behind wall time; resynchronizing pacing\n";
                        next_tick = now;
                    }
                    std::this_thread::sleep_until(next_tick);
                }
                sim->exitrequest.store(1);
            });
            sim->RenderLoop();
            sim->exitrequest.store(1);
            physics.join();
            if (failed && !stopped) throw std::runtime_error("simulation stopped after IO/control failure");
        }
        sample();
        recorder.Finish();
        ::shutdown(options.fd, SHUT_WR);
        ::close(options.fd);
        std::cout << "[simulate] stopped at t=" << data->time << std::endl;
        return 0;
    } catch (const std::exception& error) {
        if (stopped) return 0;
        std::cerr << "[simulate] ERROR: " << error.what() << '\n';
        return 1;
    }
}
