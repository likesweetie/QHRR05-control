#include "homing_controller.hpp"

#include <GLFW/glfw3.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void OnSignal(int) { stop_requested = 1; }
struct Options {
    std::string model;
    std::string config;
    std::string csv;
    bool headless = false;
    double duration = 0;
};

Options Parse(int argc, char** argv) {
    Options result;
    const char* robot = std::getenv("ROBOT_NAME");
    if (robot && std::string(robot) != qhrr05::kRobotName) throw std::runtime_error("only qhrr-05 is supported");
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--headless") { result.headless = true; continue; }
        if (i + 1 >= argc) throw std::runtime_error("missing value for " + arg);
        const std::string value = argv[++i];
        if (arg == "--model") result.model = value;
        else if (arg == "--control-config") result.config = value;
        else if (arg == "--csv") result.csv = value;
        else if (arg == "--duration") {
            std::size_t used = 0;
            result.duration = std::stod(value, &used);
            if (used != value.size() || !std::isfinite(result.duration) || result.duration <= 0)
                throw std::runtime_error("duration must be positive and finite");
        } else throw std::runtime_error("unknown argument: " + arg);
    }
    if (result.config.empty()) throw std::runtime_error("--control-config is required");
    if (result.model.empty()) throw std::runtime_error("--model is required");
    if (result.headless && result.duration <= 0) throw std::runtime_error("--headless requires --duration");
    return result;
}

struct Sample {
    double time = 0;
    qhrr05::JointValues position{};
    qhrr05::JointValues reference{};
    qhrr05::JointValues torque{};
};
Sample Capture(const mjData* data, const qhrr05::HomingController& controller) {
    Sample sample;
    sample.time = data->time;
    sample.reference = controller.Reference(data->time).position;
    for (int i = 0; i < qhrr05::kJointCount; ++i) {
        sample.position[i] = data->qpos[i];
        sample.torque[i] = data->actuator_force[i];
    }
    return sample;
}

class Recorder {
public:
    explicit Recorder(const std::string& path) {
        if (path.empty()) return;
        if (std::filesystem::exists(path)) throw std::runtime_error("CSV already exists: " + path);
        file_.open(path);
        if (!file_) throw std::runtime_error("cannot open CSV: " + path);
        file_ << "run_id,sim_time";
        for (const auto& field : {"q", "q_ref", "torque"})
            for (const char* joint : qhrr05::kJointNames) file_ << ',' << field << '_' << joint;
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

struct View {
    mjvCamera camera{};
    const mjModel* model = nullptr;
    mjvScene* scene = nullptr;
    bool paused = false;
    bool restart = false;
    double mouse_x = 0;
    double mouse_y = 0;
};
void Key(GLFWwindow* window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    auto& view = *static_cast<View*>(glfwGetWindowUserPointer(window));
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
    if (key == GLFW_KEY_SPACE) view.paused = !view.paused;
    if (key == GLFW_KEY_R) view.restart = true;
}
void Cursor(GLFWwindow* window, double x, double y) {
    auto& view = *static_cast<View*>(glfwGetWindowUserPointer(window));
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    const double dx = x-view.mouse_x, dy = y-view.mouse_y;
    view.mouse_x = x;
    view.mouse_y = y;
    if (height <= 0) return;
    const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    if (left || right) mjv_moveCamera(view.model, right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V,
                                    dx/height, dy/height, view.scene, &view.camera);
}
void Scroll(GLFWwindow* window, double, double dy) {
    auto& view = *static_cast<View*>(glfwGetWindowUserPointer(window));
    mjv_moveCamera(view.model, mjMOUSE_ZOOM, 0, -.05*dy, view.scene, &view.camera);
}

int Run(const Options& options) {
    auto config = qhrr05::LoadControlConfig(options.config);
    char error[2048] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(options.model.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) throw std::runtime_error(std::string("model load failed: ") + error);
    qhrr05::ValidateControlConfig(config, model.get());
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
    if (!data) throw std::runtime_error("could not allocate MuJoCo data");
    qhrr05::HomingController controller(config);
    controller.Reset(model.get(), data.get());
    Recorder recorder(options.csv);
    auto plots = std::make_unique<LivePlots>(config.history_sec);
    std::cout << "[homing] robot=qhrr-05 config=" << options.config
              << " duration=" << config.duration_sec << "s goal=[" << config.home_position[0]
              << ',' << config.home_position[1] << ',' << config.home_position[2] << "] rad"
              << (options.headless ? " headless" : " GUI") << std::endl;

    GLFWwindow* window = nullptr;
    View view;
    mjvScene scene{};
    mjrContext context{};
    mjvOption visual_option;
    mjv_defaultOption(&visual_option);
    bool graphics_ready = false;
    auto cleanup = [&] {
        if (graphics_ready) { mjr_freeContext(&context); mjv_freeScene(&scene); }
        if (window) glfwDestroyWindow(window);
        if (!options.headless) glfwTerminate();
    };
    try {
        if (!options.headless) {
            glfwSetErrorCallback([](int code, const char* text) {
                std::cerr << "[GLFW " << code << "] " << text << '\n';
            });
            if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; no headless fallback was attempted");
            window = glfwCreateWindow(1600, 900, "qhrr-05 | Homing control and live telemetry", nullptr, nullptr);
            if (!window) throw std::runtime_error("could not create MuJoCo window");
            glfwMakeContextCurrent(window);
            glfwSwapInterval(1);
            mjv_defaultCamera(&view.camera);
            view.camera.lookat[0] = .05; view.camera.lookat[1] = .04; view.camera.lookat[2] = .30;
            view.camera.distance = 1.05; view.camera.azimuth = 115; view.camera.elevation = -20;
            mjv_defaultScene(&scene);
            mjr_defaultContext(&context);
            mjv_makeScene(model.get(), &scene, 2000);
            mjr_makeContext(model.get(), &context, mjFONTSCALE_100);
            graphics_ready = true;
            visual_option.geomgroup[3] = 0;
            visual_option.geomgroup[4] = 0;
            view.model = model.get(); view.scene = &scene;
            glfwSetWindowUserPointer(window, &view);
            glfwSetKeyCallback(window, Key);
            glfwSetCursorPosCallback(window, Cursor);
            glfwSetScrollCallback(window, Scroll);
        }
        const double dt = model->opt.timestep;
        double next_sample = 0;
        double next_step = options.headless ? 0 : glfwGetTime();
        bool holding_reported = false;
        while (!stop_requested && (!window || !glfwWindowShouldClose(window))) {
            if (view.restart) {
                controller.Reset(model.get(), data.get());
                plots->Clear();
                recorder.Restart();
                next_sample = 0;
                holding_reported = false;
                next_step = options.headless ? 0 : glfwGetTime();
                view.restart = false;
            }
            const double now = options.headless ? 0 : glfwGetTime();
            if (view.paused) next_step = now;
            int steps = 0;
            while (!view.paused && (options.headless || now >= next_step) && steps < 100) {
                controller.Apply(model.get(), data.get());
                mj_step(model.get(), data.get());
                mj_forward(model.get(), data.get());
                for (int i = 0; i < mjNWARNING; ++i)
                    if (data->warning[i].number) throw std::runtime_error("MuJoCo dynamics warning during homing");
                if (data->time + 1e-9 >= next_sample) {
                    const auto sample = Capture(data.get(), controller);
                    plots->Push(sample);
                    recorder.Write(sample);
                    next_sample = data->time + 1.0/config.sample_hz;
                }
                if (!holding_reported && data->time >= config.duration_sec) {
                    std::cout << "[homing] HOLD: maintaining configured homing pose" << std::endl;
                    holding_reported = true;
                }
                next_step += dt;
                ++steps;
                if (options.duration > 0 && data->time + 1e-9 >= options.duration) break;
            }
            if (options.duration > 0 && data->time + 1e-9 >= options.duration) break;
            if (!options.headless && steps == 100 && now >= next_step) {
                std::cerr << "[homing] rendering is behind wall time; resynchronizing display clock\n";
                next_step = now;
            }
            if (window) {
                int width, height;
                glfwGetFramebufferSize(window, &width, &height);
                if (width > 0 && height > 0) {
                    const int model_width = static_cast<int>(width * .62);
                    const mjrRect viewport{0, 0, model_width, height};
                    mjv_updateScene(model.get(), data.get(), &visual_option, nullptr, &view.camera, mjCAT_ALL, &scene);
                    mjr_render(viewport, &scene, &context);
                    double max_error = 0;
                    for (int i = 0; i < 3; ++i)
                        max_error = std::max(max_error, std::abs(config.home_position[i]-data->qpos[i]));
                    char status[256];
                    std::snprintf(status, sizeof(status), "%s  |  t=%.2f s  |  home error max=%.4f rad",
                        view.paused ? "PAUSED" : (holding_reported ? "HOLD" : "HOMING"), data->time, max_error);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "qhrr-05", status, &context);
                    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                        "SPACE: pause/resume   R: restart homing   ESC: exit\nMouse: orbit/pan   Wheel: zoom", nullptr, &context);
                    plots->Draw({model_width, 0, width-model_width, height}, &context);
                    glfwSwapBuffers(window);
                }
                glfwPollEvents();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        recorder.Write(Capture(data.get(), controller));
        recorder.Finish();
        std::cout << "[homing] stopped at t=" << data->time << " q=[" << data->qpos[0]
                  << ',' << data->qpos[1] << ',' << data->qpos[2] << "]" << std::endl;
        cleanup();
        return 0;
    } catch (...) { cleanup(); throw; }
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "task_controller --model MODEL --control-config YAML [--headless --duration SEC] [--csv PATH]\n";
        return 0;
    }
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);
    try { return Run(Parse(argc, argv)); }
    catch (const std::exception& error) {
        std::cerr << "[homing] ERROR: " << error.what() << '\n';
        return 1;
    }
}
