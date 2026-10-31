#include "renderer.hpp"
#include "snapshot.hpp"
#include "operator_panel.hpp"
#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace {
using namespace qhrr05::visualization;
volatile std::sig_atomic_t stop_requested = 0;

void OnSignal(int) {
    stop_requested = 1;
}

struct Options {
    std::string backend = "filament", model, shm = "/qhrr05", assets, screenshot, ibl;
    std::string control_config = QHRR_DEFAULT_CONTROL_CONFIG;
    double fps = 60, render_scale = 0;
    int msaa = 0;
    int frames = 0, width = 1600, height = 1000, slow_render_ms = 0;
    bool preview = false, pbr_materials = false, hidden = false, no_ui = false;
};

double ParseNumber(const std::string& text) {
    std::size_t count;
    const double value = std::stod(text, &count);
    if (count != text.size())
        throw std::runtime_error("invalid numeric value: " + text);
    return value;
}

int ParseInteger(const std::string& text) {
    std::size_t count;
    const int value = std::stoi(text, &count);
    if (count != text.size())
        throw std::runtime_error("invalid integer value: " + text);
    return value;
}

Options Parse(int argc, char** argv) {
    Options result;
    bool msaa_set = false, scale_set = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto value = [&]() -> std::string {
            if (++i >= argc)
                throw std::runtime_error("missing value for " + arg);
            return argv[i];
        };
        if (arg == "--backend")
            result.backend = value();
        else if (arg == "--model")
            result.model = value();
        else if (arg == "--control-config")
            result.control_config = value();
        else if (arg == "--no-ui")
            result.no_ui = true;
        else if (arg == "--shm-name")
            result.shm = value();
        else if (arg == "--assets")
            result.assets = value();
        else if (arg == "--ibl")
            result.ibl = value();
        else if (arg == "--msaa") {
            result.msaa = ParseInteger(value());
            msaa_set = true;
        } else if (arg == "--render-scale") {
            result.render_scale = ParseNumber(value());
            scale_set = true;
        } else if (arg == "--fps")
            result.fps = ParseNumber(value());
        else if (arg == "--frames")
            result.frames = ParseInteger(value());
        else if (arg == "--width")
            result.width = ParseInteger(value());
        else if (arg == "--height")
            result.height = ParseInteger(value());
        else if (arg == "--slow-render-ms")
            result.slow_render_ms = ParseInteger(value());
        else if (arg == "--screenshot")
            result.screenshot = value();
        else if (arg == "--preview")
            result.preview = true;
        else if (arg == "--pbr-materials")
            result.pbr_materials = true;
        else if (arg == "--hidden")
            result.hidden = true;
        else if (arg == "--help") {
            std::cout
                << "qhrr_visualizer --model MJCF [--backend filament|classic] [--shm-name /qhrr05]\n"
                   "  --preview              Explicit static model preview without SHM or physics\n"
                   "  --pbr-materials        Apply QHRR render-only MJCF material mappings\n"
                   "  --fps 60 --width 1600 --height 1000 --frames N --screenshot FILE.ppm\n"
                   "  --msaa 1|2|4|8 --render-scale 1..2 (Filament default: 8x, 1.333333x)\n"
                   "  --assets DIRECTORY --ibl FILE.ktx --hidden --slow-render-ms N (latency test)\n"
                   "Mouse: left orbit, right pan, middle zoom, wheel zoom; Shift changes axis\n"
                   "Keys: D/H/G/M/T/B task, 1/2/3 tracking plane, Space pause, R reset, Right step\n"
                   "      W workspace, L trail, Tab panel, F camera tracking, Home camera reset\n"
                   "      Alt+0..5 geom groups, C contact wrench, V frames, S shadows\n"
                   "Double-click select; Ctrl+right drag force/mocap, Ctrl+left rotate\n"
                   "--control-config FILE.yaml --no-ui\n";
            std::exit(0);
        } else
            throw std::runtime_error("unknown option: " + arg);
    }
    if (result.model.empty() || !std::isfinite(result.fps) || result.fps < 1 || result.fps > 240 ||
        result.frames < 0 || result.width < 64 || result.height < 64 || result.width > 8192 ||
        result.height > 8192 || result.slow_render_ms < 0 || result.slow_render_ms > 10000)
        throw std::runtime_error(
            "invalid model, FPS, size, frame count or artificial render delay");
    if (!msaa_set)
        result.msaa = result.backend == "filament" ? 8 : 4;
    if (!scale_set)
        result.render_scale = result.backend == "filament" ? 4.0 / 3.0 : 1;
    if ((result.msaa != 1 && result.msaa != 2 && result.msaa != 4 && result.msaa != 8) ||
        !std::isfinite(result.render_scale) || result.render_scale < 1 || result.render_scale > 2)
        throw std::runtime_error("--msaa requires 1/2/4/8 and --render-scale requires 1..2");
    if (result.backend == "classic" && result.render_scale != 1)
        throw std::runtime_error("fixed supersampling is supported only by Filament");
    if (result.assets.empty())
        result.assets =
            (std::filesystem::canonical("/proc/self/exe").parent_path() / "assets").string();
    if (result.backend == "classic" && !result.ibl.empty())
        throw std::runtime_error("--ibl is supported only by the Filament backend");
    return result;
}

// Only the viewer's mjModel is changed. The simulation loads its original scene.xml.
void ApplyPbrMaterials(mjModel* model) {
    for (const auto& [source, target] :
        std::array<std::pair<const char*, const char*>, 5>{{{"metal", "pbr_metal"},
            {"black", "pbr_black"},
            {"white", "pbr_white"},
            {"gray", "pbr_gray"},
            {"ground", "pbr_ground"}}}) {
        const int old_id = mj_name2id(model, mjOBJ_MATERIAL, source);
        const int new_id = mj_name2id(model, mjOBJ_MATERIAL, target);
        if (old_id < 0 || new_id < 0)
            throw std::runtime_error(
                std::string("missing PBR material mapping: ") + source + " -> " + target);
        for (int geom = 0; geom < model->ngeom; ++geom)
            if (model->geom_matid[geom] == old_id)
                model->geom_matid[geom] = new_id;
    }

    const int target = mj_name2id(model, mjOBJ_GEOM, "touch_object");
    const int target_material = mj_name2id(model, mjOBJ_MATERIAL, "pbr_target");
    if (target < 0 || target_material < 0)
        throw std::runtime_error("PBR profile requires touch_object and pbr_target");
    model->geom_matid[target] = target_material;

    // CAD parts retain their geometry; assign surface finishes only on this viewer copy.
    for (const auto& [geom_name, material_name] :
        std::array<std::pair<const char*, const char*>, 3>{{{"qhrr_hip_visual", "pbr_black"},
            {"qhrr_thigh_visual", "pbr_metal"},
            {"FL", "pbr_rubber"}}}) {
        const int geom = mj_name2id(model, mjOBJ_GEOM, geom_name);
        const int material = mj_name2id(model, mjOBJ_MATERIAL, material_name);
        if (geom < 0 || material < 0)
            throw std::runtime_error("studio surface mapping is incomplete");
        model->geom_matid[geom] = material;
    }

    const int background = mj_name2id(model, mjOBJ_TEXTURE, "pbr_studio_background");
    if (background < 0)
        throw std::runtime_error("PBR profile requires pbr_studio_background");
    for (int texture = 0; texture < model->ntex; ++texture)
        if (texture != background && model->tex_type[texture] == mjTEXTURE_SKYBOX)
            model->tex_type[texture] = mjTEXTURE_CUBE;

    const auto numeric = [&](const char* name, int size) {
        const int id = mj_name2id(model, mjOBJ_NUMERIC, name);
        if (id < 0 || model->numeric_size[id] != size)
            throw std::runtime_error(std::string("missing PBR lighting setting: ") + name);
        return model->numeric_data + model->numeric_adr[id];
    };
    const auto* direction = numeric("pbr.key.direction", 3);
    const auto* color = numeric("pbr.key.color", 3);
    const double radius = numeric("pbr.key.radius", 1)[0];
    for (int light = 0; light < model->nlight; ++light) {
        if (model->light_type[light] != mjLIGHT_DIRECTIONAL)
            continue;
        std::copy_n(direction, 3, model->light_dir + 3 * light);
        std::copy_n(color, 3, model->light_diffuse + 3 * light);
        model->light_bulbradius[light] = radius;
    }
}

struct StateBinding {
    std::array<int, 3> qpos;
    int mocap;

    explicit StateBinding(const mjModel* model) {
        if (model->nq != 3 || model->nv != 3 || model->nmocap != 1)
            throw std::runtime_error(
                "current SHM visualization supports QHRR's 3 fixed-base joints and one mocap object only");
        const char* names[] = {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
        for (int i = 0; i < 3; ++i) {
            const int joint = mj_name2id(model, mjOBJ_JOINT, names[i]);
            if (joint < 0 || model->jnt_type[joint] != mjJNT_HINGE)
                throw std::runtime_error(std::string("missing expected hinge: ") + names[i]);
            qpos[i] = model->jnt_qposadr[joint];
        }
        const int object = mj_name2id(model, mjOBJ_BODY, "touch_object");
        if (object < 0 || (mocap = model->body_mocapid[object]) < 0)
            throw std::runtime_error("SHM object_position requires mocap body touch_object");
    }

    void Apply(const Snapshot& state, mjData* data) const {
        if (!state.simulation_available) {
            for (int i = 0; i < 3; ++i) {
                data->qpos[qpos[i]] =
                    state.motor.position_offset[i] + state.motor.feedback[i].position;
                data->qvel[qpos[i]] = state.motor.feedback[i].velocity;
            }
            data->mocap_pos[3 * mocap] = 5;  // No physical object pose sensor exists.
            data->time = state.control.control_time;
            return;
        }
        const auto& snapshot = state.simulation;
        if (!std::isfinite(snapshot.time))
            throw std::runtime_error("non-finite simulation time in snapshot");
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(snapshot.position_gt[i]) ||
                !std::isfinite(snapshot.object_position[i]))
                throw std::runtime_error("non-finite transform in simulation snapshot");
            data->qpos[qpos[i]] = snapshot.position_gt[i];
            data->qvel[qpos[i]] = snapshot.velocity_gt[i];
            data->mocap_pos[3 * mocap + i] = snapshot.object_position[i];
        }
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(snapshot.object_quaternion[i]))
                throw std::runtime_error("non-finite mocap quaternion in simulation IPC");
            data->mocap_quat[4 * mocap + i] = snapshot.object_quaternion[i];
        }
        data->time = snapshot.time;
    }
};

void UpdateVisualState(const mjModel* model, mjData* data) {
    // No mj_step, integration, controller, or contact force reconstruction in the viewer.
    mj_kinematics(model, data);
    mj_comPos(model, data);
    mj_camlight(model, data);
    mj_tendon(model, data);
    mj_transmission(model, data);
}

struct Interaction {
    const mjModel* model;
    mjData* data = nullptr;
    OperatorPanel* panel = nullptr;
    qhrr05::InputClient* input = nullptr;
    mjvCamera camera;
    mjvOption options;
    mjvPerturb perturb{};
    mjvScene pick_scene{};
    RenderingConfig rendering;
    bool wrench = false, simulation_controls = false;
    bool capture = false;
    double x = 0, y = 0, last_click = -1;
    int view_width = 1600, view_height = 1000;

    Interaction(const mjModel* m, mjData* d, OperatorPanel* p, qhrr05::InputClient* i)
        : model(m), data(d), panel(p), input(i) {
        mjv_defaultPerturb(&perturb);
        mjv_defaultScene(&pick_scene);
        mjv_makeScene(model, &pick_scene, 50000);
        mjv_defaultOption(&options);
        options.geomgroup[3] = 0;
        ResetCamera();
    }

    ~Interaction() {
        Release();
        mjv_freeScene(&pick_scene);
    }

    void ResetCamera() {
        mjv_defaultFreeCamera(model, &camera);
    }

    void UpdatePickScene() {
        mjv_updateScene(model, data, &options, &perturb, &camera, mjCAT_ALL, &pick_scene);
    }

    static Interaction& Get(GLFWwindow* window) {
        return *static_cast<Interaction*>(glfwGetWindowUserPointer(window));
    }

    void Release() {
        perturb.active = 0;
        if (input && simulation_controls)
            input->ReleasePerturb();
    }

    void RefreshDrag() {
        if (!input || !simulation_controls || !perturb.active)
            return;
        const char* name = mj_id2name(model, mjOBJ_BODY, perturb.select);
        if (!name || std::strlen(name) >= PerturbTarget{}.body.size())
            throw std::runtime_error("selected body has no valid simulation IPC name");
        PerturbTarget target;
        target.active = perturb.active;
        std::copy(name, name + std::strlen(name), target.body.begin());
        std::copy_n(perturb.refpos, 3, target.reference_position.begin());
        std::copy_n(perturb.refselpos, 3, target.reference_selection.begin());
        std::copy_n(perturb.localpos, 3, target.local_position.begin());
        std::copy_n(perturb.refquat, 4, target.reference_quaternion.begin());
        input->SetPerturb(target);
    }

    int Pick(double x, double y) {
        UpdatePickScene();
        mjtNum point[3];
        int geom = -1, flex = -1, skin = -1;
        const int body = mjv_select(model,
            data,
            &options,
            double(view_width) / view_height,
            x / view_width,
            1 - y / view_height,
            &pick_scene,
            point,
            &geom,
            &flex,
            &skin);
        if (body > 0) {
            perturb.select = body;
            perturb.flexselect = perturb.skinselect = -1;
            mjtNum offset[3];
            mju_sub3(offset, point, data->xpos + 3 * body);
            mju_mulMatTVec(perturb.localpos, data->xmat + 9 * body, offset, 3, 3);
            std::cout << "[operator] selected " << mj_id2name(model, mjOBJ_BODY, body) << std::endl;
        }
        return body;
    }

    static void Focus(GLFWwindow* window, int focused) {
        if (!focused)
            Get(window).Release();
    }

    static void MouseButton(GLFWwindow* window, int button, int action, int mods) {
        auto& ui = Get(window);
        glfwGetCursorPos(window, &ui.x, &ui.y);
        if (action == GLFW_RELEASE) {
            ui.Release();
            return;
        }
        if (action != GLFW_PRESS || ui.x >= ui.view_width || ui.y >= ui.view_height ||
            ImGui::GetIO().WantCaptureMouse)
            return;
        try {
            const double now = glfwGetTime();
            const bool double_click = button == GLFW_MOUSE_BUTTON_LEFT && now - ui.last_click < .3;
            ui.last_click = now;
            if ((mods & GLFW_MOD_CONTROL) || double_click) {
                ui.Pick(ui.x, ui.y);
                if ((mods & GLFW_MOD_CONTROL) && ui.perturb.select <= 0) {
                    ui.panel->Error(
                        "No body selected; double-click a visible robot link or test object");
                    return;
                }
                if ((mods & GLFW_MOD_CONTROL) && ui.perturb.select > 0) {
                    if (!ui.input || !ui.simulation_controls) {
                        ui.panel->Error("Perturbation requires a live simulation");
                        return;
                    }
                    mj_crb(ui.model, ui.data);
                    mj_factorM(ui.model, ui.data);
                    mjv_initPerturb(ui.model, ui.data, &ui.pick_scene, &ui.perturb);
                    ui.perturb.active = button == GLFW_MOUSE_BUTTON_RIGHT  ? mjPERT_TRANSLATE
                                        : button == GLFW_MOUSE_BUTTON_LEFT ? mjPERT_ROTATE
                                                                           : 0;
                    ui.RefreshDrag();
                }
            }
        } catch (const std::exception& error) {
            ui.Release();
            ui.panel->Error(error.what());
        }
    }

    static void Move(GLFWwindow* window, double x, double y) {
        auto& ui = Get(window);
        const double dx = x - ui.x, dy = y - ui.y;
        ui.x = x;
        ui.y = y;
        const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        const bool middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        if ((!left && !right && !middle) || ui.view_height <= 0 ||
            (!ui.perturb.active && (x >= ui.view_width || ImGui::GetIO().WantCaptureMouse)))
            return;
        if (!ui.perturb.active && (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                                      glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS))
            return;
        const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                           glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
        const int action = right  ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V)
                           : left ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V)
                                  : mjMOUSE_ZOOM;
        try {
            if (ui.perturb.active) {
                mjv_movePerturb(ui.model,
                    ui.data,
                    action,
                    dx / ui.view_height,
                    dy / ui.view_height,
                    &ui.pick_scene,
                    &ui.perturb);
                ui.RefreshDrag();
            } else
                mjv_moveCamera(
                    ui.model, action, dx / ui.view_height, dy / ui.view_height, &ui.camera);
        } catch (const std::exception& error) {
            ui.Release();
            ui.panel->Error(error.what());
        }
    }

    static void Scroll(GLFWwindow* window, double, double y) {
        auto& ui = Get(window);
        if (ui.x >= ui.view_width || ImGui::GetIO().WantCaptureMouse)
            return;
        mjv_moveCamera(ui.model, mjMOUSE_ZOOM, 0, -.05 * y, &ui.camera);
    }

    static void Key(GLFWwindow* window, int key, int, int action, int mods) {
        if (action != GLFW_PRESS || ImGui::GetIO().WantTextInput)
            return;
        auto& ui = Get(window);
        try {
            if (key == GLFW_KEY_ESCAPE) {
                ui.Release();
                glfwSetWindowShouldClose(window, true);
                return;
            }
            if ((mods & GLFW_MOD_ALT) && key >= GLFW_KEY_0 && key <= GLFW_KEY_5) {
                ui.options.geomgroup[key - GLFW_KEY_0] = !ui.options.geomgroup[key - GLFW_KEY_0];
                return;
            }
            if (mods & (GLFW_MOD_CONTROL | GLFW_MOD_ALT))
                return;
            if (key == GLFW_KEY_R || key == GLFW_KEY_SPACE)
                ui.Release();
            if (ui.panel->Key(key))
                return;
            if (key == GLFW_KEY_F12)
                ui.capture = true;
            if (key == GLFW_KEY_HOME)
                ui.ResetCamera();
            if (key == GLFW_KEY_C)
                ui.wrench = !ui.wrench;
            if (key == GLFW_KEY_V)
                ui.options.frame = ui.options.frame == mjFRAME_NONE ? mjFRAME_BODY : mjFRAME_NONE;
            if (key == GLFW_KEY_S)
                ui.rendering.shadows = !ui.rendering.shadows;
            if (key == GLFW_KEY_F) {
                if (ui.camera.type == mjCAMERA_TRACKING)
                    ui.ResetCamera();
                else {
                    ui.camera.type = mjCAMERA_TRACKING;
                    ui.camera.trackbodyid = ui.perturb.select > 0
                                                ? ui.perturb.select
                                                : mj_name2id(ui.model, mjOBJ_BODY, "FL_calf");
                }
            }
        } catch (const std::exception& error) {
            ui.panel->Error(error.what());
        }
    }
};

std::vector<mjvGeom> ContactWrench(const SimulationData& snapshot) {
    std::vector<mjvGeom> result;
    if (!snapshot.contact)
        return result;
    const mjtNum identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const mjtNum size[3] = {.004, .004, .004};
    for (int kind = 0; kind < 2; ++kind) {
        const auto& vector = kind ? snapshot.contact_torque : snapshot.contact_force;
        const float rgba[4] = {kind ? .95f : .1f, kind ? .5f : .9f, .25f, 1};
        mjtNum from[3], to[3];
        double norm2 = 0;
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(vector[i]) || !std::isfinite(snapshot.toe_position[i]))
                throw std::runtime_error("non-finite published contact wrench");
            from[i] = snapshot.toe_position[i];
            to[i] = from[i] + vector[i] * (kind ? .1 : .01);
            norm2 += vector[i] * vector[i];
        }
        if (norm2 < 1e-12)
            continue;
        mjvGeom geom{};
        mjv_initGeom(&geom, mjGEOM_ARROW, size, from, identity, rgba);
        mjv_connector(&geom, mjGEOM_ARROW, .004, from, to);
        result.push_back(geom);
    }
    return result;
}

void WritePpm(
    const std::string& name, const std::vector<unsigned char>& pixels, int width, int height) {
    std::ofstream out(name, std::ios::binary);
    if (!out)
        throw std::runtime_error("cannot open screenshot: " + name);
    out << "P6\n" << width << ' ' << height << "\n255\n";
    for (int row = 0; row < height; ++row)
        out.write(reinterpret_cast<const char*>(pixels.data() + row * width * 3), width * 3);
    if (!out)
        throw std::runtime_error("screenshot write failed: " + name);
    std::cout << "[renderer] screenshot=" << name << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::cout << std::unitbuf;
        const auto args = Parse(argc, argv);
        if (mj_version() != 3016000)
            throw std::runtime_error(
                "unexpected runtime MuJoCo version: " + std::string(mj_versionString()));
        std::cout << "[visualizer] MuJoCo " << mj_versionString() << " backend=" << args.backend
                  << " mode=" << (args.preview ? "STATIC PREVIEW (no SHM)" : "SHM observer")
                  << '\n';
        char error[2048];
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(args.model.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model)
            throw std::runtime_error(error);
        if (args.pbr_materials)
            ApplyPbrMaterials(model.get());
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        if (!data)
            throw std::runtime_error("cannot allocate renderer mjData");
        if (model->nkey > 0)
            mj_resetDataKeyframe(model.get(), data.get(), 0);
        UpdateVisualState(model.get(), data.get());
        std::unique_ptr<SnapshotSource> source;
        std::unique_ptr<StateBinding> binding;
        if (!args.preview) {
            binding = std::make_unique<StateBinding>(model.get());
            source = std::make_unique<SnapshotSource>(args.shm);
        }
        glfwSetErrorCallback([](int code, const char* text) {
            std::cerr << "[glfw] " << code << ": " << text << '\n';
        });
        if (!glfwInit())
            throw std::runtime_error("GLFW initialization failed (X11 display required)");

        struct GlfwLifetime {
            ~GlfwLifetime() {
                glfwTerminate();
            }
        } glfw_lifetime;

        if (args.backend == "filament")
            glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        if (args.backend == "classic")
            glfwWindowHint(GLFW_SAMPLES, args.msaa);
        if (args.hidden)
            glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> window(
            glfwCreateWindow(args.width, args.height, "QHRR05 visualization", nullptr, nullptr),
            glfwDestroyWindow);
        if (!window)
            throw std::runtime_error("cannot create visualization window");
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGui::StyleColorsDark();
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::GetIO().LogFilename = nullptr;

        struct ImGuiLifetime {
            bool platform = false;

            ~ImGuiLifetime() {
                if (platform)
                    ImGui_ImplGlfw_Shutdown();
                ImGui::DestroyContext();
            }
        } imgui_lifetime;

        std::unique_ptr<qhrr05::InputClient> input;
        if (!args.preview)
            input = std::make_unique<qhrr05::InputClient>(args.shm);
        OperatorPanel panel(args.control_config, input.get(), !args.preview && !args.no_ui);
        Interaction ui(model.get(), data.get(), &panel, input.get());
        glfwSetWindowUserPointer(window.get(), &ui);
        glfwSetMouseButtonCallback(window.get(), Interaction::MouseButton);
        glfwSetCursorPosCallback(window.get(), Interaction::Move);
        glfwSetScrollCallback(window.get(), Interaction::Scroll);
        glfwSetKeyCallback(window.get(), Interaction::Key);
        glfwSetWindowFocusCallback(window.get(), Interaction::Focus);
        const bool platform_ok = args.backend == "filament"
                                     ? ImGui_ImplGlfw_InitForVulkan(window.get(), true)
                                     : ImGui_ImplGlfw_InitForOpenGL(window.get(), true);
        if (!platform_ok)
            throw std::runtime_error("ImGui GLFW initialization failed");
        imgui_lifetime.platform = true;
        auto renderer = MakeRenderer(args.backend);
        ui.rendering.environment_map = args.ibl;
        ui.rendering.msaa = args.msaa;
        ui.rendering.render_scale = args.render_scale;
        renderer->Configure(ui.rendering);
        renderer->Initialize(window.get(), model.get(), args.assets);
        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);
        const auto period = std::chrono::duration<double>(1 / args.fps);
        const auto start = std::chrono::steady_clock::now();
        auto title_time = start;
        auto next_frame = start;
        Snapshot snapshot;
        bool stale_reported = false;
        int frames = 0;
        std::vector<mjvGeom> debug_geometry;
        int width = args.width, height = args.height;
        while (!stop_requested && !glfwWindowShouldClose(window.get())) {
            int window_width, window_height;
            glfwGetWindowSize(window.get(), &window_width, &window_height);
            ui.view_width = panel.CanvasWidth(window_width);
            ui.view_height = window_height;
            glfwPollEvents();
            if (source) {
                snapshot = source->Latest();
                ui.simulation_controls = snapshot.simulation_available;
                panel.Update(snapshot, source->Drain(), source->Dropped());
                if (snapshot.failed)
                    throw std::runtime_error("device session failed");
                if (snapshot.stopping || snapshot.finished)
                    break;
                const bool stale = !snapshot.pose_valid || !snapshot.published_ns ||
                                   shm_utils::MonotonicNs() - snapshot.published_ns > 500000000;
                if (stale && !stale_reported)
                    std::cerr
                        << "[visualizer] STALE/WAITING: no valid current joint pose; pose is held\n";
                if (!stale && stale_reported)
                    std::cout << "[visualizer] valid joint observations resumed\n";
                stale_reported = stale;
                if (snapshot.pose_valid) {
                    binding->Apply(snapshot, data.get());
                    UpdateVisualState(model.get(), data.get());
                }
            }
            glfwGetFramebufferSize(window.get(), &width, &height);
            if (width > 0 && height > 0) {
                debug_geometry = ui.wrench && source && snapshot.simulation_available
                                     ? ContactWrench(snapshot.simulation)
                                     : std::vector<mjvGeom>{};
                ui.RefreshDrag();
                ui.UpdatePickScene();
                ui.pick_scene.ngeom = 0;
                panel.AddGeometry(ui.pick_scene);
                debug_geometry.insert(debug_geometry.end(),
                    ui.pick_scene.geoms,
                    ui.pick_scene.geoms + ui.pick_scene.ngeom);
                renderer->Resize(width, height);
                renderer->SetViewport({0,
                    0,
                    int(double(panel.CanvasWidth(window_width)) * width / window_width),
                    height});
                renderer->SetPerturb(ui.perturb);
                renderer->BeginFrame();
                ImGui_ImplGlfw_NewFrame();
                ImGui::NewFrame();
                panel.Draw(window_width, window_height);
                renderer->UpdateCamera(ui.camera);
                renderer->Configure(ui.rendering);
                if (args.slow_render_ms)
                    std::this_thread::sleep_for(std::chrono::milliseconds(args.slow_render_ms));
                renderer->RenderFrame(data.get(), ui.options, debug_geometry);
                if (ui.capture) {
                    const auto path = args.screenshot.empty()
                                          ? "artifacts/visualization-capture.ppm"
                                          : args.screenshot;
                    const auto parent = std::filesystem::path(path).parent_path();
                    if (!parent.empty())
                        std::filesystem::create_directories(parent);
                    WritePpm(path,
                        renderer->ReadPixels(data.get(), ui.options, debug_geometry),
                        width,
                        height);
                    ui.capture = false;
                }
                ++frames;
                if (args.frames && frames >= args.frames)
                    break;
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - title_time > std::chrono::milliseconds(500)) {
                std::ostringstream title;
                title << "QHRR05 | " << args.backend << " | "
                      << (args.preview        ? "STATIC PREVIEW"
                             : stale_reported ? "STALE / POSE HELD"
                                              : "LIVE")
                      << " | t=" << data->time << " | D/H/G/M/T/B task | Space pause | Tab graphs";
                glfwSetWindowTitle(window.get(), title.str().c_str());
                title_time = now;
            }
            next_frame += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            if (next_frame < now)
                next_frame = now;
            std::this_thread::sleep_until(next_frame);
        }
        if (!args.screenshot.empty() && width > 0 && height > 0)
            WritePpm(args.screenshot,
                renderer->ReadPixels(data.get(), ui.options, debug_geometry),
                width,
                height);
        ui.Release();
        renderer->Shutdown();
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::cout << "[visualizer] frames=" << frames << " elapsed_s=" << elapsed;
        if (source)
            std::cout << " snapshots=" << source->Reads() << " busy_skips=" << source->Misses()
                      << " max_shm_hold_ns=" << source->MaxHoldNs()
                      << " simulation_time=" << data->time;
        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[visualizer] ERROR: " << error.what() << '\n';
        return 1;
    }
}
