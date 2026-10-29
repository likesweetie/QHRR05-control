#define GLFW_EXPOSE_NATIVE_X11
#include "renderer.hpp"
#include <GLFW/glfw3native.h>
#undef Success  // X11 macro conflicts with Filament RenderableManager::Result.
#include <vulkan/vulkan.h>
#include <mujoco/mjrfilament.h>
#include "render/filament/support/mjrf_ptr.h"
#include "render/filament/support/model_decorations.h"
#include "render/filament/support/model_lights.h"
#include "render/filament/support/model_renderables.h"
#include "render/filament/support/model_resources.h"
#include "render/filament/support/imgui_bridge.h"
#include "render/filament/core/scene_view.h"
#include "render/filament/core/filament_context.h"
#include <filament/Renderer.h>
#include <filament/SwapChain.h>
#include <cmath>
#include <backends/imgui_impl_opengl3.h>
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace qhrr05::visualization {
namespace {
std::filesystem::path asset_root;
std::filesystem::path environment_map;

void RegisterAssets(const std::string& directory, const std::string& ibl) {
    asset_root = std::filesystem::canonical(directory);
    environment_map.clear();
    if (!ibl.empty()) {
        environment_map = std::filesystem::canonical(ibl);
        if (environment_map.extension() != ".ktx" ||
            !std::filesystem::is_regular_file(environment_map))
            throw std::runtime_error("--ibl requires a prefiltered Filament .ktx environment");
    }
    std::cout << "[renderer] environment IBL="
              << (environment_map.empty() ? asset_root / "ibl.ktx" : environment_map) << '\n';
    for (const char* name : {"ibl.ktx", "pbr.filamat", "qhrr_pbr.filamat", "decor.filamat"}) {
        if (!std::filesystem::is_regular_file(asset_root / name))
            throw std::runtime_error(
                "missing Filament runtime resource: " + (asset_root / name).string());
    }
    std::cout << "[renderer] material=QHRR brushed PBR, anisotropy + specular AA\n";
    mjpResourceProvider provider;
    mjp_defaultProvider(&provider);
    provider.prefix = "filament";
    provider.open = [](mjResource* resource) {
        const std::string name(resource->name);
        const auto path = std::filesystem::path(name.substr(name.find(':') + 1));
        if (path.is_absolute() || path.string().find("..") != std::string::npos)
            return 0;
        const auto filename =
            path == "ibl.ktx" && !environment_map.empty()
                ? environment_map
                : asset_root / (path == "pbr.filamat" ? "qhrr_pbr.filamat" : path);
        std::ifstream file(filename, std::ios::binary | std::ios::ate);
        if (!file)
            return 0;
        const auto size = file.tellg();
        if (size <= 0 || size > std::numeric_limits<int>::max())
            return 0;
        auto bytes = std::make_unique<std::vector<char>>(static_cast<std::size_t>(size));
        file.seekg(0);
        if (!file.read(bytes->data(), size))
            return 0;
        resource->data = bytes.release();
        return 1;
    };
    provider.read = [](mjResource* resource, const void** output) {
        const auto& bytes = *static_cast<std::vector<char>*>(resource->data);
        *output = bytes.data();
        return static_cast<int>(bytes.size());
    };
    provider.close = [](mjResource* resource) {
        delete static_cast<std::vector<char>*>(resource->data);
        resource->data = nullptr;
    };
    if (mjp_registerProvider(&provider) < 0)
        throw std::runtime_error("could not register Filament resource provider");
}

class ClassicRenderer final : public Renderer {
public:
    void BeginFrame() override {
        ImGui_ImplOpenGL3_NewFrame();
    }

    ~ClassicRenderer() override {
        Shutdown();
    }

    void Initialize(GLFWwindow* window, const mjModel* model, const std::string&) override {
        window_ = window;
        model_ = model;
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(0);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);
        mjv_makeScene(model_, &scene_, 4000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);
        initialized_ = true;
        if (!ImGui_ImplOpenGL3_Init("#version 130"))
            throw std::runtime_error("Classic ImGui OpenGL initialization failed");
        GLint samples = 0;
        glGetIntegerv(GL_SAMPLES, &samples);
        if (config_.msaa > 1 && samples < config_.msaa)
            throw std::runtime_error("OpenGL window did not provide requested MSAA samples");
        std::cout << "[renderer] classic / " << glGetString(GL_RENDERER) << " MSAA=" << samples
                  << "x\n";
    }

    void Shutdown() override {
        if (initialized_) {
            glfwMakeContextCurrent(window_);
            ImGui_ImplOpenGL3_Shutdown();
            mjr_freeContext(&context_);
            mjv_freeScene(&scene_);
            initialized_ = false;
        }
    }

    void Resize(int width, int height) override {
        full_viewport_ = {0, 0, width, height};
        viewport_ = full_viewport_;
    }

    void SetViewport(const mjrRect& viewport) override {
        viewport_ = viewport;
    }

    void SetPerturb(const mjvPerturb& perturb) override {
        perturb_ = perturb;
    }

    void UpdateCamera(const mjvCamera& camera) override {
        camera_ = camera;
    }

    void Configure(const RenderingConfig& config) override {
        config_ = config;
    }

    void RenderFrame(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) override {
        glfwMakeContextCurrent(window_);
        mjv_updateScene(model_, data, &options, &perturb_, &camera_, mjCAT_ALL, &scene_);
        scene_.flags[mjRND_SHADOW] = config_.shadows;
        for (const auto& geom : debug_geometry) {
            if (scene_.ngeom < scene_.maxgeom)
                scene_.geoms[scene_.ngeom++] = geom;
        }
        mjr_render(viewport_, &scene_, &context_);
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
    }

    std::vector<unsigned char> ReadPixels(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) override {
        RenderFrame(data, options, debug_geometry);
        std::vector<unsigned char> pixels(full_viewport_.width * full_viewport_.height * 3);
        glReadBuffer(GL_FRONT);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0,
            0,
            full_viewport_.width,
            full_viewport_.height,
            GL_RGB,
            GL_UNSIGNED_BYTE,
            pixels.data());
        // OpenGL's origin is bottom-left; the common interface returns top-left rows.
        const auto stride = full_viewport_.width * 3;
        for (int row = 0; row < full_viewport_.height / 2; ++row)
            std::swap_ranges(pixels.begin() + row * stride,
                pixels.begin() + (row + 1) * stride,
                pixels.begin() + (full_viewport_.height - row - 1) * stride);
        return pixels;
    }

private:
    GLFWwindow* window_ = nullptr;
    const mjModel* model_ = nullptr;
    bool initialized_ = false;
    mjvScene scene_{};
    mjrContext context_{};
    mjvCamera camera_{};
    mjrRect viewport_{};
    mjrRect full_viewport_{};
    mjvPerturb perturb_{};
    RenderingConfig config_{};
};

class FilamentRenderer final : public Renderer {
public:
    void BeginFrame() override {
    }

    ~FilamentRenderer() override {
        Shutdown();
    }

    void Initialize(
        GLFWwindow* window, const mjModel* model, const std::string& assets_directory) override {
        model_ = model;
        RegisterAssets(assets_directory, config_.environment_map);
        VerifyVulkanGpu(config_.msaa);
        mjrfContextConfig config;
        mjrf_defaultContextConfig(&config);
        config.graphics_api = mjGRAPHICS_API_VULKAN;
        config.software_rendering = false;
        config.native_window = reinterpret_cast<void*>(glfwGetX11Window(window));
        context_ = mujoco::CreateContext(config);
        if (!context_)
            throw std::runtime_error("could not create Vulkan Filament context");
        mjrRendererInfo info;
        mjrf_getRendererInfo(context_.get(), &info);
        if (std::string(info.backend) != "vulkan")
            throw std::runtime_error("Filament did not create the requested Vulkan backend");
        std::cout << "[renderer] " << info.renderer << " / " << info.backend << '\n';
        mjrfSceneParams scene_config;
        mjrf_defaultSceneParams(&scene_config);
        scene_ = mujoco::CreateScene(context_.get(), scene_config);
        mjrf_configureSceneFromModel(scene_.get(), model_);
        auto* view = mujoco::SceneView::downcast(scene_.get());
        // MuJoCo 3.16 builds the LUT from the previous ColorGradingOptions.
        // Reapply the now-current values so the configured exposure/contrast take effect.
        view->SetColorGradingOptions(view->GetColorGradingOptions());
        // Persistent resources: created/uploaded once, never re-created in RenderFrame.
        resources_ = std::make_unique<mujoco::ModelResources>(model_, context_.get());
        lights_ = std::make_unique<mujoco::ModelLights>(scene_.get(), resources_.get());
        renderables_ = std::make_unique<mujoco::ModelRenderables>(scene_.get(), resources_.get());
        decorations_ =
            std::make_unique<mujoco::ModelDecorations>(context_.get(), scene_.get(), model_, 50000);
        ui_scene_ = mujoco::CreateScene(context_.get(), scene_config);
        ui_bridge_ = std::make_unique<mujoco::ImguiBridge>(context_.get(), ui_scene_.get());
        auto& io = ImGui::GetIO();
        io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
        io.BackendRendererName = "QHRR mjrf Vulkan";
        ImGui::GetPlatformIO().Renderer_TextureMaxWidth = 4096;
        ImGui::GetPlatformIO().Renderer_TextureMaxHeight = 4096;
        std::cout << "[renderer] uploaded model resources once: meshes=" << model_->nmesh
                  << " textures=" << model_->ntex << '\n';
        CreateStudioLights();
        Configure(config_);
    }

    void Shutdown() override {
        // Scene objects must die before scene, and resources before context.
        for (auto& light : studio_lights_)
            mjrf_removeSceneLight(scene_.get(), light.get());
        studio_lights_.clear();
        ui_bridge_.reset();
        ui_scene_.reset();
        decorations_.reset();
        renderables_.reset();
        lights_.reset();
        resources_.reset();
        scene_.reset();
        context_.reset();
    }

    void Resize(int width, int height) override {
        full_viewport_ = {0, 0, width, height};
        viewport_ = full_viewport_;
    }

    void SetViewport(const mjrRect& viewport) override {
        viewport_ = viewport;
    }

    void SetPerturb(const mjvPerturb& perturb) override {
        perturb_ = perturb;
    }

    void UpdateCamera(const mjvCamera& camera) override {
        camera_ = camera;
    }

    void Configure(const RenderingConfig& config) override {
        if ((config.msaa != 1 && config.msaa != 2 && config.msaa != 4 && config.msaa != 8) ||
            !std::isfinite(config.render_scale) || config.render_scale < 1 ||
            config.render_scale > 2)
            throw std::runtime_error("rendering requires MSAA 1/2/4/8 and render scale 1..2");
        const bool changed = !configured_ || config_.msaa != config.msaa ||
                             config_.render_scale != config.render_scale;
        config_ = config;
        if (!scene_ || !changed)
            return;
        auto* view = mujoco::SceneView::downcast(scene_.get())->GetDefaultRenderView();
        auto msaa = view->getMultiSampleAntiAliasingOptions();
        msaa.enabled = config.msaa > 1;
        msaa.sampleCount = config.msaa;
        msaa.customResolve = true;
        view->setMultiSampleAntiAliasingOptions(msaa);
        view->setAntiAliasing(filament::View::AntiAliasing::NONE);
        auto taa = view->getTemporalAntiAliasingOptions();
        taa.enabled = false;
        taa.sharpness = 0;
        view->setTemporalAntiAliasingOptions(taa);
        // Filament supports fixed supersampling up to 2x through equal DRS bounds.
        // Equal bounds explicitly prevent a load-dependent quality reduction.
        auto resolution = view->getDynamicResolutionOptions();
        resolution.enabled = config.render_scale != 1;
        resolution.homogeneousScaling = true;
        resolution.minScale = resolution.maxScale = float(config.render_scale);
        resolution.sharpness = 0;
        resolution.quality = filament::View::QualityLevel::LOW;  // Bilinear downsample, no sharpen.
        view->setDynamicResolutionOptions(resolution);
        const auto applied = view->getDynamicResolutionOptions();
        if (resolution.enabled && (!applied.enabled || applied.minScale != resolution.minScale ||
                                      applied.maxScale != resolution.maxScale))
            throw std::runtime_error("Filament rejected the requested fixed render scale");
        auto quality = view->getRenderQuality();
        quality.hdrColorBuffer = filament::View::QualityLevel::HIGH;
        view->setRenderQuality(quality);
        configured_ = true;
        std::cout << "[renderer] MSAA=" << config.msaa
                  << "x fixed render scale=" << config.render_scale
                  << "x, FXAA/TAA/sharpen=off, HDR=16F" << '\n';
    }

    void RenderFrame(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) override {
        std::array<mjrfRenderRequest, 2> requests{
            Prepare(data, options, debug_geometry), PrepareUi()};
        mjrf_render(context_.get(), requests.data(), requests.size(), nullptr, 0);
        auto* view = mujoco::SceneView::downcast(scene_.get())->GetDefaultRenderView();
        const auto scale = view->getLastDynamicResolutionScale();
        if (std::abs(scale.x - config_.render_scale) > .01 ||
            std::abs(scale.y - config_.render_scale) > .01)
            throw std::runtime_error("Filament changed the requested internal resolution");
        if (reported_width_ != viewport_.width || reported_height_ != viewport_.height) {
            reported_width_ = viewport_.width;
            reported_height_ = viewport_.height;
            std::cout << "[renderer] viewport=" << viewport_.width << 'x' << viewport_.height
                      << " internal~=" << std::lround(viewport_.width * scale.x) << 'x'
                      << std::lround(viewport_.height * scale.y) << '\n';
        }
    }

    std::vector<unsigned char> ReadPixels(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) override {
        std::array<mjrfRenderRequest, 2> requests{
            Prepare(data, options, debug_geometry), PrepareUi()};
        // The pinned mjrf render-target path disables MSAA. Use a readable
        // headless swapchain with the same SceneView path as the window instead.
        auto* context = mujoco::FilamentContext::downcast(context_.get());
        auto* engine = context->GetEngine();
        engine->flushAndWait();
        auto destroy_renderer = [engine](filament::Renderer* r) {
            engine->destroy(r);
        };
        auto destroy_swapchain = [engine](filament::SwapChain* s) {
            engine->destroy(s);
        };
        std::unique_ptr<filament::Renderer, decltype(destroy_renderer)> renderer(
            engine->createRenderer(), destroy_renderer);
        std::unique_ptr<filament::SwapChain, decltype(destroy_swapchain)> swapchain(
            engine->createSwapChain(
                full_viewport_.width, full_viewport_.height, filament::SwapChain::CONFIG_READABLE),
            destroy_swapchain);
        filament::Renderer::ClearOptions clear;
        clear.clear = true;
        clear.discard = true;
        renderer->setClearOptions(clear);
        context->GetMaterialManager()->PrepareToRender();
        for (const auto& request : requests) {
            const mjrfRenderRequest* pointer = &request;
            mujoco::SceneView::downcast(request.scene)->PrepareToRender({&pointer, 1});
        }
        context->GetMaterialManager()->RemoveUnusedMaterials();
        if (!renderer->beginFrame(swapchain.get()))
            throw std::runtime_error("cannot begin antialiased screenshot frame");
        for (const auto& request : requests)
            mujoco::SceneView::downcast(request.scene)->Render(renderer.get(), request);
        std::vector<unsigned char> pixels(full_viewport_.width * full_viewport_.height * 3);
        filament::backend::PixelBufferDescriptor read(pixels.data(),
            pixels.size(),
            filament::backend::PixelDataFormat::RGB,
            filament::backend::PixelDataType::UBYTE,
            1);
        renderer->readPixels(0, 0, full_viewport_.width, full_viewport_.height, std::move(read));
        renderer->endFrame();
        engine->flushAndWait();
        std::cout << "[renderer] screenshot uses window-equivalent MSAA and fixed supersampling\n";
        return pixels;
    }

private:
    void CreateStudioLights() {
        if (mj_name2id(model_, mjOBJ_NUMERIC, "pbr.fill.position") < 0)
            return;
        const auto number = [&](const std::string& name, int count) {
            const int id = mj_name2id(model_, mjOBJ_NUMERIC, name.c_str());
            if (id < 0 || model_->numeric_size[id] != count)
                throw std::runtime_error("missing studio light parameter: " + name);
            return model_->numeric_data + model_->numeric_adr[id];
        };
        for (const auto* label : {"fill", "rim"}) {
            const std::string prefix = std::string("pbr.") + label;
            mjrfLightParams params;
            mjrf_defaultLightParams(&params);
            params.type = mjLIGHT_SPOT;
            params.intensity = number(prefix + ".intensity", 1)[0];
            params.cast_shadow = false;
            params.cutoff = 65;
            params.softness = 1;
            params.bulb_radius = .15;
            std::copy_n(number(prefix + ".color", 3), 3, params.color);
            auto light = mujoco::CreateLight(context_.get(), params);
            if (!light)
                throw std::runtime_error("studio light creation failed: " + prefix);
            float position[3], direction[3];
            std::copy_n(number(prefix + ".position", 3), 3, position);
            std::copy_n(number(prefix + ".direction", 3), 3, direction);
            mjrf_setLightPose(light.get(), position, direction);
            mjrf_addSceneLight(scene_.get(), light.get());
            studio_lights_.push_back(std::move(light));
        }
        std::cout << "[renderer] studio: soft key + fill + rim + HDR IBL, neutral ACES\n";
    }

    mjrfRenderRequest Prepare(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) {
        lights_->Update(data);
        renderables_->SetOptions(options);
        renderables_->Update(data);
        decorations_->Update(
            data, &options, &perturb_, &camera_, viewport_, nullptr, debug_geometry);
        renderables_->MarkAsSelected(perturb_.select > 0 ? mjOBJ_BODY : mjOBJ_UNKNOWN,
            perturb_.select > 0 ? perturb_.select : -1);
        mjrfRenderRequest request;
        mjrf_defaultRenderRequest(&request);
        request.scene = scene_.get();
        request.camera = mjv_camera2GLCamera(model_, data, &camera_);
        request.viewport = viewport_;
        request.shadows = config_.shadows;
        request.post_processing = config_.post_processing;
        request.headlight = model_->vis.headlight.active;
        std::copy_n(model_->vis.headlight.diffuse, 3, request.headlight_color);
        request.headlight_intensity = lights_->GetHeadlightIntensity();
        return request;
    }

    mjrfRenderRequest PrepareUi() {
        ui_bridge_->Update();
        mjrfRenderRequest request;
        mjrf_defaultRenderRequest(&request);
        request.scene = ui_scene_.get();
        request.camera = ui_bridge_->GetCamera(full_viewport_.width, full_viewport_.height);
        request.viewport = full_viewport_;
        request.shadows = request.reflections = request.post_processing = false;
        return request;
    }

    std::vector<mujoco::UniquePtr<mjrfLight>> studio_lights_;
    bool configured_ = false;
    int reported_width_ = 0, reported_height_ = 0;
    const mjModel* model_ = nullptr;
    mujoco::UniquePtr<mjrfContext> context_{nullptr, mjrf_destroyContext};
    mujoco::UniquePtr<mjrfScene> scene_{nullptr, mjrf_destroyScene};
    mujoco::UniquePtr<mjrfScene> ui_scene_{nullptr, mjrf_destroyScene};
    std::unique_ptr<mujoco::ImguiBridge> ui_bridge_;
    std::unique_ptr<mujoco::ModelResources> resources_;
    std::unique_ptr<mujoco::ModelLights> lights_;
    std::unique_ptr<mujoco::ModelRenderables> renderables_;
    std::unique_ptr<mujoco::ModelDecorations> decorations_;
    mjvCamera camera_{};
    mjrRect viewport_{};
    mjrRect full_viewport_{};
    mjvPerturb perturb_{};
    RenderingConfig config_{};
};
}  // namespace

void VerifyVulkanGpu(int samples) {
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "QHRR05 visualizer GPU verification";
    app.apiVersion = VK_API_VERSION_1_1;
    VkInstanceCreateInfo config{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    config.pApplicationInfo = &app;
    VkInstance instance = VK_NULL_HANDLE;
    if (vkCreateInstance(&config, nullptr, &instance) != VK_SUCCESS)
        throw std::runtime_error("Vulkan instance creation failed; check NVIDIA driver/runtime");
    std::uint32_t count = 0;
    VkResult status = vkEnumeratePhysicalDevices(instance, &count, nullptr);
    std::vector<VkPhysicalDevice> devices(count);
    if (status == VK_SUCCESS && count)
        status = vkEnumeratePhysicalDevices(instance, &count, devices.data());
    bool accepted = status == VK_SUCCESS && count == 1;
    if (status != VK_SUCCESS)
        devices.clear();
    for (auto device : devices) {
        VkPhysicalDeviceProperties info;
        vkGetPhysicalDeviceProperties(device, &info);
        std::cout << "[vulkan] device=" << info.deviceName << " vendor=" << info.vendorID
                  << " api=" << VK_VERSION_MAJOR(info.apiVersion) << '.'
                  << VK_VERSION_MINOR(info.apiVersion) << '\n';
        const auto supported_samples =
            info.limits.framebufferColorSampleCounts & info.limits.framebufferDepthSampleCounts;
        if (!(supported_samples & static_cast<VkSampleCountFlagBits>(samples)))
            throw std::runtime_error(
                "NVIDIA Vulkan device does not support requested color/depth MSAA");
        std::cout << "[vulkan] verified color/depth MSAA=" << samples << "x\n";
        accepted &= info.vendorID == 0x10de &&
                    info.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU &&
                    std::string(info.deviceName).find("RTX 5090") != std::string::npos;
    }
    vkDestroyInstance(instance, nullptr);
    if (!accepted)
        throw std::runtime_error(
            "expected exactly one NVIDIA RTX 5090 Vulkan device; set "
            "VK_DRIVER_FILES and VK_ICD_FILENAMES to NVIDIA's ICD. Software/other GPU fallback is forbidden");
}

std::unique_ptr<Renderer> MakeRenderer(const std::string& backend) {
    if (backend == "filament")
        return std::make_unique<FilamentRenderer>();
    if (backend == "classic")
        return std::make_unique<ClassicRenderer>();
    throw std::runtime_error("unknown renderer backend: " + backend);
}
}  // namespace qhrr05::visualization
