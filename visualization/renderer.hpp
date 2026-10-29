#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace qhrr05::visualization {
struct RenderingConfig {
    bool shadows = true;
    bool post_processing = true;
    int msaa = 8;
    double render_scale = 4.0 / 3.0;  // Fixed supersampling; never reduced to meet FPS.
    std::string environment_map;  // Prefiltered HDR environment in Filament KTX format.
};

class Renderer {
public:
    virtual ~Renderer() = default;
    virtual void Initialize(
        GLFWwindow* window, const mjModel* model, const std::string& assets_directory) = 0;
    virtual void Shutdown() = 0;
    virtual void BeginFrame() = 0;
    virtual void Resize(int width, int height) = 0;
    virtual void SetViewport(const mjrRect& viewport) = 0;
    virtual void SetPerturb(const mjvPerturb& perturb) = 0;
    virtual void UpdateCamera(const mjvCamera& camera) = 0;
    virtual void Configure(const RenderingConfig& config) = 0;
    virtual void RenderFrame(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) = 0;
    // RGB rows start at the top left. Readback may wait for the GPU, exclusively
    // in the visualization process.
    virtual std::vector<unsigned char> ReadPixels(
        mjData* data, const mjvOption& options, std::span<const mjvGeom> debug_geometry) = 0;
};

std::unique_ptr<Renderer> MakeRenderer(const std::string& backend);
void VerifyVulkanGpu(int samples);
}  // namespace qhrr05::visualization
