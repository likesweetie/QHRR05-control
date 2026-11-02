#pragma once

#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include "robot_model/backend.hpp"
#include <string>
#include <memory>
#include <array>
#include <vector>

namespace qhrr05 {
class WebGuiBridge {
public:
    WebGuiBridge(const std::string& shm_name, const std::string& control_config);
    void Run();

private:
    std::string Snapshot();
    void SampleTelemetry(bool cogging, bool tracking);
    shm_utils::Mapping mapping_;
    std::unique_ptr<simulation_ipc::Client> simulation_;
    ObservationOrigin origin_ = ObservationOrigin::Device;
    double maximum_force_ = 0;
    std::string zero_config_json_ = "null";
    std::array<std::uint64_t, 3> cogging_sequence_{};
    std::vector<std::string> cogging_samples_;
    std::uint64_t cogging_read_misses_ = 0;
    std::string control_config_;
    std::unique_ptr<ModelBackend> tracking_model_;
    std::array<std::uint64_t, 3> tracking_sequence_{};
    std::vector<std::string> tracking_samples_;
    std::uint64_t tracking_read_misses_ = 0, tracking_motion_sequence_ = 0;
    std::uint64_t tracking_generation_ = 0;
    int tracking_plane_ = 0;
    bool tracking_active_ = false;
    std::string tracking_problem_ = "트래킹 실행 대기";
};
}  // namespace qhrr05
