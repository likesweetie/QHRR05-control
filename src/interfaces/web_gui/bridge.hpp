#pragma once

#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include <string>
#include <memory>

namespace qhrr05 {
class WebGuiBridge {
public:
    WebGuiBridge(const std::string& shm_name, const std::string& control_config);
    void Run();

private:
    std::string Snapshot();
    shm_utils::Mapping mapping_;
    std::unique_ptr<simulation_ipc::Client> simulation_;
    ObservationOrigin origin_ = ObservationOrigin::Device;
    double maximum_force_ = 0;
    std::string zero_config_json_ = "null";
};
}  // namespace qhrr05
