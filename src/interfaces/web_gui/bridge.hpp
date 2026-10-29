#pragma once

#include "ipc/shm_utils.hpp"
#include "ipc/simulation/channel.hpp"
#include <string>

namespace qhrr05 {
class WebGuiBridge {
public:
    WebGuiBridge(const std::string& shm_name, const std::string& control_config);
    void Run();

private:
    std::string Snapshot();
    shm_utils::Mapping mapping_;
    simulation_ipc::Client simulation_;
    double maximum_force_ = 0;
};
}  // namespace qhrr05
