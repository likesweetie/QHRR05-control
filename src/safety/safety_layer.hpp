#pragma once

#include "ipc/shared_memory.hpp"
#include <string>

namespace qhrr05 {
class SafetyLayer {
public:
    explicit SafetyLayer(const std::string& config_path);
    void Validate(const ControlData& input, bool stopping, bool failed) const;
    void ValidateAnticoggingPhase(int axis, int order, double delta) const;
    void ValidateAnticoggingAmplitude(int axis, int order, double delta) const;
    void ValidateRmdDampingConstant(double value) const;

private:
    double maximum_force_ = 0;
};
}  // namespace qhrr05
