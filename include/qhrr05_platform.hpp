#pragma once

#include <array>

namespace qhrr05 {

inline constexpr char kRobotName[] = "qhrr-05";
inline constexpr int kJointCount = 3;
inline constexpr std::array<const char*, kJointCount> kJointNames = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"};
inline constexpr std::array<const char*, kJointCount> kActuatorNames = {
    "FL_hip", "FL_thigh", "FL_calf"};
inline constexpr std::array<const char*, kJointCount> kLinkNames = {
    "FL_hip", "FL_thigh", "FL_calf"};

inline bool HasDimensions(int nq, int nv, int nu) {
    return nq == kJointCount && nv == kJointCount && nu == kJointCount;
}

}  // namespace qhrr05
