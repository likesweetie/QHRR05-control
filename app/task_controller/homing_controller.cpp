#include "homing_controller.hpp"
#include "qhrr05_model.hpp"

#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qhrr05 {
namespace {
void Require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
void CheckKeys(const YAML::Node& node, std::initializer_list<const char*> keys) {
    Require(node && node.IsMap(), "control configuration section must be a map");
    for (const char* key : keys) Require(node[key].IsDefined(), std::string("missing control key: ") + key);
    for (const auto& entry : node) {
        const std::string name = entry.first.as<std::string>();
        Require(std::any_of(keys.begin(), keys.end(), [&](const char* key) { return name == key; }),
                "unknown control key: " + name);
    }
}
JointValues ReadValues(const YAML::Node& node, const char* name, bool allow_scalar = false) {
    JointValues values{};
    if (allow_scalar && node.IsScalar()) {
        values.fill(node.as<double>());
    } else {
        Require(node.IsSequence() && node.size() == kJointCount,
                std::string(name) + " must contain exactly three values");
        for (int i = 0; i < kJointCount; ++i) values[i] = node[i].as<double>();
    }
    for (double value : values) Require(std::isfinite(value), std::string(name) + " must be finite");
    return values;
}
}  // namespace

ControlConfig LoadControlConfig(const std::string& path) {
    Require(!path.empty(), "control configuration path is required");
    const auto root = YAML::LoadFile(path);
    CheckKeys(root, {"robot", "joint_names", "kp", "kd", "bias_compensation", "homing", "plots"});
    Require(root["robot"].as<std::string>() == kRobotName, "only qhrr-05 is supported");
    const auto names = root["joint_names"];
    Require(names.IsSequence() && names.size() == kJointCount, "joint_names must contain three FL joints");
    for (int i = 0; i < kJointCount; ++i) {
        Require(names[i].as<std::string>() == kJointNames[i], "joint_names order does not match qhrr-05");
    }
    CheckKeys(root["homing"], {"initial_position_rad", "position_rad", "duration_sec"});
    CheckKeys(root["plots"], {"history_sec", "sample_hz"});
    ControlConfig result;
    result.kp = ReadValues(root["kp"], "kp", true);
    result.kd = ReadValues(root["kd"], "kd", true);
    result.initial_position = ReadValues(root["homing"]["initial_position_rad"], "initial_position_rad");
    result.home_position = ReadValues(root["homing"]["position_rad"], "position_rad");
    result.duration_sec = root["homing"]["duration_sec"].as<double>();
    result.bias_compensation = root["bias_compensation"].as<bool>();
    result.history_sec = root["plots"]["history_sec"].as<double>();
    result.sample_hz = root["plots"]["sample_hz"].as<double>();
    for (int i = 0; i < kJointCount; ++i) {
        Require(result.kp[i] > 0 && result.kd[i] >= 0, "kp must be positive and kd nonnegative");
    }
    Require(std::isfinite(result.duration_sec) && result.duration_sec > 0, "homing duration must be positive and finite");
    Require(std::isfinite(result.history_sec) && result.history_sec > 0 &&
            std::isfinite(result.sample_hz) && result.sample_hz > 0 &&
            result.sample_hz * result.history_sec <= mjMAXLINEPNT,
            "plot history_sec * sample_hz must be positive, finite and at most 1000");
    return result;
}

void ValidateControlConfig(const ControlConfig& config, const mjModel* model) {
    ValidateModel(model);
    for (int i = 0; i < kJointCount; ++i) {
        Require(model->jnt_limited[i] && model->actuator_ctrllimited[i], "model must define joint and torque limits");
        for (double q : {config.initial_position[i], config.home_position[i]}) {
            Require(std::isfinite(q) && q >= model->jnt_range[2*i] && q <= model->jnt_range[2*i+1],
                    std::string(kJointNames[i]) + " initial or homing position is outside model limits");
        }
    }
    Require(config.sample_hz <= 1.0 / model->opt.timestep, "plot sample_hz exceeds the simulation rate");
}

HomingController::HomingController(ControlConfig config) : config_(std::move(config)) {}

void HomingController::Reset(const mjModel* model, mjData* data) {
    ValidateControlConfig(config_, model);
    mj_resetData(model, data);
    std::copy(config_.initial_position.begin(), config_.initial_position.end(), data->qpos);
    mj_forward(model, data);
}

HomingReference HomingController::Reference(double sim_time) const {
    HomingReference ref;
    const double s = std::clamp(sim_time / config_.duration_sec, 0.0, 1.0);
    const double blend = s*s*s*(10.0 + s*(-15.0 + 6.0*s));
    const double rate = 30.0*s*s*(1.0-s)*(1.0-s) / config_.duration_sec;
    ref.holding = sim_time >= config_.duration_sec;
    for (int i = 0; i < kJointCount; ++i) {
        const double delta = config_.home_position[i] - config_.initial_position[i];
        ref.position[i] = config_.initial_position[i] + blend * delta;
        ref.velocity[i] = rate * delta;
    }
    return ref;
}

HomingReference HomingController::Apply(const mjModel* model, mjData* data) const {
    const auto ref = Reference(data->time);
    mj_forward(model, data);
    for (int i = 0; i < kJointCount; ++i) {
        Require(std::isfinite(data->qpos[i]) && std::isfinite(data->qvel[i]), "non-finite joint state");
        const double command = config_.kp[i] * (ref.position[i] - data->qpos[i]) +
                               config_.kd[i] * (ref.velocity[i] - data->qvel[i]) +
                               (config_.bias_compensation ? data->qfrc_bias[i] : 0.0);
        Require(std::isfinite(command), "non-finite torque command");
        data->ctrl[i] = std::clamp(command, model->actuator_ctrlrange[2*i], model->actuator_ctrlrange[2*i+1]);
    }
    return ref;
}
}  // namespace qhrr05
