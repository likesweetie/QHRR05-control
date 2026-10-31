#include <array>
#include "simulation/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <random>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace qhrr05 {
VirtualMotors::VirtualMotors(const std::string& hal_config_path,
    const std::string& simulation_config_path,
    const std::array<double, 3>& encoder_origin,
    ShmData& shared)
    : iq_sensors_{VirtualIqSensor(simulation_config_path, 0),
          VirtualIqSensor(simulation_config_path, 1),
          VirtualIqSensor(simulation_config_path, 2)},
      shared_(shared) {
    const auto config = YAML::LoadFile(hal_config_path);
    const auto interface = config["interface"].as<std::string>();
    const auto ids = config["motor_ids"].as<std::vector<unsigned>>();
    const auto motor_path = std::filesystem::path(hal_config_path).parent_path() /
                            config["motor_config"].as<std::string>();
    const auto motor = YAML::LoadFile(motor_path.string());
    const auto mit = motor["mit"];

    if (ids.size() != 3 || interface.empty() || interface.size() >= IFNAMSIZ)
        throw std::runtime_error("virtual motors require an interface and three IDs");

    for (int i = 0; i < 3; ++i) {
        if (!ids[i] || ids[i] > 254 ||
            std::find(ids.begin(), ids.begin() + i, ids[i]) != ids.begin() + i)
            throw std::runtime_error("virtual motor IDs must be distinct and in [1, 254]");

        ids_[i] = 0x140 + ids[i];
    }

    velocity_max_ = mit["velocity_max_rad_s"].as<double>();
    torque_max_ = mit["torque_max_nm"].as<double>();
    kt_output_ = mit["kt_output_nm_a"].as<double>();
    kd_scale_ = mit["kd_scale"].as<unsigned>();
    rom_parameters_ = openrobot::EncodeProfile(
        velocity_max_, torque_max_, kt_output_, mit["gear_ratio"].as<double>(), kd_scale_);

    const auto simulation = YAML::LoadFile(simulation_config_path);
    const auto actual_motor = simulation["motor"];
    const bool require_zero = actual_motor["require_zero_position_setup"].as<bool>();
    require_zero_ = require_zero;
    {
        shm_utils::Lock lock(shared_);
        if (shared_.simulation_available) {
            auto& motors = shm_utils::Next(shared_.motors, shared_.motor_sequence);
            motors.coordinate_valid.fill(!require_zero);
            ++shared_.motor_sequence;
        }
    }

    if (require_zero) {
        std::mt19937_64 generator(std::random_device{}());
        std::uniform_real_distribution<double> initial_position(
            -3.141592653589793, 3.141592653589793);

        for (int i = 0; i < 3; ++i)
            mit_origin_[i] = encoder_origin[i] - initial_position(generator);
    }

    std::cout << "[simulation] encoder zero: "
              << (require_zero ? "RANDOM / sequential stop calibration required"
                               : "MODEL / calibrated joint coordinates")
              << std::endl;
    const auto axis_value = [&](const char* key, int axis) {
        const auto values = actual_motor[key];

        if (!values.IsSequence() || values.size() != 3)
            throw std::runtime_error(std::string("expected three simulation motor values: ") + key);

        const double result = values[axis].as<double>();

        if (!std::isfinite(result))
            throw std::runtime_error(std::string("simulation motor value must be finite: ") + key);

        return result;
    };

    for (int i = 0; i < 3; ++i) {
        const double kt_motor = axis_value("kt_motor_nm_a", i);
        gear_ratio_[i] = axis_value("gear_ratio", i);
        actual_kt_output_[i] = kt_motor * gear_ratio_[i];
        pole_pairs_[i] = axis_value("pole_pairs", i);
        electrical_zero_[i] = axis_value("electrical_zero_rad", i);

        if (kt_motor <= 0 || gear_ratio_[i] <= 0 || !std::isfinite(actual_kt_output_[i]) ||
            actual_kt_output_[i] <= 0 || pole_pairs_[i] < 1 ||
            pole_pairs_[i] != std::floor(pole_pairs_[i]))
            throw std::runtime_error("simulation motor Kt/gear/pole-pairs must be positive");
    }

    std::cout << "[simulation] Iq sensor "
              << (iq_sensors_[0].enabled() ? "ENABLED" : "DISABLED (ideal CAN feedback)") << " | "
              << simulation_config_path << std::endl;

    const auto peer = ReadFeedback();

    if (!peer.running)
        throw std::runtime_error("CAN HAL is not running");

    const auto index = ::if_nametoindex(interface.c_str());

    if (!index)
        throw std::runtime_error("CAN interface not found: " + interface);

    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);

    if (socket_ < 0)
        shm_utils::Check(errno, "virtual motor SocketCAN");

    try {
        ifreq request{};
        std::copy(interface.begin(), interface.end(), request.ifr_name);

        if (::ioctl(socket_, SIOCGIFHWADDR, &request) < 0)
            shm_utils::Check(errno, "virtual motor interface type");

        if (request.ifr_hwaddr.sa_family != ARPHRD_CAN)
            throw std::runtime_error("virtual motor interface is not CAN");

        if (::ioctl(socket_, SIOCGIFFLAGS, &request) < 0 || !(request.ifr_flags & IFF_UP))
            throw std::runtime_error("virtual motor CAN interface is not UP");

        std::array<can_filter, 3> filters{};

        for (int i = 0; i < 3; ++i)
            filters[i] = {ids_[i], CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG};

        const int receive_own = 0;

        if (::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), sizeof(filters)) <
                0 ||
            ::setsockopt(
                socket_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &receive_own, sizeof(receive_own)) < 0)
            shm_utils::Check(errno, "virtual motor CAN options");

        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = int(index);

        if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
            shm_utils::Check(errno, "virtual motor CAN bind");
    } catch (...) {
        ::close(socket_);
        socket_ = -1;
        throw;
    }
}

VirtualMotors::~VirtualMotors() {
    if (socket_ >= 0)
        ::close(socket_);
}

MotorData VirtualMotors::ReadFeedback() const {
    auto& shm = shared_;
    shm_utils::Lock lock(shm);
    auto state = shm.motors[shm.motor_sequence % kShmBufferCount];

    if (shm.failed)
        throw std::runtime_error("CAN HAL failed");

    state.running &= !shm.stopping;

    return state;
}

void VirtualMotors::Reply(int axis, const openrobot::Payload& bytes) {
    can_frame frame{};
    frame.can_id = ids_[axis];
    frame.can_dlc = 8;
    std::copy(bytes.begin(), bytes.end(), frame.data);
    ssize_t count;

    do {
        count = ::write(socket_, &frame, sizeof(frame));
    } while (count < 0 && errno == EINTR);

    if (count != sizeof(frame))
        throw std::runtime_error(
            "virtual motor CAN reply failed: " + std::string(std::strerror(errno)));
}

void VirtualMotors::Update(const mjModel* model, mjData* data) {
    // Use physics time: a pause does not consume the Set Zero protection interval.
    // Rewinding MuJoCo is a mechanical reset, not a motor power cycle.
    const double motor_dt = motor_time_ < 0 ? 0 : std::max(0.0, data->time - motor_time_);

    for (auto& remaining : zero_hold_remaining_)
        remaining = std::max(0.0, remaining - motor_dt);

    motor_time_ = data->time;

    if (sensor_time_ < 0 || data->time != sensor_time_) {
        const bool reset = sensor_time_ < 0 || data->time < sensor_time_;
        const double dt = reset ? model->opt.timestep : data->time - sensor_time_;

        for (int i = 0; i < 3; ++i) {
            if (reset)
                iq_sensors_[i].Reset();

            const double kt = iq_sensors_[i].enabled() ? actual_kt_output_[i] : kt_output_;
            const double iq_true = data->qfrc_actuator[i] / kt;
            const double theta_e = std::remainder(
                pole_pairs_[i] * gear_ratio_[i] * data->qpos[i] + electrical_zero_[i],
                2 * 3.141592653589793);
            reported_iq_[i] = iq_sensors_[i].Sample(iq_true, theta_e, dt);
        }

        sensor_time_ = data->time;
    }

    for (unsigned batch = 0; batch < 256; ++batch) {
        can_frame frame{};
        const auto count = ::read(socket_, &frame, sizeof(frame));

        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            break;

        if (count < 0 && errno == EINTR)
            continue;

        if (count != sizeof(frame))
            throw std::runtime_error("virtual motor CAN read failed");

        const auto found = std::find(ids_.begin(), ids_.end(), frame.can_id);

        if (found == ids_.end())
            continue;

        if (frame.can_dlc != 8) {
            std::cerr << "[virtual motor] rejected CAN DLC " << int(frame.can_dlc) << '\n';
            continue;
        }

        const int axis = int(found - ids_.begin());
        openrobot::Payload bytes{};
        std::copy_n(frame.data, 8, bytes.begin());

        if (bytes[0] == 0xC4)
            Reply(axis, rom_parameters_);
        else if (bytes[0] == 0xC1) {
            if (modes_[axis] == MotorMode::Released) {
                // Both bootstrap policies initialize an encoder reference. Enabling
                // never changes it; only an explicit C3 can rebase the coordinates.
                targets_[axis] = {};
                modes_[axis] = MotorMode::Enabled;
            }

            Reply(axis, {0xC1, std::uint8_t(kd_scale_)});
        } else if (bytes[0] == 0xC2) {
            modes_[axis] = MotorMode::Released;
            targets_[axis] = {};
            Reply(axis, {0xC2});
        } else if (bytes[0] == 0xC3) {
            const unsigned raw = unsigned(bytes[6]) | (unsigned(bytes[7]) << 8);
            const int offset = raw >= 0x8000 ? int(raw) - 0x10000 : int(raw);
            const double offset_rad = offset * (3.141592653589793 / 18000.0);
            mit_origin_[axis] = data->qpos[axis] - offset_rad;
            zero_hold_remaining_[axis] = 0.020;
            targets_[axis] = {};

            if (modes_[axis] != MotorMode::Released)
                modes_[axis] = MotorMode::Enabled;

            Reply(axis, {0xC3, 0, 0, 0, 0, 0, bytes[6], bytes[7]});
        } else if (bytes[0] == 0xC0) {
            const auto target = openrobot::DecodeMitCommand(bytes, velocity_max_, kd_scale_);

            if (modes_[axis] != MotorMode::Released && zero_hold_remaining_[axis] <= 1e-12) {
                targets_[axis] = target;
                modes_[axis] = MotorMode::MitControl;
            }

            Reply(axis,
                openrobot::EncodeFeedback({data->qpos[axis] - mit_origin_[axis],
                    data->qvel[axis],
                    iq_sensors_[axis].enabled() ? reported_iq_[axis]
                                                : data->qfrc_actuator[axis] / kt_output_,
                    25}));
        } else
            throw std::runtime_error(
                "unsupported virtual motor command: " + std::to_string(bytes[0]));
    }

    // Only received CAN targets drive the torque actuators. FF is not used.
    for (int i = 0; i < 3; ++i) {
        const auto& target = targets_[i];
        const double torque =
            modes_[i] == MotorMode::MitControl
                ? target.kp * (target.position - (data->qpos[i] - mit_origin_[i])) +
                      target.kd * (target.velocity - data->qvel[i])
                : 0;
        const double minimum = std::max(-torque_max_, model->actuator_ctrlrange[2 * i]);
        const double maximum = std::min(torque_max_, model->actuator_ctrlrange[2 * i + 1]);
        data->ctrl[i] = std::clamp(torque, minimum, maximum);
    }
}

std::array<double, 3> ReadPosition(const YAML::Node& value, const mjModel* model) {
    const auto values = value.as<std::vector<double>>();

    if (values.size() != 3)
        throw std::runtime_error("expected three joint positions");

    std::array<double, 3> result;

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(values[i]) || values[i] < model->jnt_range[2 * i] ||
            values[i] > model->jnt_range[2 * i + 1])
            throw std::runtime_error("joint position is outside model limits");

        result[i] = values[i];
    }

    return result;
}

void ResetSimulation(const std::array<double, 3>& initial, const mjModel* model, mjData* data) {
    mj_resetData(model, data);
    std::copy(initial.begin(), initial.end(), data->qpos);
    mj_forward(model, data);
}

ControlData UpdateControlIO(ShmData& shm,
    const mjModel* model,
    mjData* data,
    mjData* gravity_data,
    std::array<double, 3>& reference,
    std::uint64_t measurement_sequence,
    const MotorData* can_feedback,
    bool publish_model) {
    ModelData state;
    if (publish_model) {
        mj_forward(model, data);
        mju_copy(gravity_data->qpos, data->qpos, model->nq);
        mju_zero(gravity_data->qvel, model->nv);
        mj_fwdPosition(model, gravity_data);
        mj_fwdVelocity(model, gravity_data);

        std::copy_n(gravity_data->qfrc_bias, 3, state.gravity_torque.begin());
        std::copy_n(data->qfrc_bias, 3, state.bias_torque.begin());
        std::copy_n(data->qfrc_passive, 3, state.passive_torque.begin());
        std::copy_n(model->dof_frictionloss, 3, state.friction_limit.begin());
        mj_fullM(model, state.mass.data(), data->qM);
        const int site = mj_name2id(model, mjOBJ_SITE, "FL_foot");

        if (site < 0)
            throw std::runtime_error("missing FL_foot site");

        mj_jacSite(model, data, state.jacobian.data(), nullptr, site);
        std::copy_n(data->site_xpos + 3 * site, 3, state.toe_position.begin());

        for (int i = 0; i < 3; ++i) {
            state.joint_min[i] = model->jnt_range[2 * i];
            state.joint_max[i] = model->jnt_range[2 * i + 1];
        }

    } else if (!can_feedback) {
        // Preserve explicit direct-torque simulation feedback; no model publication.
        mj_forward(model, data);
    }
    ControlData control;
    MotorData motors;
    {
        shm_utils::Lock lock(shm);

        if (shm.stopping)
            throw std::runtime_error("SHM stopped during control");

        state.published_ns = shm_utils::MonotonicNs();
        state.valid = true;
        state.input_measurement_sequence = measurement_sequence;
        if (publish_model) {
            shm.model[(shm.model_sequence + 1) % kShmBufferCount] = state;
            ++shm.model_sequence;
        }

        if (!can_feedback) {
            auto& next = shm_utils::Next(shm.motors, shm.motor_sequence);
            next.sample_dt = model->opt.timestep;
            next.supported.fill(kDebugSignals);
            next.valid.fill(measurement_sequence ? kDebugSignals : 0);
            next.profile_verified.fill(true);
            next.enabled = next.command_enabled;
            ++next.feedback_sequence;
            next.feedback_published_ns = state.published_ns;

            for (int i = 0; i < 3; ++i) {
                next.feedback[i] = {data->qpos[i] - next.position_offset[i],
                    data->qvel[i],
                    0,  // No current measurement in explicit direct-torque debug mode.
                    25};
                next.motor_torque[i] = data->qfrc_actuator[i];

                if (next.received_sequence[i] != measurement_sequence || !next.received_ns[i])
                    next.received_ns[i] = state.published_ns;

                next.received_sequence[i] = measurement_sequence;
                next.measurement_ns[i] =
                    static_cast<std::uint64_t>(measurement_sequence * model->opt.timestep * 1e9);
            }

            ++shm.motor_sequence;
        }

        motors = shm.motors[shm.motor_sequence % kShmBufferCount];
        control = shm.control[shm.control_sequence % kShmBufferCount];
        shm_utils::Notify(shm);
    }

    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(motors.debug_torque[i]) || !std::isfinite(motors.reference[i]))
            throw std::runtime_error("non-finite joint output");

        if (!can_feedback)
            data->ctrl[i] = std::clamp(motors.command_enabled[i] ? motors.debug_torque[i] : 0.0,
                model->actuator_ctrlrange[2 * i],
                model->actuator_ctrlrange[2 * i + 1]);

        reference[i] = motors.reference[i];
    }

    return control;
}

std::array<double, 6> ReadContactGroundTruth(const mjModel* model, const mjData* data) {
    const int foot = mj_name2id(model, mjOBJ_GEOM, "FL");
    const int object = mj_name2id(model, mjOBJ_GEOM, "touch_object");
    const int site = mj_name2id(model, mjOBJ_SITE, "FL_foot");

    if (foot < 0 || object < 0 || site < 0)
        throw std::runtime_error("missing Magnetic Touch scene geometry");

    std::array<double, 6> total{};

    for (int i = 0; i < data->ncon; ++i) {
        const auto& contact = data->contact[i];

        if (!((contact.geom[0] == foot && contact.geom[1] == object) ||
                (contact.geom[1] == foot && contact.geom[0] == object)))
            continue;

        mjtNum wrench[6], force[3], moment[3], offset[3], lever[3];
        mj_contactForce(model, data, i, wrench);
        mju_mulMatTVec(force, contact.frame, wrench, 3, 3);
        mju_mulMatTVec(moment, contact.frame, wrench + 3, 3, 3);
        mju_sub3(offset, contact.pos, data->site_xpos + 3 * site);
        mju_cross(lever, offset, force);
        const double sign = contact.geom[1] == foot ? 1 : -1;

        for (int axis = 0; axis < 3; ++axis) {
            total[axis] += sign * force[axis];
            total[3 + axis] += sign * (moment[axis] + lever[axis]);
        }
    }

    return total;
}
}  // namespace qhrr05
