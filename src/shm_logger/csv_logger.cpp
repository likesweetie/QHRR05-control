#include "shm_logger/logger_service.hpp"
#include "ipc/shm_utils.hpp"
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <memory>
#include <cmath>
#include <thread>
#include <chrono>

namespace {
int LogCan(const char* name, const char* csv_path) {
    shm_utils::Mapping mapping(name);
    auto& shm = *mapping.get();

    {
        shm_utils::Lock lock(shm);

        if (!shm.can_enabled)
            throw std::runtime_error("CAN logging requires a CAN-mode session");
    }

    const int descriptor = ::open(csv_path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);

    if (descriptor < 0)
        throw std::runtime_error(std::string("CAN CSV create failed: ") + std::strerror(errno));

    FILE* stream = ::fdopen(descriptor, "w");

    if (!stream) {
        ::close(descriptor);
        throw std::runtime_error("CAN CSV fdopen failed");
    }

    std::unique_ptr<FILE, decltype(&std::fclose)> file(stream, std::fclose);
    std::fprintf(stream,
        "state_seq,cycle,consumed_target_seq,axis,tx_frames,rx_frames,feedback_seq,q_rad,dq_rad_s,iq_a,temp_c,profile_verified,enabled,invalid_frames,socket_errors,feedback_age_us,observed_feedback_interval_us,skipped_publications,latest_target_seq,target_enabled,q_target_rad,dq_target_rad_s,kp,kd\n");
    std::uint64_t last_sequence = 0, skipped = 0, rows = 0;
    std::array<std::uint64_t, 3> last_feedback{}, last_received{};

    for (;;) {
        MotorData state;
        bool done;
        {
            shm_utils::Lock lock(shm);

            while (shm.motors[shm.motor_sequence % kShmBufferCount].feedback_sequence ==
                       last_sequence &&
                   !shm.stopping)
                lock.Wait();

            if (shm.motors[shm.motor_sequence % kShmBufferCount].feedback_sequence ==
                    last_sequence &&
                shm.stopping)
                break;

            state = shm.motors[shm.motor_sequence % kShmBufferCount];
            done = shm.stopping;
        }

        const auto missed = state.feedback_sequence - last_sequence - 1;
        skipped += missed;
        last_sequence = state.feedback_sequence;
        const auto now = shm_utils::MonotonicNs();

        for (int i = 0; i < 3; ++i) {
            std::fprintf(stream,
                "%llu,%llu,%llu,%d,%llu,%llu,%llu,%.12g,%.12g,%.12g,%d,%d,%d,%llu,%llu,",
                static_cast<unsigned long long>(state.feedback_sequence),
                static_cast<unsigned long long>(state.cycles),
                static_cast<unsigned long long>(state.consumed_command_sequence),
                i,
                static_cast<unsigned long long>(state.tx_frames[i]),
                static_cast<unsigned long long>(state.rx_frames[i]),
                static_cast<unsigned long long>(state.received_sequence[i]),
                state.feedback[i].position,
                state.feedback[i].velocity,
                state.feedback[i].current,
                state.feedback[i].temperature,
                state.profile_verified[i],
                state.enabled[i],
                static_cast<unsigned long long>(state.invalid_frames[i]),
                static_cast<unsigned long long>(state.socket_errors));

            if (state.received_sequence[i])
                std::fprintf(stream, "%.3f", (now - state.received_ns[i]) / 1000.0);

            std::fprintf(stream, ",");

            if (state.received_sequence[i] != last_feedback[i]) {
                if (last_received[i])
                    std::fprintf(
                        stream, "%.3f", (state.received_ns[i] - last_received[i]) / 1000.0);

                last_feedback[i] = state.received_sequence[i];
                last_received[i] = state.received_ns[i];
            }

            const auto& target = state.command[i];
            std::fprintf(stream,
                ",%llu,%llu,%d,%.12g,%.12g,%.12g,%.12g\n",
                static_cast<unsigned long long>(missed),
                static_cast<unsigned long long>(state.command_sequence),
                state.command_enabled[i],
                target.position,
                target.velocity,
                target.kp,
                target.kd);
            ++rows;
        }

        if (std::ferror(stream) || std::fflush(stream) != 0)
            throw std::runtime_error("CAN CSV write failed");

        if (done)
            break;
    }

    std::cout << "[shm_logger] CAN rows=" << rows << " skipped_publications=" << skipped << '\n';
    return 0;
}
}  // namespace

int qhrr05::logging::RunCsvLogger(int argc, char** argv) {
    try {
        if (argc == 4 && std::string(argv[1]) == "--can")
            return LogCan(argv[2], argv[3]);

        if (argc != 3)
            throw std::runtime_error("usage: shm_logger SHM_NAME CSV_PATH");

        shm_utils::Mapping mapping(argv[1]);
        auto& shm = *mapping.get();
        simulation_ipc::Client simulation(argv[1]);

        const int fd = ::open(argv[2], O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);

        if (fd < 0)
            throw std::runtime_error(std::string("CSV create failed: ") + std::strerror(errno));

        FILE* stream = ::fdopen(fd, "w");

        if (!stream) {
            ::close(fd);
            throw std::runtime_error("CSV fdopen failed");
        }

        std::unique_ptr<FILE, decltype(&std::fclose)> file(stream, std::fclose);

        std::fprintf(stream, "run_id,sim_time,controller");

        for (const char* field : {"q", "q_ref", "torque"})
            for (const char* joint : {"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"})
                std::fprintf(stream, ",%s_%s", field, joint);

        std::fprintf(stream,
            ",sample_seq,state_seq,command_seq,control_time,control_latency_us,sample_age_us,skipped_samples");
        std::fprintf(stream,
            ",magnetic_phase,magnetic_reason,observer_valid,force_est_n,force_gt_n,force_target_n,force_command_n,tracking_error_m,displacement_m,event_seq,event_time,torque_saturated,min_singular_m");

        for (const char* field : {"toe",
                 "toe_ref",
                 "force_est",
                 "force_gt",
                 "moment_gt",
                 "normal",
                 "external_torque",
                 "object"})
            for (const char* axis : {"x", "y", "z"})
                std::fprintf(stream, ",%s_%s", field, axis);

        std::fprintf(
            stream, ",sim_contact,sim_contact_event_seq,sim_contact_event_time,sim_paused\n");

        std::uint64_t last_event = 0, last_motion = 0;
        std::uint64_t last_sequence = 0, skipped_total = 0, rows = 0;
        double max_latency_us = 0, max_command_interval_us = 0;
        std::uint64_t observed_command = 0, previous_command_ns = 0, skipped_commands = 0;

        for (;;) {
            SimulationData sample;
            ControlData command;
            MotorData motors;
            std::uint64_t sample_sequence;
            bool has_sample;

            simulation.Poll();
            sample = simulation.Latest();
            sample_sequence = sample.sequence;
            has_sample = sample_sequence != last_sequence;
            bool stopping;
            {
                shm_utils::Lock lock(shm);
                if (shm.failed)
                    throw std::runtime_error("SHM stopped after a process failure");
                stopping = shm.stopping;
                motors = shm.motors[shm.motor_sequence % kShmBufferCount];
                command = shm.control[shm.control_sequence % kShmBufferCount];
            }
            if (!has_sample && (stopping || sample.finished))
                break;
            if (!has_sample && command.command_sequence == observed_command) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }

            if (command.command_sequence > observed_command) {
                skipped_commands += command.command_sequence - observed_command - 1;

                if (previous_command_ns)
                    max_command_interval_us = std::max(max_command_interval_us,
                        (command.published_ns - previous_command_ns) / 1000.0);

                if (command.input_model_sequence)
                    max_latency_us = std::max(max_latency_us,
                        (command.published_ns - command.input_model_published_ns) / 1000.0);

                if (command.mode == 2 && (command.active_motion_sequence != last_motion ||
                                             command.magnetic.event_sequence != last_event)) {
                    const auto& magnetic = command.magnetic;
                    std::cout << "[magnetic] t=" << magnetic.event_time << " "
                              << MagneticPhaseName(magnetic.phase)
                              << " reason=" << MagneticReasonName(magnetic.reason)
                              << " Fhat=" << magnetic.estimated_normal_force
                              << " target=" << magnetic.target_force << std::endl;
                    last_motion = command.active_motion_sequence;
                    last_event = magnetic.event_sequence;
                }

                observed_command = command.command_sequence;
                previous_command_ns = command.published_ns;
            }

            if (!has_sample)
                continue;

            const auto skipped = sample_sequence - last_sequence - 1;
            skipped_total += skipped;
            last_sequence = sample_sequence;

            const bool has_source_state =
                command.input_model_sequence != 0 && command.command_sequence != 0;
            const double latency_us =
                has_source_state
                    ? (command.published_ns - command.input_model_published_ns) / 1000.0
                    : 0;

            const double age_us = (shm_utils::MonotonicNs() - sample.published_ns) / 1000.0;
            max_latency_us = std::max(max_latency_us, latency_us);

            std::fprintf(stream,
                "%llu,%.12g,%s",
                static_cast<unsigned long long>(sample.run_id),
                sample.time,
                command.mode == 0   ? "homing"
                : command.mode == 1 ? "gravity"
                : command.mode == 2 ? "magnetic"
                : command.mode == 3 ? "tracking"
                : command.mode == 4 ? "ball"
                : command.mode == 5 ? "damping"
                : command.mode == 6 ? "zero_set"
                                    : "invalid");

            std::array<double, 3> reference;

            for (int i = 0; i < 3; ++i)
                reference[i] = motors.reference[i];

            for (const auto& values : {sample.position_gt, reference, sample.torque_gt})
                for (double value : values)
                    std::fprintf(stream, ",%.12g", value);

            std::fprintf(stream,
                ",%llu,%llu,%llu,%.9f,",
                static_cast<unsigned long long>(sample_sequence),
                static_cast<unsigned long long>(command.input_model_sequence),
                static_cast<unsigned long long>(command.command_sequence),
                command.control_time);
            // No real measurement has been consumed yet for source sequence 0.
            if (has_source_state)
                std::fprintf(stream, "%.3f", latency_us);

            std::fprintf(stream, ",%.3f,%llu", age_us, static_cast<unsigned long long>(skipped));

            const auto& magnetic = command.magnetic;
            double error_squared = 0, truth_squared = 0, normal_truth = 0;

            for (int i = 0; i < 3; ++i) {
                error_squared += std::pow(sample.toe_position[i] - magnetic.toe_reference[i], 2);
                truth_squared += sample.contact_force[i] * sample.contact_force[i];
                normal_truth -= magnetic.normal[i] * sample.contact_force[i];
            }

            const bool following = magnetic.phase == MagneticPhase::Following ||
                                   magnetic.phase == MagneticPhase::Releasing;

            std::fprintf(stream,
                ",%s,%s,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%llu,%.9g,%d,%.9g",
                MagneticPhaseName(magnetic.phase),
                MagneticReasonName(magnetic.reason),
                magnetic.observer_valid,
                magnetic.estimated_normal_force,
                following ? normal_truth : std::sqrt(truth_squared),
                magnetic.target_force,
                magnetic.force_command,
                command.mode == 2 ? std::sqrt(error_squared) : 0.0,
                magnetic.displacement,
                static_cast<unsigned long long>(magnetic.event_sequence),
                magnetic.event_time,
                magnetic.torque_saturated,
                magnetic.minimum_singular_value);

            for (const auto& values : {sample.toe_position,
                     magnetic.toe_reference,
                     magnetic.estimated_force,
                     sample.contact_force,
                     sample.contact_torque,
                     magnetic.normal,
                     magnetic.external_torque,
                     sample.object_position})
                for (double value : values)
                    std::fprintf(stream, ",%.12g", value);

            std::fprintf(stream,
                ",%d,%llu,%.9g,%d\n",
                sample.contact,
                static_cast<unsigned long long>(sample.contact_event_sequence),
                sample.contact_event_time,
                sample.paused);

            if (std::ferror(stream) || std::fflush(stream) != 0)
                throw std::runtime_error("CSV write failed");

            ++rows;
        }

        if (std::fflush(stream) != 0)
            throw std::runtime_error("CSV flush failed");

        std::cout << "[shm_logger] rows=" << rows << " skipped_samples=" << skipped_total
                  << " max_observed_control_latency_us=" << max_latency_us
                  << " observed_command_seq=" << observed_command
                  << " skipped_commands=" << skipped_commands
                  << " max_observed_command_interval_us=" << max_command_interval_us << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[shm_logger] " << error.what() << '\n';
        return 1;
    }
}
