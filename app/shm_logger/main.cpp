#include "shm_utils.hpp"
#include <cstdio>
#include <algorithm>
#include <iostream>
#include <memory>
#include <cmath>

int main(int argc, char** argv) {
    try {
        if (argc != 3)
            throw std::runtime_error("usage: shm_logger SHM_NAME CSV_PATH");

        shm_utils::Mapping mapping(argv[1]);
        auto& shm = *mapping.get();

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

        std::fprintf(stream, "\n");

        std::uint64_t last_event = 0, last_motion = 0;
        std::uint64_t last_sequence = 0, skipped_total = 0, rows = 0;
        double max_latency_us = 0, max_command_interval_us = 0;
        std::uint64_t observed_command = 0, previous_command_ns = 0, skipped_commands = 0;

        for (;;) {
            ShmSample sample;
            ShmCommand command;
            bool has_sample;

            {
                shm_utils::Lock lock(shm);

                while (shm.sample_sequence == last_sequence &&
                       shm.command_sequence == observed_command && !shm.stopping &&
                       !shm.simulation_finished)
                    lock.Wait();

                if (shm.failed)
                    throw std::runtime_error("SHM stopped after a process failure");

                if (shm.sample_sequence == last_sequence &&
                    (shm.stopping || shm.simulation_finished))
                    break;

                sample = shm.samples[shm.sample_sequence % 2];
                command = shm.commands[shm.command_sequence % 2];
                has_sample = shm.sample_sequence != last_sequence;
            }

            if (command.sequence > observed_command) {
                skipped_commands += command.sequence - observed_command - 1;

                if (previous_command_ns)
                    max_command_interval_us = std::max(max_command_interval_us,
                        (command.published_ns - previous_command_ns) / 1000.0);

                if (command.state_sequence)
                    max_latency_us = std::max(max_latency_us,
                        (command.published_ns - command.state_published_ns) / 1000.0);

                if (command.mode == 2 && (command.request_sequence != last_motion ||
                                             command.magnetic.event_sequence != last_event)) {
                    const auto& magnetic = command.magnetic;
                    std::cout << "[magnetic] t=" << magnetic.event_time << " "
                              << MagneticPhaseName(magnetic.phase)
                              << " reason=" << MagneticReasonName(magnetic.reason)
                              << " Fhat=" << magnetic.estimated_normal_force
                              << " target=" << magnetic.target_force << std::endl;
                    last_motion = command.request_sequence;
                    last_event = magnetic.event_sequence;
                }

                observed_command = command.sequence;
                previous_command_ns = command.published_ns;
            }

            if (!has_sample)
                continue;

            const auto skipped = sample.sequence - last_sequence - 1;
            skipped_total += skipped;
            last_sequence = sample.sequence;

            const bool has_source_state =
                sample.state_sequence != 0 && sample.command_sequence != 0;
            const double latency_us =
                has_source_state
                    ? (sample.command_published_ns - sample.state_published_ns) / 1000.0
                    : 0;

            const double age_us = (shm_utils::MonotonicNs() - sample.published_ns) / 1000.0;
            max_latency_us = std::max(max_latency_us, latency_us);

            std::fprintf(stream,
                "%llu,%.12g,%s",
                static_cast<unsigned long long>(sample.run_id),
                sample.time,
                sample.mode == 0 ? "homing" : (sample.mode == 1 ? "gravity" : "magnetic"));

            for (const auto& values : {sample.position, sample.reference, sample.torque})
                for (double value : values)
                    std::fprintf(stream, ",%.12g", value);

            std::fprintf(stream,
                ",%llu,%llu,%llu,%.9f,",
                static_cast<unsigned long long>(sample.sequence),
                static_cast<unsigned long long>(sample.state_sequence),
                static_cast<unsigned long long>(sample.command_sequence),
                sample.control_time);
            // No real measurement has been consumed yet for source sequence 0.
            if (has_source_state)
                std::fprintf(stream, "%.3f", latency_us);

            std::fprintf(stream, ",%.3f,%llu", age_us, static_cast<unsigned long long>(skipped));

            const auto& magnetic = sample.magnetic;
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
                sample.mode == 2 ? std::sqrt(error_squared) : 0.0,
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

            std::fprintf(stream, "\n");

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
