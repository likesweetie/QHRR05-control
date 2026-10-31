#include "snapshot.hpp"
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <sys/wait.h>
#include <unistd.h>

using namespace qhrr05::visualization;
using namespace std::chrono_literals;

void Require(bool condition, const char* text) {
    if (!condition)
        throw std::runtime_error(text);
}

int main() {
    try {
        {
            const auto hardware_name = "/qhrr_view_hw_" + std::to_string(getpid());
            shm_utils::Mapping hardware(hardware_name, true);
            SnapshotSource source(hardware_name);
            {
                shm_utils::Lock lock(*hardware.get());
                auto& m = hardware.get()->motors[0];
                m.supported.fill(kCanSignals);
                m.valid.fill(kCanSignals);
                m.coordinate_valid.fill(true);
                m.received_sequence.fill(1);
                m.feedback_sequence = 1;
                m.feedback_published_ns = shm_utils::MonotonicNs();
                m.feedback[0].position = .25;
            }
            std::this_thread::sleep_for(30ms);
            auto view = source.Latest();
            Require(!view.simulation_available && view.pose_valid && view.sequence > 0,
                "physical viewer required simulation IPC");
            Require(view.motor.feedback[0].position == .25, "physical pose did not update");
        }
        const auto name = "/qhrr_visualization_test_" + std::to_string(getpid());
        shm_utils::Mapping owner(name, true);
        auto& shm = *owner.get();
        shm.simulation_available = true;
        qhrr05::simulation_ipc::Server simulation_ipc(name);
        {
            SnapshotSource source(name);
            {
                shm_utils::Lock lock(shm);
                std::this_thread::sleep_for(30ms);
            }
            Require(source.Misses() > 0, "busy SHM samples must be skipped");
            std::jthread producer([&]() {
                for (int frame = 1; frame <= 200; ++frame) {
                    {
                        shm_utils::Lock lock(shm);
                        ++shm.model_sequence;
                    }
                    SimulationData next;
                    next.time = frame;
                    next.position_gt.fill(frame);
                    next.object_position.fill(-frame);
                    next.published_ns = shm_utils::MonotonicNs();
                    simulation_ipc.Poll();
                    simulation_ipc.Publish(next);
                    std::this_thread::sleep_for(1ms);
                }
            });
            // Simulate a blocked GPU/consumer. The observer must not hold SHM while waiting
            // for the consumer, and the physics producer must keep publishing.
            std::this_thread::sleep_for(100ms);
            std::uint64_t progressed;
            {
                shm_utils::Lock lock(shm);
                progressed = shm.model_sequence;
            }
            Require(progressed > 20, "producer blocked by stalled visualization consumer");
            int coherent_samples = 0;
            while (coherent_samples < 50) {
                auto snapshot = source.Latest();
                for (int axis = 0; axis < 3; ++axis) {
                    Require(snapshot.simulation.position_gt[axis] == snapshot.simulation.time,
                        "torn joint state snapshot");
                    Require(snapshot.simulation.object_position[axis] == -snapshot.simulation.time,
                        "torn object snapshot");
                }
                ++coherent_samples;
                std::this_thread::sleep_for(2ms);
            }
            producer.join();
            {
                shm_utils::Lock lock(shm);
                Require(!shm.stopping && !shm.failed && shm.motor_sequence == 0 &&
                            shm.control_sequence == 0,
                    "visualizer changed commands or session flags");
            }
            std::cout << "snapshot coherence/stalled consumer: reads=" << source.Reads()
                      << " skips=" << source.Misses() << " max_hold_ns=" << source.MaxHoldNs()
                      << '\n';
        }
        const pid_t child = fork();
        Require(child >= 0, "fork failed");
        if (!child) {
            _exit(pthread_mutex_lock(&shm.mutex) == 0 ? 0 : 1);
        }
        int status;
        Require(
            waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "mutex owner death fixture failed");
        {
            SnapshotSource source(name);
            bool rejected = false;
            for (int attempt = 0; attempt < 100 && !rejected; ++attempt) {
                try {
                    source.Latest();
                } catch (const std::runtime_error&) {
                    rejected = true;
                }
                std::this_thread::sleep_for(2ms);
            }
            Require(rejected, "owner death must be reported, not silently displayed");
            Require(!shm.failed && !shm.stopping,
                "observer must not change session flags on owner death");
        }
        shm.abi_version = 0;
        bool rejected = false;
        try {
            SnapshotSource invalid(name);
        } catch (const std::runtime_error&) {
            rejected = true;
        }
        Require(rejected, "incompatible SHM ABI must fail explicitly");
        std::cout << "owner death / ABI rejection: passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
