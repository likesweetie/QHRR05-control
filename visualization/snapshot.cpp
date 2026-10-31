#include "snapshot.hpp"
#include <algorithm>
#include <chrono>

namespace qhrr05::visualization {
SnapshotSource::SnapshotSource(const std::string& name) : mapping_(name) {
    {
        auto& shm = *mapping_.get();
        // Session capabilities are immutable before participants are started.
        if (shm.simulation_available)
            simulation_channel_ = std::make_unique<simulation_ipc::Client>(name);
    }
    worker_ = std::jthread([this](std::stop_token stop) {
        Observe(stop);
    });
}

SnapshotSource::~SnapshotSource() {
    worker_.request_stop();
    worker_.join();
}

Snapshot SnapshotSource::Latest() {
    std::lock_guard lock(mailbox_mutex_);
    if (error_)
        std::rethrow_exception(error_);
    return latest_;
}

std::vector<Snapshot> SnapshotSource::Drain() {
    std::lock_guard lock(mailbox_mutex_);
    if (error_)
        std::rethrow_exception(error_);
    std::vector<Snapshot> result(pending_.begin(), pending_.end());
    pending_.clear();
    return result;
}

void SnapshotSource::Observe(std::stop_token stop) {
    try {
        auto& shm = *mapping_.get();
        std::uint64_t previous_sequence = 0, previous_motor = 0, previous_control = 0, revision = 0;
        while (!stop.stop_requested()) {
            Snapshot next;
            next.simulation_available = bool(simulation_channel_);
            if (simulation_channel_) {
                simulation_channel_->Poll();
                next.simulation = simulation_channel_->Latest();
                next.finished = next.simulation.finished;
                next.simulation_status = simulation_channel_->Status();
            } else
                next.simulation_status = "unavailable (physical device session)";
            const int result = ::pthread_mutex_trylock(&shm.mutex);
            if (result == EBUSY) {
                ++misses_;
            } else {
                if (result == EOWNERDEAD) {
                    // Do not display possibly torn data or change the robot's session flags.
                    const int repair = ::pthread_mutex_consistent(&shm.mutex);
                    const int unlock = ::pthread_mutex_unlock(&shm.mutex);
                    shm_utils::Check(repair, "visualizer SHM mutex consistency");
                    shm_utils::Check(unlock, "visualizer SHM recovery unlock");
                    throw std::runtime_error("visualizer: SHM mutex owner died");
                }
                shm_utils::Check(result, "visualizer SHM trylock");
                const auto acquired = shm_utils::MonotonicNs();
                next.motor = shm.motors[shm.motor_sequence % kShmBufferCount];
                next.model = shm.model[shm.model_sequence % kShmBufferCount];
                next.control = shm.control[shm.control_sequence % kShmBufferCount];
                next.stopping = shm.stopping;
                next.failed = shm.failed;
                next.published_ns = next.simulation_available ? next.simulation.published_ns
                                                              : next.motor.feedback_published_ns;
                const auto observed = ReadObservation(next.motor);
                next.pose_valid =
                    next.simulation_available
                        ? next.simulation.published_ns != 0
                        : observed.position_velocity && CoordinatesValid(observed.coordinate_valid);
                if (next.simulation.sequence != previous_sequence ||
                    next.motor.feedback_sequence != previous_motor ||
                    next.control.command_sequence != previous_control)
                    ++revision;
                next.sequence = revision;

                const int unlock = ::pthread_mutex_unlock(&shm.mutex);
                const auto released = shm_utils::MonotonicNs();
                shm_utils::Check(unlock, "visualizer SHM unlock");
                max_hold_ns_ = std::max(max_hold_ns_.load(), released - acquired);
                ++reads_;
                {
                    std::lock_guard lock(mailbox_mutex_);
                    latest_ = next;
                    if (next.pose_valid && (next.simulation.sequence != previous_sequence ||
                                               next.motor.feedback_sequence != previous_motor ||
                                               next.control.command_sequence != previous_control)) {
                        previous_sequence = next.simulation.sequence;
                        previous_motor = next.motor.feedback_sequence;
                        previous_control = next.control.command_sequence;
                        if (pending_.size() == 1000) {
                            pending_.pop_front();
                            ++dropped_;
                        }
                        pending_.push_back(next);
                    }
                }
                if (next.stopping || next.failed || next.finished)
                    return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    } catch (...) {
        std::lock_guard lock(mailbox_mutex_);
        error_ = std::current_exception();
    }
}
}  // namespace qhrr05::visualization
