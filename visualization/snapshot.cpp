#include "snapshot.hpp"
#include <algorithm>
#include <chrono>

namespace qhrr05::visualization {
SnapshotSource::SnapshotSource(const std::string& name)
    : mapping_(name), simulation_channel_(name), worker_([this](std::stop_token stop) {
          Observe(stop);
      }) {
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
        std::uint64_t previous_sequence = 0;
        while (!stop.stop_requested()) {
            Snapshot next;
            simulation_channel_.Poll();
            next.simulation = simulation_channel_.Latest();
            next.sequence = next.simulation.sequence;
            next.finished = next.simulation.finished;
            next.simulation_status = simulation_channel_.Status();
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
                const int unlock = ::pthread_mutex_unlock(&shm.mutex);
                const auto released = shm_utils::MonotonicNs();
                shm_utils::Check(unlock, "visualizer SHM unlock");
                max_hold_ns_ = std::max(max_hold_ns_.load(), released - acquired);
                ++reads_;
                {
                    std::lock_guard lock(mailbox_mutex_);
                    latest_ = next;
                    if (next.sequence != previous_sequence && next.simulation.published_ns) {
                        previous_sequence = next.sequence;
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
