#include "shm_logger/shm_snapshot_reader.hpp"
#include "ipc/shm_utils.hpp"
#include <sys/mman.h>
#include <sys/stat.h>
#include <algorithm>

namespace qhrr05::logging {
SnapshotReader::SnapshotReader(const std::string& name, bool simulation) : name_(name) {
    if (simulation)
        simulation_ = std::make_unique<simulation_ipc::Client>(name);
    if (name.size() < 2 || name.front() != '/' || name.find('/', 1) != std::string::npos)
        throw std::runtime_error("logger SHM name must be / followed by a single name");

    fd_ = ::shm_open(name.c_str(), O_RDWR | O_CLOEXEC, kShmPermissions);
    if (fd_ < 0)
        shm_utils::Check(errno, "logger shm_open");

    try {
        struct stat info {};

        if (::fstat(fd_, &info) != 0)
            shm_utils::Check(errno, "logger SHM stat");
        if (info.st_size != kShmByteSize)
            throw std::runtime_error("logger SHM size mismatch");

        device_ = info.st_dev;
        inode_ = info.st_ino;
        void* address = ::mmap(nullptr, kShmByteSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (address == MAP_FAILED)
            shm_utils::Check(errno, "logger SHM mmap");
        shm_ = static_cast<ShmData*>(address);

        if (shm_->magic != kShmMagic || shm_->abi_version != kShmAbiVersion ||
            shm_->byte_size != kShmByteSize || shm_->buffer_count != kShmBufferCount)
            throw std::runtime_error("logger SHM ABI/header mismatch");
    } catch (...) {
        if (shm_)
            ::munmap(shm_, kShmByteSize);
        ::close(fd_);
        throw;
    }
}

SnapshotReader::~SnapshotReader() {
    ::munmap(shm_, kShmByteSize);
    ::close(fd_);
}

bool SnapshotReader::Read(LogFrame& frame) {
    if (simulation_)
        simulation_->Poll();
    const auto before = shm_utils::MonotonicNs();
    ++attempts_;
    const int result = ::pthread_mutex_trylock(&shm_->mutex);
    const auto acquired = shm_utils::MonotonicNs();
    const auto wait = acquired - before;
    wait_ns_ += wait;
    wait_max_ns_ = std::max(wait_max_ns_.load(), wait);

    if (result == EBUSY) {
        ++misses_;
        return false;
    }

    if (result == EOWNERDEAD) {
        // Do not log a potentially torn publication or change the robot's state flags.
        ++recoveries_;
        const int repair = ::pthread_mutex_consistent(&shm_->mutex);
        ::pthread_mutex_unlock(&shm_->mutex);
        shm_utils::Check(repair, "logger robust mutex recovery");
        return false;
    }

    if (result) {
        ++failures_;
        shm_utils::Check(result, "logger SHM trylock");
    }

    frame.capture_ns = acquired;
    frame.lock_acquired_ns = acquired;
    frame.abi_version = shm_->abi_version;
    frame.motor_sequence = shm_->motor_sequence;
    frame.model_sequence = shm_->model_sequence;
    frame.control_sequence = shm_->control_sequence;
    frame.stopping = shm_->stopping;
    frame.failed = shm_->failed;
    frame.can_enabled = shm_->can_enabled;
    frame.motor = shm_->motors[frame.motor_sequence % kShmBufferCount];
    frame.model = shm_->model[frame.model_sequence % kShmBufferCount];
    frame.control = shm_->control[frame.control_sequence % kShmBufferCount];
    const int unlock = ::pthread_mutex_unlock(&shm_->mutex);
    frame.lock_released_ns = shm_utils::MonotonicNs();
    shm_utils::Check(unlock, "logger SHM unlock");
    if (simulation_) {
        frame.simulation = simulation_->Latest();
        frame.simulation_sequence = frame.simulation.sequence;
        frame.simulation_finished = frame.simulation.finished;
    }

    const auto hold = frame.lock_released_ns - acquired;
    hold_ns_ += hold;
    hold_max_ns_ = std::max(hold_max_ns_.load(), hold);
    ++successes_;
    return true;
}

bool SnapshotReader::SameObject() const {
    const int fd = ::shm_open(name_.c_str(), O_RDONLY | O_CLOEXEC, kShmPermissions);
    if (fd < 0) {
        if (errno == ENOENT)
            return false;
        shm_utils::Check(errno, "logger SHM identity open");
    }

    struct stat info {};

    const int status = ::fstat(fd, &info);
    const int error = errno;
    ::close(fd);
    if (status != 0)
        shm_utils::Check(error, "logger SHM identity stat");
    return info.st_dev == device_ && info.st_ino == inode_ && info.st_size == kShmByteSize;
}

void SnapshotReader::ResetCounters() {
    attempts_ = successes_ = misses_ = recoveries_ = failures_ = 0;
    wait_ns_ = wait_max_ns_ = hold_ns_ = hold_max_ns_ = 0;
}

ReaderCounters SnapshotReader::Counters() const {
    const auto attempts = attempts_.load(), successes = successes_.load();
    return {attempts,
        successes,
        misses_.load(),
        recoveries_.load(),
        failures_.load(),
        attempts ? double(wait_ns_.load()) / attempts / 1000 : 0,
        double(wait_max_ns_.load()) / 1000,
        successes ? double(hold_ns_.load()) / successes / 1000 : 0,
        double(hold_max_ns_.load()) / 1000};
}
}  // namespace qhrr05::logging
