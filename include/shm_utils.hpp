#pragma once

#include "shared_memory.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <new>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

namespace shm_utils {
inline void Check(int error, const char* operation) {
    if (error)
        throw std::runtime_error(std::string(operation) + ": " + std::strerror(error));
}

inline std::uint64_t MonotonicNs() {
    timespec time{};

    if (::clock_gettime(CLOCK_MONOTONIC, &time) != 0)
        Check(errno, "clock_gettime");

    return static_cast<std::uint64_t>(time.tv_sec) * 1000000000ULL + time.tv_nsec;
}

inline void Notify(ShmData& shm) {
    Check(::pthread_cond_broadcast(&shm.changed), "SHM notify");
}

class Lock {
public:
    explicit Lock(ShmData& shm) : shm_(shm) {
        const int result = ::pthread_mutex_lock(&shm_.mutex);

        if (result == EOWNERDEAD) {
            FailOwner();
            ::pthread_mutex_unlock(&shm_.mutex);
        }

        Check(result, "SHM lock");
    }

    ~Lock() {
        ::pthread_mutex_unlock(&shm_.mutex);
    }

    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;

    // Periodic scheduler wake-up, not a response deadline.
    void WaitUntil(std::uint64_t wake_ns) {
        const timespec wake{static_cast<time_t>(wake_ns / 1000000000ULL),
            static_cast<long>(wake_ns % 1000000000ULL)};
        const int result = ::pthread_cond_timedwait(&shm_.changed, &shm_.mutex, &wake);

        if (result == EOWNERDEAD)
            FailOwner();

        if (result != ETIMEDOUT)
            Check(result, "SHM scheduler wait");
    }

    void Wait() {
        const int result = ::pthread_cond_wait(&shm_.changed, &shm_.mutex);

        if (result == EOWNERDEAD)
            FailOwner();

        Check(result, "SHM wait");
    }

private:
    void FailOwner() {
        shm_.stopping = shm_.failed = true;
        ::pthread_mutex_consistent(&shm_.mutex);
        ::pthread_cond_broadcast(&shm_.changed);
    }

    ShmData& shm_;
};

inline void Stop(ShmData& shm, bool failed = false) {
    Lock lock(shm);
    shm.stopping = true;
    shm.failed |= failed;
    Notify(shm);
}

// The owning program creates exclusively; other programs attach. Existing objects are
// never reused, resized or unlinked by an attaching process.
class Mapping {
public:
    explicit Mapping(const std::string& name, bool create = false) : name_(name), owner_(create) {
        if (name.size() < 2 || name[0] != '/' || name.find('/', 1) != std::string::npos)
            throw std::runtime_error("SHM name must be / followed by a single name");

        const int flags = O_RDWR | O_CLOEXEC | (create ? O_CREAT | O_EXCL : 0);
        const int fd = ::shm_open(name.c_str(), flags, 0600);

        if (fd < 0)
            throw std::runtime_error("shm_open " + name + ": " + std::strerror(errno));

        try {
            if (create && ::ftruncate(fd, sizeof(ShmData)) != 0)
                Check(errno, "SHM size");

            struct stat stat {};

            if (::fstat(fd, &stat) != 0)
                Check(errno, "SHM stat");

            if (stat.st_size != sizeof(ShmData))
                throw std::runtime_error("SHM size mismatch: " + name);

            void* address =
                ::mmap(nullptr, sizeof(ShmData), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

            if (address == MAP_FAILED)
                Check(errno, "SHM mmap");

            data_ = static_cast<ShmData*>(address);

            if (create) {
                new (data_) ShmData{};
                pthread_mutexattr_t mutex_attr;
                Check(::pthread_mutexattr_init(&mutex_attr), "SHM mutex attributes");
                Check(::pthread_mutexattr_setpshared(&mutex_attr, PTHREAD_PROCESS_SHARED),
                    "SHM mutex sharing");
                Check(::pthread_mutexattr_setrobust(&mutex_attr, PTHREAD_MUTEX_ROBUST),
                    "SHM mutex robustness");
                const int result = ::pthread_mutex_init(&data_->mutex, &mutex_attr);
                ::pthread_mutexattr_destroy(&mutex_attr);
                Check(result, "SHM mutex init");
                pthread_condattr_t cond_attr;
                Check(::pthread_condattr_init(&cond_attr), "SHM condition attributes");
                Check(::pthread_condattr_setpshared(&cond_attr, PTHREAD_PROCESS_SHARED),
                    "SHM condition sharing");
                Check(::pthread_condattr_setclock(&cond_attr, CLOCK_MONOTONIC),
                    "SHM condition clock");
                const int cond_result = ::pthread_cond_init(&data_->changed, &cond_attr);
                ::pthread_condattr_destroy(&cond_attr);
                Check(cond_result, "SHM condition init");
            }

            ::close(fd);
        } catch (...) {
            ::close(fd);

            if (data_)
                ::munmap(data_, sizeof(ShmData));

            if (create)
                ::shm_unlink(name.c_str());

            throw;
        }
    }

    ~Mapping() {
        ::munmap(data_, sizeof(ShmData));

        if (owner_)
            ::shm_unlink(name_.c_str());
    }

    Mapping(const Mapping&) = delete;
    Mapping& operator=(const Mapping&) = delete;

    ShmData* get() const {
        return data_;
    }

private:
    std::string name_;
    bool owner_;
    ShmData* data_ = nullptr;
};
}  // namespace shm_utils
