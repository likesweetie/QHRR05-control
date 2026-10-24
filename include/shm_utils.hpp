#pragma once

#include "shared_memory.hpp"

#include <cstddef>
#include <cerrno>


#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>           /* For O_* constants */

#include <unistd.h>

namespace shm_utils
{


inline ShmData* OpenShm(const char* name)
{
    int fd = shm_open(name, O_RDWR, 0666);
    if (fd < 0) return nullptr;

    void* addr = mmap(nullptr, sizeof(ShmData),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (addr == MAP_FAILED) return nullptr;

    auto* shm = reinterpret_cast<ShmData*>(addr);
    return shm;
}


inline ShmData* CreateShm(const char* name, bool create, bool* out_created)
{
    if (out_created) *out_created = false;

    int fd = -1;

    if (create) {
        // 1) "새로 생성"을 먼저 시도해서 out_created를 정확히 채움
        fd = shm_open(name, O_RDWR | O_CREAT | O_EXCL, 0666);
        if (fd >= 0) {
            if (out_created) *out_created = true;
        } else {
            // 이미 존재하면(exist) 그냥 open으로 폴백
            if (errno != EEXIST) {
                return nullptr;
            }
            fd = shm_open(name, O_RDWR, 0666);
            if (fd < 0) return nullptr;
        }
    } else {
        // 반드시 기존에 있어야 함
        fd = shm_open(name, O_RDWR, 0666);
        if (fd < 0) return nullptr;
    }

    // 2) 크기 보장: 기존 shm는 축소하지 않고, ShmData보다 작을 때만 확장
    struct stat st{};
    if (fstat(fd, &st) != 0) {
        close(fd);
        return nullptr;
    }
    if (st.st_size < static_cast<off_t>(sizeof(ShmData))) {
        if (ftruncate(fd, static_cast<off_t>(sizeof(ShmData))) != 0) {
            close(fd);
            return nullptr;
        }
    }

    // 3) mmap으로 매핑
    void* addr = mmap(nullptr, sizeof(ShmData),
                      PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    close(fd);

    if (addr == MAP_FAILED) {
        return nullptr;
    }

    return static_cast<ShmData*>(addr);
}

inline void CloseShm(ShmData* shm)
{
    if (!shm) return;
    ::munmap(static_cast<void*>(shm), sizeof(ShmData));
}

inline void DeleteShmByName(const char* name)
{
    if (!name) return;
    ::shm_unlink(name);
}

}
