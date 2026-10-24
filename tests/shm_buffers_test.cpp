#include "shm_utils.hpp"
#include <iostream>
#include <sys/wait.h>

void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void WaitChild(pid_t child) {
    int status = 0;
    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
    Require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "SHM writer failed");
}
int main() {
    pid_t writer = -1;
    const auto name = "/qhrr05-buffers-"+std::to_string(::getpid());
    try {
        {
            shm_utils::Mapping mapping(name, true);
            auto& shm = *mapping.get();
            writer = ::fork();
            Require(writer >= 0, "fork failed");
            if (writer == 0) {
                for (std::uint64_t sequence = 1; sequence <= 15000; ++sequence) {
                    shm_utils::Lock lock(shm);
                    auto& state = shm.states[sequence % 2];
                    state.sequence = sequence;
                    for (int i = 0; i < 3; ++i) {
                        state.position[i] = sequence*(i+1);
                        state.velocity[i] = -state.position[i];
                        state.gravity_torque[i] = 2*state.position[i];
                    }
                    shm.state_sequence = sequence;
                    shm_utils::Notify(shm);
                }
                { shm_utils::Lock lock(shm); shm.simulation_finished = true; shm_utils::Notify(shm); }
                ::_exit(0);
            }
            std::uint64_t seen = 0;
            for (;;) {
                ShmState snapshot;
                bool finished;
                {
                    shm_utils::Lock lock(shm);
                    while (shm.state_sequence == seen && !shm.simulation_finished) lock.Wait();
                    snapshot = shm.states[shm.state_sequence % 2];
                    finished = shm.simulation_finished;
                }
                for (int i = 0; i < 3; ++i) {
                    Require(snapshot.position[i] == snapshot.sequence*(i+1), "torn position snapshot");
                    Require(snapshot.velocity[i] == -snapshot.position[i], "mixed state versions");
                    Require(snapshot.gravity_torque[i] == 2*snapshot.position[i], "torn gravity snapshot");
                }
                seen = snapshot.sequence;
                if (finished) break;
                ::usleep(50);  // Reader may fall behind and a slot may be reused repeatedly.
            }
            WaitChild(writer); writer = -1;
            Require(seen == 15000, "last published state was lost");
            Require(shm.states[0].sequence == 15000 && shm.states[1].sequence == 14999,
                    "writer did not alternate double-buffer slots");
        }
        const int stale = ::shm_open(name.c_str(), O_RDWR, 0600);
        if (stale >= 0) ::close(stale);
        Require(stale < 0 && errno == ENOENT, "creator did not unlink owned SHM");
        {
            shm_utils::Mapping mapping(name, true);
            auto& shm = *mapping.get();
            writer = ::fork();
            Require(writer >= 0, "fork failed");
            if (writer == 0) { ::pthread_mutex_lock(&shm.mutex); ::_exit(0); }
            WaitChild(writer); writer = -1;
            bool rejected = false;
            try { shm_utils::Lock lock(shm); } catch (const std::runtime_error&) { rejected = true; }
            Require(rejected, "dead writer must be reported, never used as valid data");
            shm_utils::Lock lock(shm);
            Require(shm.failed && shm.stopping, "dead writer did not stop the SHM session");
        }
        const int small = ::shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
        Require(small >= 0, "test SHM create failed");
        Require(::ftruncate(small, 1) == 0, "test SHM resize failed");
        bool rejected = false;
        try { shm_utils::Mapping wrong_size(name); } catch (const std::runtime_error&) { rejected = true; }
        struct stat stat{};
        Require(::fstat(small, &stat) == 0 && stat.st_size == 1, "attach resized an existing SHM");
        ::close(small); ::shm_unlink(name.c_str());
        Require(rejected, "wrong SHM size was accepted");
        std::cout << "PASS: process-shared coherent double buffers, writer death, unlink, wrong-size rejection\n";
        return 0;
    } catch (const std::exception& error) {
        if (writer > 0) { ::kill(writer, SIGKILL); while (::waitpid(writer, nullptr, 0) < 0 && errno == EINTR) {} }
        ::shm_unlink(name.c_str());
        std::cerr << error.what() << '\n';
        return 1;
    }
}
