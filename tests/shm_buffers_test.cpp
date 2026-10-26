#include "shm_utils.hpp"
#include <iostream>
#include <sys/wait.h>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

void WaitChild(pid_t child) {
    int status = 0;

    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {
    }

    Require(WIFEXITED(status) && WEXITSTATUS(status) == 0, "SHM writer failed");
}

int main() {
    pid_t writer = -1, feedback_writer = -1;
    const auto name = "/qhrr05-buffers-" + std::to_string(::getpid());

    try {
        {
            shm_utils::Mapping mapping(name, true);
            auto& shm = *mapping.get();
            const auto launch = [&](bool feedback) {
                const auto child = ::fork();
                Require(child >= 0, "fork failed");

                if (child == 0) {
                    for (std::uint64_t sequence = 1; sequence <= 15000; ++sequence) {
                        shm_utils::Lock lock(shm);
                        auto& motors = shm_utils::Next(shm.motors, shm.motor_sequence);

                        if (feedback)
                            motors.feedback_sequence = sequence;
                        else
                            motors.command_sequence = sequence;

                        for (int i = 0; i < 3; ++i) {
                            if (feedback)
                                motors.feedback[i].position = -double(sequence * (i + 1));
                            else
                                motors.command[i].position = sequence * (i + 1);
                        }

                        ++shm.motor_sequence;
                        shm_utils::Notify(shm);
                    }

                    ::_exit(0);
                }

                return child;
            };
            writer = launch(false);
            feedback_writer = launch(true);
            std::uint64_t seen = 0;

            for (;;) {
                MotorData snapshot;
                {
                    shm_utils::Lock lock(shm);

                    while (shm.motor_sequence == seen)
                        lock.Wait();

                    snapshot = shm.motors[shm.motor_sequence % 2];
                    seen = shm.motor_sequence;
                }

                for (int i = 0; i < 3; ++i) {
                    Require(snapshot.command[i].position == snapshot.command_sequence * (i + 1),
                        "torn command snapshot or feedback writer overwrote a command");
                    Require(snapshot.feedback[i].position ==
                                -double(snapshot.feedback_sequence * (i + 1)),
                        "torn feedback snapshot or command writer overwrote feedback");
                }

                if (snapshot.command_sequence == 15000 && snapshot.feedback_sequence == 15000)
                    break;

                ::usleep(50);
            }

            WaitChild(writer);
            writer = -1;
            WaitChild(feedback_writer);
            feedback_writer = -1;
            Require(seen == 30000, "multiwriter publication lost an update");
            Require(shm.motors[0].command_sequence + shm.motors[0].feedback_sequence == 30000 &&
                        shm.motors[1].command_sequence + shm.motors[1].feedback_sequence == 29999,
                "writers did not alternate double-buffer slots");
        }

        const int stale = ::shm_open(name.c_str(), O_RDWR, 0600);

        if (stale >= 0)
            ::close(stale);

        Require(stale < 0 && errno == ENOENT, "creator did not unlink owned SHM");
        {
            shm_utils::Mapping mapping(name, true);
            auto& shm = *mapping.get();
            writer = ::fork();
            Require(writer >= 0, "fork failed");

            if (writer == 0) {
                ::pthread_mutex_lock(&shm.mutex);
                ::_exit(0);
            }

            WaitChild(writer);
            writer = -1;
            bool rejected = false;

            try {
                shm_utils::Lock lock(shm);
            } catch (const std::runtime_error&) {
                rejected = true;
            }

            Require(rejected, "dead writer must be reported, never used as valid data");
            shm_utils::Lock lock(shm);
            Require(shm.failed && shm.stopping, "dead writer did not stop the SHM session");
        }

        const int small = ::shm_open(name.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
        Require(small >= 0, "test SHM create failed");
        Require(::ftruncate(small, 1) == 0, "test SHM resize failed");
        bool rejected = false;

        try {
            shm_utils::Mapping wrong_size(name);
        } catch (const std::runtime_error&) {
            rejected = true;
        }

        struct stat stat {};

        Require(::fstat(small, &stat) == 0 && stat.st_size == 1, "attach resized an existing SHM");
        ::close(small);
        if (feedback_writer > 0) {
            ::kill(feedback_writer, SIGKILL);

            while (::waitpid(feedback_writer, nullptr, 0) < 0 && errno == EINTR) {
            }
        }

        ::shm_unlink(name.c_str());
        Require(rejected, "wrong SHM size was accepted");
        std::cout
            << "PASS: process-shared coherent double buffers, writer death, unlink, wrong-size rejection\n";
        return 0;
    } catch (const std::exception& error) {
        if (writer > 0) {
            ::kill(writer, SIGKILL);

            while (::waitpid(writer, nullptr, 0) < 0 && errno == EINTR) {
            }
        }

        ::shm_unlink(name.c_str());
        std::cerr << error.what() << '\n';

        return 1;
    }
}
