#include "daemon/daemon.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main() {
    // Same executable name, but not owned by Daemon: it must survive TerminateAll.
    const pid_t independent = ::fork();

    if (independent < 0)
        return 1;

    if (independent == 0) {
        ::execl("/bin/sleep", "sleep", "30", static_cast<char*>(nullptr));
        _exit(127);
    }

    int result = 0;

    try {
        Daemon daemon;
        ProcessSpec missing;
        missing.name = "missing";
        missing.exec_path = "/does-not-exist/qhrr05-test";
        Require(!daemon.Spawn(missing), "exec failure must be reported to the parent");
        ProcessSpec owned;
        owned.name = "owned";
        owned.exec_path = "/bin/sleep";
        owned.args = {"30"};
        Require(daemon.Spawn(owned).has_value(), "could not start owned child");
        Require(!daemon.Signal("unknown", SIGTERM), "unknown process must not be signalled");
        daemon.TerminateAll(SIGTERM);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);

        while (daemon.Get("owned")->running && std::chrono::steady_clock::now() < deadline) {
            daemon.Reap();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        Require(!daemon.Get("owned")->running, "owned child was not stopped/reaped");
        int status = 0;
        Require(::waitpid(independent, &status, WNOHANG) == 0,
            "unowned process with the same executable name was terminated");
        std::cout << "PASS: exec failure, owned process stop/reap, independent process survives\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        result = 1;
    }

    ::kill(independent, SIGTERM);

    while (::waitpid(independent, nullptr, 0) < 0 && errno == EINTR) {
    }

    return result;
}
