#include "ipc/simulation/channel.hpp"

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

namespace {
void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
}

int main() {
    try {
        using namespace qhrr05::simulation_ipc;
        const std::string name = "/qhrr-ipc-test-" + std::to_string(getpid());
        const std::string path =
            "/tmp/qhrr05-simulation-" + std::to_string(getuid()) + name + ".sock";
        {
            Server server(name);
            Client observer(name), controller(name, true), slow(name);
            observer.Poll();
            controller.Poll();
            slow.Poll();
            server.Poll();
            SimulationData state;
            state.published_ns = 100;
            state.run_id = 42;
            state.object_quaternion = {0, 1, 0, 0};
            server.Publish(state);
            observer.Poll();
            Require(observer.Latest().run_id == 42 && observer.Latest().object_quaternion[1] == 1,
                "simulation snapshot lost pose or run id");
            bool active_rejected = false;
            try {
                Server duplicate(name);
            } catch (const std::runtime_error&) {
                active_rejected = true;
            }
            Require(active_rejected, "a second server replaced an active endpoint");
            SimulationCommand command;
            command.action = SimulationAction::Pause;
            command.pause = 1;
            bool rejected = false;
            try {
                observer.SendApproved(command);
            } catch (const std::runtime_error&) {
                rejected = true;
            }
            Require(rejected, "observer sent a controller command");
            const auto sequence = controller.SendApproved(command);
            const auto approved = server.Poll();
            Require(approved.size() == 1 && approved[0].sequence == sequence &&
                        approved[0].command.pause == 1,
                "approved command was lost or changed");

            // Stop one subscriber while maintaining a healthy reader. Publishing
            // must stay nonblocking, with loss reported only to the slow subscriber.
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < 10000; ++i) {
                state.time = i * .001;
                server.Publish(state);
                observer.Poll();
            }
            Require(std::chrono::steady_clock::now() - start < std::chrono::seconds(3),
                "slow subscriber blocked simulation publication");
            Require(server.Latest().transport_drops > 0, "backpressure did not report drops");
            Require(observer.Latest().transport_drops == 0 && observer.Latest().sequence == 10001,
                "slow subscriber contaminated healthy reader or delayed its latest state");
            slow.Poll();
            server.Poll();
            slow.Poll();
            Require(slow.Latest().sequence == 10001 && slow.Latest().transport_drops > 0,
                "slow subscriber did not recover with explicit gap count");

            // A mismatched version must be rejected, never interpreted as v1.
            const int bad = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
            Require(bad >= 0, "raw socket failed");
            sockaddr_un address{};
            address.sun_family = AF_UNIX;
            std::memcpy(address.sun_path, path.c_str(), path.size() + 1);
            Require(connect(bad, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
                "raw connect failed");

            struct Header {
                std::uint64_t magic = 0x5148525253494d31ULL;
                std::uint32_t version = 999, type = 1, size = 0, reserved = 0;
            } header;

            Require(send(bad, &header, sizeof(header), MSG_NOSIGNAL) == sizeof(header),
                "invalid version send failed");
            server.Poll();
            char byte;
            Require(recv(bad, &byte, 1, MSG_DONTWAIT) == 0, "invalid peer was not disconnected");
            close(bad);
            server.Finish();
            observer.Poll();
            Require(observer.Latest().finished, "simulation finish was not published");
        }
        Require(access(path.c_str(), F_OK) != 0, "simulation IPC endpoint was not removed");
        const auto child = fork();
        Require(child >= 0, "fork failed");
        if (child == 0) {
            Server abandoned(name);
            _exit(0);  // Simulate abrupt termination: no destructor/endpoint cleanup.
        }
        int status = 0;
        Require(
            waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "abrupt-exit fixture failed");
        Require(access(path.c_str(), F_OK) == 0, "abrupt exit did not leave the fixture socket");
        {
            Server recovered(name);
            Client reader(name);
            reader.Poll();
            recovered.Poll();
            SimulationData truth;
            truth.published_ns = 1;
            truth.run_id = 99;
            recovered.Publish(truth);
            reader.Poll();
            Require(reader.Latest().run_id == 99,
                "stale socket recovery did not restore communication");
        }
        {
            std::ofstream marker(path);
            marker << "preserve";
        }
        bool file_rejected = false;
        try {
            Server invalid(name);
        } catch (const std::runtime_error&) {
            file_rejected = true;
        }
        Require(
            file_rejected && access(path.c_str(), F_OK) == 0, "non-socket endpoint was removed");
        unlink(path.c_str());
        { Server lock_released(name); }
        std::cout
            << "PASS: active endpoint preserved, abrupt-exit socket recovered, foreign files preserved\n";
        std::cout
            << "PASS: separate simulation IPC, roles, version rejection, backpressure, finish/cleanup\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
