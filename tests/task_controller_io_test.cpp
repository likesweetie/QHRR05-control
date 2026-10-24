#include <array>
#include "task_controller/task_controller.hpp"
#include "mujoco_simulate/simulation.hpp"
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>

void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
    try {
        Require(argc == 3, "expected config and model");
        const auto config = YAML::LoadFile(argv[1]);
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(argv[2], nullptr, error, sizeof(error)), mj_deleteModel);
        Require(model != nullptr, error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        const auto initial = qhrr05::ReadPosition(config["homing"]["initial_position_rad"], model.get());
        qhrr05::ResetSimulation(initial, model.get(), data.get());
        for (int scenario = 0; scenario < 5; ++scenario) {
            int fd[2];
            Require(::socketpair(AF_UNIX, SOCK_SEQPACKET, 0, fd) == 0, "socketpair failed");
            std::exception_ptr worker_error;
            std::thread worker([&] {
                try {
                    if (scenario == 0) {
                        qhrr05::TaskController task(config, fd[1]);
                        Require(task.ReadInput(), "expected joint input");
                        task.Control();
                        task.WriteOutput();
                        Require(!task.ReadInput(), "closed simulation must end the loop");
                        return;
                    }
                    double input[10];
                    Require(::recv(fd[1], input, sizeof(input), 0) == sizeof(input), "input read failed");
                    if (scenario != 4) {
                        double output[6] = {};
                        if (scenario == 2) output[0] = std::numeric_limits<double>::quiet_NaN();
                        if (scenario == 3) output[0] = 1000;
                        const auto length = scenario == 1 ? sizeof(double) : sizeof(output);
                        Require(::send(fd[1], output, length, MSG_NOSIGNAL) == length, "output write failed");
                    }
                    ::close(fd[1]);
                } catch (...) { worker_error = std::current_exception(); ::close(fd[1]); }
            });
            std::array<double, 3> reference;
            bool rejected = false;
            try { qhrr05::ExchangeControl(fd[0], model.get(), data.get(), reference); }
            catch (const std::runtime_error&) { rejected = true; }
            ::close(fd[0]);
            worker.join();
            if (worker_error) std::rethrow_exception(worker_error);
            Require(rejected == (scenario == 1 || scenario == 2 || scenario == 4), "wrong IO result");
            if (scenario == 0)
                for (int i = 0; i < 3; ++i) Require(reference[i] == initial[i], "wrong initial reference");
            if (scenario == 3)
                Require(data->ctrl[0] == model->actuator_ctrlrange[1], "torque was not limited by the model");
        }
        std::cout << "PASS: direct joint loop, short/non-finite output rejection, model torque limit, disconnect\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
