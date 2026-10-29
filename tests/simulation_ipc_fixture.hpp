#pragma once
#include "ipc/simulation/channel.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

class SimulationIpcFixture {
public:
    SimulationIpcFixture(const std::string& name, SimulationData state = {}, int count = 1)
        : server_(name) {
        for (int i = 0; i < count; ++i)
            server_.Publish(state);
        worker_ = std::thread([this] {
            while (!stopping_) {
                server_.Poll();
                {
                    std::lock_guard lock(mutex_);
                    if (pending_) {
                        server_.Publish(next_);
                        pending_ = false;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    ~SimulationIpcFixture() {
        stopping_ = true;
        worker_.join();
    }

    void Publish(SimulationData state) {
        std::lock_guard lock(mutex_);
        next_ = state;
        pending_ = true;
    }

private:
    qhrr05::simulation_ipc::Server server_;
    std::atomic<bool> stopping_{false};
    std::mutex mutex_;
    SimulationData next_{};
    bool pending_ = false;
    std::thread worker_;
};
