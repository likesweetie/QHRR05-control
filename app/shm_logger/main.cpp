#include "shm_utils.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_run{true};

void HandleSignal(int) {
    g_run.store(false, std::memory_order_relaxed);
}

const char* ResolveShmName() {
    const char* name = std::getenv("SHM_NAME");
    return (name && name[0]) ? name : "/shm0";
}

std::string ResolveCsvPath(int argc, char** argv) {
    if (argc > 1 && argv[1] && argv[1][0]) {
        return argv[1];
    }

    const char* env = std::getenv("SHM_LOG_PATH");
    if (env && env[0]) {
        return std::string(env);
    }
    return "shm_log.csv";
}

void WriteHeader(std::ofstream& out, int nq, int nv, int nu) {
    out << "wall_time_ns,sim_time,state_seq,applied_command_seq,nq,nv,nu";
    for (int i = 0; i < nq; ++i) {
        out << ",q_" << i;
    }
    for (int i = 0; i < nv; ++i) {
        out << ",qd_" << i;
    }
    for (int i = 0; i < nu; ++i) {
        out << ",ctrl_applied_" << i;
    }
    out << '\n';
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    const char* shm_name = ResolveShmName();
    const std::string csv_path = ResolveCsvPath(argc, argv);

    std::cout << "[shm_logger] waiting shm: " << shm_name << std::endl;

    ShmData* shm = nullptr;
    while (g_run.load(std::memory_order_relaxed) && !shm) {
        shm = shm_utils::OpenShm(shm_name);
        if (!shm) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (!shm) {
        std::cerr << "[shm_logger] failed to open shm" << std::endl;
        return 1;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (g_run.load(std::memory_order_relaxed) && shm->state_seq == 0 &&
           std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (shm->state_seq == 0 || (shm->nq != 3 || shm->nv != 3 || shm->nu != 3)) {
        std::cerr << "[shm_logger] expected published qhrr-05 state with nq=nv=nu=3" << std::endl;
        shm_utils::CloseShm(shm);
        return 1;
    }
    constexpr int nq = 3;
    constexpr int nv = 3;
    constexpr int nu = 3;

    std::ofstream out(csv_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "[shm_logger] failed to open csv: " << csv_path << std::endl;
        shm_utils::CloseShm(shm);
        return 1;
    }

    WriteHeader(out, nq, nv, nu);
    std::cout << "[shm_logger] logging to: " << csv_path
              << " (nq=" << nq << ", nv=" << nv << ", nu=" << nu << ")" << std::endl;

    std::uint64_t rows = 0;
    const auto period = std::chrono::milliseconds(20);  // 50 Hz
    auto next_tick = std::chrono::steady_clock::now();
    int exit_code = 0;

    while (g_run.load(std::memory_order_relaxed)) {
        next_tick += period;
        if ((shm->nq != 3 || shm->nv != 3 || shm->nu != 3)) {
            std::cerr << "[shm_logger] qhrr-05 state dimensions changed" << std::endl;
            exit_code = 1;
            break;
        }

        std::uint64_t seq0 = 0;
        std::uint64_t seq1 = 0;
        std::uint64_t applied_command_seq = 0;
        double sim_time = 0.0;

        double q[nq] = {0.0};
        double qd[nv] = {0.0};
        double ctrl_applied[nu] = {0.0};

        // Best-effort consistent snapshot.
        for (int retry = 0; retry < 3; ++retry) {
            seq0 = shm->state_seq;
            sim_time = shm->sim_time;
            applied_command_seq = shm->applied_command_seq;

            for (int i = 0; i < nq; ++i) {
                q[i] = shm->qpos[i];
            }
            for (int i = 0; i < nv; ++i) {
                qd[i] = shm->qvel[i];
            }
            for (int i = 0; i < nu; ++i) {
                ctrl_applied[i] = shm->ctrl_applied[i];
            }

            seq1 = shm->state_seq;
            if (seq0 == seq1) {
                break;
            }
        }

        const auto now = std::chrono::time_point_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now());
        const auto wall_time_ns = static_cast<std::int64_t>(now.time_since_epoch().count());

        out << wall_time_ns << ',' << sim_time << ',' << seq1 << ',' << applied_command_seq
            << ',' << nq << ',' << nv << ',' << nu;

        for (int i = 0; i < nq; ++i) {
            out << ',' << q[i];
        }
        for (int i = 0; i < nv; ++i) {
            out << ',' << qd[i];
        }
        for (int i = 0; i < nu; ++i) {
            out << ',' << ctrl_applied[i];
        }
        out << '\n';

        ++rows;

        if ((rows % 200) == 0) {
            out.flush();
        }

        std::this_thread::sleep_until(next_tick);
    }

    out.flush();
    shm_utils::CloseShm(shm);
    std::cout << "[shm_logger] stopped. rows=" << rows << std::endl;
    return exit_code;
}
