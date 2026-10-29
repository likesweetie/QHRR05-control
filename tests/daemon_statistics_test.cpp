#include "daemon/statistics.hpp"
#include "ipc/shm_utils.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

WindowSummary Row(const std::string& report, const std::string& name) {
    std::istringstream lines(report);
    std::string line;

    while (std::getline(lines, line)) {
        if (line.rfind(name, 0) != 0)
            continue;

        WindowSummary result;
        std::istringstream values(line.substr(26));
        values >> result.minimum >> result.maximum >> result.mean >> result.standard_deviation >>
            result.count;
        Require(bool(values), "statistics row has no numeric samples");
        return result;
    }

    throw std::runtime_error("missing statistics row: " + name);
}

int main() {
    try {
        WindowStatistics window;
        window.Add(0, 1);
        window.Add(1000000000, 3);
        auto summary = window.Read(1000000000, 2000000000);
        Require(summary.count == 2 && summary.minimum == 1 && summary.maximum == 3 &&
                    summary.mean == 2 && summary.standard_deviation == 1,
            "window min/max/mean/population deviation is wrong");
        Require(summary.latest == 3 && summary.trend.size() == 10, "latest/trend summary is wrong");
        summary = window.Read(2000000000, 2000000000);
        Require(summary.count == 1 && summary.mean == 3 && summary.standard_deviation == 0,
            "expired observations remain in the window");
        Require(window.Read(3000000000, 2000000000).count == 0, "empty window must be N/A");

        const auto name = "/qhrr05-statistics-" + std::to_string(::getpid());
        shm_utils::Mapping owner(name, true);
        auto& shm = *owner.get();
        shm.can_enabled = true;
        ShmStatistics statistics(name, 10);
        const auto now = shm_utils::MonotonicNs();
        auto& motors = shm.motors[0];
        motors.command_sequence = motors.feedback_sequence = 1;
        motors.command_published_ns = now - 50000000;
        motors.feedback_published_ns = now - 40000000;
        motors.received_ns = {now - 10000000, now - 20000000, now - 30000000};
        motors.received_sequence.fill(1);
        shm.model_sequence = shm.control_sequence = 1;
        shm.model[1].published_ns = now - 20000000;
        shm.control[1].published_ns = now - 9000000;
        statistics.Sample(shm);

        motors.command_sequence = 6;
        motors.command_published_ns += 10000000;
        motors.feedback_sequence = 3;
        motors.feedback_published_ns += 8000000;
        shm.model_sequence = 2;
        shm.model[0].published_ns = now - 17000000;
        shm.control_sequence = 3;
        shm.control[1].published_ns += 4000000;
        shm.control[1].command_sequence = 6;
        shm.control[1].published_ns = now - 5000000;
        shm.control[1].input_model_published_ns = now - 17000000;
        statistics.Sample(shm);
        statistics.Sample(shm);  // Unchanged publications must not create interval/gap samples.
        {
            shm_utils::Lock lock(shm);
            statistics.Sample(shm);  // Trylock must skip, never wait for this held mutex.
        }

        const auto report = statistics.Report(shm_utils::MonotonicNs());
        Require(Row(report, "Obs skips # / command").mean == 4, "observer sequence skip is wrong");
        Require(Row(report, "Pub mean us / command").mean == 2000 &&
                    Row(report, "Pub mean us / feedback").mean == 4000 &&
                    Row(report, "Pub mean us / model").mean == 3000 &&
                    Row(report, "Pub mean us / control").mean == 2000,
            "publication timestamps were paired with unrelated domain sequences");
        Require(Row(report, "Pub mean us / command").count == 1,
            "unchanged publication was counted twice");
        Require(Row(report, "Age us / command").count == 3,
            "data age must still be sampled while the producer is unchanged");
        Require(Row(report, "Model->control lag us").mean == 12000,
            "model-to-control publication lag is wrong");
        Require(Row(report, "RX worst age us").minimum >= 30000,
            "feedback age was incorrectly taken from the HAL publication timestamp");
        Require(Row(report, "Cmd unconsumed distance #").maximum == 6,
            "unconsumed command distance is wrong");
        Require(report.find("busy skips=1") != std::string::npos, "busy snapshot is not visible");
        Require(report.find("NOT consumer loss") != std::string::npos &&
                    report.find("N/A: mutex wait/hold, IPC overhead") != std::string::npos,
            "unobservable metrics were presented as measured values");
        Require(
            shm.motor_sequence == 0 && shm.control_sequence == 3 && !shm.stopping && !shm.failed,
            "statistics changed the shared-memory publication state");

        shm.can_enabled = false;
        ShmStatistics debug_statistics(name, 10);
        debug_statistics.Sample(shm);
        const auto debug_report = debug_statistics.Report(shm_utils::MonotonicNs());
        const auto distance_row =
            debug_report.substr(debug_report.find("Cmd unconsumed distance #"));
        Require(distance_row.substr(0, distance_row.find('\n')).find("N/A") != std::string::npos,
            "CAN command consumption was reported for a session without CAN");
        std::cout
            << "PASS: window math, coherent sequences, sampled interval/gap semantics, nonblocking observer\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
