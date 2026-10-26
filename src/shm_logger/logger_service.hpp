#pragma once

#include "shm_logger/log_schema.hpp"
#include "shm_logger/session_storage.hpp"
#include "data_tamer_v2/sinks/mcap_sink.hpp"
#include "data_tamer_v2/sinks/mcap_ring_sink.hpp"
#include <atomic>
#include <deque>
#include <mutex>
#include <set>
#include <thread>
#include <csignal>

namespace qhrr05::logging {
struct LoggerHealth {
    std::uint64_t capture_ns = 0, snapshot_attempts = 0, snapshot_successes = 0,
                  snapshot_failures = 0, shm_lock_misses = 0, mutex_recoveries = 0,
                  abi_failures = 0, source_regressions = 0, invalid_timestamps = 0,
                  last_successful_snapshot_ns = 0, schedule_skips = 0;
    std::array<std::uint64_t, 6> source_sequence_gaps{};
    std::array<double, 4> source_age_ms{};
    std::array<bool, 4> source_valid{};
    double mutex_wait_average_us = 0, mutex_wait_max_us = 0, mutex_hold_average_us = 0,
           mutex_hold_max_us = 0, input_rate_hz = 0, output_rate_hz = 0, drop_rate = 0;
    std::uint64_t datatamer_attempts = 0, datatamer_accepted = 0, pool_exhausted = 0, oversize = 0,
                  serialization_failures = 0, blocked_snapshots = 0, sink_delivered = 0,
                  sink_errors = 0, queue_high_water = 0, disk_available_bytes = 0,
                  flight_dump_failures = 0, triggers_accepted = 0, triggers_rejected = 0,
                  flight_evictions = 0, flight_stored_bytes = 0, flight_stored_snapshots = 0,
                  flight_report_drops = 0;
    std::string state = "INITIALIZING", last_error;
};

// Owns the local snapshot, channels and sinks in lifetime order. Only sampler_ writes frame_.
class LoggerService {
public:
    LoggerService(const std::string& config_path,
        const std::string& shm_name,
        const std::string& daemon_config);
    ~LoggerService();
    int Run(volatile std::sig_atomic_t& stop);

private:
    void Start();
    void Stop(const std::string& result);
    void SampleLoop() noexcept;
    void UpdateHealth();
    void Event(const std::string& code, const std::string& detail = "");
    bool Trigger(const std::string& code);
    void Fault(const std::string& message);
    void OpenSocket();
    void PollCommands();
    YAML::Node Command(const std::string& request);
    YAML::Node Status() const;
    void CheckTriggers(const LogFrame& frame);
    void Attach();

    fs::path config_path_, daemon_config_, socket_path_;
    std::string shm_name_, runtime_mode_, label_;
    YAML::Node config_;
    double rate_hz_ = 500, health_hz_ = 10, display_sec_ = 2, attach_timeout_sec_ = 10;
    double pre_sec_ = 5, post_sec_ = 2, stale_sec_ = .1, can_timeout_sec_ = .1, rotation_sec_ = 600;
    std::uint64_t segment_bytes_ = 512ULL << 20, ring_bytes_ = 64ULL << 20;
    std::size_t pool_slots_ = 1024;
    bool continuous_enabled_ = true, flight_enabled_ = true, auto_start_ = true;
    bool storage_writable_ = true;
    std::unique_ptr<SessionStorage> storage_;
    std::unique_ptr<SnapshotReader> reader_;
    LogFrame frame_{}, latest_{};
    LoggerHealth health_{};
    std::string event_code_, event_detail_, session_text_;
    std::uint64_t event_ns_ = 0, event_id_ = 0;
    bool event_accepted_ = false;
    std::shared_ptr<DataTamer2::LogChannel> state_channel_, health_channel_, event_channel_,
        session_channel_;
    std::shared_ptr<DataTamer2::SinkWorkerOf<DataTamer2::MCAPSink>> continuous_;
    std::shared_ptr<DataTamer2::SinkWorkerOf<DataTamer2::MCAPRingSink>> flight_;
    std::optional<DataTamer2::RegistrationID> simulation_id_;
    std::thread sampler_;
    std::atomic<bool> collecting_{false}, sampler_failed_{false}, source_fault_{false};
    std::array<std::atomic<std::uint64_t>, 6> source_gaps_{};
    std::atomic<std::uint64_t> source_regressions_{0}, invalid_times_{0};
    std::uint64_t health_previous_ns_ = 0, health_previous_input_ = 0, health_previous_output_ = 0,
                  fault_since_ns_ = 0;
    std::atomic<std::uint64_t> accepted_{0}, rejected_{0}, last_success_{0}, schedule_skips_{0},
        blocked_{0};
    std::mutex latest_mutex_, error_mutex_, dump_mutex_;
    std::string sampler_error_;
    std::deque<DataTamer2::MCAPRingDump> dumps_;
    std::deque<std::pair<std::uint64_t, std::string>> dump_requests_;
    std::uint64_t callback_drops_ = 0, auxiliary_failures_ = 0;
    std::set<std::string> active_triggers_;
    LogFrame last_observed_{};
    bool have_observed_ = false;
    fs::path active_file_;
    std::uint64_t start_ns_ = 0, segment_start_ns_ = 0, last_recoveries_ = 0;
    std::uint64_t session_number_ = 0, segment_number_ = 0;
    std::uint64_t socket_device_ = 0, socket_inode_ = 0;
    int socket_fd_ = -1;
};

std::string ControlSocketPath(const std::string& shm_name);
int RunCsvLogger(int argc, char** argv);
}  // namespace qhrr05::logging
