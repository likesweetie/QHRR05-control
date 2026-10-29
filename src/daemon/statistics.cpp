#include "daemon/statistics.hpp"
#include "ipc/shm_utils.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

void WindowStatistics::Add(std::uint64_t time_ns, double value) {
    if (!std::isfinite(value) || (!values_.empty() && time_ns < values_.back().first))
        throw std::runtime_error("invalid monitoring sample");

    values_.emplace_back(time_ns, value);
}

WindowSummary WindowStatistics::Read(std::uint64_t now_ns, std::uint64_t window_ns) {
    while (!values_.empty() && now_ns >= values_.front().first &&
           now_ns - values_.front().first >= window_ns)
        values_.pop_front();

    WindowSummary result;
    double squared_deviations = 0;

    for (const auto& sample : values_) {
        const double value = sample.second;

        if (result.count == 0)
            result.minimum = result.maximum = value;

        result.minimum = std::min(result.minimum, value);
        result.maximum = std::max(result.maximum, value);
        result.latest = value;
        ++result.count;
        const double delta = value - result.mean;
        result.mean += delta / result.count;
        squared_deviations += delta * (value - result.mean);
    }

    if (result.count)
        result.standard_deviation = std::sqrt(std::max(0.0, squared_deviations / result.count));

    if (result.count) {
        std::array<double, 10> sums{};
        std::array<std::size_t, 10> counts{};
        const auto start = now_ns >= window_ns ? now_ns - window_ns : 0;
        for (const auto& sample : values_) {
            const auto bucket = std::min<std::uint64_t>(9, (sample.first - start) * 10 / window_ns);
            sums[bucket] += sample.second;
            ++counts[bucket];
        }
        constexpr char levels[] = ".:-=+*#%@";
        result.trend.assign(10, ' ');
        for (std::size_t i = 0; i < counts.size(); ++i) {
            if (!counts[i])
                continue;
            const double normalized =
                result.maximum > result.minimum
                    ? (sums[i] / counts[i] - result.minimum) / (result.maximum - result.minimum)
                    : .5;
            result.trend[i] = levels[std::clamp(int(std::lround(normalized * 8)), 0, 8)];
        }
    }
    return result;
}

ShmStatistics::ShmStatistics(std::string name, double window_sec)
    : name_(std::move(name)), window_ns_(0) {
    if (!std::isfinite(window_sec) || window_sec <= 0 || window_sec > 60)
        throw std::runtime_error("monitoring.window_sec must be in (0, 60]");

    window_ns_ = static_cast<std::uint64_t>(window_sec * 1e9);
}

void ShmStatistics::Sample(ShmData& shm) {
    const int result = ::pthread_mutex_trylock(&shm.mutex);

    if (result == EBUSY) {
        ++busy_;
        return;
    }

    if (result == EOWNERDEAD) {
        shm.stopping = shm.failed = true;
        ::pthread_mutex_consistent(&shm.mutex);
        ::pthread_cond_broadcast(&shm.changed);
        ::pthread_mutex_unlock(&shm.mutex);
    }

    shm_utils::Check(result, "daemon SHM snapshot trylock");
    const auto& motors = shm.motors[shm.motor_sequence % kShmBufferCount];
    const auto& model = shm.model[shm.model_sequence % kShmBufferCount];
    const auto& control = shm.control[shm.control_sequence % kShmBufferCount];
    const std::array<std::uint64_t, 4> sequence{motors.command_sequence,
        motors.feedback_sequence,
        shm.model_sequence,
        shm.control_sequence};
    const std::array<std::uint64_t, 4> publication{motors.command_published_ns,
        motors.feedback_published_ns,
        model.published_ns,
        control.published_ns};
    const auto control_sequence = control.command_sequence;
    const auto control_published_ns = control.published_ns;
    const auto input_model_published_ns = control.input_model_published_ns;
    const auto received_ns = motors.received_ns;
    const auto received_sequence = motors.received_sequence;
    const auto consumed_command_sequence = motors.consumed_command_sequence;
    stopping_ = shm.stopping;
    failed_ = shm.failed;
    can_enabled_ = shm.can_enabled;
    shm_utils::Check(::pthread_mutex_unlock(&shm.mutex), "daemon SHM snapshot unlock");
    const auto now = shm_utils::MonotonicNs();
    ++samples_;

    for (std::size_t i = 0; i < sequence.size(); ++i) {
        if (!sequence[i] || !publication[i])
            continue;

        if (publication[i] > now) {
            ++invalid_times_;
            continue;
        }

        ages_[i].Add(now, (now - publication[i]) / 1000.0);

        if (previous_sequence_[i] && sequence[i] != previous_sequence_[i]) {
            if (sequence[i] < previous_sequence_[i] || publication[i] <= previous_publication_[i]) {
                ++regressions_;
            } else {
                const auto delta = sequence[i] - previous_sequence_[i];
                gaps_[i].Add(now, double(delta - 1));
                intervals_[i].Add(
                    now, (publication[i] - previous_publication_[i]) / (1000.0 * delta));
            }
        }

        previous_sequence_[i] = sequence[i];
        previous_publication_[i] = publication[i];
    }

    if (control_sequence && control_sequence != previous_control_sequence_ &&
        input_model_published_ns && control_published_ns >= input_model_published_ns &&
        control_published_ns <= now)
        model_to_control_.Add(now, (control_published_ns - input_model_published_ns) / 1000.0);

    previous_control_sequence_ = control_sequence;

    if (can_enabled_ && sequence[0] && consumed_command_sequence <= sequence[0])
        unconsumed_commands_.Add(now, double(sequence[0] - consumed_command_sequence));

    bool all_encoders_received = true;
    std::uint64_t oldest_encoder_ns = now;

    for (std::size_t i = 0; i < received_ns.size(); ++i) {
        if (!received_sequence[i] || !received_ns[i] || received_ns[i] > now)
            all_encoders_received = false;
        else
            oldest_encoder_ns = std::min(oldest_encoder_ns, received_ns[i]);
    }

    if (all_encoders_received)
        encoder_age_.Add(now, (now - oldest_encoder_ns) / 1000.0);
}

std::string ShmStatistics::Report(std::uint64_t now_ns, bool color) {
    const char* reset = color ? "\033[0m" : "";
    const char* title = color ? "\033[1;36m" : "";
    const char* dim = color ? "\033[2m" : "";
    const char* status_color =
        color ? (failed_ ? "\033[1;31m" : (stopping_ ? "\033[1;33m" : "\033[1;32m")) : "";
    std::ostringstream text;
    text << title << "QHRR-05 | SHM DASHBOARD" << reset << "   " << status_color
         << (failed_ ? "FAILED" : (stopping_ ? "STOPPING" : "RUNNING")) << reset << '\n'
         << name_ << " | " << (can_enabled_ ? "CAN" : "DEBUG / DIRECT") << " | ABI "
         << kShmAbiVersion << " | " << kShmByteSize << " bytes | window " << window_ns_ / 1e9
         << " s\n"
         << dim << "reads=" << samples_ << "  busy skips=" << busy_ << "  regress=" << regressions_
         << "  bad-ts=" << invalid_times_ << reset << '\n'
         << std::left << std::setw(26) << "Metric / channel" << std::right << std::setw(10) << "min"
         << std::setw(10) << "max" << std::setw(10) << "mean" << std::setw(10) << "sigma"
         << std::setw(6) << "N" << std::setw(11) << "latest" << "  trend\n";
    const auto number = [](double value) {
        std::ostringstream output;
        if (std::abs(value) >= 1e6)
            output << std::scientific << std::setprecision(1);
        else
            output << std::fixed << std::setprecision(2);
        output << value;
        return output.str();
    };
    const auto row = [&](const std::string& name, WindowStatistics& values) {
        const auto summary = values.Read(now_ns, window_ns_);
        text << std::left << std::setw(26) << name << std::right;
        if (summary.count)
            text << std::setw(10) << number(summary.minimum) << std::setw(10)
                 << number(summary.maximum) << std::setw(10) << number(summary.mean)
                 << std::setw(10) << number(summary.standard_deviation) << std::setw(6)
                 << summary.count << std::setw(11) << number(summary.latest) << "  |"
                 << summary.trend << '|';
        else
            text << dim << "  N/A -- no samples in window" << reset;
        text << '\n';
    };
    const auto section = [&](const char* heading) {
        text << title << heading << reset << '\n';
    };
    const std::array<const char*, 4> names{"command", "feedback", "model", "control"};

    section("--- DATA AGE [us] ---------------------------------------------------------------");
    for (std::size_t i = 0; i < names.size(); ++i)
        row(std::string("Age us / ") + names[i], ages_[i]);
    row("RX worst age us", encoder_age_);

    section("--- PUBLICATION / INPUT [us] ----------------------------------------------------");
    for (std::size_t i = 0; i < names.size(); ++i)
        row(std::string("Pub mean us / ") + names[i], intervals_[i]);
    row("Model->control lag us", model_to_control_);

    section("--- SEQUENCE DISTANCE [#] -------------------------------------------------------");
    for (std::size_t i = 0; i < names.size(); ++i)
        row(std::string("Obs skips # / ") + names[i], gaps_[i]);
    row("Cmd unconsumed distance #", unconsumed_commands_);

    text
        << dim
        << "Window min/max/mean/sigma | trend: oldest->newest, normalized per row (no alarm threshold).\n"
        << "Pub=averaged interval; skips are NOT consumer loss. N/A: mutex wait/hold, IPC overhead."
        << reset << '\n';
    return text.str();
}
