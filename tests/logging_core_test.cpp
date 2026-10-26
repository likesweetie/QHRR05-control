#include "shm_logger/log_schema.hpp"
#include "shm_logger/session_storage.hpp"
#include "ipc/shm_utils.hpp"
#include "data_tamer_v2/mcap_reader.hpp"
#include "data_tamer_v2/sinks/mcap_sink.hpp"
#include "data_tamer_v2/sinks/mcap_ring_sink.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sys/wait.h>
#include <type_traits>

using namespace qhrr05::logging;
using namespace std::chrono_literals;

void Require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

// Seed every explicitly described field with a nonzero value, including nested arrays/enums.
template <typename T> void Fill(T& value) {
    if constexpr (std::is_enum_v<T>)
        value = static_cast<T>(1);
    else if constexpr (std::is_same_v<T, bool>)
        value = true;
    else if constexpr (std::is_floating_point_v<T>)
        value = 1.25;
    else if constexpr (std::is_arithmetic_v<T>)
        value = 17;
    else if constexpr (DataTamer2::has_TypeDefinition<T>::value) {
        auto add = [](std::string_view, auto* field) {
            Fill(*field);
        };
        DataTamer2::TypeDefinitionTrait<T>::define(value, add);
    } else {
        for (auto& item : value)
            Fill(item);
    }
}

class CountingSink : public DataTamer2::DataSink {
    void onSchema(const DataTamer2::Schema&) override {
    }

    void onSnapshot(const DataTamer2::SnapshotRef&) override {
    }
};

int main() {
    char directory[] = "/tmp/qhrr05-log-core-XXXXXX";
    try {
        Require(::mkdtemp(directory), "temp directory failed");
        const fs::path root(directory);
        const auto name = "/qhrr05-log-core-" + std::to_string(::getpid());
        shm_utils::Mapping owner(name, true);
        auto& shm = *owner.get();
        SnapshotReader reader(name);
        LogFrame frame;
        {
            shm_utils::Lock lock(shm);
            shm.motor_sequence = 3;
            shm.motors[1].feedback[2].position = .7;
            Require(!reader.Read(frame), "trylock blocked instead of reporting busy");
        }
        Require(reader.Read(frame) && frame.motor_sequence == 3 &&
                    frame.motor.feedback[2].position == .7,
            "wrong current published slot copied");
        Require(reader.Counters().lock_misses == 1 && reader.Counters().successes == 1,
            "lock/copy counters incorrect");
        Require(!shm.stopping && !shm.failed, "logger changed robot flags");
        Require(reader.SameObject(), "identity rejected original SHM");
        {
            const pid_t child = ::fork();
            Require(child >= 0, "fork failed");
            if (!child) {
                ::pthread_mutex_lock(&shm.mutex);
                ::_exit(0);
            }
            ::waitpid(child, nullptr, 0);
            Require(!reader.Read(frame) && reader.Counters().recoveries == 1,
                "owner death did not skip torn snapshot");
            Require(reader.Read(frame) && !shm.stopping && !shm.failed,
                "recovery interfered with robot state");
        }
        {
            const auto old = shm.abi_version;
            shm.abi_version = old + 1;
            bool rejected = false;
            try {
                SnapshotReader bad(name);
            } catch (const std::exception&) {
                rejected = true;
            }
            shm.abi_version = old;
            Require(rejected, "ABI mismatch was accepted");
        }
        ::shm_unlink(name.c_str());
        {
            shm_utils::Mapping replacement(name, true);
            Require(!reader.SameObject(), "recreated SHM silently reused");
        }
        std::cout << "PASS: snapshot slots, trylock, robust recovery, ABI and replacement\n";

        Fill(frame.motor);
        Fill(frame.model);
        Fill(frame.control);
        Fill(frame.simulation);
        frame.capture_ns = 1000000;
        auto channel = DataTamer2::LogChannel::create("qhrr/state");
        auto gt = RegisterState(*channel, frame, true);
        auto disk = DataTamer2::MCAPSink::create((root / "roundtrip.mcap").string());
        channel->addDataSink(disk);
        channel->setEnabled(*gt, true);
        channel->startLogging();
        Require(DataTamer2::accepted(channel->tryTakeSnapshot(1ms)), "schema snapshot failed");
        bool frozen = false;
        try {
            channel->registerValue("late", &frame.capture_ns);
        } catch (const std::exception&) {
            frozen = true;
        }
        Require(frozen, "schema was not frozen");
        disk->stop();
        std::map<std::string, double> decoded;
        DataTamer2::ForEachValue((root / "roundtrip.mcap").string(),
            [&](const auto&, auto stamp, const std::string& key, const auto& value) {
                Require(stamp == 1000000, "monotonic timestamp changed");
                decoded[key] = std::visit(
                    [](auto v) {
                        return double(v);
                    },
                    value);
            });
        Require(decoded.size() > 190, "schema missing arrays/structure fields");
        Require(decoded.at("motor/HR/command/kp") == 1.25 &&
                    decoded.at("motor/feedback[2]/temperature") == 17 &&
                    decoded.at("model/jacobian[8]") == 1.25 &&
                    decoded.at("control/magnetic/phase") == 1 &&
                    decoded.at("control/joystick_buttons[10]") == 1 &&
                    decoded.at("simulation/ground_truth/contact_event_sequence") == 17,
            "roundtrip lost nested motor/model/control/GT values");
        std::cout << "PASS: " << decoded.size()
                  << " numeric fields, enums, arrays, schema freeze and MCAP roundtrip\n";

        auto bounded = DataTamer2::LogChannel::create("bounded");
        auto slow = std::make_shared<DataTamer2::SinkWorkerOf<CountingSink>>(
            std::make_unique<CountingSink>(), DataTamer2::SinkWorker::Delivery::Manual);
        bounded->registerValue("x", &frame.capture_ns);
        bounded->setPoolCapacity(2);
        bounded->addDataSink(slow);
        bounded->startLogging();
        Require(DataTamer2::accepted(bounded->tryTakeSnapshot(1ms)), "pool sample 1");
        Require(DataTamer2::accepted(bounded->tryTakeSnapshot(2ms)), "pool sample 2");
        Require(bounded->tryTakeSnapshot(3ms) == DataTamer2::SnapshotResult::pool_exhausted,
            "stalled sink did not exhaust the bounded pool");
        slow->drain();
        Require(DataTamer2::accepted(bounded->tryTakeSnapshot(4ms)), "pool did not recover");
        slow->stop();
        std::cout << "PASS: slow sink bounded memory and observable pool exhaustion\n";

        DataTamer2::MCAPRingOptions options;
        options.filepath = (root / "fault.mcap").string();
        options.window = 5s;
        options.capacity_bytes = 64 * 1024;
        auto ring = DataTamer2::MCAPRingSink::create(options);
        auto flight = DataTamer2::LogChannel::create("flight");
        flight->registerValue("x", &frame.capture_ns);
        flight->addDataSink(ring);
        flight->startLogging();
        DataTamer2::MCAPRingDump dump;
        ring->sink().setDumpCallback([&](const auto& outcome) {
            dump = outcome;
        });
        for (int i = 0; i <= 900; ++i) {
            if (i == 600) {
                Require(ring->sink().requestDump(2s), "trigger refused");
                Require(!ring->sink().requestDump(2s), "duplicate trigger accepted");
            }
            Require(
                DataTamer2::accepted(flight->tryTakeSnapshot(std::chrono::milliseconds(i * 10))),
                "flight sample lost");
            ring->drain();
        }
        ring->stop();
        Require(dump.ok && !dump.truncated && dump.first_message == 1s && dump.last_message == 8s,
            "flight 5-second pre / 2-second post bounds incorrect");
        auto tiny_options = options;
        tiny_options.filepath = (root / "tiny.mcap").string();
        tiny_options.capacity_bytes = 256;
        auto tiny = DataTamer2::MCAPRingSink::create(tiny_options);
        auto small = DataTamer2::LogChannel::create("small");
        small->registerValue("x", &frame.capture_ns);
        small->addDataSink(tiny);
        small->startLogging();
        for (int i = 0; i < 100; ++i) {
            (void)small->tryTakeSnapshot(std::chrono::milliseconds(i));
            tiny->drain();
        }
        Require(
            tiny->sink().stats().evicted_by_capacity > 0, "small RAM ring did not report eviction");
        Require(tiny->sink().requestDump(2s), "pending shutdown trigger failed");
        tiny->stop();
        Require(tiny->sink().stats().dumps_written == 1, "pending dump lost on shutdown");
        std::cout << "PASS: pre/post, duplicates, capacity eviction and pending dump shutdown\n";

        const auto storage_root = root / "storage";
        fs::create_directories(storage_root / "old/faults");
        std::ofstream(storage_root / "old/session.json") << "{}";
        std::ofstream(storage_root / "old/a.mcap") << std::string(100, 'a');
        std::ofstream(storage_root / "old/faults/f.mcap") << std::string(100, 'f');
        fs::last_write_time(storage_root / "old/a.mcap",
            fs::file_time_type::clock::now() - std::chrono::hours(24 * 365));
        SessionStorage ample_storage(storage_root, 1000, 0, true);
        ample_storage.Enforce({});
        Require(fs::exists(storage_root / "old/a.mcap"),
            "old recording removed despite sufficient storage capacity");

        SessionStorage storage(storage_root, 150, 0, true);
        storage.Enforce({});
        Require(!fs::exists(storage_root / "old/a.mcap") &&
                    fs::exists(storage_root / "old/faults/f.mcap"),
            "FIFO retention deleted a protected fault");
        std::ofstream(storage_root / "old/current.mcap") << std::string(100, 'c');
        bool full = false;
        try {
            storage.Enforce({storage_root / "old/current.mcap"});
        } catch (const std::exception&) {
            full = true;
        }
        Require(full && fs::exists(storage_root / "old/current.mcap"),
            "active file not protected at budget limit");
        std::cout << "PASS: FIFO storage, protected faults and active-file budget exhaustion\n";
        fs::remove_all(root);
        return 0;
    } catch (const std::exception& e) {
        fs::remove_all(directory);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
