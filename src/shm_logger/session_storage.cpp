#include "shm_logger/session_storage.hpp"
#include "ipc/shm_utils.hpp"
#include <openssl/evp.h>
#include <sys/file.h>
#include <array>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <regex>
#include <sstream>
#include <vector>
#include <map>
#include <memory>
#include <iostream>

namespace qhrr05::logging {
namespace {
std::string Quote(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\')
            out << '\\' << char(c);
        else if (c < 32)
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
        else
            out << char(c);
    }
    out << '"';
    return out.str();
}
}

std::string Json(const YAML::Node& node) {
    if (!node || node.IsNull())
        return "null";
    if (node.IsScalar()) {
        const auto s = node.Scalar();
        static const std::regex number("-?(0|[1-9][0-9]*)(\\.[0-9]+)?([eE][+-]?[0-9]+)?");
        if (node.Tag() != "!" && node.Tag() != "tag:yaml.org,2002:str" &&
            (s == "true" || s == "false" || std::regex_match(s, number)))
            return s;
        return Quote(s);
    }

    std::string out = node.IsMap() ? "{" : "[";
    bool first = true;
    if (node.IsMap()) {
        for (const auto& item : node) {
            if (!first)
                out += ',';
            first = false;
            out += Quote(item.first.as<std::string>()) + ':' + Json(item.second);
        }
    } else {
        for (const auto& item : node) {
            if (!first)
                out += ',';
            first = false;
            out += Json(item);
        }
    }
    return out + (node.IsMap() ? '}' : ']');
}

void WriteJson(const fs::path& path, const YAML::Node& node) {
    const auto temporary = path.string() + ".tmp";
    std::ofstream out(temporary, std::ios::trunc);
    out << Json(node) << '\n';
    out.close();
    if (!out)
        throw std::runtime_error("cannot write metadata: " + path.string());
    fs::rename(temporary, path);
}

std::string Sha256(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input)
        throw std::runtime_error("cannot hash source file: " + path.string());
    auto context =
        std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("SHA256 initialization failed");
    std::array<char, 65536> buffer{};
    while (input) {
        input.read(buffer.data(), buffer.size());
        if (EVP_DigestUpdate(context.get(), buffer.data(), input.gcount()) != 1)
            throw std::runtime_error("SHA256 update failed");
    }
    if (!input.eof())
        throw std::runtime_error("SHA256 source read failed: " + path.string());
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned size = 0;
    if (EVP_DigestFinal_ex(context.get(), digest.data(), &size) != 1)
        throw std::runtime_error("SHA256 finalization failed");
    std::ostringstream out;
    for (unsigned i = 0; i < size; ++i)
        out << std::hex << std::setfill('0') << std::setw(2) << unsigned(digest[i]);
    return out.str();
}

SessionStorage::SessionStorage(const fs::path& root,
    std::uint64_t maximum_bytes,
    std::uint64_t minimum_free_bytes,
    bool protect_faults)
    : root_(fs::absolute(root)), maximum_bytes_(maximum_bytes),
      minimum_free_bytes_(minimum_free_bytes), protect_faults_(protect_faults) {
    fs::create_directories(root_);
    fs::permissions(root_, fs::perms::owner_all, fs::perm_options::replace);
}

SessionStorage::~SessionStorage() {
    if (lease_fd_ >= 0)
        ::close(lease_fd_);
}

fs::path SessionStorage::Begin(const std::string& label,
    const fs::path& logger_config,
    const fs::path& daemon_config,
    const std::string& runtime_mode,
    const std::string& shm_name,
    double rate_hz,
    const YAML::Node& sources) {
    const auto utc_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
                            .count();
    const auto monotonic = shm_utils::MonotonicNs();
    const auto utc = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    ::gmtime_r(&utc, &tm);
    std::ostringstream name;
    name << std::put_time(&tm, "%Y-%m-%d_%H%M%S") << '_' << label << '_' << ::getpid() << '_'
         << monotonic;
    directory_ = root_ / name.str();
    if (!fs::create_directory(directory_))
        throw std::runtime_error("session directory already exists");
    if (lease_fd_ >= 0)
        ::close(lease_fd_);
    lease_fd_ = ::open((directory_ / "active.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lease_fd_ < 0 || ::flock(lease_fd_, LOCK_EX | LOCK_NB) != 0)
        throw std::runtime_error("cannot lock the recording session");
    fs::create_directory(directory_ / "config");
    fs::create_directory(directory_ / "faults");

    metadata_ = YAML::Node(YAML::NodeType::Map);
    metadata_["logging_schema_version"] = 1;
    metadata_["session_id"] = name.str();
    metadata_["start_utc_ns"] = utc_ns;
    metadata_["monotonic_origin_ns"] = monotonic;
    metadata_["git_commit"] = QHRR_GIT_COMMIT;
    metadata_["git_commit"].SetTag("!");
    metadata_["git_dirty"] = bool(QHRR_GIT_DIRTY);
    metadata_["build_compiler"] = QHRR_BUILD_COMPILER;
    metadata_["build_type"] = QHRR_BUILD_TYPE;
    metadata_["data_tamer_version"] = DATA_TAMER_VERSION;
    metadata_["robot_model"] = "qhrr-05";
    metadata_["runtime_mode"] = runtime_mode;
    metadata_["shm_name"] = shm_name;
    metadata_["shm_abi"] = kShmAbiVersion;
    metadata_["sampling_rate_hz"] = rate_hz;
    metadata_["result"] = "recording";
    metadata_["time_basis"] = "CLOCK_MONOTONIC; start_utc_ns maps to monotonic_origin_ns";
    metadata_["limits"] =
        "best effort; channel publications are asynchronous; no lossless producer history";

    std::map<std::string, fs::path> paths;
    paths["shm_logger"] = logger_config;
    if (!daemon_config.empty()) {
        paths["daemon"] = daemon_config;
        const auto daemon = YAML::LoadFile(daemon_config.string());
        // Read both manifests; matching transport is selected below from the attached SHM mode.
        const auto key = sources["without_can"].as<bool>() ? "debug_processes" : "processes";
        for (const auto& process : daemon[key]) {
            const auto cwd = process["working_directory"]
                                 ? fs::absolute(daemon_config.parent_path() /
                                                process["working_directory"].as<std::string>())
                                 : fs::current_path();
            const auto args = process["args"].as<std::vector<std::string>>();
            const auto process_name = process["name"].as<std::string>();
            for (std::size_t i = 0; i + 1 < args.size(); ++i) {
                if (args[i] == "--model")
                    paths["model"] = cwd / args[i + 1];
                if (args[i] == "--control-config")
                    paths["control"] = cwd / args[i + 1];
                if (args[i] == "--hal-config")
                    paths["can_hal"] = cwd / args[i + 1];
                if (args[i] == "--simulation-config")
                    paths["simulation"] = cwd / args[i + 1];
            }
            if (process_name == "control" && !args.empty())
                paths["control"] = cwd / args[0];
            if (process_name == "can_hal" && !args.empty())
                paths["can_hal"] = cwd / args[0];
        }
    } else if (sources["files"]) {
        for (const auto& item : sources["files"])
            paths[item.first.as<std::string>()] =
                logger_config.parent_path() / item.second.as<std::string>();
    }

    if (paths.count("can_hal")) {
        const auto hal = YAML::LoadFile(paths.at("can_hal").string());
        paths["motor"] = paths.at("can_hal").parent_path() / hal["motor_config"].as<std::string>();
    }
    YAML::Node model_manifest;
    for (const auto& [role, path] : paths) {
        const auto source = fs::canonical(path);
        const auto hash = Sha256(source);
        metadata_["sources"][role]["path"] = source.string();
        metadata_["sources"][role]["sha256"] = hash;
        if (role != "model") {
            const auto destination = directory_ / "config" / (role + ".yaml");
            fs::copy_file(source, destination, fs::copy_options::none);
            if (Sha256(destination) != hash)
                throw std::runtime_error(
                    "configuration changed during session snapshot: " + source.string());
            metadata_["sources"][role]["copy"] = fs::relative(destination, directory_).string();
        } else {
            for (const auto& item : fs::recursive_directory_iterator(source.parent_path())) {
                if (item.is_symlink())
                    throw std::runtime_error(
                        "model manifest does not follow symlinks: " + item.path().string());
                if (item.is_regular_file())
                    model_manifest[fs::relative(item.path(), source.parent_path()).string()] =
                        Sha256(item.path());
            }
        }
    }
    WriteJson(directory_ / "model_manifest.json", model_manifest);
    WriteJson(directory_ / "session.json", metadata_);
    return directory_;
}

void SessionStorage::Finish(const std::string& result, const YAML::Node& quality) {
    metadata_["result"] = result;
    metadata_["end_monotonic_ns"] = shm_utils::MonotonicNs();
    metadata_["quality"] = quality;
    WriteJson(directory_ / "quality_report.json", quality);
    WriteJson(directory_ / "session.json", metadata_);
    if (lease_fd_ >= 0) {
        ::close(lease_fd_);
        lease_fd_ = -1;
    }
}

std::uint64_t SessionStorage::Available() const {
    return fs::space(root_).available;
}

void SessionStorage::Enforce(const std::set<fs::path>& active) {
    struct Record {
        fs::path path;
        fs::file_time_type time;
        std::uint64_t size;
        bool protected_record;
    };

    std::vector<Record> records;
    std::vector<fs::path> inactive_sessions;
    std::uint64_t total = 0;
    for (const auto& session : fs::directory_iterator(root_)) {
        if (session.is_symlink() || !session.is_directory() ||
            !fs::is_regular_file(session.path() / "session.json"))
            continue;
        bool protected_session = fs::exists(session.path() / ".protect");
        if (session.path() != directory_) {
            const int lease =
                ::open((session.path() / "active.lock").c_str(), O_RDONLY | O_CLOEXEC);
            if (lease >= 0) {
                if (::flock(lease, LOCK_EX | LOCK_NB) != 0) {
                    const int error = errno;
                    ::close(lease);
                    if (error != EWOULDBLOCK)
                        shm_utils::Check(error, "session lease probe");
                    protected_session = true;
                } else
                    ::close(lease);
            } else if (errno != ENOENT)
                shm_utils::Check(errno, "session lease open");
        }
        if (!protected_session && !(session.path() == directory_ && lease_fd_ >= 0))
            inactive_sessions.push_back(session.path());
        for (const auto& entry : fs::recursive_directory_iterator(session.path())) {
            if (entry.is_symlink())
                throw std::runtime_error(
                    "logging storage contains a symlink: " + entry.path().string());
            if (!entry.is_regular_file())
                continue;
            const auto size = entry.file_size();
            total += size;
            if (entry.path().extension() != ".mcap")
                continue;
            const bool fault = entry.path().parent_path().filename() == "faults";
            records.push_back({entry.path(),
                entry.last_write_time(),
                size,
                protected_session || (protect_faults_ && fault) ||
                    (active.count(entry.path()) != 0 ||
                        active.count(entry.path().parent_path()) != 0)});
        }
    }
    std::sort(records.begin(), records.end(), [](const Record& a, const Record& b) {
        return a.time < b.time;
    });
    for (const auto& record : records) {
        if (record.protected_record ||
            (total <= maximum_bytes_ && Available() >= minimum_free_bytes_))
            continue;
        fs::remove(record.path);
        total -= record.size;
        std::cout << "[shm_logger] retention removed " << record.path << '\n';
    }
    for (const auto& session : inactive_sessions) {
        bool has_recording = false;
        std::uint64_t remaining = 0;
        for (const auto& item : fs::recursive_directory_iterator(session)) {
            if (item.is_regular_file()) {
                remaining += item.file_size();
                has_recording |= item.path().extension() == ".mcap";
            }
        }
        if (!has_recording && (total > maximum_bytes_ || Available() < minimum_free_bytes_)) {
            fs::remove_all(session);
            total -= remaining;
            std::cout << "[shm_logger] retention removed empty session " << session << '\n';
        }
    }
    if (Available() < minimum_free_bytes_ || total > maximum_bytes_)
        throw std::runtime_error(
            "logging storage budget exhausted; active/protected files retained");
}
}  // namespace qhrr05::logging
