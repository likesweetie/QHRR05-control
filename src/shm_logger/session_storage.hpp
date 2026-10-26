#pragma once

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <set>
#include <string>

namespace qhrr05::logging {
namespace fs = std::filesystem;
std::string Json(const YAML::Node& node);
void WriteJson(const fs::path& path, const YAML::Node& node);
std::string Sha256(const fs::path& path);

class SessionStorage {
public:
    SessionStorage(const fs::path& root,
        std::uint64_t maximum_bytes,
        std::uint64_t minimum_free_bytes,
        bool protect_faults);
    ~SessionStorage();
    SessionStorage(const SessionStorage&) = delete;
    SessionStorage& operator=(const SessionStorage&) = delete;
    fs::path Begin(const std::string& label,
        const fs::path& logger_config,
        const fs::path& daemon_config,
        const std::string& runtime_mode,
        const std::string& shm_name,
        double rate_hz,
        const YAML::Node& sources);
    void Finish(const std::string& result, const YAML::Node& quality);
    void Enforce(const std::set<fs::path>& active);
    std::uint64_t Available() const;

    const fs::path& Directory() const {
        return directory_;
    }

    const fs::path& Root() const {
        return root_;
    }

    const YAML::Node& Metadata() const {
        return metadata_;
    }

private:
    fs::path root_, directory_;
    std::uint64_t maximum_bytes_, minimum_free_bytes_;
    bool protect_faults_;
    YAML::Node metadata_;
    int lease_fd_ = -1;
};
}  // namespace qhrr05::logging
