#include "rb_servo/network/state_publisher.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

namespace rb_servo {
namespace {

constexpr int kStateSchemaVersion = 1;
constexpr std::chrono::milliseconds kPublishPeriod{50};

nlohmann::json jointArrayJson(const JointArray& joints) {
    nlohmann::json out = nlohmann::json::array();
    for (double value : joints) out.push_back(value);
    return out;
}

nlohmann::json poseJson(const Pose6D& pose) {
    return {
        {"x", pose.x},
        {"y", pose.y},
        {"z", pose.z},
        {"rx", pose.rx},
        {"ry", pose.ry},
        {"rz", pose.rz},
    };
}

nlohmann::json armStateJson(
    const RobotState& state,
    const ArmCommand& command,
    const JointArray& sent_q_deg,
    const JointArray& previous_sent_q_deg,
    bool send_ok,
    uint64_t send_start_ns,
    uint64_t send_end_ns,
    double send_duration_us
) {
    return {
        {"mode", toString(command.mode)},
        {"q_actual_deg", jointArrayJson(state.q_actual_deg)},
        {"q_sent_deg", jointArrayJson(sent_q_deg)},
        {"q_previous_sent_deg", jointArrayJson(previous_sent_q_deg)},
        {"send_ok", send_ok},
        {"send_start_ns", send_start_ns},
        {"send_end_ns", send_end_ns},
        {"send_duration_us", send_duration_us},
        {"has_valid_joint_state", state.has_valid_joint_state},
        {"connection_state", state.connection_state == RobotConnectionState::Connected
            ? "Connected"
            : state.connection_state == RobotConnectionState::Error ? "Error" : "Disconnected"},
        {"robot_time_ns", state.robot_time_ns},
        {"host_time_ns", state.host_time_ns},
        {"error_code", state.error_code},
        {"tcp_stand", nullptr},
        {"tcp_base", nullptr},
        {"tcp_deferred", true},
    };
}

}  // namespace

StatePublisher::StatePublisher(const DualArmConfig& config, SnapshotProvider provider)
    : config_(config), snapshot_provider_(std::move(provider)) {}

StatePublisher::StatePublisher(const NetworkConfig& config) {
    config_.network = config;
}

StatePublisher::~StatePublisher() {
    stop();
}

void StatePublisher::updateSnapshot(const ServoSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_snapshot_ = snapshot;
}

std::string StatePublisher::serializeSnapshot(const ServoSnapshot& snapshot) const {
    nlohmann::json message;
    message["schema_version"] = kStateSchemaVersion;
    message["tick"] = snapshot.tick;
    message["loop_start_time_ns"] = snapshot.loop_start_time_ns;
    message["loop_end_time_ns"] = snapshot.loop_end_time_ns;
    message["host_time_ns"] = snapshot.loop_end_time_ns;
    message["period_ms"] = snapshot.period_ms;
    message["jitter_ms"] = snapshot.jitter_ms;
    message["filter_dt_ms"] = snapshot.filter_dt_ms;
    message["command_seq"] = snapshot.command.seq;

    message["left"] = armStateJson(
        snapshot.left_state,
        snapshot.command.left,
        snapshot.left_sent_q_deg,
        snapshot.left_prev_sent_q_deg,
        snapshot.left_send_ok,
        snapshot.left_send_start_ns,
        snapshot.left_send_end_ns,
        snapshot.left_send_duration_us
    );
    message["right"] = armStateJson(
        snapshot.right_state,
        snapshot.command.right,
        snapshot.right_sent_q_deg,
        snapshot.right_prev_sent_q_deg,
        snapshot.right_send_ok,
        snapshot.right_send_start_ns,
        snapshot.right_send_end_ns,
        snapshot.right_send_duration_us
    );

    message["send_skew_us"] = snapshot.send_skew_us;
    message["safety_verdict"] = toString(snapshot.safety_verdict);
    message["motion_state"] = toString(snapshot.motion_state);
    message["fault_latched"] = snapshot.fault_latched;
    message["latched_fault_reason"] = toString(snapshot.latched_fault_reason);
    message["fault_reason"] = snapshot.fault_reason;
    message["logger_dropped_samples"] = snapshot.logger_dropped_samples;
    message["logger_health"] = {
        {"dropped_samples", snapshot.logger_dropped_samples},
        {"ok", snapshot.logger_dropped_samples == 0},
    };
    message["mount_transform_deferred"] = false;
    message["mounts"] = {
        {"left", {
            {"frame", "stand"},
            {"base_pose_in_stand", poseJson(config_.left_mount.base_pose_in_stand)},
        }},
        {"right", {
            {"frame", "stand"},
            {"base_pose_in_stand", poseJson(config_.right_mount.base_pose_in_stand)},
        }},
    };
    message["tcp_fields_deferred"] = true;
    return message.dump();
}

bool StatePublisher::start() {
    if (running_) return true;

    std::string host;
    int port = 0;
    if (!parseEndpoint(&host, &port)) {
        std::cerr << "[ERROR] StatePublisher only supports udp://host:port endpoints, got "
                  << config_.network.state_pub_bind << "\n";
        return false;
    }

    running_ = true;
    thread_ = std::thread(&StatePublisher::threadMain, this);
    return true;
}

void StatePublisher::stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void StatePublisher::threadMain() {
    std::string host;
    int port = 0;
    if (!parseEndpoint(&host, &port)) {
        running_ = false;
        return;
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "[ERROR] StatePublisher socket failed: " << std::strerror(errno) << "\n";
        running_ = false;
        return;
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &dest.sin_addr) != 1) {
        std::cerr << "[ERROR] StatePublisher invalid IPv4 host: " << host << "\n";
        ::close(fd);
        running_ = false;
        return;
    }

    bool send_warned = false;
    while (running_) {
        ServoSnapshot snapshot;
        if (snapshot_provider_) {
            snapshot = snapshot_provider_();
            updateSnapshot(snapshot);
        } else {
            std::lock_guard<std::mutex> lock(snapshot_mutex_);
            snapshot = latest_snapshot_;
        }

        const std::string payload = serializeSnapshot(snapshot);
        const ssize_t sent = ::sendto(
            fd,
            payload.data(),
            payload.size(),
            0,
            reinterpret_cast<const sockaddr*>(&dest),
            sizeof(dest)
        );
        if (sent < 0 && !send_warned) {
            std::cerr << "[WARN] StatePublisher send failed: " << std::strerror(errno) << "\n";
            send_warned = true;
        }
        std::this_thread::sleep_for(kPublishPeriod);
    }

    ::close(fd);
}

bool StatePublisher::parseEndpoint(std::string* host, int* port) const {
    const std::string& endpoint = config_.network.state_pub_bind;
    constexpr const char* prefix = "udp://";
    if (endpoint.rfind(prefix, 0) != 0) return false;

    std::string rest = endpoint.substr(std::strlen(prefix));
    if (rest.empty()) return false;
    if (rest == "localhost") rest = "127.0.0.1";

    const auto colon = rest.rfind(':');
    if (colon == std::string::npos || colon + 1 >= rest.size()) return false;

    std::string parsed_host = rest.substr(0, colon);
    if (parsed_host == "localhost") parsed_host = "127.0.0.1";
    if (parsed_host.empty() || parsed_host == "0.0.0.0") return false;

    int parsed_port = 0;
    try {
        parsed_port = std::stoi(rest.substr(colon + 1));
    } catch (const std::exception&) {
        return false;
    }
    if (parsed_port <= 0 || parsed_port > 65535) return false;
    in_addr addr{};
    if (::inet_pton(AF_INET, parsed_host.c_str(), &addr) != 1) return false;

    if (host) *host = parsed_host;
    if (port) *port = parsed_port;
    return true;
}

}  // namespace rb_servo
