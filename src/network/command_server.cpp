#include "rb_servo/network/command_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "rb_servo/core/clock.hpp"

namespace rb_servo {
namespace {

struct UdpEndpoint {
    std::string host = "0.0.0.0";
    int port = 0;
};

UdpEndpoint parseUdpUri(const std::string& uri) {
    const std::string prefix = "udp://";
    if (uri.rfind(prefix, 0) != 0) {
        throw std::runtime_error("Only udp:// command_bind is supported now: " + uri);
    }
    const std::string rest = uri.substr(prefix.size());
    const auto colon = rest.rfind(':');
    if (colon == std::string::npos) {
        throw std::runtime_error("Invalid udp uri: " + uri);
    }
    UdpEndpoint ep;
    ep.host = rest.substr(0, colon);
    ep.port = std::stoi(rest.substr(colon + 1));
    return ep;
}

std::string trim(std::string s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::string stripQuotes(std::string s) {
    s = trim(s);
    if (s.size() >= 2 && ((s.front() == '"' && s.back() == '"') ||
                          (s.front() == '\'' && s.back() == '\''))) {
        return s.substr(1, s.size() - 2);
    }
    return s;
}

bool extractString(const std::string& text, const std::string& key, std::string* out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
    std::smatch m;
    if (!std::regex_search(text, m, re)) return false;
    *out = m[1].str();
    return true;
}

bool extractDouble(const std::string& text, const std::string& key, double* out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?(?:[eE][-+]?[0-9]+)?)");
    std::smatch m;
    if (!std::regex_search(text, m, re)) return false;
    *out = std::stod(m[1].str());
    return true;
}

bool extractUint64(const std::string& text, const std::string& key, uint64_t* out) {
    double v = 0.0;
    if (!extractDouble(text, key, &v)) return false;
    *out = static_cast<uint64_t>(v);
    return true;
}

bool extractBool(const std::string& text, const std::string& key, bool* out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false|1|0)", std::regex_constants::icase);
    std::smatch m;
    if (!std::regex_search(text, m, re)) return false;
    const std::string v = m[1].str();
    *out = (v == "true" || v == "True" || v == "TRUE" || v == "1");
    return true;
}

bool extractArray(const std::string& text, const std::string& key, std::vector<double>* out) {
    const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\[([^\\]]*)\\]");
    std::smatch m;
    if (!std::regex_search(text, m, re)) return false;
    out->clear();
    std::stringstream ss(m[1].str());
    std::string token;
    while (std::getline(ss, token, ',')) {
        token = trim(token);
        if (!token.empty()) out->push_back(std::stod(token));
    }
    return true;
}

bool copyJointArray(const std::vector<double>& values, JointArray* out) {
    if (values.size() != kDof) return false;
    for (int i = 0; i < kDof; ++i) (*out)[i] = values[static_cast<size_t>(i)];
    return true;
}

bool copyPose6D(const std::vector<double>& values, Pose6D* out) {
    if (values.size() != 6) return false;
    *out = Pose6D{values[0], values[1], values[2], values[3], values[4], values[5]};
    return true;
}

bool copyWrench6D(const std::vector<double>& values, Wrench6D* out) {
    if (values.size() != 6) return false;
    *out = Wrench6D{values[0], values[1], values[2], values[3], values[4], values[5]};
    return true;
}

std::string extractObject(const std::string& text, const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const size_t key_pos = text.find(needle);
    if (key_pos == std::string::npos) return "";
    const size_t brace_begin = text.find('{', key_pos + needle.size());
    if (brace_begin == std::string::npos) return "";
    int depth = 0;
    for (size_t i = brace_begin; i < text.size(); ++i) {
        if (text[i] == '{') ++depth;
        else if (text[i] == '}') {
            --depth;
            if (depth == 0) return text.substr(brace_begin, i - brace_begin + 1);
        }
    }
    return "";
}

void parseForceControlObject(const std::string& object, ForceControlCommand* cmd) {
    const std::string force = extractObject(object, "force_control");
    if (force.empty()) return;

    std::string mode;
    if (extractString(force, "mode", &mode)) cmd->mode = forceControlModeFromString(mode);

    std::vector<double> arr;
    if (extractArray(force, "target_wrench", &arr)) copyWrench6D(arr, &cmd->target_wrench);
    if (extractDouble(force, "max_pos_offset_m", &cmd->max_pos_offset_m)) {}
    if (extractDouble(force, "max_rot_offset_rad", &cmd->max_rot_offset_rad)) {}
    if (extractDouble(force, "max_pos_step_m", &cmd->max_pos_step_m)) {}
    if (extractDouble(force, "max_rot_step_rad", &cmd->max_rot_step_rad)) {}

    const std::string axis = extractObject(force, "enabled_axis");
    if (!axis.empty()) {
        extractBool(axis, "x", &cmd->enabled_axis.x);
        extractBool(axis, "y", &cmd->enabled_axis.y);
        extractBool(axis, "z", &cmd->enabled_axis.z);
        extractBool(axis, "roll", &cmd->enabled_axis.roll);
        extractBool(axis, "pitch", &cmd->enabled_axis.pitch);
        extractBool(axis, "yaw", &cmd->enabled_axis.yaw);
    }
}

void parseArmObject(
    const std::string& object,
    ArmId arm_id,
    uint64_t seq,
    uint64_t receive_time_ns,
    ControlMode default_mode,
    double default_timeout_sec,
    ArmCommand* out
) {
    out->arm_id = arm_id;
    out->seq = seq;
    out->host_time_ns = receive_time_ns;
    out->mode = default_mode;
    out->timeout_sec = default_timeout_sec;

    if (!object.empty()) {
        std::string mode;
        if (extractString(object, "mode", &mode)) out->mode = controlModeFromString(mode);
        double timeout = 0.0;
        if (extractDouble(object, "timeout_sec", &timeout)) out->timeout_sec = timeout;
        double gripper = 0.0;
        if (extractDouble(object, "gripper_target", &gripper) || extractDouble(object, "gripper", &gripper)) {
            out->gripper_target = gripper;
        }

        std::vector<double> arr;
        if (extractArray(object, "q_target_deg", &arr)) out->has_joint_target = copyJointArray(arr, &out->q_target_deg);
        if (extractArray(object, "dq_target_deg_s", &arr)) out->has_joint_velocity = copyJointArray(arr, &out->dq_target_deg_s);
        if (extractArray(object, "tcp_target_stand", &arr)) out->has_tcp_target = copyPose6D(arr, &out->tcp_target_stand);
        if (extractArray(object, "tcp_delta_stand", &arr)) out->has_tcp_delta_stand = copyPose6D(arr, &out->tcp_delta_stand);
        if (extractArray(object, "tcp_delta_local", &arr)) out->has_tcp_delta_local = copyPose6D(arr, &out->tcp_delta_local);
        parseForceControlObject(object, &out->force_control);
    }
}

}  // namespace

CommandServer::CommandServer(
    const NetworkConfig& config,
    CommandBuffer* command_buffer
) : config_(config), command_buffer_(command_buffer) {}

CommandServer::~CommandServer() {
    stop();
}

bool CommandServer::start() {
    if (running_) return true;
    running_ = true;
    thread_ = std::thread(&CommandServer::threadMain, this);
    return true;
}

void CommandServer::stop() {
    if (!running_) return;
    running_ = false;
    if (socket_fd_ >= 0) {
        ::shutdown(socket_fd_, SHUT_RDWR);
        ::close(socket_fd_);
        socket_fd_ = -1;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
}

void CommandServer::threadMain() {
    try {
        const UdpEndpoint ep = parseUdpUri(config_.command_bind);
        socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            throw std::runtime_error(std::string("socket() failed: ") + std::strerror(errno));
        }

        int reuse = 1;
        ::setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(ep.port));
        if (::inet_pton(AF_INET, ep.host.c_str(), &addr.sin_addr) != 1) {
            throw std::runtime_error("Invalid bind host: " + ep.host);
        }
        if (::bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            throw std::runtime_error(std::string("bind() failed: ") + std::strerror(errno));
        }

        std::cerr << "[INFO] CommandServer listening on " << config_.command_bind << "\n";
        std::array<char, 8192> buffer{};
        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(socket_fd_, &fds);
            timeval tv{};
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            const int ready = ::select(socket_fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (ready <= 0) continue;

            sockaddr_in src{};
            socklen_t src_len = sizeof(src);
            const ssize_t n = ::recvfrom(socket_fd_, buffer.data(), buffer.size() - 1, 0,
                                         reinterpret_cast<sockaddr*>(&src), &src_len);
            if (n <= 0) continue;
            buffer[static_cast<size_t>(n)] = '\0';
            const uint64_t receive_time_ns = nowSteadyNs();
            DualArmCommand cmd;
            if (parseMessage(std::string(buffer.data(), static_cast<size_t>(n)), receive_time_ns, &cmd)) {
                if (command_buffer_) command_buffer_->setCommand(cmd);
            } else {
                std::cerr << "[WARN] failed to parse command packet\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] CommandServer failed: " << e.what() << "\n";
        running_ = false;
    }
}

bool CommandServer::parseMessage(
    const std::string& message,
    uint64_t receive_time_ns,
    DualArmCommand* out_command
) {
    if (!out_command) return false;

    DualArmCommand cmd;
    cmd.host_time_ns = receive_time_ns;  // authoritative timestamp for timeout checks

    extractUint64(message, "seq", &cmd.seq);
    extractBool(message, "coupled_timeout", &cmd.coupled_timeout);

    std::string mode_string = "Hold";
    extractString(message, "mode", &mode_string);
    const ControlMode default_mode = controlModeFromString(mode_string);

    double timeout_sec = 0.2;
    extractDouble(message, "timeout_sec", &timeout_sec);

    const std::string left_object = extractObject(message, "left");
    const std::string right_object = extractObject(message, "right");

    parseArmObject(left_object, ArmId::Left, cmd.seq, receive_time_ns, default_mode, timeout_sec, &cmd.left);
    parseArmObject(right_object, ArmId::Right, cmd.seq, receive_time_ns, default_mode, timeout_sec, &cmd.right);

    if (left_object.empty() && right_object.empty() && default_mode == ControlMode::JointTarget) {
        std::vector<double> arr;
        if (extractArray(message, "q_target_deg", &arr)) {
            cmd.left.has_joint_target = copyJointArray(arr, &cmd.left.q_target_deg);
            cmd.right.has_joint_target = copyJointArray(arr, &cmd.right.q_target_deg);
        }
    }

    // Fail-safe validation: never allow missing payloads to become zero joint arrays.
    // Invalid motion commands are converted to Hold; the servo loop also checks the flags.
    auto sanitize = [](ArmCommand* arm) {
        if (!arm) return;
        if ((arm->mode == ControlMode::JointTarget && !arm->has_joint_target) ||
            (arm->mode == ControlMode::JointVelocity && !arm->has_joint_velocity) ||
            (arm->mode == ControlMode::TcpPoseTarget && !arm->has_tcp_target) ||
            (arm->mode == ControlMode::TcpDeltaStand && !arm->has_tcp_delta_stand) ||
            (arm->mode == ControlMode::TcpDeltaLocal && !arm->has_tcp_delta_local)) {
            std::cerr << "[WARN] command seq " << arm->seq
                      << " has mode " << toString(arm->mode)
                      << " but missing required payload; converted to Hold\n";
            arm->mode = ControlMode::Hold;
        }
    };
    sanitize(&cmd.left);
    sanitize(&cmd.right);

    *out_command = cmd;
    return true;
}

}  // namespace rb_servo
