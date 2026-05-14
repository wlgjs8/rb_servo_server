#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"

namespace rb_servo {

class CommandServer {
public:
    CommandServer(
        const NetworkConfig& config,
        CommandBuffer* command_buffer
    );

    ~CommandServer();

    bool start();
    void stop();

    bool parseMessage(
        const std::string& message,
        uint64_t receive_time_ns,
        DualArmCommand* out_command
    );

private:
    void threadMain();

private:
    NetworkConfig config_;
    CommandBuffer* command_buffer_ = nullptr;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
