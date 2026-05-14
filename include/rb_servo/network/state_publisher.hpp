#pragma once

#include <atomic>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/thread_safe_buffer.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

struct DualRobotState {
    uint64_t host_time_ns = 0;
    RobotState left;
    RobotState right;
};

class StatePublisher {
public:
    explicit StatePublisher(const NetworkConfig& config);
    ~StatePublisher();

    void updateState(const DualRobotState& state);

    bool start();
    void stop();

private:
    void threadMain();

private:
    NetworkConfig config_;
    ThreadSafeBuffer<DualRobotState> latest_state_;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace rb_servo
