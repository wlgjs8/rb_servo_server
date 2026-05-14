#include "rb_servo/network/state_publisher.hpp"

#include <chrono>
#include <iostream>

namespace rb_servo {

StatePublisher::StatePublisher(const NetworkConfig& config) : config_(config) {}

StatePublisher::~StatePublisher() {
    stop();
}

void StatePublisher::updateState(const DualRobotState& state) {
    latest_state_.set(state);
}

bool StatePublisher::start() {
    if (running_) return true;
    running_ = true;
    thread_ = std::thread(&StatePublisher::threadMain, this);
    return true;
}

void StatePublisher::stop() {
    if (!running_) return;
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

void StatePublisher::threadMain() {
    // TODO: implement optional state publisher.
    std::cerr << "[WARN] StatePublisher is not implemented yet: " << config_.state_pub_bind << "\n";
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

}  // namespace rb_servo
