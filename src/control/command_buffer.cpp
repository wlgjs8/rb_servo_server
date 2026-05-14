#include "rb_servo/control/command_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace rb_servo {
namespace {
DualArmCommand makeHold(uint64_t now_ns) {
    DualArmCommand hold;
    hold.host_time_ns = now_ns;
    hold.left.arm_id = ArmId::Left;
    hold.right.arm_id = ArmId::Right;
    hold.left.mode = ControlMode::Hold;
    hold.right.mode = ControlMode::Hold;
    return hold;
}

bool validTimeout(double timeout_sec) {
    return timeout_sec > 0.0 && std::isfinite(timeout_sec);
}
}  // namespace

void CommandBuffer::setCommand(const DualArmCommand& command) {
    latest_command_.set(command);
}

DualArmCommand CommandBuffer::latestOrHold(uint64_t now_ns) const {
    auto maybe = latest_command_.get();
    if (!maybe.has_value()) {
        return makeHold(now_ns);
    }

    DualArmCommand cmd = *maybe;
    if (!validTimeout(cmd.left.timeout_sec) || !validTimeout(cmd.right.timeout_sec)) {
        return makeHold(now_ns);
    }

    const double timeout = std::min(cmd.left.timeout_sec, cmd.right.timeout_sec);
    const uint64_t timeout_ns = static_cast<uint64_t>(timeout * 1e9);
    const bool stale = cmd.host_time_ns > 0 && now_ns > cmd.host_time_ns + timeout_ns;

    if (stale) {
        cmd.left.mode = ControlMode::Hold;
        cmd.right.mode = ControlMode::Hold;
    }
    return cmd;
}

}  // namespace rb_servo
