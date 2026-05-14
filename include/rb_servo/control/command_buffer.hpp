#pragma once

#include "rb_servo/core/thread_safe_buffer.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class CommandBuffer {
public:
    void setCommand(const DualArmCommand& command);
    DualArmCommand latestOrHold(uint64_t now_ns) const;

private:
    ThreadSafeBuffer<DualArmCommand> latest_command_;
};

}  // namespace rb_servo
