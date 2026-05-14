#pragma once

#include <atomic>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/core/types.hpp"

namespace rb_servo {

class ServoLogger {
public:
    explicit ServoLogger(const LoggingConfig& config);
    ~ServoLogger();

    bool start();
    void stop();

    void push(const ServoSample& sample);

private:
    void threadMain();
    void writeHeader();
    void writeSample(const ServoSample& sample);

private:
    LoggingConfig config_;

    std::atomic<bool> running_{false};
    std::thread thread_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<ServoSample> queue_;

    std::ofstream file_;
};

}  // namespace rb_servo
