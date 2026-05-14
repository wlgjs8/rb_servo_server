#include "rb_servo/logging/servo_logger.hpp"

#include <filesystem>
#include <iostream>

namespace rb_servo {

ServoLogger::ServoLogger(const LoggingConfig& config) : config_(config) {}

ServoLogger::~ServoLogger() {
    stop();
}

bool ServoLogger::start() {
    if (!config_.enable) return true;
    if (running_) return true;

    std::filesystem::create_directories(config_.directory);
    file_.open(config_.directory + "/servo_log.csv", std::ios::out | std::ios::trunc);
    if (!file_) {
        std::cerr << "[ERROR] failed to open servo log file\n";
        return false;
    }
    writeHeader();

    running_ = true;
    thread_ = std::thread(&ServoLogger::threadMain, this);
    return true;
}

void ServoLogger::stop() {
    if (!running_) {
        if (file_.is_open()) file_.close();
        return;
    }
    running_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
    if (file_.is_open()) {
        file_.flush();
        file_.close();
    }
}

void ServoLogger::push(const ServoSample& sample) {
    if (!config_.enable || !running_) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.push(sample);
    }
    cv_.notify_one();
}

void ServoLogger::threadMain() {
    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(config_.flush_period_ms), [&] {
            return !queue_.empty() || !running_;
        });
        while (!queue_.empty()) {
            ServoSample sample = queue_.front();
            queue_.pop();
            lock.unlock();
            writeSample(sample);
            lock.lock();
        }
        if (file_) file_.flush();
    }
}

void ServoLogger::writeHeader() {
    file_ << "tick,loop_start_time_ns,loop_end_time_ns,period_ms,jitter_ms,filter_dt_ms,safety_verdict,fault_latched,fault_reason,command_seq,left_mode,right_mode,left_send_ok,right_send_ok";
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_actual_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",left_q_sent_" << i;
    for (int i = 0; i < kDof; ++i) file_ << ",right_q_sent_" << i;
    file_ << ",left_error_code,right_error_code\n";
}

void ServoLogger::writeSample(const ServoSample& sample) {
    file_ << sample.tick << ','
          << sample.loop_start_time_ns << ','
          << sample.loop_end_time_ns << ','
          << sample.period_ms << ','
          << sample.jitter_ms << ','
          << sample.filter_dt_ms << ','
          << toString(sample.safety_verdict) << ','
          << sample.fault_latched << ','
          << sample.fault_reason << ','
          << sample.command.seq << ','
          << toString(sample.command.left.mode) << ','
          << toString(sample.command.right.mode) << ','
          << sample.left_send_ok << ','
          << sample.right_send_ok;
    for (double v : sample.left_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.right_state.q_actual_deg) file_ << ',' << v;
    for (double v : sample.left_sent_q_deg) file_ << ',' << v;
    for (double v : sample.right_sent_q_deg) file_ << ',' << v;
    file_ << ',' << sample.left_state.error_code << ',' << sample.right_state.error_code << '\n';
}

}  // namespace rb_servo
