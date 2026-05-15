#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <string>
#include <thread>
#include <unistd.h>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/control/safety_filter.hpp"
#include "rb_servo/core/clock.hpp"
#include "rb_servo/logging/servo_logger.hpp"
#include "rb_servo/network/command_server.hpp"
#include "rb_servo/robot/i_robot_backend.hpp"

namespace {

constexpr double kEpsilon = 1e-9;

#define RB_CHECK(expr) \
    do { \
        if (!(expr)) { \
            std::cerr << "CHECK failed: " #expr << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
            return false; \
        } \
    } while (0)

rb_servo::JointArray joints(double value) {
    rb_servo::JointArray out{};
    out.fill(value);
    return out;
}

bool sameJointArray(const rb_servo::JointArray& a, const rb_servo::JointArray& b) {
    for (int i = 0; i < rb_servo::kDof; ++i) {
        if (std::abs(a[i] - b[i]) > kEpsilon) return false;
    }
    return true;
}

class TestBackend final : public rb_servo::IRobotBackend {
public:
    TestBackend(rb_servo::ArmId arm_id, rb_servo::JointArray initial, bool fail_send)
        : arm_id_(arm_id), q_actual_(initial), q_target_(initial), fail_send_(fail_send) {}

    bool connect() override {
        connected_ = true;
        return true;
    }

    bool initialize() override {
        initialized_ = true;
        return true;
    }

    bool readState(rb_servo::RobotState& out_state) override {
        if (!read_ok_) return false;
        out_state.arm_id = arm_id_;
        out_state.host_time_ns = rb_servo::nowSteadyNs();
        out_state.q_actual_deg = q_actual_;
        out_state.q_target_deg = q_target_;
        out_state.has_valid_joint_state = valid_joint_state_;
        out_state.connection_state = connected_
            ? rb_servo::RobotConnectionState::Connected
            : rb_servo::RobotConnectionState::Disconnected;
        out_state.servo_enabled = initialized_;
        out_state.has_error = has_error_;
        return true;
    }

    bool sendServoJ(const rb_servo::JointArray& q_target_deg) override {
        if (fail_send_) return false;
        q_target_ = q_target_deg;
        q_actual_ = q_target_deg;
        return true;
    }

    bool stop() override { return true; }
    bool resetFault() override { return true; }
    bool isConnected() const override { return connected_; }
    rb_servo::ArmId armId() const override { return arm_id_; }
    std::string name() const override { return "test"; }

    void setValidJointState(bool valid) { valid_joint_state_ = valid; }
    void setReadOk(bool ok) { read_ok_ = ok; }
    void setConnected(bool connected) { connected_ = connected; }
    void setHasError(bool has_error) { has_error_ = has_error; }

private:
    rb_servo::ArmId arm_id_;
    rb_servo::JointArray q_actual_{};
    rb_servo::JointArray q_target_{};
    bool fail_send_ = false;
    bool valid_joint_state_ = true;
    bool read_ok_ = true;
    bool has_error_ = false;
    bool connected_ = false;
    bool initialized_ = false;
};

rb_servo::DualArmConfig testConfig() {
    rb_servo::DualArmConfig cfg;
    cfg.left_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.right_robot.run_mode = rb_servo::RunMode::Mock;
    cfg.servo.rate_hz = 200;
    cfg.servo.command_timeout_sec = 0.2;
    cfg.servo.enable_realtime_priority = false;
    cfg.servo.filter_dt_min_ratio = 0.5;
    cfg.servo.filter_dt_max_ratio = 1.5;
    cfg.safety.q_min_deg = joints(-180.0);
    cfg.safety.q_max_deg = joints(180.0);
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    cfg.safety.max_tracking_error_deg = 1000.0;
    cfg.safety.tracking_error_policy = rb_servo::TrackingErrorPolicy::SnapToActual;
    cfg.safety.stop_both_arms_on_single_arm_error = false;
    cfg.safety.latch_fault_on_robot_state_error = true;
    return cfg;
}

rb_servo::DualArmCommand command(rb_servo::ControlMode mode) {
    rb_servo::DualArmCommand cmd;
    cmd.seq = 1;
    cmd.host_time_ns = rb_servo::nowSteadyNs();
    cmd.left.arm_id = rb_servo::ArmId::Left;
    cmd.right.arm_id = rb_servo::ArmId::Right;
    cmd.left.mode = mode;
    cmd.right.mode = mode;
    cmd.left.timeout_sec = 0.2;
    cmd.right.timeout_sec = 0.2;
    return cmd;
}

void sleepTicks() {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
}

int reserveLoopbackUdpPort() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }

    socklen_t len = sizeof(addr);
    if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0) {
        ::close(fd);
        return -1;
    }
    const int port = ntohs(addr.sin_port);
    ::close(fd);
    return port;
}

bool testCommandValidation() {
    rb_servo::NetworkConfig network;
    network.command_timeout_sec = 0.35;
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    rb_servo::DualArmCommand out;
    const uint64_t now = rb_servo::nowSteadyNs();

    RB_CHECK(!server.parseMessage("{", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"Unknown"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"JointTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"JointTarget","q_target_deg":[0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"JointTarget","q_target_deg":[0,0,0,0,0,"bad"]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"JointTarget","timeout_sec":0,"q_target_deg":[0,0,0,0,0,0]})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"JointVelocity"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"TcpPoseTarget"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"TcpDeltaStand"})", now, &out));
    RB_CHECK(!server.parseMessage(R"({"mode":"TcpDeltaLocal"})", now, &out));

    RB_CHECK(server.parseMessage(R"({"mode":"EmergencyStop"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::EmergencyStop);
    RB_CHECK(server.parseMessage(R"({"mode":"ArmMotion"})", now, &out));
    RB_CHECK(out.left.mode == rb_servo::ControlMode::ArmMotion);

    RB_CHECK(server.parseMessage(R"({"mode":"JointTarget","q_target_deg":[1,2,3,4,5,6]})", now, &out));
    RB_CHECK(out.left.has_joint_target);
    RB_CHECK(out.right.has_joint_target);
    RB_CHECK(std::abs(out.left.timeout_sec - 0.35) < kEpsilon);
    return true;
}

bool testCommandBufferInvalidTimeoutHolds() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.q_target_deg = joints(12.0);
    target.right.q_target_deg = joints(12.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    target.left.timeout_sec = -1.0;
    target.right.timeout_sec = 0.2;
    buffer.setCommand(target);

    rb_servo::DualArmCommand latest = buffer.latestOrHold(target.host_time_ns + 1);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);

    target.left.timeout_sec = 0.1;
    target.right.timeout_sec = 0.1;
    buffer.setCommand(target);
    latest = buffer.latestOrHold(target.host_time_ns + 200'000'000ULL);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testCoupledTimeoutUsesEarliestArmTimeout() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.host_time_ns = rb_servo::nowSteadyNs();
    target.left.timeout_sec = 0.2;
    target.right.timeout_sec = 0.05;
    target.coupled_timeout = false;
    target.left.q_target_deg = joints(4.0);
    target.right.q_target_deg = joints(4.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);

    rb_servo::DualArmCommand latest = buffer.latestOrHold(target.host_time_ns + 100'000'000ULL);
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testLifecycleCommandSurvivesMotionOverwrite() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmCommand arm = command(rb_servo::ControlMode::ArmMotion);
    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(3.0);
    target.right.q_target_deg = joints(3.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;

    const uint64_t now = rb_servo::nowSteadyNs();
    arm.host_time_ns = now;
    target.host_time_ns = now + 1;

    buffer.setCommand(arm);
    buffer.setCommand(target);

    rb_servo::DualArmCommand first = buffer.latestOrHold(now + 2);
    RB_CHECK(first.left.mode == rb_servo::ControlMode::ArmMotion);
    RB_CHECK(first.right.mode == rb_servo::ControlMode::ArmMotion);

    rb_servo::DualArmCommand second = buffer.latestOrHold(now + 3);
    RB_CHECK(second.left.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(second.right.mode == rb_servo::ControlMode::JointTarget);
    RB_CHECK(sameJointArray(second.left.q_target_deg, joints(3.0)));
    return true;
}

bool testOversizedUdpPacketDoesNotUpdateCommandBuffer() {
    const int port = reserveLoopbackUdpPort();
    RB_CHECK(port > 0);

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    RB_CHECK(server.start());

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    RB_CHECK(fd >= 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    RB_CHECK(::inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) == 1);
    const std::string oversized(9000, 'x');
    const ssize_t sent = ::sendto(
        fd,
        oversized.data(),
        oversized.size(),
        0,
        reinterpret_cast<sockaddr*>(&addr),
        sizeof(addr)
    );
    ::close(fd);
    RB_CHECK(sent == static_cast<ssize_t>(oversized.size()));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    const rb_servo::DualArmCommand latest = buffer.latestOrHold(rb_servo::nowSteadyNs());
    server.stop();
    RB_CHECK(latest.left.mode == rb_servo::ControlMode::Hold);
    RB_CHECK(latest.right.mode == rb_servo::ControlMode::Hold);
    return true;
}

bool testCommandServerStartFailsOnInvalidBind() {
    rb_servo::NetworkConfig network;
    network.command_bind = "udp://invalid-host:50010";
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    RB_CHECK(!server.start());
    server.stop();
    return true;
}

bool testCommandServerStartFailsOnPortConflict() {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    RB_CHECK(fd >= 0);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::cerr << "bind test socket failed: " << std::strerror(errno) << "\n";
        ::close(fd);
        return false;
    }

    socklen_t len = sizeof(addr);
    RB_CHECK(::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) == 0);
    const int port = ntohs(addr.sin_port);

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer buffer;
    rb_servo::CommandServer server(network, &buffer);
    const bool started = server.start();
    server.stop();
    ::close(fd);
    RB_CHECK(!started);
    return true;
}

bool testSecondCommandServerStartFailsOnSamePort() {
    const int port = reserveLoopbackUdpPort();
    RB_CHECK(port > 0);

    rb_servo::NetworkConfig network;
    network.command_bind = "udp://127.0.0.1:" + std::to_string(port);
    rb_servo::CommandBuffer first_buffer;
    rb_servo::CommandBuffer second_buffer;
    rb_servo::CommandServer first(network, &first_buffer);
    rb_servo::CommandServer second(network, &second_buffer);

    RB_CHECK(first.start());
    const bool second_started = second.start();
    second.stop();
    first.stop();
    RB_CHECK(!second_started);
    return true;
}

bool testRealModeTcpStatePublisherExposureRequiresOverride() {
    const char* old_allow_real = std::getenv("RB_ALLOW_REAL_ROBOT");
    const char* old_allow_network = std::getenv("RB_ALLOW_NETWORK_EXPOSURE");
    const std::string saved_allow_real = old_allow_real ? old_allow_real : "";
    const std::string saved_allow_network = old_allow_network ? old_allow_network : "";

    setenv("RB_ALLOW_REAL_ROBOT", "1", 1);
    unsetenv("RB_ALLOW_NETWORK_EXPOSURE");

    const std::string path = "/tmp/rb-servo-real-exposure-" + std::to_string(getpid()) + ".yaml";
    {
        std::ofstream file(path);
        file << "left_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "right_robot:\n"
             << "  backend_type: rbpodo\n"
             << "  run_mode: real\n"
             << "network:\n"
             << "  command_bind: \"udp://127.0.0.1:50010\"\n"
             << "  state_pub_bind: \"tcp://0.0.0.0:50110\"\n";
    }

    bool rejected = false;
    try {
        (void)rb_servo::loadConfigFromYaml(path);
    } catch (const std::exception&) {
        rejected = true;
    }
    ::unlink(path.c_str());

    if (old_allow_real) {
        setenv("RB_ALLOW_REAL_ROBOT", saved_allow_real.c_str(), 1);
    } else {
        unsetenv("RB_ALLOW_REAL_ROBOT");
    }
    if (old_allow_network) {
        setenv("RB_ALLOW_NETWORK_EXPOSURE", saved_allow_network.c_str(), 1);
    } else {
        unsetenv("RB_ALLOW_NETWORK_EXPOSURE");
    }

    RB_CHECK(rejected);
    return true;
}

bool testSafetyFilterVelocityClampMaxStep() {
    rb_servo::SafetyConfig cfg;
    cfg.q_min_deg = joints(-180.0);
    cfg.q_max_deg = joints(180.0);
    cfg.dq_max_deg_s = joints(10.0);
    cfg.ddq_max_deg_s2 = joints(100000.0);
    cfg.max_tracking_error_deg = 1000.0;
    rb_servo::SafetyFilter filter(cfg);

    rb_servo::RobotState state;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);

    const rb_servo::SafetyCheckResult result = filter.filterJointTarget(
        joints(100.0),
        joints(0.0),
        joints(0.0),
        state,
        0.01
    );

    RB_CHECK(result.ok);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.1 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    return true;
}

bool testSafetyFilterAccelerationClampDoesNotOvershoot() {
    rb_servo::SafetyConfig cfg;
    cfg.q_min_deg = joints(-180.0);
    cfg.q_max_deg = joints(180.0);
    cfg.dq_max_deg_s = joints(1000.0);
    cfg.ddq_max_deg_s2 = joints(100.0);
    cfg.max_tracking_error_deg = 1000.0;
    rb_servo::SafetyFilter filter(cfg);

    rb_servo::RobotState state;
    state.connection_state = rb_servo::RobotConnectionState::Connected;
    state.has_valid_joint_state = true;
    state.q_actual_deg = joints(0.0);

    const rb_servo::SafetyCheckResult result = filter.filterJointTarget(
        joints(0.005),
        joints(0.0),
        joints(0.0),
        state,
        0.01
    );

    RB_CHECK(result.ok);
    for (double q : result.filtered_q_deg) {
        RB_CHECK(q <= 0.005 + kEpsilon);
        RB_CHECK(q >= -kEpsilon);
    }
    return true;
}

bool testRobotStateErrorRealPolicyLatchesFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.left_robot.run_mode = rb_servo::RunMode::Real;
    cfg.safety.latch_fault_on_robot_state_error = false;
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    TestBackend* left_raw = left.get();
    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    left_raw->setHasError(true);
    sleepTicks();
    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.fault_latched);
    RB_CHECK(snapshot.latched_fault_reason == rb_servo::SafetyVerdict::RobotStateError);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::FaultLatched);
    return true;
}

bool testLatestSnapshotContainsSendTimingAndPreviousTargets() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.dq_max_deg_s = joints(10000.0);
    cfg.safety.ddq_max_deg_s2 = joints(100000.0);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(2.0);
    target.right.q_target_deg = joints(2.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();

    const rb_servo::ServoSnapshot snapshot = loop.latestSnapshot();
    loop.stop();
    RB_CHECK(snapshot.tick > 0);
    RB_CHECK(snapshot.loop_end_time_ns >= snapshot.loop_start_time_ns);
    RB_CHECK(snapshot.motion_state == rb_servo::ServerMotionState::Running);
    RB_CHECK(snapshot.left_send_ok);
    RB_CHECK(snapshot.right_send_ok);
    RB_CHECK(snapshot.left_send_start_ns > 0);
    RB_CHECK(snapshot.left_send_end_ns >= snapshot.left_send_start_ns);
    RB_CHECK(snapshot.right_send_start_ns > 0);
    RB_CHECK(snapshot.right_send_end_ns >= snapshot.right_send_start_ns);
    RB_CHECK(snapshot.left_send_duration_us >= 0.0);
    RB_CHECK(snapshot.right_send_duration_us >= 0.0);
    RB_CHECK(sameJointArray(snapshot.left_prev_sent_q_deg, joints(2.0)));
    RB_CHECK(sameJointArray(snapshot.right_prev_sent_q_deg, joints(2.0)));
    return true;
}

bool testLoggerZeroCapacityDropsWithoutBlocking() {
    rb_servo::LoggingConfig cfg;
    cfg.enable = true;
    cfg.directory = "/tmp/rb-servo-logger-test-" + std::to_string(getpid());
    cfg.queue_capacity = 0;
    cfg.flush_period_ms = 1;

    rb_servo::ServoLogger logger(cfg);
    RB_CHECK(logger.start());
    rb_servo::ServoSample sample;
    logger.push(sample);
    logger.stop();
    std::filesystem::remove_all(cfg.directory);
    RB_CHECK(logger.droppedSamples() == 1);
    return true;
}

bool testInvalidStartupRobotStateFailsStart() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    auto left = std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false);
    auto right = std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false);
    left->setValidJointState(false);

    rb_servo::DualArmServoLoop loop(
        std::move(left),
        std::move(right),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(!loop.start());
    return true;
}

bool testEmergencyWinsAndResetDoesNotRun() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    rb_servo::DualArmCommand mixed = command(rb_servo::ControlMode::ResetFault);
    mixed.right.mode = rb_servo::ControlMode::EmergencyStop;
    buffer.setCommand(mixed);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::EmergencyLatched);
    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::EmergencyStop);

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(!loop.faultLatched());
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(5.0);
    target.right.q_target_deg = joints(5.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);

    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();
    buffer.setCommand(target);
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::Running);

    buffer.setCommand(command(rb_servo::ControlMode::ResetFault));
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);
    loop.stop();
    return true;
}

bool testDisarmAndCartesianHoldPreviousTarget() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand cartesian = command(rb_servo::ControlMode::TcpPoseTarget);
    cartesian.left.has_tcp_target = true;
    cartesian.right.has_tcp_target = true;
    cartesian.left.tcp_target_stand.x = 0.1;
    cartesian.right.tcp_target_stand.x = 0.1;
    buffer.setCommand(cartesian);
    sleepTicks();
    RB_CHECK(sameJointArray(loop.previousSentTarget().left_q_target_deg, initial));
    RB_CHECK(sameJointArray(loop.previousSentTarget().right_q_target_deg, initial));

    buffer.setCommand(command(rb_servo::ControlMode::DisarmMotion));
    sleepTicks();
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::ConnectedHold);
    loop.stop();
    return true;
}

bool testJointLimitClamp() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.q_max_deg = joints(5.0);
    cfg.safety.q_min_deg = joints(-5.0);
    cfg.safety.dq_max_deg_s = joints(100000.0);
    cfg.safety.ddq_max_deg_s2 = joints(1000000.0);
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, false),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(100.0);
    target.right.q_target_deg = joints(100.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    loop.stop();

    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    for (double q : previous.left_q_target_deg) RB_CHECK(q <= 5.0 + kEpsilon);
    for (double q : previous.right_q_target_deg) RB_CHECK(q <= 5.0 + kEpsilon);
    return true;
}

bool testSendFailureDoesNotAdvancePreviousTarget() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, true),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();
    loop.stop();

    const rb_servo::ServoTarget previous = loop.previousSentTarget();
    RB_CHECK(sameJointArray(previous.left_q_target_deg, initial));
    RB_CHECK(!sameJointArray(previous.right_q_target_deg, initial));
    return true;
}

bool testStopBothOnSendFailureLatchesFault() {
    rb_servo::CommandBuffer buffer;
    rb_servo::DualArmConfig cfg = testConfig();
    cfg.safety.stop_both_arms_on_single_arm_error = true;
    const rb_servo::JointArray initial = joints(0.0);
    rb_servo::DualArmServoLoop loop(
        std::make_unique<TestBackend>(rb_servo::ArmId::Left, initial, true),
        std::make_unique<TestBackend>(rb_servo::ArmId::Right, initial, false),
        cfg,
        &buffer,
        nullptr
    );

    RB_CHECK(loop.start());
    buffer.setCommand(command(rb_servo::ControlMode::ArmMotion));
    sleepTicks();

    rb_servo::DualArmCommand target = command(rb_servo::ControlMode::JointTarget);
    target.left.q_target_deg = joints(7.0);
    target.right.q_target_deg = joints(7.0);
    target.left.has_joint_target = true;
    target.right.has_joint_target = true;
    buffer.setCommand(target);
    sleepTicks();

    RB_CHECK(loop.faultLatched());
    RB_CHECK(loop.latchedFaultReason() == rb_servo::SafetyVerdict::SendFailure);
    RB_CHECK(loop.motionState() == rb_servo::ServerMotionState::FaultLatched);
    loop.stop();
    return true;
}

}  // namespace

int main() {
    if (!testCommandValidation()) return 1;
    if (!testCommandBufferInvalidTimeoutHolds()) return 1;
    if (!testCoupledTimeoutUsesEarliestArmTimeout()) return 1;
    if (!testLifecycleCommandSurvivesMotionOverwrite()) return 1;
    if (!testOversizedUdpPacketDoesNotUpdateCommandBuffer()) return 1;
    if (!testCommandServerStartFailsOnInvalidBind()) return 1;
    if (!testCommandServerStartFailsOnPortConflict()) return 1;
    if (!testSecondCommandServerStartFailsOnSamePort()) return 1;
    if (!testRealModeTcpStatePublisherExposureRequiresOverride()) return 1;
    if (!testSafetyFilterVelocityClampMaxStep()) return 1;
    if (!testSafetyFilterAccelerationClampDoesNotOvershoot()) return 1;
    if (!testRobotStateErrorRealPolicyLatchesFault()) return 1;
    if (!testLatestSnapshotContainsSendTimingAndPreviousTargets()) return 1;
    if (!testLoggerZeroCapacityDropsWithoutBlocking()) return 1;
    if (!testInvalidStartupRobotStateFailsStart()) return 1;
    if (!testEmergencyWinsAndResetDoesNotRun()) return 1;
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testJointLimitClamp()) return 1;
    if (!testSendFailureDoesNotAdvancePreviousTarget()) return 1;
    if (!testStopBothOnSendFailureLatchesFault()) return 1;
    return 0;
}
