#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rb_servo/config/config.hpp"
#include "rb_servo/control/command_buffer.hpp"
#include "rb_servo/control/dual_arm_servo_loop.hpp"
#include "rb_servo/core/clock.hpp"
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
        out_state.arm_id = arm_id_;
        out_state.host_time_ns = rb_servo::nowSteadyNs();
        out_state.q_actual_deg = q_actual_;
        out_state.q_target_deg = q_target_;
        out_state.connection_state = connected_
            ? rb_servo::RobotConnectionState::Connected
            : rb_servo::RobotConnectionState::Disconnected;
        out_state.servo_enabled = initialized_;
        out_state.has_error = false;
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

private:
    rb_servo::ArmId arm_id_;
    rb_servo::JointArray q_actual_{};
    rb_servo::JointArray q_target_{};
    bool fail_send_ = false;
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
    if (!testEmergencyWinsAndResetDoesNotRun()) return 1;
    if (!testDisarmAndCartesianHoldPreviousTarget()) return 1;
    if (!testJointLimitClamp()) return 1;
    if (!testSendFailureDoesNotAdvancePreviousTarget()) return 1;
    if (!testStopBothOnSendFailureLatchesFault()) return 1;
    return 0;
}
