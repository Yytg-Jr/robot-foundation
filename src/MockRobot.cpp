#include "robot/mock_robot.hpp"
#include "robot/types.hpp"
#include <algorithm>
#include <cmath>
#include <string>
namespace robot {




MockRobot::MockRobot(std::uint64_t /*seed*/) {}
//加入生命周期的意义：connected_不能只是一个裸的bool，需要有状态约束
//disconnect()在没连的时候调用 也静默成功
RobotResult MockRobot::connect() {
    if(connected_ == true) return {RobotError::AlreadyConnected,"already connected"};
    read_count_ = 0;
    step_count_ = 0;
    faulted_ = false;
    connected_ = true;
    target_ = {};
    current_ = {};
    state_valid_ = true;
    return {};
}
//RAII也需要 代表着异常时的安全
//如果在connect之后 disconnect之前发出异常或提前return disconnect永远执行不到，connect就存在泄露
//RAII“把资源的获取和释放绑在对象生命周期上，构造时获取，析构时释放

MockRobot::~MockRobot(){
    if(connected_) disconnect();
}

RobotResult MockRobot::configure_fault(FaultMode m, std::uint64_t n) {
    fault_mode_ = m;
    fail_on_read_n_ = n;
    return {};
}


void MockRobot::disconnect() {
    connected_ = false;
    state_valid_ = false;
    target_ = {};
    current_ = {};
}//幂等化 无论执行多少次 结果保持一致

ReadResult MockRobot::read() {
    if(faulted_) return {RobotError::FaultInjected,{}};
    if(!connected_) return {RobotError::NotConnected,{}};
    ++read_count_;
    //模式一：读操作本身超时 —— 表现为错误码
    if(fault_mode_ == FaultMode::ReadTimeoutAtN&&read_count_ == fail_on_read_n_){
        faulted_ = true;
        state_valid_ = false;
        return {RobotError::ReadTimeout,{}};
    }
    // 模式二：读“成功”，但这一帧不可信 —— 表现为 state.valid=false
    RobotState s;
    
    s.connected = connected_;
    s.valid = connected_ && state_valid_;
    if (fault_mode_ == FaultMode::InvalidFeedbackAtN && read_count_ == fail_on_read_n_)
        s.valid = false;
    if (s.valid) {
        s.arm_rad = current_.arm_rad;
        s.gripper_opening = current_.gripper_opening;
    }
    return {RobotError::None, s};   // 拷贝，不返回内部引用
}

RobotResult MockRobot::submit_target(const JointValues& t) {
    if (!connected_) return {RobotError::NotConnected, "not connected"};
    if(faulted_) return {RobotError::FaultInjected,"session faulted"};
    // 复用 Case A 的有限性+范围规则
    for (std::size_t i = 0; i < t.arm_rad.size(); ++i) {
        double v = t.arm_rad[i];
        if (!std::isfinite(v) || v < kJointMin || v > kJointMax)
            return {RobotError::InvalidTarget, "joint " + std::to_string(i)};
    }
    if (!std::isfinite(t.gripper_opening) ||
        t.gripper_opening < kGripperMin || t.gripper_opening > kGripperMax)
        return {RobotError::InvalidTarget, "gripper"};
    target_ = t;   // 仅写 target_，current_ 不动
    return {};
}

namespace {
double advance(double cur, double tgt, double max_step) {
    double err = tgt - cur;
    double step = std::clamp(err, -max_step, max_step);
    return cur + step;
}
}

RobotResult MockRobot::step() {
    if (!connected_) return {RobotError::NotConnected, "not connected"};
    if(faulted_) return {RobotError::FaultInjected,"session faulted"};
    const double joint_step = kJointMaxSpeed * kDt;   // 0.025
    const double grip_step  = kGripperMaxSpeed * kDt; // 0.05
    ++step_count_;
    for (std::size_t i = 0; i < current_.arm_rad.size(); ++i)
        current_.arm_rad[i] = advance(current_.arm_rad[i],
                                      target_.arm_rad[i], joint_step);
    current_.gripper_opening = advance(current_.gripper_opening,
                                       target_.gripper_opening, grip_step);
    state_valid_ = true;
    return {};
}

} // namespace robot