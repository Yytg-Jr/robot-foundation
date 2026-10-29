#include "robot/executor.hpp"
#include <chrono>
#include <thread>
#include <cmath>

namespace robot {
namespace {

// 到达判定：5 个关节 + 夹爪都在误差内才算到达。
// 必须用 eps：advance() 里 cur + (tgt-cur) 有浮点舍入，
// 例如 0.7 + 0.1 = 0.7999999999999999，永远 != 0.8，用 == 会判不到达、卡住。
bool reached(const RobotState& s, const JointValues& target, double eps = 1e-9) {
    for (std::size_t i = 0; i < s.arm_rad.size(); ++i)
        if (std::fabs(s.arm_rad[i] - target.arm_rad[i]) > eps) return false;
    return std::fabs(s.gripper_opening - target.gripper_opening) <= eps;
}

} // namespace

Executor::~Executor() {
    shutdown();
}

void Executor::start() {
    // 原子抢占“启动权”：只有把 false 换成 true 成功的那一个线程才创建 worker
    bool expected = false;
    if (!started_.compare_exchange_strong(expected, true))
        return;   // 已启动，静默忽略

    worker_ = std::thread(&Executor::run, this);
}

bool Executor::submit(const Command& c) {
    return queue_.try_push(c);   // 输入线程只入队，不碰 robot_
}

void Executor::cancel() {
    // 只发请求；清空队列 + 保持位置由执行线程在 run() 里独占完成
    cancel_requested_.store(true);
}

void Executor::shutdown() {
    stop_.store(true);          // 1) 让 run() 循环退出
    queue_.close();             // 2) 丢弃剩余命令、唤醒等待者、拒绝后续 submit
    if (worker_.joinable())     // 3) join（第二次调用时已 join，跳过）
        worker_.join();
}

RobotState Executor::snapshot() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return latest_;   // 值拷贝，不是引用
}

void Executor::run() {
    constexpr auto kPeriod = std::chrono::milliseconds(50);
    auto next = std::chrono::steady_clock::now();

    while (!stop_) {
        // 1) cancel：丢弃待执行（不执行），再把当前位置设为目标 → 机器人停住
        if (cancel_requested_.exchange(false)) {
            Command junk;
            while (queue_.try_pop(junk)) {}   // drain：丢弃，不是边 pop 边执行
            has_active_target_ = false;

            ReadResult r = robot_.read();
            if (r.ok()) {                     // 故障态 read() 不 ok，自然无法 hold
                JointValues hold;
                hold.arm_rad         = r.state.arm_rad;
                hold.gripper_opening = r.state.gripper_opening;
                robot_.submit_target(hold);   // target == current → step() 不再移动
            }
        }

        // 2) 空闲（上一目标已到达/已取消）才消费下一条
        if (!has_active_target_) {
            Command cmd;
            if (queue_.try_pop(cmd)) {
                RobotResult res = robot_.submit_target(cmd.target);
                if (res.ok()) {
                    active_target_ = cmd.target;
                    has_active_target_ = true;
                }
                // 失败（InvalidTarget/FaultInjected）：记录并跳过，不清队列
            }
        }

        // 3) 推进一个 dt
        robot_.step();

        // 4) 读状态 → 加锁快照；同时判断当前目标是否到达
        ReadResult r = robot_.read();
        if (r.ok()) {
            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                latest_ = r.state;
            }
            if (has_active_target_ && reached(r.state, active_target_))
                has_active_target_ = false;   // 到达 → 下轮才会取新命令
        }

        // 5) 对齐到下一个 50ms 边界（落后则重新对齐，避免空转）
        auto now = std::chrono::steady_clock::now();
        if (now < next) {
            std::this_thread::sleep_until(next);
        } else {
            next = now;   // 本周期超时，别追落后的边界
        }
        next += kPeriod;
    }
}

} // namespace robot
