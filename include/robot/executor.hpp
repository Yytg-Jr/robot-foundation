#pragma once
#include "robot/types.hpp"
#include "robot/irobot.hpp"
#include "robot/boundedqueue.hpp"
#include <atomic>
#include <mutex>
#include <thread>

namespace robot {

// 执行线程唯一写 IRobot；输入线程只 submit；状态通过加锁快照对外读。
class Executor {
public:
    Executor(IRobot& robot, std::size_t cap = 8)
        : robot_(robot), queue_(cap) {}

    ~Executor();                    // 内部调 shutdown() + join

    void start();                   // 启动执行线程（幂等：已启动则忽略）
    bool submit(const Command& c);  // 转发给 queue_.try_push
    void cancel();                  // 请求取消：清空待执行 + 保持当前位置
    void shutdown();                // close 队列 + 唤醒 + join，可重复调用

    RobotState snapshot() const;    // 加锁返回 latest_ 副本

private:
    void run();                     // 执行线程主循环

    IRobot& robot_;
    BoundedQueue queue_;
    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> cancel_requested_{false};
    std::atomic<bool> started_{false};

    bool has_active_target_{false};   // 当前是否有正在执行的目标
    JointValues active_target_{};     // 正在执行的目标，用于到达判定

    mutable std::mutex state_mtx_;
    RobotState latest_{};
};

} // namespace robot
