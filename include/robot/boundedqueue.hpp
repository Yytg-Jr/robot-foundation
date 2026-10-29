#pragma once
#include "robot/types.hpp"
#include <deque>
#include <mutex>
#include <cstddef>
#include <condition_variable>
#include <chrono>
#include <functional>

namespace robot{

// 有界队列：容量满则拒绝；rate_per_sec > 0 时启用令牌桶限速（0 = 不限速）。
// 队列关闭后不再打开。
class BoundedQueue {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    // clock 可注入，测试里手动推进时间即可精确测限速，无需真实 sleep
    explicit BoundedQueue(std::size_t capacity = 8,
                          std::size_t rate_per_sec = 0,
                          std::function<TimePoint()> clock = std::chrono::steady_clock::now);
    bool try_push(const Command& c);   // 有可能会拒绝（满/超速/已关闭）
    bool wait_pop(Command& out);       // 空且未关闭时阻塞，关闭后返回 false 表示结束
    bool try_pop(Command& out);   // 非阻塞：队列空返回 false；有数据则弹出并返回 true
    void close();
    std::size_t size() const;

    // 观察计数
    std::size_t pushed() const;
    std::size_t rejected() const;
    std::size_t dropped() const;
    std::size_t consumed() const;

private:
    mutable std::mutex mtx_;           // mutable：const getter 也能锁

    std::condition_variable not_empty_; // 等到非空或已关闭
    std::condition_variable not_full_;  // 预留：阻塞式 push（背压）
    std::deque<Command> queue_;
    bool closed_ = false;
    std::size_t capacity_;
    std::size_t rate_per_sec_ = 0;
    double tokens_ = 0;
    std::function<TimePoint()> clock_;  // 声明在 last_refill_ 之前，构造时要用它初始化 last_refill_
    std::chrono::steady_clock::time_point last_refill_{};
    std::size_t pushed_ = 0, rejected_ = 0, dropped_ = 0, consumed_ = 0;
};

}//namespace robot
