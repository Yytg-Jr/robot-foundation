#include "robot/boundedqueue.hpp"
#include <cstdio>
#include <cstdint>
#include <thread>
#include <atomic>
#include <chrono>

namespace {
int g_checks = 0, g_failures = 0;
void check_true(bool c, const char* e, int l) {
    ++g_checks; if (!c) { ++g_failures; std::printf("FAIL line %d: %s\n", l, e); }
}
#define CHECK_TRUE(c) check_true((c), #c, __LINE__)

// 不变量 1：容量。push 20 条，只进 8，rejected == 12（默认不限速）
void test_capacity() {
    robot::BoundedQueue q(8);
    for (int i = 0; i < 20; ++i) {
        robot::Command c; c.sequence_id = static_cast<std::uint64_t>(i);
        if (i < 8) CHECK_TRUE(q.try_push(c));
        else       CHECK_TRUE(!q.try_push(c));
    }
    CHECK_TRUE(q.size() == 8);
    CHECK_TRUE(q.rejected() == 12);
}

// 不变量 2：FIFO 与序号单调不重复
void test_fifo() {
    robot::BoundedQueue q(8);
    for (int i = 0; i < 5; ++i) {
        robot::Command c; c.sequence_id = static_cast<std::uint64_t>(i);
        CHECK_TRUE(q.try_push(c));
    }
    for (int i = 0; i < 5; ++i) {
        robot::Command out;
        CHECK_TRUE(q.wait_pop(out));
        CHECK_TRUE(out.sequence_id == static_cast<std::uint64_t>(i));
    }
}

// 不变量 3：close 语义。close 后 push 永远失败；空队列 wait_pop 立即返回 false
void test_close_semantics() {
    robot::BoundedQueue q(8);
    q.close();
    robot::Command c;
    CHECK_TRUE(!q.try_push(c));
    CHECK_TRUE(!q.wait_pop(c));
}

// 不变量 4：守恒 pushed == consumed + dropped（rejected 从未入队，不计入）
void test_conservation() {
    robot::BoundedQueue q(4);
    robot::Command c;
    for (int i = 0; i < 6; ++i) CHECK_TRUE(q.try_push(c) == (i < 4)); // 4 成功 2 拒
    robot::Command out;
    CHECK_TRUE(q.wait_pop(out));   // consumed 1
    q.close();                     // 丢弃剩余 3
    CHECK_TRUE(q.pushed() == 4);
    CHECK_TRUE(q.consumed() == 1);
    CHECK_TRUE(q.rejected() == 2);
    CHECK_TRUE(q.dropped() == 3);
    CHECK_TRUE(q.pushed() == q.consumed() + q.dropped());
}

// 不变量 5 的线程侧：空队列上 wait_pop 阻塞，close 后应醒来返回 false，join 不卡死
void test_wait_pop_blocks_then_close() {
    robot::BoundedQueue q(8);
    std::atomic<bool> got_false{false};
    std::thread t([&]{ robot::Command c; got_false = !q.wait_pop(c); });
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 确保消费者进入 wait
    q.close();
    t.join();   // join 前 q 仍在作用域，安全
    CHECK_TRUE(got_false);
}

// ---- 限速（令牌桶）测试：注入假时钟，手动推进时间，不 sleep ----

// 冷启动：tokens=0，第一条被拒；推进一个令牌周期后放一条
void test_rate_cold_start() {
    using TP = robot::BoundedQueue::TimePoint;
    TP now{};
    robot::BoundedQueue q(8, 100, [&]{ return now; });
    robot::Command c;
    CHECK_TRUE(!q.try_push(c));                    // tokens=0
    now += std::chrono::milliseconds(10);          // 补 1 令牌
    CHECK_TRUE(q.try_push(c));
    CHECK_TRUE(!q.try_push(c));                    // 令牌耗尽
}

// 突发上限：空闲补到桶满（=rate），最多瞬间放 rate 条
void test_rate_burst_cap() {
    using TP = robot::BoundedQueue::TimePoint;
    TP now{};
    robot::BoundedQueue q(200, 100, [&]{ return now; });
    now += std::chrono::seconds(10);               // tokens 补满 = 100
    robot::Command c;
    int ok = 0;
    for (int i = 0; i < 150; ++i) if (q.try_push(c)) ++ok;
    CHECK_TRUE(ok == 100);
    CHECK_TRUE(q.rejected() == 50);
}

// 补充速率：半个周期 0.5 令牌不够，满一个周期才够 1
void test_rate_refill() {
    using TP = robot::BoundedQueue::TimePoint;
    TP now{};
    robot::BoundedQueue q(8, 100, [&]{ return now; });
    now += std::chrono::milliseconds(10);          // 1 令牌
    robot::Command c;
    CHECK_TRUE(q.try_push(c));                     // 用掉
    CHECK_TRUE(!q.try_push(c));
    now += std::chrono::milliseconds(5);           // 0.5 令牌
    CHECK_TRUE(!q.try_push(c));
    now += std::chrono::milliseconds(5);           // 满 1
    CHECK_TRUE(q.try_push(c));
}

// 满队列拒绝不消耗令牌（先查满、再限速）
void test_full_queue_no_token_consume() {
    using TP = robot::BoundedQueue::TimePoint;
    TP now{};
    robot::BoundedQueue q(2, 100, [&]{ return now; });
    now += std::chrono::seconds(1);                // 补满
    robot::Command c;
    CHECK_TRUE(q.try_push(c)); CHECK_TRUE(q.try_push(c)); // 填满容量 2
    CHECK_TRUE(!q.try_push(c));                    // 队列满 → 拒，不应扣令牌
    robot::Command out;
    q.wait_pop(out); q.wait_pop(out);              // 清空
    CHECK_TRUE(q.try_push(c));                     // 未推进时间仍放行 → 令牌未被误扣
}

} // namespace

int main() {
    test_capacity();
    test_fifo();
    test_close_semantics();
    test_conservation();
    test_wait_pop_blocks_then_close();
    test_rate_cold_start();
    test_rate_burst_cap();
    test_rate_refill();
    test_full_queue_no_token_consume();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
