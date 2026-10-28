#include "robot/boundedqueue.hpp"
#include <stdexcept>
#include <algorithm>   // std::min

namespace robot{

    BoundedQueue::BoundedQueue(std::size_t capacity, std::size_t rate_per_sec,
                               std::function<TimePoint()> clock)
        : capacity_(capacity),
          rate_per_sec_(rate_per_sec),
          clock_(std::move(clock)),
          last_refill_(clock_()) {
        if (capacity_ == 0)
            throw std::invalid_argument("boundedQueue capacity must > 0");
    }

    bool BoundedQueue::try_push(const Command& c){
        {
            std::lock_guard<std::mutex> lk(mtx_);
            // 先查容量，再查限速：满队列拒绝不应消耗速率令牌
            if (closed_ || queue_.size() >= capacity_){
                ++rejected_;
                return false;
            }
            if (rate_per_sec_ > 0) {
                auto now = clock_();
                auto dt = std::chrono::duration<double>(now - last_refill_).count();
                last_refill_ = now;
                tokens_ = std::min((double)rate_per_sec_, tokens_ + dt * rate_per_sec_); // 桶容量=rate
                if (tokens_ < 1.0) { ++rejected_; return false; }
                tokens_ -= 1.0;
            }
            queue_.push_back(c);
            ++pushed_;
        }// 放锁后再通知，避免被唤醒线程立刻又阻塞在锁上
        not_empty_.notify_one();
        return true;
    }

    bool BoundedQueue::wait_pop(Command& out){
        {
            std::unique_lock<std::mutex> lk(mtx_);
            not_empty_.wait(lk, [this]{ return closed_ || !queue_.empty(); });
            // 谓词循环防虚假唤醒：一个消费者被唤醒时另一消费者可能已抢先 pop，
            // 此时队列又空，谓词不满足会继续 wait
            if (closed_) return false;   // close() 已清空队列，无数据可给
            out = std::move(queue_.front());
            ++consumed_;
            queue_.pop_front();
            lk.unlock();
            not_full_.notify_one();      // 预留：有空间了，唤醒可能的阻塞生产者
        }
        return true;
    }

    void BoundedQueue::close() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            dropped_ += queue_.size();
            queue_.clear();
            closed_ = true;
        }
        not_empty_.notify_all();
        not_full_.notify_all();          // 预留：阻塞生产者也要醒来发现 closed_
    }

    std::size_t BoundedQueue::size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return queue_.size();
    }

    std::size_t BoundedQueue::pushed() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return pushed_;
    }

    std::size_t BoundedQueue::rejected() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return rejected_;
    }

    std::size_t BoundedQueue::dropped() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return dropped_;
    }

    std::size_t BoundedQueue::consumed() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return consumed_;
    }

}//namespace robot
