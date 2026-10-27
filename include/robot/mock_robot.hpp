#pragma once
#include <array>
#include <cstdint>
#include <string>
#include "robot/types.hpp"
#include "robot/irobot.hpp"

namespace robot{

class MockRobot:public IRobot{
    public:
        static constexpr double kJointMaxSpeed   = 0.5;   // rad/s
        static constexpr double kGripperMaxSpeed = 1.0;   // 1/s
        static constexpr double kDt              = 0.05;  // s

        explicit MockRobot(std::uint64_t seed = 0);
        ~MockRobot();
        RobotResult connect();
        void disconnect();
        bool is_connected() const {return connected_;}

        // 只接受合法目标；成功仅更新 target_，不动 current_
        RobotResult submit_target(const JointValues& target);
        // 推进一个 dt；按 clamp(error, ±max_speed*dt) 更新 current_
        RobotResult step();
        RobotResult configure_fault(FaultMode m, std::uint64_t n);
        ReadResult read();

    private:
        bool connected_{false};
        bool state_valid_{false};
        JointValues target_{};
        JointValues current_{};
        
        FaultMode    fault_mode_{FaultMode::None};
        std::uint64_t fail_on_read_n_{0};
        std::uint64_t read_count_{0};      // 本会话已读次数
        bool         faulted_{false};      // 一旦注入故障就置位，直到显式重连
        std::uint64_t step_count_{0};      // 模拟时间 = step_count_ * kDt

    };

    class ConnectionGuard {
        public:
            explicit ConnectionGuard(MockRobot& r)
                :robot_(r), result_(r.connect()), owns_(result_.ok()) {}
            ~ConnectionGuard() {if(owns_)robot_.disconnect();}
            //删除拷贝构造与拷贝复制
            ConnectionGuard(const ConnectionGuard&) = delete;
            ConnectionGuard& operator=(const ConnectionGuard&)=delete;
            
            bool ok() const {return owns_;}
            const RobotResult& result() const {return result_;}
        private:
            MockRobot& robot_;
            RobotResult result_;
            bool owns_;  
    };

}//namespace robot