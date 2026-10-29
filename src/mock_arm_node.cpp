#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>

#include "robot/executor.hpp"
#include "robot/robot_factory.hpp"

namespace {

// 校验并拆解 6 个值 → JointValues（5 关节 + 1 夹爪）。合法返回 true。
bool to_target(const std_msgs::msg::Float64MultiArray& msg, robot::JointValues& out) {
    if (msg.data.size() != 6) return false;
    for (std::size_t i = 0; i < 5; ++i) {
        double v = msg.data[i];
        if (!std::isfinite(v) || v < robot::kJointMin || v > robot::kJointMax)
            return false;
    }
    double g = msg.data[5];
    if (!std::isfinite(g) || g < robot::kGripperMin || g > robot::kGripperMax)
        return false;
    for (std::size_t i = 0; i < 5; ++i)
        out.arm_rad[i] = msg.data[i];
    out.gripper_opening = g;
    return true;
}

} // namespace

class MockArmNode : public rclcpp::Node {
public:
    MockArmNode()
        : Node("mock_arm"),
          robot_(robot::make_robot()),
          executor_(*robot_) {
        // 生命周期：先 connect，再启动执行线程
        robot_->connect();
        executor_.start();

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/mock_arm/joint_states", 10);
        gripper_pub_ = create_publisher<std_msgs::msg::Float64>("/mock_arm/gripper_opening", 10);

        target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mock_arm/target", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                robot::JointValues t;
                if (!to_target(*msg, t)) {
                    RCLCPP_WARN(get_logger(), "目标非法：需要 6 个有限且在限位内的值");
                    return;   // 丢弃，不 submit
                }
                robot::Command cmd;
                cmd.sequence_id = seq_++;
                cmd.target = t;
                executor_.submit(cmd);   // 只投递，不在这里跑 step
            });

        cancel_srv_ = create_service<std_srvs::srv::Trigger>(
            "/mock_arm/cancel",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                executor_.cancel();
                res->success = true;
                res->message = "accepted";
            });

        // 50ms 周期发布状态（与执行线程 kPeriod 对齐，无需严格同步）
        pub_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]() { publish_state(); });
    }

private:
    void publish_state() {
        robot::RobotState s = executor_.snapshot();
        if (!s.valid) return;   // 未连接/故障时没有可信状态，跳过

        auto joint = sensor_msgs::msg::JointState();
        joint.header.stamp = now();
        joint.name = {"j0", "j1", "j2", "j3", "j4"};
        joint.position.assign(s.arm_rad.begin(), s.arm_rad.end());
        joint_pub_->publish(joint);

        auto grip = std_msgs::msg::Float64();
        grip.data = s.gripper_opening;
        gripper_pub_->publish(grip);
    }

    std::unique_ptr<robot::IRobot> robot_;   // 必须先于 executor_ 声明（executor_ 用 *robot_ 初始化）
    robot::Executor executor_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::uint64_t seq_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MockArmNode>());
    rclcpp::shutdown();
    return 0;
}
