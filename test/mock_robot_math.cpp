#include <cmath>
#include <cstdio>
#include <stdexcept>
#include "robot/mock_robot.hpp"
#include <initializer_list>
namespace {

int g_checks = 0;
int g_failures = 0;

void check_close(double actual, double expected, double tol,
                 const char* expr, int line) {
    ++g_checks;
    if (std::fabs(actual - expected) > tol) {
        ++g_failures;
        std::printf("FAIL line %d: %s  actual=%.17g expected=%.17g tol=%g\n",
                    line, expr, actual, expected, tol);
    }
}

void check_true(bool cond, const char* expr, int line) {
    ++g_checks;
    if (!cond) {
        ++g_failures;
        std::printf("FAIL line %d: %s\n", line, expr);
    }
}

#define CHECK_CLOSE(a, e, tol) check_close((a), (e), (tol), #a, __LINE__)
#define CHECK_TRUE(c)          check_true((c), #c, __LINE__)
//# 是字符串化操作符  __LINE__ __FILE__ __func__让断言报告是哪一个文件的哪一个函数 或哪一行

constexpr double kTol = 1e-9;   // 0.025 累加 12 次的浮点误差远小于此

// D8 验收①：未连接不能提交
void test_not_connected() {
    robot::MockRobot r;                     // 不调用 connect()
    robot::JointValues t;
    auto res = r.submit_target(t);
    CHECK_TRUE(res.code == robot::RobotError::NotConnected);

    auto s = r.read();                      // 副本，且标记为无效
    CHECK_TRUE(s.code == robot::RobotError::NotConnected);
    CHECK_TRUE(!s.state.valid);
}

// D8 验收②：接受目标后当前状态尚未瞬间改变
void test_no_instant_jump() {
    robot::MockRobot r;
    r.connect();

    robot::JointValues t;
    t.arm_rad[0] = 0.3;
    CHECK_TRUE(r.submit_target(t).ok());

    auto before = r.read();                 // 提交后立刻读
    CHECK_CLOSE(before.state.arm_rad[0], 0.0, kTol);   // 仍是旧值
    CHECK_CLOSE(before.state.gripper_opening, 0.0, kTol);

    r.step();
    auto after = r.read();
    CHECK_CLOSE(after.state.arm_rad[0], 0.025, kTol);  // 一步后才动
}

// 手算：q0 0 -> 0.3，每步 0.025；第1步 0.025、第4步 0.1、第12步 0.3、之后保持
void test_joint_ramp() {
    robot::MockRobot r;
    r.connect();

    robot::JointValues t;
    t.arm_rad[0] = 0.3;
    CHECK_TRUE(r.submit_target(t).ok());

    const double expected_first4[] = {0.025, 0.05, 0.075, 0.1};
    for (int n = 1; n <= 13; ++n) {
        r.step();
        double q0 = r.read().state.arm_rad[0];
        if (n <= 4)  CHECK_CLOSE(q0, expected_first4[n - 1], kTol);
        if (n == 12) CHECK_CLOSE(q0, 0.3, kTol);   // 第12步到达
        if (n == 13) CHECK_CLOSE(q0, 0.3, kTol);   // 不越过、保持
    }
}

// 手算：gripper 0 -> 0.8，每步 0.05；第16步到达，第17步保持
void test_gripper_ramp() {
    robot::MockRobot r;
    r.connect();

    robot::JointValues t;
    t.gripper_opening = 0.8;
    CHECK_TRUE(r.submit_target(t).ok());

    for (int n = 1; n <= 15; ++n) {
        r.step();
        if (n == 1)  CHECK_CLOSE(r.read().state.gripper_opening, 0.05, kTol);
        if (n == 15) CHECK_CLOSE(r.read().state.gripper_opening, 0.75, kTol);
    }
    r.step();   // 第16步
    CHECK_CLOSE(r.read().state.gripper_opening, 0.8, kTol);
    r.step();   // 第17步
    CHECK_CLOSE(r.read().state.gripper_opening, 0.8, kTol);
}

// 非法目标被拒，且不污染 current_
void test_invalid_target_rejected() {
    robot::MockRobot r;
    r.connect();

    robot::JointValues t;
    t.arm_rad[0] = 1.5;                     // 越界
    CHECK_TRUE(r.submit_target(t).code == robot::RobotError::InvalidTarget);

    r.step();                               // target_ 未更新，current_ 不动
    CHECK_CLOSE(r.read().state.arm_rad[0], 0.0, kTol);
}
// 生命周期：重复 connect 被拒；断开后能重连
  void test_connect_lifecycle() {
      robot::MockRobot r;
      CHECK_TRUE(r.connect().ok());
      CHECK_TRUE(r.connect().code == robot::RobotError::AlreadyConnected);
      r.disconnect();
      CHECK_TRUE(r.connect().ok());   // 断开后可重连
  }

  // RAII：正常离开作用域自动断开
  void test_raii_normal() {
      robot::MockRobot r;
      {
          robot::ConnectionGuard g(r);
          CHECK_TRUE(g.ok());
          CHECK_TRUE(r.is_connected());
      }                                  // 离开作用域，无需手动 disconnect
      CHECK_TRUE(!r.is_connected());
  }

  // RAII：异常展开也自动断开（D9 验收核心）
  void test_raii_exception() {
      robot::MockRobot r;
      try {
          robot::ConnectionGuard g(r);
          CHECK_TRUE(r.is_connected());
          throw std::runtime_error("boom");
      } catch (...) {
          // 抛异常期间 g 的析构已执行 disconnect
      }
      CHECK_TRUE(!r.is_connected());
  }

  void test_read_timeout_fault() {
      robot::MockRobot r;
      r.connect();
      r.configure_fault(robot::FaultMode::ReadTimeoutAtN, 3);

      CHECK_TRUE(r.read().ok());                 // 读1
      CHECK_TRUE(r.read().ok());                 // 读2
      CHECK_TRUE(r.read().code == robot::RobotError::ReadTimeout);   // 读3 超时
      CHECK_TRUE(r.submit_target(robot::JointValues{}).code == robot::RobotError::FaultInjected);

      r.disconnect();
      r.connect();                                // 重连=显式 reset
      CHECK_TRUE(r.read().ok());                  // 计数已清零，可重新跑
  }
} // namespace

int main() {
    test_not_connected();
    test_connect_lifecycle();
    test_no_instant_jump();
    test_joint_ramp();
    test_gripper_ramp();
    test_invalid_target_rejected();
    test_raii_normal();
    test_raii_exception();
    test_read_timeout_fault();
    std::printf("%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;         // 非零退出码，ctest 可判失败
}


//----------------------------------验收方式---------------------------------
/*cd ~/robot-foundations
  cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
  cmake --build build -j
  ctest --test-dir build --output-on-failure  */