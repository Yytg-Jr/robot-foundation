#include "robot/robot_factory.hpp"   // 不是 "robot_factory.hpp"
  #include "robot/mock_robot.hpp"      // 不是 "mock_robot.hpp"
  namespace robot {

  std::unique_ptr<IRobot> make_robot() {
      return std::make_unique<MockRobot>();   // MockRobot 的 seed 参数有默认值 0，可省略
  }

  }  // namespace robot