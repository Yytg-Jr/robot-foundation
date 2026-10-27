#pragma once
  #include <memory>

  #include "robot/irobot.hpp"   // 拿到 IRobot 的完整接口；调用方本来也要用它

  namespace robot {

  // 工厂：返回基类指针，调用方只依赖接口，不知道具体是 MockRobot
  std::unique_ptr<IRobot> make_robot();

}  // namespace robot