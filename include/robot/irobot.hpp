#pragma once
#include "robot/types.hpp"

namespace robot {

class IRobot {
public:
    virtual ~IRobot() = default;
    virtual RobotResult connect() = 0;
    virtual void disconnect() = 0;
    virtual ReadResult read() = 0;
    virtual RobotResult submit_target(const JointValues&) = 0;
    virtual RobotResult step() = 0;
};

}  // namespace robot
//virtual .. =0 只声明不是先，子类必须实现，否则子类也是抽象类不能实例化
//virtual ~IRobot() = default;
//虚析构函数 析构时通过基类指针delete，如果析构不是虚的，那么会导致IROBOT指向的内容内存
//泄露，加上virual后delete会正确调用到mockrobot
