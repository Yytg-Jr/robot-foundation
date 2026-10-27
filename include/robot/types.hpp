#pragma once
#include <array>
#include <cstdint>
#include <string>
namespace robot{
	struct JointValues{
		std::array<double,5> arm_rad{};
		double gripper_opening{0.0};
	};
	struct Command{
		std::uint64_t sequence_id{0};
		JointValues target;
	};
	struct RobotState {
    	bool connected{false};//连接状态
    	bool valid{false}; //本帧状态是否可信
    	std::array<double,5> arm_rad{};
    	double gripper_opening{0.0};
	};

	enum class RobotError{
    	None,
   		NotConnected,
    	AlreadyConnected,
    	InvalidTarget, //目标越界/非有限
    	ReadTimeout,    //故障注入
    		FaultInjected,  
	};

	struct ReadResult {
    	RobotError code{RobotError::None};
    	RobotState state;
    		bool ok() const { return code == RobotError::None && state.valid; }
	};

	enum class FaultMode {
    	None,
   		ReadTimeoutAtN,
    	InvalidFeedbackAtN,
	};

	struct RobotResult {
    	RobotError code{RobotError::None};
    	std::string reason;
    	bool ok() const {return code == RobotError::None;}
	};

	constexpr double kJointMin = -1.0;
	constexpr double kJointMax =  1.0;
	constexpr double kGripperMin = 0.0;
	constexpr double kGripperMax =  1.0;
}


