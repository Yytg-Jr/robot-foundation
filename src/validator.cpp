#include "robot/validator.hpp"
#include <cmath>
#include <sstream>
#include <vector>
//用到的语法：const& 用原值 try..catch 抓异常 stod string to double 直到不能转换为double为止，并记录位置
//move 减少一次拷贝，因为原值调用到成员时要拷贝一次，这时用Move不要成员函数的存储减少一次存储
namespace robot{

bool is_finite(double v){
	return std::isfinite(v);
}
namespace {
bool parse_double(const std::string& tok, double& out) {
    // 只接受本练习的简单数值格式：整段可被完整消费
    try {
        std::size_t pos = 0;
        out = std::stod(tok, &pos);
        return pos == tok.size();
    } catch (...) {
        return false;
    }
}

bool parse_u64(const std::string& tok,uint64_t& out){
	try {
		std::size_t pos = 0;
		out = std::stoull(tok,&pos);
		return pos == tok.size();
	}catch(...){
		return false;
	}
}

std::vector<std::string> split_csv(const std::string& line){
	std::vector<std::string> out;
	std::string cur;
	for(char c : line){
		if(c ==','){out.push_back(cur);cur.clear();}
		else       {cur.push_back(c); }
	}
	out.push_back(cur);
	return out;
}
ValidationError make_err(ErrorCode c, std::string field, std::string reason) {
    return ValidationError{c, std::move(field), std::move(reason)};
}
}//namespace

ParseResult parse_and_validate(const std::string& line,std::size_t line_no){
    (void)line_no;//行号由调用方补进输出
    ParseResult r;

    auto fields = split_csv(line);
    constexpr std::size_t kExpected = 7;//seq + 5 joints + gripper
    if(fields.size() !=kExpected){
        r.error = make_err(ErrorCode::FieldCount, "<row>",
                           "expected 7 fields, got " +
                           std::to_string(fields.size()));
	return r;
    }
    std::uint64_t seq = 0;
    if (!parse_u64(fields[0], seq)) {
        r.error = make_err(ErrorCode::Parse, "sequence_id", "not a uint64");
        return r;
    }

    std::array<double, 5> arm{};
    for (std::size_t i = 0; i < arm.size(); ++i) {
        if (!parse_double(fields[1 + i], arm[i])) {
            r.error = make_err(ErrorCode::Parse, "q" + std::to_string(i),
                               "not a number");
            return r;
        }
    }
    double grip = 0.0;
    if (!parse_double(fields[6], grip)) {
        r.error = make_err(ErrorCode::Parse, "gripper", "not a number");
        return r;
    }

    // 2) 有限性检查（NaN/Inf 必须单独用 isfinite，不能用大小比较）
    for (std::size_t i = 0; i < arm.size(); ++i) {
        if (!is_finite(arm[i])) {
            r.error = make_err(ErrorCode::NotFinite,
                               "q" + std::to_string(i), "NaN or Inf");
            return r;
        }
    }
    if (!is_finite(grip)) {
        r.error = make_err(ErrorCode::NotFinite, "gripper", "NaN or Inf");
        return r;
    }

    // 3) 范围检查（教学限制）
    for (std::size_t i = 0; i < arm.size(); ++i) {
        //if (arm[i] < kJointMin || arm[i] > kJointMax) {
        if (arm.at(i) < kJointMin || arm.at(i) > kJointMax) {
            r.error = make_err(ErrorCode::JointOutOfRange,
                               "q" + std::to_string(i),
                               "outside [-1, 1]");
            return r;
        }
    }
    if (grip < kGripperMin || grip > kGripperMax) {
        r.error = make_err(ErrorCode::GripperOutOfRange, "gripper",
                           "outside [0, 1]");
        return r;
    }
    Command cmd;
    cmd.sequence_id = seq;
    cmd.target.arm_rad = arm;
    cmd.target.gripper_opening = grip;
    r.command = cmd;
    return r;
}

std::string format_accept(const Command& c, std::size_t line_no) {
    std::ostringstream os;
    os << "ACCEPT line=" << line_no << " seq=" << c.sequence_id
       << " target=[";
    for (std::size_t i = 0; i < c.target.arm_rad.size(); ++i) {
        if (i) os << ',';
        os << c.target.arm_rad[i];
    }
    os << "] gripper=" << c.target.gripper_opening;
    return os.str();
}


}//robot

