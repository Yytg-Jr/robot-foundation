#pragma once
#include <cstddef>
#include <optional>
#include <string>
#include "robot/types.hpp"

namespace robot {

enum class ErrorCode {
    None,
    FieldCount,      // 字段少/多
    Parse,           // abc 之类
    NotFinite,       // NaN/Inf
    JointOutOfRange,
    GripperOutOfRange,
    BadSequenceId,
};

struct ValidationError {
    ErrorCode code{ErrorCode::None};
    std::string field;   // 如 "q0" / "gripper" / "sequence_id"
    std::string reason;  // 人类可读
};

struct ParseResult {
    std::optional<Command> command;   // 成功才有值
    ValidationError error;            // 成功时 code==None
};

// line 仅用于错误定位；纯函数，不修改任何外部状态
ParseResult parse_and_validate(const std::string& line, std::size_t line_no);

// 是否有限（NaN/Inf 都会被拒）。注意 NaN 不能用大小比较过滤。
bool is_finite(double v);

// 把已解析命令渲染成 ACCEPT 行
std::string format_accept(const Command& c, std::size_t line_no);

} // namespace robot