#include <iostream>
#include <string>
#include "robot/validator.hpp"

namespace {
bool is_header(const std::string& line) {
    return line.rfind("sequence_id", 0) == 0;
}
} // namespace

int main() {
    robot::Command last_accepted{};   // 初始全零
    std::string line;
    std::size_t line_no = 0;

    while (std::getline(std::cin, line)) {
        ++line_no;
        if (line.empty()) continue;
        if (line_no == 1 && is_header(line)) continue; // header 单独识别

        auto res = robot::parse_and_validate(line, line_no);
        if (res.command) {
            last_accepted = *res.command; // 仅在完整校验通过后提交
            std::cout << robot::format_accept(last_accepted, line_no) << '\n';
        } else {
            std::cout << "REJECT line=" << line_no
                      << " reason=" << res.error.reason
                      << " field="   << res.error.field << '\n';
        }
    }
    return 0;
}
