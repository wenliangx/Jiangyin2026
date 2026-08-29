#ifndef FSM_CTRL_FLIGHT_FSM_COMMAND_METADATA_HPP_
#define FSM_CTRL_FLIGHT_FSM_COMMAND_METADATA_HPP_

#include <cstddef>

namespace fsm_ctrl {
namespace flight_fsm {

// UDP 命令菜单元数据。新增命令时可让终端/UI 复用这一张表；状态转换
// 仍由 machine.hpp 唯一定义。
struct CommandMeta {
  int command;
  const char* target;
};

constexpr CommandMeta kCommandMeta[] = {
    {0, "Idle"},          {1, "ArmOnly"},       {2, "NmpcHover"},
    {3, "SuperSegment1"}, {4, "SuperSegment2"}, {5, "SuperSegment3"},
    {6, "Landing"},       {7, "SafeNoop"},      {8, "SafeNoop"},
    {9, "Emergency"},
};

constexpr std::size_t kCommandMetaCount =
    sizeof(kCommandMeta) / sizeof(kCommandMeta[0]);

}  // namespace flight_fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FLIGHT_FSM_COMMAND_METADATA_HPP_
