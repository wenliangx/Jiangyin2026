#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_COMMAND_META_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_COMMAND_META_HPP_

#include <cstddef>

#include <fsm_ctrl/single_offboard_sml/states.hpp>

namespace fsm_ctrl {
namespace single_sml {

// 每个 UDP cmd 的菜单元数据：swarm 终端菜单的编译期唯一来源。
// 必须与 machines/mission.hpp 中 FSM_CTRL_SML_MISSION_COMMAND_TRANSITIONS
// 转换表保持同步（cmd → 目标状态）；目标状态名引用 states.hpp 的 kName。
struct CommandMeta {
  int command;
  const char* target;  // 按下该 cmd 后进入的状态名。
};

constexpr CommandMeta kCommandMeta[] = {
    {0, Idle::kName},
    {1, ArmOnly::kName},
    {2, NmpcHover::kName},
    {3, SuperTrack::kName},  // 守卫条件: MissionAvailable
    {4, Landing::kName},     // 守卫条件: MissionAvailable
    {5, SafeNoop::kName},
    {6, SafeNoop::kName},
    {7, SafeNoop::kName},
    {8, SafeNoop::kName},
    {9, Emergency::kName},
};

constexpr std::size_t kCommandMetaCount =
    sizeof(kCommandMeta) / sizeof(kCommandMeta[0]);

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_COMMAND_META_HPP_
