#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_

namespace fsm_ctrl {
namespace single_sml {

struct Idle {};
struct ArmOnly {};
struct CoreHover {};
struct CoreSuperLanding {};
struct CoreLanding {};
struct LowThrust {};
struct PositionHold {};
struct NmpcHover {};
struct Landing {};
struct NmpcTrack {};
struct SuperTrack {};
struct SuperSegment1 {};
struct SuperSegment2 {};
struct SuperSegment3 {};
struct Emergency {};
struct SafeNoop {};

struct OnCommand0 {};
struct OnCommand1 {};
struct OnCommand2 {};
struct OnCommand3 {};
struct OnCommand4 {};
struct OnCommand5 {};
struct OnCommand6 {};
struct OnCommand7 {};
struct OnCommand8 {};
struct OnCommand9 {};
struct OnUnsupportedCommand {};
struct Tick {};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_
