#ifndef FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_
#define FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_

#include <string>

namespace fsm_ctrl {
namespace single_sml {

struct Idle {};
struct ArmOnly {};
struct NmpcHover {};
struct Landing {};
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
struct OnTargetRecognized {
  std::string label;
};
struct OnSegmentTimeout {};
struct OnFinalSegmentComplete {};
struct Tick {};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_
