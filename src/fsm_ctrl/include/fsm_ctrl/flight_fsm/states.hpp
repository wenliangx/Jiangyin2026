#ifndef FSM_CTRL_FLIGHT_FSM_STATES_HPP_
#define FSM_CTRL_FLIGHT_FSM_STATES_HPP_

namespace fsm_ctrl {
namespace flight_fsm {

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

}  // namespace flight_fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FLIGHT_FSM_STATES_HPP_
