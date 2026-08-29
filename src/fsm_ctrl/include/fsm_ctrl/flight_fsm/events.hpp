#ifndef FSM_CTRL_FLIGHT_FSM_EVENTS_HPP_
#define FSM_CTRL_FLIGHT_FSM_EVENTS_HPP_

#include <string>

namespace fsm_ctrl {
namespace flight_fsm {

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

}  // namespace flight_fsm
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_FLIGHT_FSM_EVENTS_HPP_
