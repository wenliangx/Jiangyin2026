#pragma once

#include <string>

namespace fsm_ctrl {

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

}  // namespace fsm_ctrl
