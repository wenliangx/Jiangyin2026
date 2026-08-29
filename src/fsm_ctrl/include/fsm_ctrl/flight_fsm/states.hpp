#pragma once

namespace fsm_ctrl {

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

}  // namespace fsm_ctrl
