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
struct MissionTrack {};
struct EgoTrack {};
struct Emergency {};
struct SafeNoop {};

struct SelectIdle {};
struct SelectLowThrust {};
struct SelectPositionHold {};
struct SelectNmpcHover {};
struct SelectLanding {};
struct SelectNmpcTrack {};
struct SelectSuperTrack {};
struct SelectMissionTrack {};
struct SelectEgoTrack {};
struct SelectEmergency {};
struct SelectSafeNoop {};
struct Tick {};

}  // namespace single_sml
}  // namespace fsm_ctrl

#endif  // FSM_CTRL_SINGLE_OFFBOARD_SML_STATES_HPP_
