#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>

namespace ocs2::legged_robot {

struct RobustWindowData {
    bool       active{false};
    bool       skip_t_a_boundary{false};   // true once the absolute start is before initTime
    scalar_t   t_a{0.0};
    scalar_t   t_b{0.0};
    scalar_t   t_nominal{0.0};  // original gait touchdown, before the configured delay
    vector3_t  p_plane{vector3_t::Zero()};
    vector3_t  n{vector3_t::UnitZ()};
    scalar_t   d{0.0};
    int        d_state_index{-1};  // auxiliary MPC state; negative selects fixed d
    // Foot frame offset above the contact point along the terrain normal.
    // EndEffectorKinematics returns the FK position of the URDF foot frame (ankle joint
    // for go2), which is `foot_frame_offset` meters above the ground contact along n.
    // BoundaryConstraint subtracts n·foot_frame_offset so g(x) = 0 means "contact point
    // on the terrain plane" (not "foot frame on the terrain plane").
    scalar_t   foot_frame_offset{0.0};
    scalar_t   dt_mpc{0.015};               // for isActive(t) tolerance
    // Derived from the full absolute window, including any liftoff clamp:
    // v_max = 2*d_max/(t_b-t_a). Replanning never shortens that denominator.
    scalar_t   v_max{0.0};
    size_t     k_a_phase_index{0};
    size_t     k_b_phase_index{0};
};

using robust_window_array_t = feet_array_t<RobustWindowData>;

}  // namespace ocs2::legged_robot
