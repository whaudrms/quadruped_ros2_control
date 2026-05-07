#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>

namespace ocs2::legged_robot {

struct RobustWindowData {
    bool       active{false};
    bool       skip_t_a_boundary{false};   // true when t_a was clamped to initTime (partial window)
    scalar_t   t_a{0.0};
    scalar_t   t_b{0.0};
    vector3_t  p_plane{vector3_t::Zero()};
    vector3_t  n{vector3_t::UnitZ()};
    scalar_t   d{0.0};
    scalar_t   dt_mpc{0.015};               // for isActive(t) tolerance
    size_t     k_a_phase_index{0};
    size_t     k_b_phase_index{0};
};

using robust_window_array_t = feet_array_t<RobustWindowData>;

}  // namespace ocs2::legged_robot
