# MuJoCo perceptive visualization

Run the MuJoCo simulator as usual, then launch the controller with a matching
terrain scene (this launch does not start the simulator itself):

```bash
source /home/tony/GO2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py \
  enable_perceptive:=true \
  terrain_scene_file:=basic_step \
  perceptive_foot_placement_boundary_margin:=0.05 \
  perceptive_foot_collision_clearance:=0.03
```

Switch the robot to OCS2 locomotion using the existing control workflow. Terrain
can appear before walking; foot placement and swing markers require MPC reference
updates. Swing curves appear when the horizon contains a swing phase.

`perceptive_foot_collision_clearance` is the minimum SDF distance of the foot
FK point, in meters: `SDF(p_foot) - clearance >= 0`. Its default is 0.03 m;
values must be finite and non-negative. It is used only when perceptive mode
and the foot collision constraint are enabled, and retains the existing
stance/transition/robust-window deactivation rules. It does not change the
placement polygon margin or the swing spline height. The value is loaded when
the controller is initialized; restart the controller launch to apply changes.

The default RViz configuration includes:

| Display | Topic | Meaning |
| --- | --- | --- |
| Perceptive Foot Placement | `/foot_placement` | Faint raw convex boundaries, thick feasible boundaries with the controller's margin, nominal foothold spheres, and terrain projection arrows |
| Swing Z Reference (XY interpolated) | `/perceptive_reference/swing_trajectories` | Actual planner z(t), with xy interpolated between adjacent stance projections for display |
| MPC Predicted Base and Feet | `/legged_robot/optimizedStateTrajectory` | Optimized MPC prediction, including feet; different from the swing height reference |
| Terrain Boundaries | `/convex_plane_decomposition_ros/boundaries` | Planar-region outer and hole boundaries |
| Terrain Insets | `/convex_plane_decomposition_ros/insets` | Inset regions used for foothold projection |
| GridMap | `/convex_plane_decomposition_ros/filtered_map` | Terrain elevation |

Foot colors follow the existing visualizer order: FL blue, FR orange, RL yellow,
RR purple (for the default Go2 foot ordering). No text markers are generated.

The feasible polygon uses the same half-space/margin helpers as MPC, including
fallback to the raw polygon if shrinking rejects the projection seed. Its RViz
namespace is then `Foot Placement Margin Fallback`. Polygons are shown for the
current/future stance selections within the MPC horizon. Initial stance regions
are raw-only until the controller's initial-stance exclusion ends. Disabling
`enable_perceptive_foot_placement_constraint` hides the feasible-region markers
while retaining the raw planning geometry. These are soft constraints; the
optimized foot can still violate their boundary at a penalty.

The swing planner supplies z and vertical velocity, not an independently tracked
xyz swing reference. The rendered xy interpolation is visualization only; it is
not fed to MPC. The robust phase can deactivate the normal swing constraint, so
the displayed height reference alone does not indicate constraint activation.

Geometry and splines are sampled together on the MPC thread after reference
updates, capped at 20 Hz and skipped without subscribers. Each message clears
the previous markers so old horizon regions/curves disappear after replanning.
The fixed frame is `odom`. To use an external PlanarTerrain source instead of
the static MuJoCo scene publisher, add `publish_static_terrain:=false` and publish
the controller's terrain topic in the same frame.

Headless regression test (build with `-DBUILD_TESTING=ON`):

```bash
source /home/tony/GO2_ws/install/setup.bash
ctest --test-dir /home/tony/GO2_ws/build/ocs2_quadruped_controller \
  -R '^perceptive_visualization_smoke_test$' --output-on-failure
```

This tests the actual marker publisher against a synthetic sloped plane and
gait schedule, including margin geometry, sampled swing height, disabled
placement, shrink fallback, and clearing swing markers after a stance replan.
