# OCS2 Quadruped Controller

This is a ros2-control controller based on [legged_control](https://github.com/qiayuanl/legged_control)
and [ocs2_ros2](https://github.com/legubiao/ocs2_ros2).

Tested environment:

* Ubuntu 24.04
    * ROS2 Jazzy
* Ubuntu 22.04
    * ROS2 Humble

* [x] **[2025-01-16]** Add support for ground truth estimator.
* [x] **[2025-03-15]** OCS2 Controller now can switch between passive and MPC mode.


[![](http://i0.hdslb.com/bfs/archive/e758ce019587032449a153cf897a543443b64bba.jpg)](https://www.bilibili.com/video/BV1UcxieuEmH/)

## 1. Interfaces

Required hardware interfaces:

* command:
    * joint position
    * joint velocity
    * joint effort
    * KP
    * KD
* state:
    * joint effort
    * joint position
    * joint velocity
    * imu sensor
        * linear acceleration
        * angular velocity
        * orientation
    * feet force sensor

## 2. Build

### 2.1 Build Dependencies
Before install OCS2 ROS2, please follow the guide to install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html). **Don't use the pinocchio install by rosdep**!

**After installed Pinocchio**, follow below step to clone ocs2 ros2 library to src folder.

```bash
cd ~/ros2_ws/src
git clone https://github.com/legubiao/ocs2_ros2

cd ocs2_ros2
git submodule update --init --recursive

cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### 2.2 Build OCS2 Quadruped Controller

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_quadruped_controller  --symlink-install
```

## Visible terrain for foothold selection

The MuJoCo static terrain publisher removes lower surfaces hidden below box
tops, including covered floor and overlapping platforms. Both region boundaries
and projection insets are clipped; newly exposed obstacle edges have a 0.02 m
inset margin. This uses the perceived heights after terrain offsets are applied.

Foothold selection additionally rejects projections inconsistent with the raw
`elevation` map (0.01 m tolerance, corrected for plane slope at the map cell
center). If no valid candidate remains, it reports an error rather than selecting
a buried plane. Smoothing remains available for body references.

`visible_terrain_test` checks flat ground, step up/down, overlapping and rotated
platforms, ramps, perception offsets, convex foothold boundaries, and preservation
of holes through ROS message conversion. It can also accept scene XML paths.

## Robust phase timing

`robustPhase.t_a` and `robustPhase.t_b` are nonnegative offsets in seconds
relative to the **original gait touchdown**: start = touchdown - t_a,
end = touchdown + t_b. Defaults are 0.05 s each. Start is capped at liftoff;
end must precede the next liftoff. Without confirmed contact, the foot stays
in swing until robust end. The nominal gait is kept separately, so the delay
is applied once and later liftoffs and gait periods do not drift.

For `standing_trot`, nominal per-leg swing/stance are 0.25/0.35 s with a 0.60 s
period. The default offsets give 0.20 s nominal swing, 0.10 s robust swing,
and 0.30 s stance. Other gaits have different durations; the Go2 `task.info`
contains these reference timings next to the parameters.

`v_max = 2*d_max/(end-start)` is calculated for each full window. At d_max=0.05 m and
T=0.10 s this is 1.0 m/s. Neither the offsets nor this rate change when MPC
replans halfway through the window; elapsed start boundary costs are skipped.
The old `P` and fixed `v_max` settings are ignored with a migration message.
The velocity envelope remains a soft constraint.

## Robust contact splice

With `robustPhase.enabled=true` and `robustPhase.enable_splice=true`, five
consecutive controller ticks of measured contact during scheduled swing inside
the robust window confirm a contact. The first tick's time and window bounds
are retained through debounce. The next MPC reference update overlays stance
only for that leg from its contact time to its configured robust end (the
delayed touchdown). This also supports contact after the original touchdown.

The nominal `GaitSchedule` is retained separately. Relative to the schedule
with configured touchdown delays, splice preserves all event times, other
legs' contact intervals, and subsequent liftoffs. Contacts
are neither snapped to nearby events nor batched at another leg's time; even
sub-shooting-interval phases retain their measured timestamps. Exactly coincident
events share one boundary, using OCS2's existing pre-event convention at the
boundary itself. Recent overlays are reapplied on each solve and discarded if
their delayed touchdown is no longer valid after a gait change.

The revised schedule feeds terrain references, swing planning, robust-window
recomputation and the MPC policy in the same solve. WBC uses the new policy's
planned mode, so stance execution follows contact confirmation and MPC latency.
Contact transitions emit logs even when `robustPhase.verbose_log=false`:

- `[robust_event]`: the first detected contact tick.
- `[robust_contact_splice]` with `stage=mpc_schedule`: MPC accepts the stance
  overlay at `apply_t`.
- `[robust_stance_enter]`: WBC first uses the spliced stance mode, once per leg
  and contact window. `contact_t` is the first contact tick, `wbc_t` is the WBC
  observation time, and `delay_ms` includes debounce and MPC latency in
  observation time. The log also identifies the foot, robust window and mode.

The `robust_contact_schedule_test` regression test checks interval preservation,
nearby/coincident contacts, subsequent swings and stale requests.

## 3. Launch

supported robot description:

* Unitree
    * go2_description
    * go1_description
    * a1_description
    * aliengo_description
    * b2_description
* Xiaomi
    * cyberdog_description
* DeepRobotics
    * lite3_description
    * x30_description
* Anybotics
    * anymal_c_description

### 3.1 About OCS2 Shared Library
OCS2 Quadruped controller depends on the OCS2 library, it required c++ automatic differentiation shared library. When first launch the controller, it will compile the OCS2 model and generate the shared library. 

You may see something similar to:
```
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_libcppadcg_tmp-27918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_position/cppad_generated/RR_foot_position_lib.so
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_libcppadcg_tmp-94918274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_velocity/cppad_generated/RR_foot_velocity_lib.so
[gazebo-5] [CppAdInterface] Compiling Shared Library: /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so
[gazebo-5] [CppAdInterface] Renaming /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_libcppadcg_tmp-83618274.so to /home/biao/ocs2_cpp_ad/b2/RR_foot_orientation/cppad_generated/RR_foot_orientation_lib.so
```
The compilation process may take a few minutes. After the compilation, restart the controller and the robot should stand up.

To config the path for the cppAD shared library, you can modify the `modelFolderCppAd` item in `task.info` file, which located at the `config/ocs2` folder under robot description package. If the path is not start with `/`, it will be considered as **relative path to the Linux Home folder**.

### 3.2 Usage
#### Keyboard State Switch
* Keyboard 1 : Passive Mode
* Keyboard 2 : OCS2 MPC Mode
  * Keyboard 2: stance
  * Keyboard 3: trot
  * Keyboard 4: standing_trot
  * Keyboard 5: flying_trot

### 3.3 Launch Controller
#### Mujoco Simulation
> **Warm Reminder**: You need to launch [Unitree Mujoco C++ Simulation](https://github.com/legubiao/unitree_mujoco) before launch the controller.
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py pkg_description:=go2_description
```

#### Gazebo Launch
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
```


### Optimized robust band width

With `robustPhase.enabled=true` and `optimize_d=true`, SQP optimizes four
constant parameters, one for each leg's first unfinished robust window.
The MPC state is `[x_physical(24), d_i(4)]`, with `dot(d_i)=0`; inputs remain
24-dimensional. A QP-only initialization stage frees the four initial widths
while keeping the physical initial state fixed. No extra stage is exported in
the MPC policy. Physical model terms receive only the first 24 states, and WBC
receives the same physical state/input dimensions as before.

`d` is the initial guess (and the fixed width when `optimize_d=false`). The GO2
configuration enables optimization with `d=0.05`, `d_min=0.02`, `d_max=0.05` m.
Hard inequalities enforce the bounds, including the initial and terminal nodes.
A leg without an unfinished robust window retains the same bounds, but its
width is unused by boundary costs. Equal bounds automatically select the
fixed-width formulation.
The speed envelope uses `2*d_max/T` and has no derivative with respect to `d_i`.
The running cost includes `-w_d*d_i` for each active window on `[t_a,t_b)`.
GO2 starts with `w_d=10.0`; zero disables the reward, and older task files without
this key default to zero. The exact state gradient is `-w_d` in the width column
and the Hessian is zero. SQP multiplies the running cost by its integration dt;
there is no extra division by window duration. The reward stops when splice
removes the window, and fixed-width mode has no reward. Larger weights favor
larger widths, but competing costs/constraints can still select `d_min`.
The total incentive decreases with the remaining robust duration after replanning.

Optional tick CSVs append `opt_d0` through `opt_d3` in model contact order
(FL, FR, RL, RR for GO2). Foothold-plan CSVs record the solved FL width in `d`,
the configured guess in `d_init`, all four `opt_d*` values and the actual window
`v_max`. Reference-manager `[robust_phase]` lines are pre-solve metadata: their
`d` still means the configured guess. For fixed-width A/B trials, set
`optimize_d=false` in task.info; `run_trial.py` preserves that setting.

Validation targets: `robust_width_test` checks the augmented OCP, physical
constraint projection and endpoint Jacobians; SQP `FreeInitialState.*` checks
interior optima, both bounds, fixed bounds, warm starts and policy feedback.
