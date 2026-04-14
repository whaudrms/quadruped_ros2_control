  # Contact Timing Bias Study Overview

  This repository currently includes a contact timing bias study built on top of the nominal OCS2 quadruped controller.
  The goal of this study is to measure how sensitive the nominal controller is to contact timing uncertainty during step-down locomotion, without relying on a perceptive terrain model
  inside the solver.

  ## Why This Study Was Added

  The original exploration direction was a perceptive height-only step-up study. That direction was stopped because it did not match the intended solver-level robustness question. In
  the nominal OCS2 path used here, explicit terrain height information was not being consumed in a way that produced meaningful robustness behavior when simple height overrides were
  injected.

  After tracing the nominal OCS2 code path, the study was redefined around **contact timing bias** rather than terrain height perturbation. This means the controller is tested by
  shifting the timing at which contact is interpreted, and then measuring how performance changes as the timing error grows.

  In short:

  - the controller remains nominal OCS2,
  - the terrain remains explicit in MuJoCo,
  - uncertainty is injected through contact timing interpretation,
  - and the resulting locomotion behavior is measured across multiple terrain heights and bias levels.

  ## Study Goal

  The main question of this study is:

  **How much contact timing error can the pure nominal OCS2 controller tolerate on step-down terrains of different heights before locomotion degrades or fails?**

  To answer that, the controller was tested on step-down terrains with heights:

  - `1 cm`
  - `3 cm`
  - `5 cm`
  - `7 cm`
  - `9 cm`

  For each terrain height, contact timing bias was applied with the following values:

  - `0.00 s`
  - `0.05 s`
  - `0.07 s`
  - `0.08 s`
  - `0.09 s`
  - `0.10 s`
  - `0.12 s`
  - `0.15 s`

  The `5 cm` terrain was used first to identify a useful bias grid, and that same grid was then reused for the other terrain heights.

  ## What Was Changed in the Code

  This study adds a contact timing bias path to the nominal OCS2 stack and includes a full experiment, filtering, validation, and handover pipeline.

  Main implementation areas:

  - contact timing bias injection in the nominal OCS2 contact interpretation path
  - per-run controller logging (`controller_state_input.csv`)
  - step-down terrain/scenario configuration for `1, 3, 5, 7, 9 cm`
  - batch execution scripts for repeated trials
  - raw-run filtering and validation scripts
  - summary table, figure, and caption generation scripts

  Relevant files are documented in detail in:

  - [`docs/contact_timing_bias_study_handover.md`](docs/contact_timing_bias_study_handover.md)

  ## Experiment Pipeline

  The experiment pipeline was organized as follows.

  ### 1. Terrain and Scenario Setup

  Step-down scenes were prepared for:

  - `1 cm`
  - `3 cm`
  - `5 cm`
  - `7 cm`
  - `9 cm`

  Main configuration files:

  - [`evaluation/go2_terrain_eval/configs/scenarios/ocs2_stepdown_d_sweep.yaml`](evaluation/go2_terrain_eval/configs/scenarios/ocs2_stepdown_d_sweep.yaml)
  - [`evaluation/go2_terrain_eval/configs/terrains.yaml`](evaluation/go2_terrain_eval/configs/terrains.yaml)

  A snapshot of the MuJoCo XML scenes used in the study is also included here:

  - [`docs/contact_timing_bias_handover_artifacts/scenes/`](docs/contact_timing_bias_handover_artifacts/scenes/)

  ### 2. Raw Trial Execution

  Raw runs were executed with the batch and trial scripts:

  - [`evaluation/go2_terrain_eval/scripts/run_contact_timing_batch.py`](evaluation/go2_terrain_eval/scripts/run_contact_timing_batch.py)
  - [`evaluation/go2_terrain_eval/scripts/run_trial.py`](evaluation/go2_terrain_eval/scripts/run_trial.py)

  Each raw run produced at least:

  - `result.json`
  - `controller.log`
  - `controller_state_input.csv`

  The raw run folders were stored under:

  - `evaluation/go2_terrain_eval/results/`

  ### 3. Filtering

  The final summary tables were **not** built from an overwritten intermediate aggregate file.
  Instead, the filtered outputs were rebuilt directly from raw run folders using:

  - [`evaluation/go2_terrain_eval/scripts/filter_contact_timing_runs.py`](evaluation/go2_terrain_eval/scripts/filter_contact_timing_runs.py)

  Primary exclusion categories were:

  - `short_duration`
  - `low_progress`
  - `startup_collapse`

  Important interpretation rule:

  - `filter_pass == True` does **not** mean the run was a successful locomotion run.
  - It only means the run was not removed by the current filtering rules.
  - Actual success/failure still depends on the run result fields.

  ### 4. Validation

  The filtered outputs were then cross-checked against the raw run folders using:

  - [`evaluation/go2_terrain_eval/scripts/validate_contact_timing_results.py`](evaluation/go2_terrain_eval/scripts/validate_contact_timing_results.py)

  This validation step checked that:

  - filtered runs and excluded runs were internally consistent,
  - terrain-by-bias summary values matched the underlying filtered rows,
  - representative spot-check combinations matched the raw run folders.

  ### 5. Handover Package

  Final researcher-facing tables were generated and packaged using:

  - [`evaluation/go2_terrain_eval/scripts/build_handover_package.py`](evaluation/go2_terrain_eval/scripts/build_handover_package.py)
  - [`evaluation/go2_terrain_eval/scripts/build_compact_researcher_table.py`](evaluation/go2_terrain_eval/scripts/build_compact_researcher_table.py)

  The validated handover package is included here:

  - [`docs/contact_timing_bias_handover_artifacts/`](docs/contact_timing_bias_handover_artifacts/)

  ## What Results Were Produced

  The final package includes:

  ### Summary Tables

  - [`main_table.csv`](docs/contact_timing_bias_handover_artifacts/main_table.csv)
  - [`exclusion_overview.csv`](docs/contact_timing_bias_handover_artifacts/exclusion_overview.csv)
  - [`contact_timing_bias_researcher_summary.csv`](docs/contact_timing_bias_handover_artifacts/contact_timing_bias_researcher_summary.csv)
  - [`contact_timing_bias_researcher_compact_summary.csv`](docs/contact_timing_bias_handover_artifacts/contact_timing_bias_researcher_compact_summary.csv)

  ### Validation Outputs

  - [`contact_timing_bias_validation_report.txt`](docs/contact_timing_bias_handover_artifacts/contact_timing_bias_validation_report.txt)
  - [`contact_timing_bias_validation_samples.csv`](docs/contact_timing_bias_handover_artifacts/contact_timing_bias_validation_samples.csv)

  ### Figures

  Final paper/presentation figures are included here:

  - [`docs/contact_timing_bias_handover_artifacts/figures/`](docs/contact_timing_bias_handover_artifacts/figures/)

  These figures include:

  - success rate vs timing bias
  - stable rate vs timing bias
  - fail rate vs timing bias
  - forward progress vs timing bias
  - roll RMS vs timing bias
  - pitch RMS vs timing bias
  - success/stable/fail heatmaps

  ### Figure Captions

  Both detailed and short captions are included:

  - [`figure_captions.txt`](docs/contact_timing_bias_handover_artifacts/figures/figure_captions.txt)
  - [`figure_captions_short.txt`](docs/contact_timing_bias_handover_artifacts/figures/figure_captions_short.txt)

  ## Main Takeaways

  The validated results support the following high-level observations:

  - increasing contact timing bias degrades nominal OCS2 locomotion performance,
  - larger step-down terrains tend to become sensitive at smaller timing errors,
  - the study package in this repository was rebuilt from raw runs after filtering and validation.

  This repository therefore contains both:

  - the implementation used to run the contact timing bias study,
  - and the validated outputs used for internal sharing and review.

  ## Where to Start

  If you are new to this repository and only want to understand this study quickly, read the following in order:

  1. [`docs/contact_timing_bias_study_handover.md`](docs/contact_timing_bias_study_handover.md)
  2. [`docs/contact_timing_bias_handover_artifacts/handover_note.txt`](docs/contact_timing_bias_handover_artifacts/handover_note.txt)
  3. [`docs/contact_timing_bias_handover_artifacts/main_table.csv`](docs/contact_timing_bias_handover_artifacts/main_table.csv)
  4. [`docs/contact_timing_bias_handover_artifacts/figures/`](docs/contact_timing_bias_handover_artifacts/figures/)



 --- --- --- --- ---



# Quadruped ROS2 Control

This repository contains the ros2-control based controllers for the quadruped robot.

* [Controllers](controllers): contains the ros2-control controllers
* [Commands](commands): contains command node used to send command to the controller
* [Descriptions](descriptions): contains the urdf model of the robot
* [Hardwares](hardwares): contains the ros2-control hardware interface for the robot

> **Warning:** Default branch was developed under ROS2 Jazzy. For ROS2 Humble, please check out **humble** branch.

Todo List:

- [x] **[2025-02-23]** Add Gazebo Playground
  - [x] OCS2 controller for Gazebo Simulation
  - [x] Refactor FSM and Unitree Guide Controller
- [x] **[2025-03-30]** Add Real Go2 Robot Support
- [x] **[2025-05-20]** Isaac Sim Support
- [ ] OCS2 Perceptive locomotion demo

Video on Real Unitree Go2 Robot:
[![](http://i0.hdslb.com/bfs/archive/7d3856b3c5e5040f24990d3eab760cf8ba4cf80d.jpg)](https://www.bilibili.com/video/BV1QpZaY8EYV/)

## 1. Quick Start

* rosdep
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```
* Compile the package
    ```bash
    colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install
    ```

### 1.1 Mujoco Simulator or Real Unitree Robot
> **Warning:** CycloneDDS ROS2 RMW may conflict with unitree_sdk2. If you cannot launch unitree mujoco simulation
> without `sudo`, then you cannot used `unitree_mujoco_hardware`. This conflict could be solved by one of below two
> methods:
> 1. Uninstall CycloneDDS ROS2 RMW, used another ROS2 RMW, such as FastDDS **[Recommended]**.
> 2. Follow the guide in [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) to configure the ROS2 RMW by
     compiling cyclone dds.

* Compile Unitree Hardware Interfaces
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to hardware_unitree_sdk2
    ```
* Follow the guide in [unitree_mujoco](https://github.com/legubiao/unitree_mujoco) to launch the unitree mujoco go2
  simulation
* Launch the ros2-control
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch unitree_guide_controller mujoco.launch.py
    ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![mujoco](.images/mujoco.png)

### 1.3 Gazebo Harmonic Simulator

* Install Gazebo
  ```bash
  sudo apt-get install ros-jazzy-ros-gz
  ```

* Compile Gazebo Playground
  ```bash
  colcon build --packages-up-to gz_quadruped_playground --symlink-install
  ```
* Launch the ros2-control
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch unitree_guide_controller gazebo.launch.py
  ```
* Run the keyboard control node
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run keyboard_input keyboard_input
    ```

![gazebo](.images/gazebo.png)

For more details, please refer to the [unitree guide controller](controllers/unitree_guide_controller/)
and [go2 description](descriptions/unitree/go2_description/).

## What's Next
Congratulations! You have successfully launched the quadruped robot in the simulation. Here are some suggestions for you to have a try:
* **More Robot Models** could be found at [description](descriptions/)
* **Try more controllers**. 
  * [OCS2 Quadruped Controller](controllers/ocs2_quadruped_controller): Robust MPC-based controller for quadruped robot
  * [RL Quadruped Controller](controllers/rl_quadruped_controller): Reinforcement learning controller for quadruped robot
* **Simulate with more sensors**
  * [Gazebo Quadruped Playground](libraries/gz_quadruped_playground): Provide gazebo simulation with lidar or depth camera.
* **Real Robot Deploy**
  * [Unitree Go2 Robot](descriptions/unitree/go2_description): Check here about how to deploy on go2 robot.

## Reference

### Conference Paper

[1] Liao, Qiayuan, et al. "Walking in narrow spaces: Safety-critical locomotion control for quadrupedal robots with
duality-based optimization." In *2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp.
2723-2730. IEEE, 2023.

### Miscellaneous

[1] Unitree Robotics. *unitree\_guide: An open source project for controlling the quadruped robot of Unitree Robotics,
and it is also the software project accompanying 《四足机器人控制算法--建模、控制与实践》 published by Unitree
Robotics*. [Online].
Available: [https://github.com/unitreerobotics/unitree_guide](https://github.com/unitreerobotics/unitree_guide)

[2] Qiayuan Liao. *legged\_control: An open-source NMPC, WBC, state estimation, and sim2real framework for legged
robots*. [Online]. Available: [https://github.com/qiayuanl/legged_control](https://github.com/qiayuanl/legged_control)

[3] Ziqi Fan. *rl\_sar: Simulation Verification and Physical Deployment of Robot Reinforcement Learning Algorithm.*

2024. Available: [https://github.com/fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 
