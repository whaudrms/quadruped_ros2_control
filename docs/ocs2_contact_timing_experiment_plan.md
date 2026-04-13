## OCS2 Contact Timing Bias Experiment Plan

### Goal
- Measure how much `contact_timing_bias = d` a pure nominal OCS2 controller can tolerate.
- Use this as a dataset-building stage before any robust solver work.

### Terrain Heights
- Primary terrain heights for the final study:
  - `1 cm`
  - `3 cm`
  - `5 cm`
  - `7 cm`
  - `9 cm`

### Current Strategy
- Use the `5 cm` step-down terrain first to determine a meaningful `d` range.
- Reuse that `d` grid on `1, 3, 7, 9 cm`.
- If the plots are not clean enough, refine the `d` grid per terrain afterward.

### Current Findings on 5 cm Step-Down
- Stable success:
  - `d = 0.00 s`
  - `d = 0.05 s`
  - `d = 0.07 s`
- Success but visibly unstable:
  - `d = 0.08 s`
- Failure:
  - `d = 0.10 s`
  - `d = 0.12 s`
  - `d = 0.15 s`

### Recommended Initial d Grid
- For the main repeated experiment:
  - `0.00`
  - `0.05`
  - `0.07`
  - `0.08`
  - `0.10`
  - `0.12`
  - `0.15`

### Repeat Count
- Final experiment:
  - `10 runs per (terrain_height, d)`

### Main Figures

#### Figure 1: Forward Progress vs Timing Bias
- x-axis:
  - `contact timing bias d [s]`
- y-axis:
  - `body_frame_forward_progress [m]`
- plot:
  - mean line
  - error bars or std band
  - optional markers for per-run samples

#### Figure 2: Stability vs Timing Bias
- Two-panel figure
- Panel A:
  - `body_frame_lateral_progress [m]` vs `d`
- Panel B:
  - `roll_rms_deg` vs `d`
  - optionally `pitch_rms_deg` in appendix

### Outcome Labels
- `stable`
  - task completed
  - forward progress sufficiently positive
  - lateral drift and roll/pitch RMS small
- `unstable`
  - task completed, but lateral drift or attitude oscillation is noticeably large
- `fail`
  - fall, stop-like behavior, or no meaningful forward progress

### Run-Level CSV Schema
- One row per run

Columns:
- `run_id`
- `timestamp`
- `terrain_key`
- `terrain_height_cm`
- `contact_timing_bias_s`
- `repeat_idx`
- `success`
- `task_completion_success`
- `outcome_label`
- `fall_reason`
- `time_to_failure_s`
- `duration_executed_s`
- `distance_xy_m`
- `body_frame_forward_progress_m`
- `body_frame_lateral_progress_m`
- `roll_rms_deg`
- `pitch_rms_deg`
- `yaw_rms_deg`
- `yaw_change_deg`
- `min_base_z_m`
- `base_z_std_m`
- `start_pose_x`
- `start_pose_y`
- `start_pose_z`
- `end_pose_x`
- `end_pose_y`
- `end_pose_z`
- `result_dir`
- `controller_csv_path`

### Aggregate CSV Schema
- One row per `(terrain_height_cm, contact_timing_bias_s)`

Columns:
- `terrain_height_cm`
- `contact_timing_bias_s`
- `n_runs`
- `success_rate`
- `stable_rate`
- `unstable_rate`
- `fail_rate`
- `forward_progress_mean`
- `forward_progress_std`
- `lateral_progress_mean`
- `lateral_progress_std`
- `roll_rms_mean`
- `roll_rms_std`
- `pitch_rms_mean`
- `pitch_rms_std`
- `time_to_failure_mean`
- `time_to_failure_std`

### Practical Rule
- First finish the `5 cm` timing-bias range cleanly.
- Then apply the same `d` list to `1, 3, 7, 9 cm`.
- Only refine the `d` list per terrain if the resulting plots are not clean enough.
