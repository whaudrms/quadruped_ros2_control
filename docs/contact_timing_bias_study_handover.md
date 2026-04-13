# Contact Timing Bias Study Handover

## 1. Scope

This document summarizes the experiment line that replaced the earlier perceptive height-only step-up direction. The new study keeps the controller in the pure nominal OCS2 setting and injects uncertainty through contact timing bias rather than through an explicit terrain-height estimate inside the solver.

The target audience is the researcher and internal teammates who need:

- the exact research question,
- the code path that was modified,
- the terrain and bias configurations that were executed,
- the filtering and validation rules used after the raw runs were collected,
- and the final handover artifacts that were generated from the validated outputs.

This document is intentionally detailed. It is written so that someone can reconstruct the experiment flow without replaying the entire chat or inspecting every script first.

## 2. Research Direction Change

### 2.1 Previous direction that was stopped

The previous direction tried to make the robot climb or step over terrain using a perceptive height-only path and several reference-side heuristics. That line was stopped because it did not align with the intended solver-level robustness study.

The key issue was conceptual:

- the pure nominal OCS2 path does not meaningfully consume explicit terrain height geometry in the way required by the intended robustness experiment,
- so simply perturbing terrain height helper values was not a valid uncertainty injection for the solver,
- and the resulting behavior did not meaningfully reflect the uncertainty mechanism that the researcher wanted to study.

### 2.2 New direction that was adopted

The new direction studies nominal OCS2 robustness under **contact timing bias**.

The working interpretation is:

- keep the controller nominal,
- keep the step-down terrain explicit in MuJoCo,
- inject uncertainty into the controller's contact interpretation timing,
- then measure how far the nominal controller can tolerate the bias before behavior becomes degrading or failing.

This direction is solver-relevant because the contact mode/contact flag interpretation is directly used by the nominal OCS2 cost and constraint path, unlike the earlier terrain-height override path that only touched swing-height helper logic.

## 3. Main Experiment Question

For step-down terrains of height:

- 1 cm
- 3 cm
- 5 cm
- 7 cm
- 9 cm

measure how the pure nominal OCS2 controller responds when the contact timing interpretation is shifted by:

- 0.00 s
- 0.05 s
- 0.07 s
- 0.08 s
- 0.09 s
- 0.10 s
- 0.12 s
- 0.15 s

The final deliverable is not a single run result. It is a validated terrain-by-bias summary after repeated runs, filtering, validation, and compact handover packaging.

## 4. Code and Configuration Changes

### 4.1 Controller path used for uncertainty injection

The contact timing bias is injected into the direct contact-flag interpretation path, not into the old swing terrain-height override path.

Relevant files:

- `controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/interface/SwitchedModelReferenceManager.cpp`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/control/CtrlComponent.cpp`
- `evaluation/go2_terrain_eval/scripts/run_trial.py`

The important design change is:

- `contact_timing_bias` is passed from the experiment configuration,
- the reference manager uses the biased timing when interpreting contact flags,
- this affects the contact-mode path that nominal OCS2 actually uses.

This replaced earlier exploratory paths that were tested and discarded:

- terrain height scalar override in the swing planner,
- event-time shifting that did not meaningfully change nominal behavior.

### 4.2 Per-run controller logging

Minimal state/input logging was added so that each run writes a controller-side CSV.

Relevant files:

- `controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/FSM/StateOCS2.h`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/FSM/StateOCS2.cpp`

Each run produces `controller_state_input.csv` containing:

- observation state,
- observation input,
- optimized state,
- optimized input,
- observed and planned contact flags,
- observed and planned mode information,
- time stamps.

### 4.3 Scenario and terrain configuration

Scenario:

- `evaluation/go2_terrain_eval/configs/scenarios/ocs2_stepdown_d_sweep.yaml`

Important scenario values:

- forward command duration: 14 s
- timeout: 22 s

Terrain registry:

- `evaluation/go2_terrain_eval/configs/terrains.yaml`

Scene XML files:

- 1 cm: `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_1cm.xml`
- 3 cm: `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_3cm.xml`
- 5 cm base scene: `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_d_sweep.xml`
- 7 cm: `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_7cm.xml`
- 9 cm: `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_9cm.xml`

The 5 cm scene was tuned first and used to identify the bias grid before expanding to the other terrain heights.

## 5. Why the Bias Grid Was Chosen

The bias grid was not chosen arbitrarily.

The 5 cm terrain was used as the tuning terrain:

- smaller biases such as 0.05 s still passed stably,
- around 0.08 s to 0.10 s the controller entered a visibly degrading regime,
- larger values such as 0.12 s and 0.15 s produced failure more consistently.

Based on that progression, the following grid was fixed for all terrains:

- 0.00
- 0.05
- 0.07
- 0.08
- 0.09
- 0.10
- 0.12
- 0.15

This gave a consistent comparison grid across terrain heights while preserving more resolution near the empirically observed transition region.

## 6. Raw Experiment Execution

### 6.1 Single-run execution script

Raw runs are produced with:

- `evaluation/go2_terrain_eval/scripts/run_trial.py`

### 6.2 Batch execution script

Repeated runs were executed with:

- `evaluation/go2_terrain_eval/scripts/run_contact_timing_batch.py`

The batch runner executes terrains sequentially. It does not run multiple trials in parallel.

For each terrain height, the batch script iterates over the terrain keys associated with:

- the terrain height itself,
- the fixed contact timing bias grid,
- and the requested repeat count.

### 6.3 Repeat count

The validated study results are based on repeated runs per terrain-bias pair. The target use here was 10 repeats per pair.

### 6.4 Raw output location

Raw run folders live under:

- `evaluation/go2_terrain_eval/results/`

Each run folder contains at minimum:

- `result.json`
- `controller.log`
- `controller_state_input.csv`

The raw result directories inside `evaluation/go2_terrain_eval/results/` were used as the source of truth during filtering and validation.

## 7. Filtering Logic After Raw Runs

### 7.1 Why filtering was needed

Raw runs contain:

- valid locomotion runs,
- early startup collapses,
- short runs,
- low-progress runs that should not be treated as representative locomotion outcomes.

The validated handover package therefore does not use raw folders directly. It uses a reconstructed filtered table built from the raw run folders.

### 7.2 Filtering script

Filtering is handled by:

- `evaluation/go2_terrain_eval/scripts/filter_contact_timing_runs.py`

This script rebuilds the analysis tables from:

- `result.json`
- `controller_state_input.csv`

and does **not** rely on the overwritten aggregate CSV as a final source of truth.

### 7.3 Primary exclusion reasons

The final validated filter uses these primary exclusion reasons:

- `short_duration`
- `low_progress`
- `startup_collapse`

An earlier overcount of `corrupted_file` was traced to a bad absolute startup-time window assumption and was fixed by switching to relative controller-log windows.

### 7.4 Important interpretation rule

`filter_pass == True` does **not** mean the run was a success run.

It only means:

- the run was not excluded by the current filtering criteria.

Actual locomotion success still comes from the run result fields such as:

- `success`
- `task_completion_success`
- `time_to_failure`

## 8. Validation of the Filtered Outputs

### 8.1 Validation script

Validation is handled by:

- `evaluation/go2_terrain_eval/scripts/validate_contact_timing_results.py`

### 8.2 Validation goal

The validation step checks whether:

- the filtered CSV and excluded log are internally consistent,
- the terrain-by-bias summary counts match the underlying filtered rows,
- representative terrain-bias combinations match the original raw run folders.

### 8.3 Validation report outputs

Validation outputs:

- `/home/ho/ros2_ws/filtered_results/contact_timing_bias_validation_report.txt`
- `/home/ho/ros2_ws/filtered_results/contact_timing_bias_validation_samples.csv`

The validation report includes fixed spot-check combinations such as:

- 1 cm, 0.10 s
- 3 cm, 0.12 s
- 5 cm, 0.09 s
- 5 cm, 0.10 s
- 7 cm, 0.07 s
- 9 cm, 0.12 s

The checked result at the time of packaging was:

- filtered runs and excluded runs were consistent,
- researcher summary rows matched the underlying filtered data,
- no `late_stabilization` or `end_only_stabilized` cases were observed in the final validated tables.

## 9. Researcher-Facing Summary Tables

### 9.1 Main filtered outputs

Final filtered outputs were written outside the repository first under:

- `/home/ho/ros2_ws/filtered_results/`

Key files:

- `contact_timing_bias_runs_filtered.csv`
- `excluded_runs_log.csv`
- `contact_timing_bias_researcher_summary.csv`
- `contact_timing_bias_researcher_compact_summary.csv`
- `contact_timing_bias_researcher_compact_summary.txt`

### 9.2 Handover package tables

The final researcher-facing compact package was then built with:

- `evaluation/go2_terrain_eval/scripts/build_handover_package.py`

Key outputs:

- `main_table.csv`
- `exclusion_overview.csv`
- `handover_note.txt`

These were built from the validated filtered outputs rather than from the overwritten intermediate aggregate CSV.

## 10. Figures and Captions

### 10.1 Figure generation

Figure generation script:

- `evaluation/go2_terrain_eval/scripts/plot_contact_timing_handover_figures.py`

This script reads:

- `/home/ho/ros2_ws/filtered_results/main_table.csv`
- `/home/ho/ros2_ws/filtered_results/exclusion_overview.csv`

and writes:

- success rate line plot,
- stable rate line plot,
- fail rate line plot,
- forward progress line plot,
- roll RMS line plot,
- pitch RMS line plot,
- success/stable/fail heatmaps.

### 10.2 Caption generation

Caption generation script:

- `evaluation/go2_terrain_eval/scripts/generate_figure_captions.py`

Outputs:

- `figure_captions.txt`
- `figure_captions_short.txt`
- `figure_caption_generation_log.txt`

### 10.3 Figure location

Final figures and caption files were generated under:

- `/home/ho/ros2_ws/filtered_results/figures/`

The figure set includes both PNG and PDF files.

## 11. Files That Matter Most for Sharing

The most useful files for teammate and researcher review are:

### Code and configuration

- `controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/interface/SwitchedModelReferenceManager.cpp`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/control/CtrlComponent.cpp`
- `controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/FSM/StateOCS2.h`
- `controllers/ocs2_quadruped_controller_height_only_v1/src/FSM/StateOCS2.cpp`
- `evaluation/go2_terrain_eval/configs/scenarios/ocs2_stepdown_d_sweep.yaml`
- `evaluation/go2_terrain_eval/configs/terrains.yaml`
- `evaluation/go2_terrain_eval/scripts/run_trial.py`
- `evaluation/go2_terrain_eval/scripts/run_contact_timing_batch.py`
- `evaluation/go2_terrain_eval/scripts/filter_contact_timing_runs.py`
- `evaluation/go2_terrain_eval/scripts/validate_contact_timing_results.py`
- `evaluation/go2_terrain_eval/scripts/build_compact_researcher_table.py`
- `evaluation/go2_terrain_eval/scripts/build_handover_package.py`
- `evaluation/go2_terrain_eval/scripts/plot_contact_timing_handover_figures.py`
- `evaluation/go2_terrain_eval/scripts/generate_figure_captions.py`

### Research outputs

- `main_table.csv`
- `exclusion_overview.csv`
- `handover_note.txt`
- `contact_timing_bias_validation_report.txt`
- `contact_timing_bias_researcher_compact_summary.csv`
- final figures under `figures/`
- caption files under `figures/`

## 12. Validated High-Level Findings That Can Be Shared Safely

Only the following high-level statements should be treated as validated package-level takeaways:

- Increasing contact timing bias degraded nominal OCS2 performance.
- Higher step-down terrain generally showed stronger sensitivity to the same timing bias.
- The final handover tables and figures were rebuilt from raw runs after filtering and validation.
- The final validated package did not contain `late_stabilization` or `end_only_stabilized` cases.

Anything more specific than that should be read from the tables and figures themselves rather than paraphrased from memory.

## 13. Recommended Sharing Strategy

For GitHub sharing, the recommended branch contents are:

- the code/config/script changes required to reproduce the study,
- this document,
- the validated handover tables,
- the final figures and captions,
- and only the minimum supporting summary artifacts needed for another person to review the result without parsing all raw run folders.

The raw run directories are still available under `evaluation/go2_terrain_eval/results/`, but they are not the easiest first entry point for review.
