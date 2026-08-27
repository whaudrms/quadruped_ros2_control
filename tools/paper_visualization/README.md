# Perceptive processing pipeline plot

저장된 실험 하나를 공통 `x-z` 단면으로 재구성해 다음 세 단계를 하나의 논문용 Figure로 생성합니다.

1. controller에 전달되는 planar-region decomposition 및 lift-off/touchdown foothold
2. terrain 높이를 적용하기 전 same-height nominal `SplineCpg`
3. 선택된 touchdown 높이를 적용한 modified `SplineCpg`

수직 궤적은 `SwingTrajectoryPlanner.cpp`의 `CubicSpline`/`SplineCpg` 수식을 그대로 계산합니다. 그림의 수평 좌표는 `tick.csv`의 OCS2 planned state를 Pinocchio FK로 복원한 실제 발 `x(t)`를 사용하므로 임의의 수평 보간을 사용하지 않습니다.
Figure의 lift-off와 touchdown 높이는 각 foothold 위치의 perceived terrain 높이에 맞춰 표시하고 spline의 끝점으로 사용합니다.

```bash
python3 tools/paper_visualization/plot_trial_swing_trajectory.py \
  --trial-dir tools/perceptive_dev_v2/results/good_data_reproduce/<trial> \
  --output-dir tools/paper_visualization/paper_output
```

생성물:

- `paper_perceptive_pipeline.pdf`, `.png`: 세 단계 perceptive 처리 과정
- `foothold_touchdowns.csv`: 네 발의 touchdown 시간과 planned/measured 좌표
- `swing_trajectory.csv`: 선택된 swing의 전 sample 수치
- `metadata.json`: trial, terrain offset, smoothing, swing 파라미터 및 추출 방법

Figure의 SplineCpg 곡선은 내부 planner 계산값입니다. 현재 Go2 설정은 `positionErrorGain=0.0`이므로 MPC에 직접 들어가는 것은 SplineCpg의 vertical-velocity reference이며, spline의 절대 `z` 위치를 그대로 추종하는 구조는 아닙니다.

## Robust-phase explainer

`task.info`의 `sqp.dt`, `robustPhase.P`, `d`, `v_max`, `foot_frame_offset`, `enable_splice`를 읽어 nominal switch와 robust contact window를 비교하는 논문용 schematic을 생성합니다.

```bash
python3 tools/paper_visualization/plot_robust_phase_explainer.py \
  --task-info descriptions/unitree/go2_description/config/ocs2/task.info \
  --output-dir tools/paper_visualization/paper_output \
  --show-replan on
```

`--show-replan auto`는 `task.info`의 `enable_splice`를 따릅니다. `on`은 contact-event stance splice와 같은-cycle MPC replan branch를 설명용으로 표시합니다.
