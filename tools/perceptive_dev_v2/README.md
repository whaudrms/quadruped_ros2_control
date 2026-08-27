# Robust phase experiment runner

이 디렉터리의 러너는 현재 워크스페이스 위치를 `run_trial.py` 기준으로 자동 탐지한다.
별도로 `/home/...` 경로를 수정하거나 ROS 환경을 먼저 source할 필요가 없다.

## 1. 실행 전 확인

```bash
cd /home/tony/GO2_ws/quadruped_ros2_control

python3 tools/perceptive_dev_v2/run_trial.py \
  --scenario standing_trot_forward \
  --terrain basic_step_short \
  --robust on \
  --mpc-frequency 10 \
  --terrain-z-offset -0.02 \
  --terrain-z-offset-only-below-z 0.15 \
  --tag robust_on_check \
  --dry-run
```

`--dry-run`은 파일 경로, MuJoCo 명령, ROS launch 명령 및 임시 파라미터를
출력하지만 시뮬레이터를 실행하거나 설정 파일을 변경하지 않는다.

최초 perceptive 실행에서는 누락된 CppAD 모델을 컴파일하므로 컨트롤러가
활성화될 때까지 수십 초 걸릴 수 있다. 러너는 최대 180초 기다리며 진행 상태를
10초마다 출력한다. 이때 중간에 Ctrl+C를 누르면 컴파일이 중단된다.

## 2. 단일 실험

Robust ON:

```bash
python3 tools/perceptive_dev_v2/run_trial.py \
  --scenario standing_trot_forward \
  --terrain basic_step_short \
  --mode perceptive_dev_v2 \
  --robust on \
  --robust-p 10 \
  --robust-d 0.03 \
  --robust-v-max 0.6 \
  --robust-splice on \
  --robust-verbose on \
  --mpc-frequency 10 \
  --terrain-z-offset -0.02 \
  --terrain-z-offset-only-below-z 0.15 \
  --tag robust_on
```

동일 조건의 Robust OFF baseline:

```bash
python3 tools/perceptive_dev_v2/run_trial.py \
  --scenario standing_trot_forward \
  --terrain basic_step_short \
  --mode perceptive_dev_v2 \
  --robust off \
  --mpc-frequency 10 \
  --terrain-z-offset -0.02 \
  --terrain-z-offset-only-below-z 0.15 \
  --tag robust_off
```

기존 ROS 또는 MuJoCo 프로세스가 발견되면 러너는 다른 실험을 종료하지 않고
실패한다. 기존 프로세스까지 강제로 정리하려는 경우에만 `--force-cleanup`을
명시한다.

## 3. A/B 실험

임계 높이 오차 `-0.02 m`에서 ON/OFF 2회 비교:

```bash
bash tools/perceptive_dev_v2/m2_critical_band_inside.sh
```

결과: `tools/perceptive_dev_v2/results/critical_band_inside/`

Robust ON/OFF와 지형 인식 오차 `0`, `+0.05`, `-0.05 m`를 조합한 6회 실험:

```bash
bash tools/perceptive_dev_v2/m2_robust_ab_sweep.sh
```

결과: `tools/perceptive_dev_v2/results/robust_ab_sweep/`

`good_data/`의 v2 `n=3` ablation 조건(`Δz=±0.03 m`, `d=0.05 m`,
Robust ON/no-splice vs OFF)을 각 3회씩 재현하는 12회 실험:

```bash
bash tools/perceptive_dev_v2/m2_good_data_reproduce.sh
```

재현 실험은 `basic_step_short_v2`, `standing_trot_forward_only_reproduce`,
MPC `10 Hz`, `P=10`, `d=0.05`, `v_max=0.6`, splice OFF를 사용한다.
원본 `standing_trot_forward_only`보다 FixedStand 후 대기를 1초 늘려
`hold_after_stand=1.5 s`, 전체 시간 9초, 전진 시작 4초로 설정한다.
결과는 `tools/perceptive_dev_v2/results/good_data_reproduce/`에
trial별 디렉터리로 저장된다.

Robust ON만 사용하여 terrain perception error `-5, -3, +3, +5 cm`를
각 5회씩 실행하는 response sweep:

```bash
# 명령과 파라미터만 검증(시뮬레이터 실행 안 함)
DRY_RUN=1 N_RUNS=1 \
  bash tools/perceptive_dev_v2/m2_robust_terrain_error_sweep.sh

# 본 실험: 4 offsets × 5 runs = 20 trials
bash tools/perceptive_dev_v2/m2_robust_terrain_error_sweep.sh
```

반복 횟수는 `N_RUNS`, 결과 경로는 `RESULTS_DIR`로 바꿀 수 있다. 기본 결과는
`tools/perceptive_dev_v2/results/robust_terrain_error_sweep_n5/`에 저장된다.
홀수 run은 음수에서 양수, 짝수 run은 양수에서 음수 순으로 실행하며, 같은 tag의
완료 trial이 있으면 건너뛰므로 중단 후 같은 명령으로 이어서 실행할 수 있다.
고정 조건은 `basic_step_short_v2`, `standing_trot_forward_only_reproduce`,
MPC 10 Hz, `P=10`, `d=0.05`, `v_max=0.6`, splice OFF이다.

각 trial은 독립적으로 `task.info`를 임시 변경하며 정상 종료, 오류 또는
Ctrl+C 시 원래 내용을 복구한다. 적용된 설정은 trial 결과 폴더의
`task.info.effective`에 보존된다.

## 4. 주요 파라미터

| 옵션 | 적용 대상 | 의미 |
|---|---|---|
| `--mode perceptive_dev_v2` | launch | perceptive MPC 및 terrain publisher 활성화 |
| `--robust on/off` | `robustPhase.enabled` | Robust OCP ON/OFF |
| `--robust-p N` | `robustPhase.P` | robust window 노드 수, `T_robust=P*sqp.dt` |
| `--robust-d M` | `robustPhase.d` | 지형 높이 불확실성 half-width `[m]` |
| `--robust-v-max V` | `robustPhase.v_max` | 발의 최대 하강속도 `[m/s]` |
| `--robust-hard-boundary on/off` | `robustPhase.hard_boundary` | boundary cost에 hard endpoint inequality를 추가할지 선택 |
| `--robust-splice on/off` | `robustPhase.enable_splice` | 조기 접촉 시 mode schedule splice 사용 여부 |
| `--robust-verbose on/off` | `robustPhase.verbose_log` | `[robust_phase]` 로그 출력 여부 |
| `--mpc-frequency HZ` | `mpcDesiredFrequency` | MPC 문제를 다시 푸는 주파수; `sqp.dt`는 변경하지 않음 |
| `--terrain-z-offset M` | terrain publisher | 물리 지형은 그대로 두고 인식 지형 높이에만 오차 추가 |
| `--terrain-z-offset-only-below-z M` | terrain publisher | 해당 실제 높이보다 낮은 non-floor 표면에만 오차 적용 |
| `--scenario NAME` | YAML | 명령 순서, 속도, 실험 시간 및 낙상 판정값 선택 |
| `--results-dir PATH` | runner | 결과 저장 루트 변경 |
| `--post-trial-hold-sec S` | runner | 측정 종료 후 simulator 유지 시간; `-1`은 Ctrl+C까지 유지 |

현재 `task.info` 기본값은 `mpcDesiredFrequency=10 Hz`, `sqp.dt=0.02 s`,
`P=10`, `d=0.05 m`, `v_max=0.6 m/s`, `enable_splice=false`이다.
따라서 기본 robust window는
`10 × 0.02 = 0.20 s`이다. `task.info`에 없는 weight나 barrier 값을 바꾸려면
원본 설정 파일을 수정한다. 러너 CLI로 지정한 값은 해당 trial 동안에만 적용된다.

## 5. 결과 위치

기본 결과 루트:

```text
tools/perceptive_dev_v2/results/
```

trial별 디렉터리 이름:

```text
YYYYMMDD_HHMMSS_<terrain>_<mode>_<tag>/
```

각 trial 디렉터리에는 다음 파일이 저장된다.

| 파일 | 내용 |
|---|---|
| `run_config.json` | 경로, 실험 상태, launch 설정, 실제 적용 파라미터와 임시 override |
| `scenario.yaml` | 해당 trial에서 사용한 입력/낙상 판정 시나리오 복사본 |
| `task.info.original` | 실행 전 설정 |
| `task.info.effective` | 실제 controller 시작 시 사용한 설정 |
| `mujoco.log` | MuJoCo stdout/stderr |
| `controller.log` | ROS launch/controller/MPC 로그 및 robust window 로그 |
| `ros_logs/` | ROS 2가 생성하는 launch/node 로그 |
| `tick.csv` | 매 control tick의 MPC 최적 상태·입력과 측정 상태 |
| `result.json` | 성공 여부, 낙상 원인, 이동량, 자세 RMS, 최소 base 높이와 실제 실험 파라미터 |
| `robust_phase_foot_z.png` | 네 발의 MPC/측정 높이와 robust window 경계 |
| `tracking_rmse.png` | base 위치·자세·관절 위치의 시간별 rolling RMSE |
| `tracking_rmse.json` | 전체 및 축/관절별 RMSE 수치 |

단일 `run_trial.py`를 기본값으로 실행한 trial의 주요 지표는
다음 파일에 누적된다.

```text
tools/perceptive_dev_v2/results/summary.csv
```

배치 실험은 각 실험 폴더에 별도 `summary.csv`를 생성한다.

```text
tools/perceptive_dev_v2/results/critical_band_inside/summary.csv
tools/perceptive_dev_v2/results/robust_ab_sweep/summary.csv
tools/perceptive_dev_v2/results/good_data_reproduce/summary.csv
tools/perceptive_dev_v2/results/robust_terrain_error_sweep_n5/summary.csv
```

배치 스크립트는 모든 trial 시도 후 `plot_all_results.py`를 자동 실행하여
각 실험 폴더에 다음 갤러리를 생성한다.

```text
tools/perceptive_dev_v2/results/<experiment>/all_visualizations/index.html
```

갤러리에는 전체 성공률/RMSE dashboard, 각 trial의 RMSE와 foot-z 그래프,
모든 terrain offset의 Robust ON/OFF WBC 추종 오차가 자동으로 포함된다.
WBC 집계는 다음 세 버전을 항상 함께 생성한다.

- `all_trials`: 실패를 실패 시점까지 포함한 모든 trial 평균
- `successful_only`: 완주한 trial만 평균
- `max_contrast_successful_pair`: 각 terrain offset의 성공 trial 중 Robust ON의
  WBC 복합 추종 오차가 가장 작은 trial과 Robust OFF의 오차가 가장 큰 trial 비교

세 번째 버전은 차이가 가장 크게 보이는 사례를 의도적으로 선택한 탐색용 그래프이며,
평균 성능이나 통계적 우월성의 근거로 사용하면 안 된다. 선택된 trial, 각 오차값과
정규화 복합 점수는 `wbc_max_contrast_selection.json`에 기록된다.
각 시계열 plot에는 저장된 `tick.csv`에서 검출한 첫 낮은 지형 착지 시점과
단차 통과 완료 시점이 표시된다. 두 평균 WBC plot은 조건별 평균 시점과
표준편차를 사용하고, 최대 대비 plot은 선택된 각 trial의 시점을 사용한다.
수치 판정 결과는 trial별 `terrain_descent_events.json`에 저장된다.
WBC deviation의 좌우 Robust ON/OFF panel은 position끼리, orientation끼리
각각 더 큰 절댓값을 기준으로 동일한 대칭 y축을 사용한다. 평균 plot에서는
평균±표준편차 영역까지 공통 축 범위에 포함된다.

`all_trials_dashboard_comparison.csv`는 각 terrain error별 `Baseline`과
`Proposed` 행, 전체 trial의 `Overall Baseline`, `Overall Proposed`,
`Overall Improvement (%)` 행으로 dashboard 지표의 평균을 기록한다.
Improvement는 성공률과 전진 거리는 `(Proposed-Baseline)/|Baseline|`, 나머지
RMS·오차 지표는 `(Baseline-Proposed)/|Baseline|`에 100을 곱하므로 양수가
개선을 뜻한다.

Robust-ON terrain-error sweep은 A/B 및 max-contrast plot 대신 다음 전용 결과를
자동 생성한다. `all_trials`가 주 분석이고 `successful_only`는 생존자 편향을
확인하기 위한 보조 분석이다.

```text
terrain_error_summary_all_trials.csv
terrain_error_summary_successful_only.csv
terrain_error_trials.csv
terrain_error_response_dashboard.png
terrain_error_success_tolerance.png
terrain_error_descent_timing.png
terrain_error_wbc_heatmap_all_trials.png
terrain_error_wbc_heatmap_successful_only.png
terrain_error_wbc_deviation_all_trials.png
terrain_error_wbc_deviation_successful_only.png
```

WBC deviation plot의 position 행과 orientation 행은 4개 offset 전체에서 각각
같은 대칭 y축을 사용한다.

```text
all_trials_dashboard.png
all_trials_dashboard_comparison.csv
wbc_tracking_error_all_trials.png
wbc_tracking_error_successful_only.png
wbc_tracking_error_max_contrast_successful_pair.png
wbc_deviation_components_all_trials_v2_dz<offset>.png
wbc_deviation_components_successful_only_v2_dz<offset>.png
wbc_deviation_components_max_contrast_successful_pair_v2_dz<offset>.png
wbc_max_contrast_selection.json
```

저장된 실험을 수동으로 전부 다시 그리려면 다음 명령을 사용한다.

```bash
python3 tools/perceptive_dev_v2/plot_all_results.py \
  --results-dir tools/perceptive_dev_v2/results/<experiment>
```

`summary.csv`에는 trial/tag, Robust ON/OFF, `P`, `d`, `v_max`, splice,
MPC 주파수, 지형 인식 오차와 주요 성능 지표가 한 행에 함께 기록된다.

Robust window와 발 높이 궤적을 그리려면:

```bash
python3 tools/perceptive_dev_v2/plot_robust_phase.py \
  tools/perceptive_dev_v2/results/<experiment>/<trial-directory>
```

RMSE 그래프를 다시 생성하거나 시간 구간을 지정하려면:

```bash
python3 tools/perceptive_dev_v2/plot_trial_rmse.py \
  tools/perceptive_dev_v2/results/<experiment>/<trial-directory> \
  --window-sec 0.25 --start-sec 0 --end-sec 16
```

기본 RMSE 범위는 scenario의 `monitoring_start_sec`부터 측정 종료까지이며,
`--post-trial-hold-sec` 시간까지 포함하려면 `--scope all`을 사용한다. 정상
완료된 새 trial에서는 두 PNG와 RMSE JSON을 자동 생성한다.
