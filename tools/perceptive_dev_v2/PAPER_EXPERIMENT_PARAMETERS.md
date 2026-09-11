# 논문 작성용 실험 파라미터 정리

작성일: 2026-09-09

대상 실험: `results/monte_carlo_n100_seed20260909`

## 1. 문서 범위와 확인 수준

이 문서는 논문의 실험 설정, 구현 세부사항, 재현성 설명에 필요한 항목을 정리한다.
다른 실험, 특히 step-up 실험에는 값을 그대로 적용하지 않는다.

| 표시 | 의미 |
|---|---|
| 실험 기록 | 대상 실험의 저장된 설정 또는 로그에서 확인 |
| 현재 코드/설정 | 현재 소스와 설정에서 확인. 실험 당시와 동일했는지 최종 확인 필요 |
| 모델 로딩 | 현재 MuJoCo 라이브러리로 해당 XML을 로딩해 확인. 과거 실행 중 변경 여부는 보장하지 않음 |
| 현재 환경 | 현재 작업 PC에서 확인. 실제 실험 수행 환경과 일치하는지 확인 필요 |

대상 디렉터리의 `run_config.json` 200개에서 기록된 effective task parameters는
`robustPhase.enabled`의 ON/OFF를 제외하면 동일했다.
다만 이 파일은 전체 `task.info`, launch 설정, 시뮬레이터 옵션을 모두 보관한 스냅샷은 아니다.

## 2. MPC 설정

| 항목 | 값 또는 방식 | 확인 수준 |
|---|---|---|
| 동역학 모델 | Full centroidal dynamics | 현재 코드/설정 |
| 최적화 프레임워크 | OCS2 | 현재 코드/설정 |
| 비선형 최적화 | Multiple-shooting SQP | 현재 코드/설정 |
| SQP 내부 QP 솔버 | HPIPM | 현재 코드/설정 |
| 예측 구간 | 2.0 s | 현재 코드/설정 |
| 이산화 간격 `sqp.dt` | 0.02 s | 실험 기록 |
| SQP 반복 설정 `sqpIteration` | 2 | 실험 기록 |
| MPC 목표 주파수 | 10 Hz | 실험 기록 |
| SQP 스레드 수 | 3 | 현재 코드/설정 |
| SQP 적분기 | RK2 | 현재 코드/설정 |
| MPC 초기화 | `coldStart=false` | 현재 코드/설정 |
| Feedback policy | `useFeedbackPolicy=false` | 현재 코드/설정 |
| 마찰 원뿔 계수 | 0.3 | 현재 코드/설정 |

### 수치 설정: 부록용

| 항목 | 값 |
|---|---|
| SQP `deltaTol` | 1e-4 |
| SQP `g_max`, `g_min` | 1e-2, 1e-6 |
| SQP inequality barrier `mu`, `delta` | 0.1, 5.0 |
| State-input equality projection | 활성화 |
| HPIPM mode | SPEED |
| HPIPM `iter_max` | 30 |
| HPIPM `tol_stat` | 1e-6 |
| HPIPM `tol_eq`, `tol_ineq`, `tol_comp` | 각각 1e-8 |
| HPIPM `reg_prim` | 1e-12 |
| HPIPM `warm_start` | 0 |
| Rollout 적분기 | ODE45 |
| Rollout `timeStep` | 0.015 s |
| Rollout 절대/상대 허용오차 | 1e-5 / 1e-3 |

이 표는 현재 코드/설정 기준이다. HPIPM 값은 인터페이스 기본 설정이다.
MPC의 이전 해 재사용과 HPIPM 내부 QP warm start는 서로 다른 설정이므로 혼동하지 않는다.
SQP RK2, rollout ODE45, MuJoCo 물리 적분기는 각각 다른 역할을 한다.

## 3. WBC 및 제어 주기

| 항목 | 값 또는 방식 | 확인 수준 |
|---|---|---|
| WBC 방식 | Weighted whole-body control | 현재 코드/설정 |
| WBC QP 솔버 | qpOASES | 현재 코드/설정 |
| QP 옵션 | `setToMPC()`, `enableEqualities=true` | 현재 코드/설정 |
| Working-set 재계산 한도 | `nWSR=20` | 현재 코드/설정 |
| OCS2 제어기 설정 주파수 | 500 Hz | 현재 코드/설정 |
| Controller manager 주파수 | 1000 Hz | 현재 코드/설정 |
| WBC 마찰계수 | 0.3 | 현재 코드/설정 |
| Swing PD gain | Kp=350, Kd=37 | 현재 코드/설정 |
| Task 가중치: swingLeg / baseAccel / contactForce | 100 / 1 / 0.05 | 현재 코드/설정 |
| 토크 제한: HAA / HFE / KFE | 23.7 / 23.7 / 35.5 N·m | 현재 코드/설정 |

설정 주파수와 실측 주파수는 구분해서 보고한다.
실시간 성능을 논의하려면 평균 및 95백분위 계산시간, 실측 실행 주파수,
deadline 만족률과 그 계산 정의를 함께 제시한다.
WBC 토크 제한을 MuJoCo actuator 한계와 동일하다고 가정하지 않는다.

## 4. 시뮬레이터와 로봇 모델

| 항목 | 값 또는 방식 | 확인 수준 |
|---|---|---|
| 로봇 | Unitree Go2 | 실험 기록 및 현재 모델 |
| 구동 관절 수 | 12 | 모델 로딩 |
| MuJoCo 버전 | 3.3.6 | 저장 로그 및 현재 라이브러리 |
| 물리 timestep | 0.002 s | 모델 로딩 |
| 물리 적분기 | Semi-implicit Euler | 모델 로딩 |
| 물리 제약 솔버 | Newton | 모델 로딩 |
| 물리 솔버 반복 한도 | 100 | 모델 로딩 |
| 물리 솔버 허용오차 | 1e-8 | 모델 로딩 |
| 중력 | 9.81 m/s² | 모델 로딩 |
| MuJoCo 모델 총질량 | 약 15.2064 kg | 모델 로딩 |
| 상태 추정 설정 | Ground-truth 기반 MuJoCo launch | 현재 코드/설정 |
| 지형 제공 방식 | 정적 지형 지도에 높이 오차 주입 | 실행 스크립트 및 실험 기록 |

물리 timestep의 역수인 500 Hz는 시뮬레이션 시간상의 주파수다.
Wall-clock 실행 속도 또는 real-time factor와 같다고 단정하지 않는다.
MuJoCo 모델 질량과 MPC에 사용하는 URDF 모델 질량의 일치 여부도 별도로 확인한다.

### 접촉 파라미터

- 발 접촉 geom: `friction="0.4 0.02 0.01"`, `condim=6`, `priority=1`.
- 발 접촉 구 형상 반경: 0.022 m.
- 지형 geom 로딩값: `friction="1 0.005 0.0001"`, `condim=3`.
- MuJoCo 마찰 원뿔 설정: `cone="elliptic"`, `impratio=100`.
- MPC와 WBC가 사용하는 마찰계수: 0.3.

위 값들은 geom별 설정이다. 접촉 쌍에 적용되는 유효 파라미터와 geom별 값을
구분해야 하며, 시뮬레이션 전체의 마찰계수를 단일 값으로 잘못 요약하지 않는다.
접촉 물리의 재현성이 중요하면 `solref`, `solimp`, margin 및 actuator 설정도 함께 보관한다.

실제 센서 기반 지형 인지 실험과 정적 지도 오차 주입 실험을 명확히 구분한다.
Ground-truth 상태를 사용했다면 상태 추정 오차에 대한 강건성까지 검증한 것으로 표현하지 않는다.

## 5. Robust-phase 핵심 파라미터

아래 항목은 대상 실험의 저장된 effective task parameters에서 확인했다.

| 항목 | 파라미터 | 값 |
|---|---|---|
| 높이 불확실성 반폭 | `d` | 0.05 m |
| Robust 구간 노드 수 | `P` | 10 |
| Robust 구간 길이 | `P * sqp.dt` | 0.20 s |
| 법선 방향 속도 제한 | `v_max` | 0.6 m/s |
| 경계 비용 가중치 | `w_boundary` | 200 |
| 법선 속도 비용 가중치 | `w_v` | 1.0 |
| 시작/끝 hard 경계 | `hard_boundary_start/end` | 모두 false |
| 시작/끝 slack 경계 | `slack_boundary_start/end` | 모두 true |
| 시작/끝 slack 가중치 | `slack_boundary_weight_start/end` | 각각 20 |
| Approach barrier | `approach_barrier_mu/delta` | 0.01 / 0.001 |
| 지형 법선/평면 소스 | `terrain_source` | convex_region |
| Foot-frame 보정 | `foot_frame_offset` | 0.06 m |
| Schedule splice | `enable_splice` | false |
| 상세 로그 | `verbose_log` | true |

### 해석상 주의사항

- 이번 Proposed는 robust OCP를 활성화하되 schedule splice는 비활성화한 구성이다.
- Baseline/Proposed의 기록된 robust 설정 차이는 `enabled=false/true`다.
- `v_max`는 최적화 문제에 넣는 제한 파라미터다. 실제 착지속도의 엄격한 상한 보장으로 표현하지 않는다.
- 명목 traversal 시간 `2*d/v_max`는 약 0.1667 s다. 이 조건만으로 폐루프 성공을 보장하지 않는다.
- 상세 로그 활성화 상태는 계산시간 측정 조건에 포함한다.

## 6. 보행과 지형 조건

| 항목 | 값 또는 방식 | 확인 수준 |
|---|---|---|
| 시나리오 | `standing_trot_forward_only_reproduce.yaml` | 실험 기록 |
| 지형 파일 | `basic_step_short_v2.xml` | 실험 기록 |
| Step-down 높이 | 0.20 m에서 0.10 m로 하강: 단차 0.10 m | 현재 XML |
| 첫 하강 경계 | x=0.6 m | 현재 XML |
| 보행 설정 | Standing trot | 현재 보행 설정 및 시나리오 |
| 보행 주기 | 0.6 s | 현재 보행 설정 |
| Swing 시간 | 0.25 s | 현재 보행 설정 |
| 명목 swing 높이 | 0.08 m | 실험 기록 |
| 전진 명령 | 0.3 m/s | 시나리오 |
| 전진 구간 길이 | 5 s | 시나리오 |
| 전체 시나리오 길이 | 9 s | 시나리오 |
| 입력 publish rate | 50 Hz | 시나리오 |
| 높이 오차 적용 대상 제한 | `terrain_z_offset_only_below_z=0.15` m | 실험 기록 |

마지막 조건은 모든 지형에 동일한 오차를 넣는 실험이 아니라,
지정된 높이 조건을 만족하는 지형 영역에 오차를 적용한다는 점에서 명시할 필요가 있다.

## 7. Monte Carlo 설계와 평가

| 항목 | 내용 |
|---|---|
| 오차 분포 | `terrain_z_offset ~ Uniform(-0.05, +0.05)` m |
| 표본 수 | 100개의 오차 표본 |
| 비교 방식 | 동일 오차를 사용하는 Baseline/Proposed paired 비교 |
| 전체 실행 수 | 100쌍, 총 200회 |
| Master seed | 20260909 |
| 실행 순서 | 각 쌍의 ON/OFF 선행 순서를 무작위화 |
| 설계 보관 | `samples.csv` |
| 실패 감시 시작 | 시나리오 시작 후 4 s |

Seed는 오차 표본과 ON/OFF 실행 순서를 재현한다.
현재 스크립트는 MuJoCo 내부 RNG나 OS 스케줄링까지 동일하게 만드는 기능을 제공하지 않는다.
동일 오차를 쓴다는 의미의 paired 설계이지, 두 실행의 전체 상태 궤적이 동일하다는 의미는 아니다.

### 실패와 성공 정의

감시 구간에서 다음 중 하나라도 관측되면 낙상으로 판정한다.

- Base 높이 `z < 0.03 m`.
- 최초 odometry 높이 대비 감소량이 `0.08 m` 초과.
- 절대 roll 또는 pitch가 `60 deg` 초과.

현재 성공 판정은 낙상 미검출 기준이다.
목표 거리 도달이나 계단 통과 완료를 별도로 요구하는 성공률로 표현하지 않는다.
센서 누락, 프로세스 오류, trial 중단을 평가에서 어떻게 처리했는지도 논문에 명시한다.

### 지표와 통계에서 반드시 정의할 항목

- RMSE 대상 신호, 좌표계, 단위, 계산 구간, 시간 정렬 기준.
- 실패 trial 포함 여부와 조기 종료 이후의 결측값 처리 방식.
- 착지 이벤트 검출 조건과 법선 방향 착지속도 계산법.
- 성공률의 분모: 전체 trial 수 또는 유효 trial 수.
- Paired 개선량의 부호 정의와 신뢰구간 계산법.
- 높이 오차 bin 경계, bin별 표본 수, 평균 계산 방식.
- 시간 곡선의 음영이 표준편차인지, 평균의 신뢰구간인지 구분.

현재 Fig1b는 요청에 따라 CI 음영을 제거한 상태다.
Fig1b에 CI가 표시되어 있다고 본문이나 캡션에 쓰지 않는다.
성공률용 Wilson 95% CI 계산 코드는 존재하지만 표시 여부는 figure별로 확인한다.

## 8. 비용함수와 재현성 정보

### 부록 또는 공개 설정에 포함할 내용

- 상태 비용 `Q`: 대각값, scaling, 상태 변수 순서와 단위.
- 입력 비용 `R`: 대각값, scaling, 입력 변수 및 Jacobian 기반 변환 정의.
- 마찰, 자기충돌, 발 배치/충돌 제약과 관련 penalty 설정.
- WBC 비용 가중치와 실제 목적함수 구성 방식.
- 토크, 관절 범위, 접촉 및 actuator 파라미터.
- 코드 commit, 로컬 수정 사항, 빌드 모드와 주요 라이브러리 버전.
- 실험 당시 `task.info`, gait, launch 및 로봇/지형 XML의 스냅샷.

가중치 숫자만 기재하지 말고 실제 비용식과 함께 제시한다.
가중치가 residual에 곱해지는지 비용에 직접 곱해지는지에 따라 해석이 달라진다.

### 계산 환경

현재 확인한 PC는 Intel Core i7-12700H, Ubuntu 22.04.5 LTS다.
이 정보는 과거 실험 환경을 자동으로 증명하지 않는다.

최종 기재 전 확인할 항목:

- [ ] 실제 실험 PC의 CPU, RAM 및 사용 코어 수
- [ ] OS, ROS 2 배포판, 커널 및 real-time 설정
- [ ] OCS2, HPIPM, qpOASES, MuJoCo 버전/commit
- [ ] Release/Debug 빌드와 최적화 옵션
- [ ] 시뮬레이터와 제어기의 동시 실행 및 GUI 사용 여부
- [ ] 로그 활성화, CPU affinity/priority, 전원 모드
- [ ] 계산시간 측정 구간과 wall-clock/시뮬레이션 시간 구분
- [ ] 실험 당시 설정과 현재 설정의 일치 여부

## 9. 권장 논문 배치

| 위치 | 포함할 내용 |
|---|---|
| Implementation details | OCS2/SQP/HPIPM, WBC/qpOASES, horizon, dt, 반복 수, 제어 주파수 |
| Simulation setup | 로봇, MuJoCo 버전, timestep, 물리 솔버, 상태/지형 정보 제공 방식 |
| Proposed method parameters | d, P, robust 구간 길이, v_max, penalty/slack 및 splice 설정 |
| Experimental protocol | 보행, 지형, 오차 분포, paired 설계, seed, trial 수, 성공/실패 정의 |
| Results 및 figure captions | 통계량, CI/SD, binning, cohort와 시간 정렬 기준 |
| Appendix / supplementary | 전체 Q/R, 수치 설정, 모델/접촉 파라미터, 환경 및 코드 스냅샷 |

## 10. 확인에 사용한 파일

아래 경로는 이 문서가 위치한 `tools/perceptive_dev_v2` 기준이다.

- [실험 결과 디렉터리](results/monte_carlo_n100_seed20260909/): 각 trial의 `run_config.json`, `result.json`, `mujoco.log`.
- [Monte Carlo 설계 및 실행](exp_monte_carlo.sh).
- [Trial 실행과 override](run_trial.py).
- [시나리오](scenarios/standing_trot_forward_only_reproduce.yaml).
- [낙상 판정](auto_input_metrics.py).
- [MPC/WBC 및 robust 설정](../../descriptions/unitree/go2_description/config/ocs2/task.info).
- [제어기 주파수](../../descriptions/unitree/go2_description/config/robot_control.yaml).
- [보행 스케줄](../../descriptions/unitree/go2_description/config/ocs2/gait.info).
- [WBC 구현](../../controllers/ocs2_quadruped_controller/src/wbc/WeightedWbc.cpp).
- [MuJoCo launch](../../controllers/ocs2_quadruped_controller/launch/mujoco.launch.py).
- [HPIPM 기본 설정](../../../ocs2_ros2/mpc/ocs2_sqp/hpipm_colcon/include/hpipm_colcon/HpipmInterfaceSettings.h).
- [MuJoCo 로봇 XML](../../../unitree_mujoco/unitree_robots/go2/go2.xml).
- [MuJoCo 지형 XML](../../../unitree_mujoco/unitree_robots/go2/basic_step_short_v2.xml).
- [Monte Carlo 집계 및 figure 생성](monte_carlo_dashboard.py).
