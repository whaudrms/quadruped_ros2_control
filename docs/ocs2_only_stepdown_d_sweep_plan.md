# OCS2-Only Step-Down `d` Sweep Plan

## 목적

현재 단계의 목표는 `perceptive nominal`이나 `robust solver`를 직접 구현하는 것이 아니다.

지금 해야 할 일은:

1. **기존 pure OCS2 nominal**만 사용한다.
2. **step-down terrain** 하나로 실험을 고정한다.
3. 실제 terrain과 OCS2가 믿는 terrain 사이에 **`±d` mismatch**를 만든다.
4. OCS2가 어느 `d` 범위까지 정상 보행 가능한지 확인한다.
5. 각 실험마다 **상태변수와 제어변수 데이터를 전부 저장**한다.

이 데이터는 이후 연구원님이 robust OCP / solver를 설계하는 입력 데이터셋으로 사용한다.


## 현재 성공 정의

지금 단계의 성공은 다음처럼 정의한다.

- step-down task를 끝까지 정상 보행으로 수행
- 낙상 없음
- 심한 lateral drift 없음
- task 구간 동안 안정적인 gait 유지

즉 지금은 box 위에 네 발을 올리는 성공 기준이 아니라,
**step-down 환경에서 nominal OCS2가 `d` mismatch에도 정상 보행을 유지하는지**가 성공 기준이다.


## 실험 개념

두 개의 지형 표현을 분리해서 생각한다.

1. **실제 MuJoCo terrain**
- 실제 step-down 높이가 존재한다.
- 실제 contact는 여기서 일어난다.

2. **OCS2 internal terrain**
- OCS2가 믿는 nominal ground이다.
- 이 값이 실제 terrain과 다르면 contact timing mismatch가 생긴다.

`d`는 이 두 세계의 차이로 정의한다.

예:
- 실제 step-down: `-3 cm`
- OCS2 internal terrain: `0 cm`

또는 반대로
- 실제 step-down: `0 cm`
- OCS2 internal terrain: `+3 cm`

이렇게 해서 `±d` mismatch를 만들고, 기존 OCS2가 어느 정도까지 견디는지 측정한다.


## 핵심 질문

이번 브랜치에서 답해야 할 질문은 다음이다.

1. 기존 OCS2 nominal은 `d = 0`에서 안정적으로 step-down을 수행하는가?
2. `|d|`가 커질수록 언제부터 실패하는가?
3. failure가 발생할 때 상태변수와 제어변수는 어떻게 변하는가?
4. 실패는 주로:
- contact timing mismatch
- body attitude 붕괴
- foot placement mismatch
- lateral drift
중 무엇으로 발생하는가?


## 실험 항목

### 1. Baseline Check

목적:
- 새 step-down 환경에서 pure OCS2 baseline이 정상 작동하는지 확인

조건:
- model: `pure_ocs2_nominal`
- terrain: `stepdown`
- `d = 0`

반복:
- 최소 `3`회

확인할 것:
- 정상 보행 여부
- time-to-failure
- body-frame forward progress
- roll/pitch RMS
- lateral drift


### 2. Positive `d` Sweep

목적:
- OCS2가 ground를 실제보다 높게/낮게 믿을 때 어느 쪽이 더 취약한지 확인

조건:
- model: `pure_ocs2_nominal`
- terrain: `stepdown`
- `d > 0`

추천 리스트:
- `d = +0.00`
- `d = +0.01`
- `d = +0.02`
- `d = +0.03`
- `d = +0.04`
- `d = +0.05`


### 3. Negative `d` Sweep

목적:
- 반대 부호 mismatch에서의 민감도 확인

조건:
- model: `pure_ocs2_nominal`
- terrain: `stepdown`
- `d < 0`

추천 리스트:
- `d = -0.01`
- `d = -0.02`
- `d = -0.03`
- `d = -0.04`
- `d = -0.05`


### 4. Repeat Runs

목적:
- 같은 `d`에서도 실패 경향이 일관적인지 확인

권장 반복 수:
- 파일럿: `N = 3`
- 본실험: `N = 10`


## `d` 리스트

실무용 추천 기본 리스트:

```text
d_list_cm = [-5, -4, -3, -2, -1, 0, +1, +2, +3, +4, +5]
d_list_m  = [-0.05, -0.04, -0.03, -0.02, -0.01, 0.00, +0.01, +0.02, +0.03, +0.04, +0.05]
```

파일럿에선 아래만 먼저 써도 된다.

```text
pilot_d_list_m = [-0.03, -0.02, -0.01, 0.00, +0.01, +0.02, +0.03]
```


## 저장할 변수 목록

이번 브랜치에서 제일 중요한 건 데이터 수집이다.

### 1. 메타데이터

- `run_id`
- `timestamp`
- `branch`
- `terrain_name`
- `scenario_name`
- `d`
- `repeat_idx`
- `seed`
- `result_dir`


### 2. 결과 요약 변수

- `success`
- `task_completion_success`
- `fall_reason`
- `time_to_failure`
- `duration_executed`
- `distance_xy`
- `path_length`
- `mean_forward_velocity`
- `body_frame_forward_progress`
- `body_frame_lateral_progress`
- `roll_rms_deg`
- `pitch_rms_deg`
- `yaw_rms_deg`
- `yaw_change_deg`
- `min_base_z`
- `base_z_std`


### 3. 상태변수 trajectory

최소한 다음을 전부 저장한다.

- base position `(x, y, z)`
- base orientation `(roll, pitch, yaw)` 또는 equivalent
- base linear velocity
- base angular velocity
- joint positions
- joint velocities
- centroidal state if available


### 4. 제어변수 trajectory

가능한 한 전부 저장한다.

- solver input trajectory
- joint torque 또는 command
- GRF
- contact force
- foot velocity command
- body attitude / pose command


### 5. contact / event 관련 변수

- contact state per leg
- touchdown time per leg
- liftoff time per leg
- contact timing mismatch if derivable
- step-down entry time
- first unstable contact time


### 6. optional but strong

- Euler angle time series
- body-frame velocity time series
- foot world position time series
- control input norm
- GRF norm per leg


## 폴더 구조

새 방향 기준으로는 아래처럼 정리한다.

```text
evaluation/go2_terrain_eval/
  configs/
    terrains.yaml
    scenarios/
      ocs2_stepdown_d_sweep.yaml
  results/
    <timestamp>_<terrain>_<mode>_<tag>/
      result.json
      controller.log
      mujoco.log
      rosbag/
      runtime_config/
  datasets/
    ocs2_stepdown_d_sweep/
      summary.csv
      runs/
        <run_id>.csv
      states/
        <run_id>_state.csv
      inputs/
        <run_id>_input.csv
      contacts/
        <run_id>_contact.csv
```


## 요약 CSV 스키마

파일:
- `evaluation/go2_terrain_eval/datasets/ocs2_stepdown_d_sweep/summary.csv`

컬럼:

```csv
run_id,timestamp,branch,terrain_name,scenario_name,d,repeat_idx,success,task_completion_success,fall_reason,time_to_failure,duration_executed,distance_xy,path_length,mean_forward_velocity,body_frame_forward_progress,body_frame_lateral_progress,roll_rms_deg,pitch_rms_deg,yaw_rms_deg,yaw_change_deg,min_base_z,base_z_std,result_dir
```


## 실험 순서

### Step 1
- 새 step-down terrain 하나 만들기
- 실제 terrain과 OCS2 nominal terrain을 분리 가능하게 만들기

### Step 2
- pure OCS2 only mode 고정
- `d = 0` baseline 확인

### Step 3
- `pilot_d_list_m`로 파일럿 sweep
- 각 run 데이터 저장

### Step 4
- 실패 경계 범위 찾기
- 그 주변 `d`를 더 촘촘히 sweep

### Step 5
- 정식 dataset 정리
- 연구원님께 전달


## 이번 브랜치의 작업 원칙

이번 브랜치에서는 아래를 하지 않는다.

- perceptive nominal 모델 완성
- robust solver 구현
- box step-up 성능 튜닝
- full perceptive pipeline 복구

이번 브랜치에서는 오직:

**pure OCS2 + step-down + `±d` mismatch + 데이터 수집**

만 한다.


## 한 줄 요약

이 브랜치의 목표는 **step-down 환경에서 기존 OCS2 nominal이 `±d` terrain mismatch에 대해 어디까지 정상 보행하는지 정량화하고, 각 실험의 상태변수/제어변수 데이터를 체계적으로 저장하는 것**이다.
