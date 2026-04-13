# OCS2 Step-Down d-Sweep Dataset Map

## 목적

이 문서는 `pure nominal OCS2 + step-down + ±d mismatch` 실험에서
어떤 데이터를 이미 저장할 수 있는지, 어떤 데이터는 추가 구현이 필요한지,
그리고 구현 위치가 어디인지를 빠르게 확인하기 위한 실무 문서다.

현재 기준 branch:

- `dev/ocs2-only-stepdown-d-sweep`

현재 baseline scene:

- `/home/ho/unitree_mujoco/unitree_robots/go2/eval_stepdown_d_sweep.xml`

현재 baseline terrain catalog entry:

- `/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/configs/terrains.yaml`


## 분류 기준

- `이미 가능`
  - 현재 ROS topic / result summary / existing logs로 바로 저장 가능
- `추가 계산 가능`
  - 현재 코드에서 정보는 존재하지만, 자동 저장 파이프라인은 없음
- `추가 logger 필요`
  - controller / OCS2 / WBC 내부 값을 직접 csv나 topic으로 내보내야 함


## 1. 상태변수

| 항목 | 현재 상태 | 저장 경로 | 구현/확인 위치 |
|---|---|---|---|
| base position | 이미 가능 | `odom`, `pose` | `GroundTruth.cpp`, `StateEstimateBase.cpp` |
| base orientation | 이미 가능 | `odom`, `pose` | `GroundTruth.cpp`, `StateEstimateBase.cpp` |
| base linear velocity | 이미 가능 | `odom` | `GroundTruth.cpp` |
| base angular velocity | 이미 가능 | `odom`, `imu` | `GroundTruth.cpp`, IMU broadcaster |
| joint positions | 이미 가능 | `joint_states` | `joint_state_broadcaster` |
| joint velocities | 이미 가능 | `joint_states` | `joint_state_broadcaster` |
| joint efforts | 이미 가능 | `joint_states` | `joint_state_broadcaster` |
| IMU orientation / gyro / accel | 이미 가능 | `imu` | `imu_sensor_broadcaster` |
| raw foot force | 이미 가능 | hardware state interface 기반, 필요시 별도 logger | `robot_control_height_only_v1.yaml`, `StateEstimateBase.cpp` |
| thresholded contact flag | 추가 계산 가능 | contact flag 변화 기록 필요 | `StateEstimateBase.cpp` |
| foot position | 추가 계산 가능 | kinematics logger 필요 | end-effector kinematics / controller 내부 |
| foot velocity | 추가 계산 가능 | kinematics logger 필요 | end-effector kinematics / controller 내부 |


## 2. 제어변수

| 항목 | 현재 상태 | 저장 경로 | 구현/확인 위치 |
|---|---|---|---|
| low-level commanded input | 추가 logger 필요 | csv/logger | `StateOCS2.cpp`, `CtrlComponent.cpp` |
| joint torque command | 추가 logger 필요 | csv/logger | WBC / command interface write path |
| joint position/velocity command | 추가 logger 필요 | csv/logger | WBC / command interface write path |
| GRF (contact forces) | 추가 logger 필요 | csv/logger | OCS2 optimized input, friction/contact force extraction |
| body attitude command | 추가 logger 필요 | csv/logger | target/reference path, optimized state/input |
| foot trajectory command | 추가 logger 필요 | csv/logger | `SwingTrajectoryPlanner`, reference manager |
| solver output input trajectory | 추가 logger 필요 | csv/logger | `StateOCS2.cpp`, MRT evaluate/policy path |
| solver output state trajectory | 추가 logger 필요 | csv/logger | `StateOCS2.cpp`, MRT evaluate/policy path |
| current observation.state | 추가 logger 필요 | csv/logger | `CtrlComponent.cpp` |
| current observation.input | 추가 logger 필요 | csv/logger | `CtrlComponent.cpp` |


## 3. 추가로 유용한 값

| 항목 | 현재 상태 | 저장 경로 | 구현/확인 위치 |
|---|---|---|---|
| success / task completion | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| time-to-failure | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| roll/pitch/yaw 변화 | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| lateral drift | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| body-frame forward progress | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| command-active window progress | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| startup gait window progress | 이미 가능 | `result.json` | `auto_input_metrics.py` |
| contact timing actual event time | 추가 계산 가능 | contact flag transition logger | `StateEstimateBase.cpp` or dedicated logger |
| touchdown timing | 추가 계산 가능 | swing/contact event logger | swing planner / reference manager |
| liftoff timing | 추가 계산 가능 | swing/contact event logger | swing planner / reference manager |
| mode schedule at runtime | 추가 logger 필요 | csv/logger | `CtrlComponent.cpp`, `StateOCS2.cpp` |
| failure mode detail | 부분 가능 | `result.json` + `controller.log` | current scripts + optional classifier |


## 4. 현재 바로 저장 가능한 최소 데이터셋

현재 추가 구현 없이 바로 모을 수 있는 최소 데이터셋은 아래다.

### ROS / runtime

- `odom`
- `pose`
- `joint_states`
- `imu`
- `tf`
- `tf_static`

### run summary

- `result.json`
- `controller.log`
- `mujoco.log`

이 조합만으로도

- base pose/velocity
- joint state
- IMU
- run-level success/failure
- progress / drift / 자세 변화

는 이미 확보 가능하다.


## 5. 연구원님 전달용으로 꼭 추가해야 하는 데이터

연구원님이 robust solver 설계에 직접 쓸 데이터를 생각하면,
다음 항목은 추가 logger가 사실상 필수다.

### 우선순위 1

- `observation.state`
- `observation.input`
- `optimized_state`
- `optimized_input`
- `mode`
- `contact_flag`

이 6개는 OCS2 nominal이 `±d`에서 어떻게 깨지는지 보는 핵심이다.

### 우선순위 2

- `GRF`
- `joint torque command`
- `body attitude / base command`
- `foot trajectory command`

이건 failure mechanism을 더 정밀하게 분석할 때 중요하다.

### 우선순위 3

- `foot position`
- `foot velocity`
- `touchdown/liftoff timing`

이건 접촉 timing mismatch를 자세히 볼 때 필요하다.


## 6. 구현 우선순위

### Step 1. 기존 저장 경로 확정

- `result.json`
- `controller.log`
- `mujoco.log`
- ROS topic recording 대상 목록 확정

### Step 2. 최소 controller csv logger 추가

첫 번째 logger는 아래 컬럼만 저장하면 된다.

- `time`
- `mode`
- `observation.state`
- `observation.input`
- `optimized_state`
- `optimized_input`
- `contact_flag`

이 정도면 nominal OCS2 failure boundary 데이터셋으로 바로 쓸 수 있다.

### Step 3. 필요시 GRF / torque logger 추가

두 번째 단계에서:

- `GRF`
- `joint torque command`
- `foot trajectory command`

를 붙인다.


## 7. 추천 저장 구조

```text
evaluation/go2_terrain_eval/results/<run_id>/
  result.json
  controller.log
  mujoco.log
  rosbag/
  controller_state_input.csv
  controller_contact.csv
  optional_wbc.csv
```


## 8. 이번 단계 결론

현재 기준으로:

- 상태변수는 상당수를 이미 저장 가능
- summary metric도 이미 상당수 저장 가능
- 그러나 연구원님이 원하는 제어변수 데이터셋은
  `controller 내부 logger`를 추가해야 한다

즉 다음 구현의 핵심은:

**ROS bag + controller csv logger**

조합이다.
