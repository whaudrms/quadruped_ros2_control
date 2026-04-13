# OCS2 d-Sweep Variable Trace

## 목적

이 문서는 step-down `±d` 실험에서 저장되는 변수들이

- 어디서 측정되는지
- 어떤 코드 경로를 거쳐 만들어지는지
- 어떤 의미를 가지는지

를 연구원님께 설명할 수 있도록 정리한 문서다.

현재 기준 branch:

- `dev/ocs2-only-stepdown-d-sweep`


## 1. 큰 흐름

현재 pure OCS2 nominal 실험에서 데이터는 크게 세 층에서 나온다.

1. **하드웨어/시뮬레이터 상태 인터페이스**
   - joint position / velocity / effort
   - IMU
   - odometer ground-truth
   - foot force

2. **controller 내부 상태**
   - `observation.state`
   - `observation.input`
   - `optimized_state`
   - `optimized_input`
   - `mode`
   - `planned_mode`

3. **평가 스크립트 summary**
   - `result.json`
   - progress / failure / roll-pitch-yaw 변화


## 2. 상태변수: 어디서 어떻게 측정되는가

### 2.1 base position / base velocity

소스:

- `robot_control_height_only_v1.yaml`
- `GroundTruth.cpp`

경로:

1. controller YAML에서 `estimator_type: ground_truth` 사용
2. `odom_name: "odometer"` 및 odom interfaces를 state interface로 읽음
3. `GroundTruth::update()`에서
   - `ctrl_component_.odom_state_interface_`
   에서 position / velocity를 읽음
4. 이 값을 기반으로:
   - 내부 `rbd_state_` 업데이트
   - `odom` 메시지 publish

관련 코드:

- [robot_control_height_only_v1.yaml](/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_height_only_v1/config/robot_control_height_only_v1.yaml)
- [GroundTruth.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/estimator/GroundTruth.cpp)


### 2.2 base orientation / angular velocity

소스:

- IMU state interface

경로:

1. `StateEstimateBase::updateImu()`에서 IMU quaternion, angular velocity, linear acceleration을 읽음
2. quaternion을 ZYX Euler로 변환
3. global angular velocity로 변환
4. `rbd_state_`의 angular state를 업데이트
5. `GroundTruth::getOdomMsg()`에서 orientation / angular velocity를 `odom` 메시지로 publish

관련 코드:

- [StateEstimateBase.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/estimator/StateEstimateBase.cpp)
- [GroundTruth.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/estimator/GroundTruth.cpp)


### 2.3 joint positions / velocities / efforts

소스:

- ros2_control joint state interfaces

경로:

1. `StateEstimateBase::updateJointStates()`에서
   - joint position
   - joint velocity
   를 읽음
2. `joint_state_broadcaster`를 통해 `joint_states` topic으로 publish됨

관련 코드:

- [StateEstimateBase.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/estimator/StateEstimateBase.cpp)
- [robot_control_height_only_v1.yaml](/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_height_only_v1/config/robot_control_height_only_v1.yaml)


### 2.4 contact-related 값

#### raw foot force

소스:

- foot force state interfaces

경로:

1. YAML에 `foot_force_name: "foot_force"`와 각 발 인터페이스가 정의됨
2. controller가 이를 state interface로 읽음

관련 코드:

- [robot_control_height_only_v1.yaml](/home/ho/ros2_ws/src/quadruped_ros2_control/descriptions/unitree/go2_description_height_only_v1/config/robot_control_height_only_v1.yaml)
- [Ocs2QuadrupedController.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/Ocs2QuadrupedController.cpp)

#### thresholded contact flag

소스:

- raw foot force + threshold

경로:

1. `StateEstimateBase::updateContact()`에서
   foot force > `feet_force_threshold`
   이면 `contact_flag_[i] = true`
2. `getMode()`는 이 contact flag를 기반으로 stance/swing mode 번호를 만든다

관련 코드:

- [StateEstimateBase.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/estimator/StateEstimateBase.cpp)
- [StateEstimateBase.h](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/estimator/StateEstimateBase.h)


## 3. controller csv 변수: 어디서 어떻게 만들어지는가

현재 run 폴더에 생성되는 파일:

- `controller_state_input.csv`

관련 코드:

- [StateOCS2.h](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/include/ocs2_quadruped_controller/FSM/StateOCS2.h)
- [StateOCS2.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/FSM/StateOCS2.cpp)
- [run_trial.py](/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/scripts/run_trial.py)

### 3.1 time

- 값: `ctrl_component_->observation_.time`
- 의미: controller 내부 누적 simulation/control time
- 갱신 위치: `CtrlComponent::updateState()`

관련 코드:

- [CtrlComponent.cpp](/home/ho/ros2_ws/src/quadruped_ros2_control/controllers/ocs2_quadruped_controller_height_only_v1/src/control/CtrlComponent.cpp)


### 3.2 obs_mode

- 값: `ctrl_component_->observation_.mode`
- 의미: estimator가 현재 contact flag로부터 계산한 mode 번호
- 생성 방식:
  - foot force thresholding
  - `stanceLeg2ModeNumber(contact_flag_)`


### 3.3 planned_mode

- 값: `planned_mode`
- 의미: 현재 policy evaluation 시점에 활성화된 MPC planned mode
- 생성 위치:
  - `StateOCS2::run()`
  - `mpc_mrt_interface_->evaluatePolicy(..., planned_mode)`


### 3.4 obs_contact_* / plan_contact_*

- `obs_contact_*`
  - 값: `modeNumber2StanceLeg(observation_.mode)`
  - 의미: 현재 추정 contact mode를 발별 boolean으로 펼친 값

- `plan_contact_*`
  - 값: `modeNumber2StanceLeg(planned_mode)`
  - 의미: planned mode를 발별 boolean으로 펼친 값


### 3.5 obs_state_*

- 값: `ctrl_component_->observation_.state`
- 의미: 현재 추정된 centroidal state
- 생성 경로:
  1. estimator가 `measured_rbd_state_` 생성
  2. `CtrlComponent::updateState()`에서
     `computeCentroidalStateFromRbdModel(measured_rbd_state_)`
     호출
  3. 그 결과가 `observation_.state`

즉 `obs_state_*`는
**하드웨어/시뮬레이터 state interface에서 읽은 현재 추정 상태를 centroidal state로 변환한 값**이다.


### 3.6 obs_input_*

- 값: `ctrl_component_->observation_.input`
- 의미: controller가 현재 시스템 입력으로 들고 있는 값
- 생성 경로:
  - `StateOCS2::run()`에서 policy evaluation 결과 `optimized_input_`를
    `ctrl_component_->observation_.input = optimized_input_`
    로 복사

즉 현재 구현상 `obs_input_*`는
**직전 정책 평가에서 선택된 input**을 의미한다.


### 3.7 opt_state_*

- 값: `optimized_state_`
- 의미: 현재 시각에서 policy evaluation으로 얻은 optimized state
- 생성 위치:
  - `StateOCS2::run()`
  - `mpc_mrt_interface_->evaluatePolicy(...)`


### 3.8 opt_input_*

- 값: `optimized_input_`
- 의미: 현재 시각에서 policy evaluation으로 얻은 optimized input
- 생성 위치:
  - `StateOCS2::run()`
  - `mpc_mrt_interface_->evaluatePolicy(...)`

이 값은 연구원님이 관심 있어 하는
**solver output input trajectory의 현재 evaluation 시점 input**에 해당한다.


## 4. result.json 변수: 어떻게 계산되는가

관련 코드:

- [auto_input_metrics.py](/home/ho/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval/scripts/auto_input_metrics.py)

### 4.1 start_pose / end_pose

- source: `/odom`
- 첫 odom callback의 pose를 `start_pose`
- 마지막 odom callback의 pose를 `end_pose`


### 4.2 distance_xy

- 계산식:
  - `sqrt((end_x - start_x)^2 + (end_y - start_y)^2)`


### 4.3 path_length

- 매 odom callback 사이의
  - `sqrt(dx^2 + dy^2)`
  누적합


### 4.4 body_forward_path_length / body_lateral_path_length

- 각 timestep의 world-frame `(dx, dy)`를
  직전 yaw 기준 body frame으로 회전시켜 누적


### 4.5 body_frame_forward_progress / lateral_progress

- 시작 yaw 기준으로
  전체 `(dx_total, dy_total)`를 body frame에 투영


### 4.6 initial_window / startup_gait_window / command_active_window 관련 progress

- scenario yaml의 step timing을 기반으로
  특정 시간 구간을 나눈 뒤
  그 구간에서의 pose 차 또는 누적 path를 계산


### 4.7 roll_rms_deg / pitch_rms_deg / yaw_rms_deg

- `/odom` quaternion을 roll/pitch/yaw로 변환
- 각 angle sample에 대해 RMS 계산
- 마지막에 degree로 변환


### 4.8 yaw_change_deg

- 첫 yaw와 마지막 yaw의 차이를
  `atan2(sin(delta), cos(delta))`
  방식으로 wrap-safe하게 계산


### 4.9 time-to-failure / fall_reason

- monitoring 구간 이후,
  아래 조건 중 하나를 만족하면 failure 처리
  - `pos.z < min_base_z`
  - `start_z - pos.z > max_base_z_drop`
  - `|roll| > max_abs_roll_deg`
  - `|pitch| > max_abs_pitch_deg`


### 4.10 base_z_std / min_base_z

- odom z sample 전체의 표준편차
- 전체 run 동안 관측된 최소 z


## 5. 현재 문서 기준으로 바로 설명 가능한 것

연구원님이 물으면 지금은 아래처럼 설명할 수 있다.

### 상태변수

- base pose/velocity는 `ground-truth odometer`에서 읽었다
- joint state는 ros2_control joint state interface에서 읽었다
- IMU는 IMU state interface에서 읽었다
- contact flag는 raw foot force에 threshold를 적용해서 만들었다

### 제어변수

- `opt_input_*`는 현재 시각에서 MPC policy evaluation 결과로 얻은 optimized input이다
- `opt_state_*`는 같은 시각의 optimized state다
- `obs_input_*`는 현재 controller가 실제 입력으로 사용 중인 input buffer 값이다

### 평가 요약값

- `result.json`의 progress, drift, RMS, failure time은 모두 `/odom`을 기준으로
  `auto_input_metrics.py`에서 계산했다


## 6. 아직 문서화는 됐지만 추가 구현이 필요한 값

아래는 이번 문서엔 포함하지 않았고, 다음 단계에서 logger를 더 붙여야 한다.

- GRF
- joint torque command
- foot trajectory command
- touchdown/liftoff timing
- foot position / velocity

이 값들은 현재 baseline logger에는 안 들어가므로,
후속 단계에서 별도 csv 혹은 topic logger를 추가해야 한다.
