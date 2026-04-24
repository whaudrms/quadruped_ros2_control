# Perceptive OCS2 + QP-WBC Auto Trial

이 브랜치는 `whaudrms/quadruped_ros2_control`의 `dev` 기반 perceptive OCS2 + QP-WBC 실험을 자동으로 재현하기 위한 최소 실행 도구를 포함한다.

## 이 브랜치 받기

```bash
git clone -b run_v1 https://github.com/<YOUR_ID>/quadruped_ros2_control.git
```

이미 repo가 있으면:

```bash
git fetch origin
git checkout run_v1
```

## 포함된 것

- `tools/perceptive_dev_v2/run_trial.py`
  - MuJoCo 실행
  - controller launch
  - readiness 확인
  - stand-only
  - OCS2 진입
  - 전진 입력
  - 결과 저장
- `tools/perceptive_dev_v2/auto_input_metrics.py`
  - `/control_input`, `/cmd_vel` 자동 송신
  - `/odom` 기반 결과 계산
- `tools/perceptive_dev_v2/scenarios/standing_trot_forward.yaml`
  - 기본 입력 시나리오

## 필요한 것

- ROS 2 Jazzy
- `unitree_mujoco` simulator build
- 이 repo 빌드 결과

## 빌드 예시

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws

colcon build \
  --packages-select ocs2_quadruped_controller \
  --base-paths ~/ros2_ws/src/quadruped_ros2_control \
  --build-base /tmp/perceptive_v2_build_patch \
  --install-base /tmp/perceptive_v2_install_patch \
  --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-O0 -DCMAKE_C_FLAGS=-O0
```

## 기본 실행

### perceptive

```bash
python3 ~/ros2_ws/src/quadruped_ros2_control/tools/perceptive_dev_v2/run_trial.py \
  --terrain basic_step.xml \
  --mode perceptive_dev_v2 \
  --tag test
```

### pure

```bash
python3 ~/ros2_ws/src/quadruped_ros2_control/tools/perceptive_dev_v2/run_trial.py \
  --terrain basic_step.xml \
  --mode pure_dev_v2 \
  --tag test
```

## 자동 탐색

스크립트는 기본적으로 아래를 자동 탐색한다.

- install setup
  - `/tmp/perceptive_v2_install_patch/setup.bash`
  - `/tmp/perceptive_v2_install_o0/setup.bash`
- scene root
  - `~/unitree_mujoco_dev/unitree_robots/go2`
  - `~/unitree_mujoco/unitree_robots/go2`
- MuJoCo build dir
  - `~/unitree_mujoco/simulate/build`
  - `~/unitree_mujoco_dev/simulate/build`

즉 일반적인 환경에서는 별도 수정 없이 실행할 수 있다.

## 경로가 다를 때

자동 탐색이 안 맞는 경우에만 아래 옵션을 넘기면 된다.

```bash
python3 ~/ros2_ws/src/quadruped_ros2_control/tools/perceptive_dev_v2/run_trial.py \
  --terrain basic_step.xml \
  --mode perceptive_dev_v2 \
  --install-setup /path/to/install/setup.bash \
  --scene-root /path/to/unitree_mujoco_dev/unitree_robots/go2 \
  --mujoco-build-dir /path/to/unitree_mujoco/simulate/build
```

또는 환경변수:

```bash
export PERCEPTIVE_INSTALL_SETUP=/path/to/install/setup.bash
export UNITREE_SCENE_ROOT=/path/to/unitree_mujoco_dev/unitree_robots/go2
export UNITREE_MUJOCO_BUILD_DIR=/path/to/unitree_mujoco/simulate/build
```

## 결과 위치

결과는 아래에 저장된다.

```text
tools/perceptive_dev_v2/results/<timestamp>_<terrain>_<mode>_<tag>/
```

생성 파일:

- `result.json`
- `mujoco.log`
- `controller.log`

## 주의

실행 전에 기존 controller 노드가 살아 있으면 스크립트가 중단된다.
