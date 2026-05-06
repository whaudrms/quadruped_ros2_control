# Perceptive 모드 실행 / 사용

## 1. 전제

- 빌드된 `ocs2_quadruped_controller` (dev 브랜치). `PerceptiveLeggedInterface` 와 `planar_terrain_publisher` executable이 같이 빌드되어 있어야 함.
- MuJoCo 시뮬레이션: scene XML을 고를 수 있어야 함. 기본 예시 scene: `basic_step.xml`.
- 런치파일의 `scene_root_dir` 경로가 로컬 환경과 일치하는지 확인:
  - `controllers/ocs2_quadruped_controller/launch/mujoco.launch.py:14`
    ```python
    scene_root_dir = os.path.expanduser("~/GO2_ws/unitree_mujoco/unitree_robots/go2")
    ```
    (원본 dev 브랜치는 `/home/tony/unitree_mujoco/...` 하드코드였음 — 포팅 시 `~/` expanduser로 교체됨.)

## 2. 런치 파라미터 (dev 브랜치 실제 기본값)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `pkg_description` | `go2_description` | 로봇 URDF 패키지 |
| `enable_perceptive` | `false` | 지형 인식 모드 전체 on/off (이것만 켜도 아래 세 constraint는 default true라 자동 활성) |
| `enable_perceptive_reference_modification` | `true` | base pitch/z를 지형에 맞춤 |
| `enable_perceptive_foot_placement_constraint` | `true` | foot placement soft constraint |
| `enable_perceptive_foot_collision_constraint` | `true` | foot collision soft constraint (swing foot vs SDF) |
| `enable_perceptive_body_collision_constraint` | `false` | body sphere SDF constraint (calf 4 spheres) — 유일하게 기본 off |
| `perceptive_foot_placement_boundary_margin` | `0.05` | polygon 제약 수축 margin [m] |
| `publish_static_terrain` | `true` | `planar_terrain_publisher` 같이 띄울지 |
| `terrain_scene_file` | `basic_step` | scene.xml 이름 (경로는 `scene_root_dir` 기준, `.xml` 자동 부착) |
| `terrain_smoothing_radius` | `0.06` | smooth_planar layer Gaussian 반경 [m] |

> 주의: 세 개의 perceptive constraint (`reference_modification`, `foot_placement_constraint`, `foot_collision_constraint`) 기본값이 `true`라서, **`enable_perceptive:=false`만 설정해도 constraint 코드 경로는 실행되지 않음** (interface 자체가 baseline `LeggedInterface`이므로). 반면 `enable_perceptive:=true`로 올리는 순간 나머지 세 flag가 자동으로 켜져 있는 셈이 됨. Ablation 실험 시 개별 flag 명시적으로 false 지정 필요.

## 3. 실행 시퀀스

### Terminal 1 — MuJoCo

```bash
cd ~/GO2_ws/unitree_mujoco/simulate/build
./unitree_mujoco -r go2 -s basic_step.xml   # 또는 로컬에 있는 scene_terrain.xml 등
```

### Terminal 2 — controller

Pure baseline (perceptive 완전히 off, publisher도 끄기):
```bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py \
    pkg_description:=go2_description \
    enable_perceptive:=false \
    publish_static_terrain:=false
```

Perceptive on:
```bash
ros2 launch ocs2_quadruped_controller mujoco.launch.py \
    pkg_description:=go2_description \
    enable_perceptive:=true \
    enable_perceptive_reference_modification:=true \
    enable_perceptive_foot_placement_constraint:=true \
    enable_perceptive_foot_collision_constraint:=true \
    enable_perceptive_body_collision_constraint:=false \
    perceptive_foot_placement_boundary_margin:=0.05 \
    publish_static_terrain:=true \
    terrain_scene_file:=basic_step
```

**첫 실행은 두 단계의 CppAD 컴파일을 거친다**:
1. Baseline 최초 실행 시: per-foot 12개 lib (`FL_foot_position_lib.so`, `FL_foot_velocity_lib.so`, ... `RR_foot_orientation_lib.so`) — 약 40초
2. Perceptive 최초 실행 시 추가: collective 3개 lib (`ALL_FOOT_position_lib.so`, `ALL_FOOT_velocity_lib.so`, `ALL_FOOT_orientation_lib.so`) — 약 26초

두 단계 모두 `controller_manager`의 spawner가 10초 × 3회 타임아웃을 초과해서 "Failed loading controller"로 실패하지만, 그 사이 MPC thread는 끝까지 초기화되어 캐시(`~/ocs2_cpp_ad/go2/*/cppad_generated/*_lib.so`)가 만들어진다. **Ctrl+C 후 두 번째 실행**부터는 캐시 로드라 즉시 성공.

### Terminal 3 — 명령

**기립 (stand)**
```bash
ros2 topic pub --once /control_input control_input_msgs/msg/Inputs \
    "{command: 2, lx: 0.0, ly: 0.0, rx: 0.0, ry: 0.0}"
```

**OCS2 gait mode (flying trot 등)**
```bash
ros2 topic pub --once /control_input control_input_msgs/msg/Inputs \
    "{command: 4, lx: 0.0, ly: 0.0, rx: 0.0, ry: 0.0}"
```

**전진 속도 명령**
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

키보드로 줄 수도 있음:
```bash
ros2 run keyboard_input keyboard_input
```

## 4. terrain scene 생성

`~/GO2_ws/unitree_mujoco/terrain_tool/terrain_generator.py` — Python XML ElementTree로 scene.xml을 만들고, MuJoCo geom type의 box/plane/hfield 를 추가하는 유틸.

주요 함수:
- `AddBox(pos, euler, size)` — 기본 박스
- `AddStairs(init_pos, yaw, width, height, length, stair_nums)` — step-up 계단
- `AddSuspendStairs(...)` — 띄엄띄엄 떠 있는 stair (stepping stones)
- `AddRoughGround(init_pos, nums, box_size, box_euler, separation, ..._rand)` — 랜덤 균일 배치된 울퉁불퉁 지형
- `AddPerlinHeighField(...)` — perlin noise heightfield PNG 생성
- `AddHeighFieldFromImage(...)` — 이미지 → heightfield

파일 말미의 `__main__`에서 원하는 조합을 호출하고 `Save()` → `scene_terrain.xml` 생성.

주의: `planar_terrain_publisher`는 `<geom type="box"/>` 와 `<geom type="plane"/>` 만 파싱한다. hfield / rough ground는 인식하지 못하므로 terrain 구성 시 box 위주로 설계해야 perceptive stack에 제대로 전달된다.

## 5. 로그 / 시각화

런치 파라미터로 CSV 로그 경로 지정 가능:
```
dataset_log_csv_path        — controller state / input snapshots
wbc_log_csv_path            — WBC command (τ, q_des, dq_des)
perceptive_debug_csv_path   — perceptive 디버그 정보
```

생성 위치 예:
```
/tmp/perceptive_logs/controller_state_input.csv
/tmp/perceptive_logs/wbc_command.csv
/tmp/perceptive_logs/perceptive_debug.csv
```

RViz에서 구독하면 유용한 토픽:
- `/perceptive_reference/raw_base_path` — MPC가 처음 받은 base target
- `/perceptive_reference/terrain_aware_base_path` — terrain pitch/z 수정된 base target
- `FootPlacementVisualization`이 발행하는 MarkerArray (convex polygon, nominal foothold)
- `SphereVisualization`이 발행하는 body sphere
- `/convex_plane_decomposition_ros/planar_terrain` — publisher → receiver 중간의 terrain 메시지

## 6. 빌드

```bash
nice -n 19 colcon build \
  --packages-select ocs2_quadruped_controller \
  --symlink-install \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
```

`parallel-workers 1` + `-DCMAKE_BUILD_PARALLEL_LEVEL=1` 로 두는 이유: OCS2 의존성이 큰 C++ 컴파일 + CppAD 라이브러리 생성이 병렬로 돌면 메모리 터지기 쉬움 (실제로 포팅 중 16GB swap 소진 + 편집기 OOM 킬 관찰됨). `nice -n 19`로 우선순위 최하위 고정하면 편집기/브라우저는 CPU를 계속 잡을 수 있다.

## 7. 디버깅 체크리스트

- `publish_static_terrain:=true` 인데 perceptive constraint가 안 먹힌다?
  → `ros2 topic echo /convex_plane_decomposition_ros/planar_terrain --qos-reliability reliable --qos-durability transient_local` 로 메시지 수신 여부 확인.
- foot placement active=[0,0,0,0] 뜸?
  → `getFootPlacementFlags`가 `initStandFinalTime` 이후에만 켜지므로, 기립 직후 초기 stance 시점에는 원래 꺼져 있음. 걷기 시작하면 `true`로 바뀌어야 정상.
- MPC가 발을 틈새에 꽂으려 함?
  → `perceptive_foot_placement_boundary_margin` 를 0.02~0.03 으로 올려 convex region을 보수적으로 수축.
- 계단 내려가면서 COM이 순간적으로 훅 떨어짐?
  → `PerceptiveLeggedReferenceManager::modifyReferences`의 `downStepCommitDistance=0.08 m`, `downStepHeightThreshold=0.03 m` 상수가 step-down을 천천히 따라가게 함. 더 보수적으로 가려면 이 상수를 올리고, raw pitch/z 유지 구간을 넓히면 됨.
- `linear_kalman` estimator 사용 시 z가 계속 떠다닌다?
  → perceptive 모드에서는 `alignPerceptiveBaseHeightToTerrain`이 매 tick `z = smooth_planar(x,y) + comHeight`로 덮어써서 교정된다. 이 함수가 안 도는 경우 (`estimator_type != "linear_kalman"` or `enable_perceptive=false`) drift 그대로 보임.

## 8. Humble로 포팅 시 주의 (Jazzy↔Humble API gap)

dev 브랜치는 Jazzy 기반으로 작성되어 있어 Humble 환경에 이식하면 다음 지점에서 빌드/런타임 실패한다. 모두 **가드/교체**만 하면 의미는 그대로.

| 증상 | 원인 | 수정 |
|---|---|---|
| `fatal error: realtime_tools/realtime_buffer.hpp: No such file or directory` | `realtime_tools` 가 `CMakeLists.txt`의 `CONTROLLER_INCLUDE_DEPENDS` 에 없음 | `realtime_tools`, `rclcpp`, `rclcpp_lifecycle` 를 deps에 추가 |
| `error: 'rclcpp' has not been declared` (GaitManager.cpp) | Jazzy는 transitive로 끌려왔지만 Humble은 명시 필요 | `#include <rclcpp/rclcpp.hpp>` 추가 |
| `error: 'rclcpp_lifecycle' has not been declared` (TargetManager.h) | 동상 | `#include <rclcpp_lifecycle/lifecycle_node.hpp>` 추가 |
| `error: 'class hardware_interface::LoanedStateInterface' has no member named 'get_optional'` (GroundTruth.cpp, StateEstimateBase.cpp 등) | Jazzy는 `get_optional() → std::optional<double>`, Humble은 `get_value() → double` | `.get_optional().value()` → `.get_value()` 일괄 치환 (19 sites) |
| `error: no match for 'operator=' (operand types are 'const std::_Swallow_assign' and 'void')` (StateOCS2.cpp) | Jazzy의 `set_value()` 는 반환형이 있어 `std::ignore = ...` 가능, Humble은 `void` 반환 | `std::ignore = ` prefix 제거 (5 sites) |
| 런타임 `terminate called after throwing an instance of 'rclcpp::exceptions::ParameterAlreadyDeclaredException'` | `controller_manager`가 launch `--params-file` 로 파라미터를 먼저 주입 → C++에서 `declare_parameter` 재선언 throw (Jazzy는 silent override) | 각 `declare_parameter` 앞에 `if (!node_->has_parameter("name"))` 가드 추가 |

`declare_parameter` 가드가 필요한 14 sites:
- `CtrlComponent.cpp` — 8개 (`robot_pkg`, `feet`, `enable_perceptive*` 6개)
- `StateEstimateBase.cpp` — 1개 (`feet_force_threshold`)
- `StateOCS2.cpp` — 2개 (`default_kp`, `default_kd`)
- `TargetManager.cpp` — 3개 (`perceptive_down_step_*`)
