# GO2 Terrain Eval

기존 `quadruped_ros2_control` launch/controller 파일을 건드리지 않고,
MuJoCo terrain scene을 자동 생성하고 자동 입력/지표 수집을 수행하는
별도 평가 프레임워크입니다.

## 목표

- `terrain_generator.py`가 만들 수 있는 terrain을 개별 scene으로 생성
- 동일한 시나리오를 자동 반복 실행
- 사람이 키보드를 누르지 않아도 gait 전환/직진 명령 수행
- trial 결과를 JSON/CSV로 저장
- 기존 실험 파일과 겹치지 않게 별도 디렉터리에서만 작업

## 구조

- `configs/terrains.yaml`
  - terrain 카탈로그
- `configs/scenarios/*.yaml`
  - 자동 입력 시나리오
- `scripts/generate_terrains.py`
  - terrain 카탈로그를 MuJoCo scene xml로 생성
- `scripts/auto_input_metrics.py`
  - `control_input` 자동 publish + `/odom` 기반 지표 수집
- `scripts/run_trial.py`
  - MuJoCo + ROS launch + evaluator를 한 번 실행
- `scripts/run_experiment.py`
  - 여러 terrain/모드를 반복 실행

## 현재 단계

현재 프레임워크는 두 가지를 우선 지원합니다.

1. baseline OCS2 자동 평가
2. perceptive용 입력 이미지가 준비된 terrain에 대한 perceptive 자동 평가

`stairs`, `slope`, `rough_ground`처럼 MuJoCo geom 기반 scene은 현재
perceptive fake elevation map과 자동 정합되지 않으므로,
초기 단계에서는 baseline 자동평가용 terrain으로 사용하는 것이 안전합니다.

반면 `flat`, `perlin_hfield`, `image_hfield`는 perceptive 입력 이미지까지
같이 정의할 수 있습니다.

## 예시 순서

```bash
cd ~/ros2_ws/src/quadruped_ros2_control/evaluation/go2_terrain_eval
python3 scripts/generate_terrains.py --all
python3 scripts/run_trial.py --terrain flat --mode baseline
python3 scripts/run_trial.py --terrain perlin_easy --mode perceptive
python3 scripts/run_experiment.py --terrains flat,stairs,perlin_easy --modes baseline
```

## 결과

기본 결과는 `results/` 아래에 저장됩니다.

- trial별 상세 JSON
- 누적 CSV

기본 지표:

- `success`
- `time_to_failure`
- `distance_xy`
- `path_length`
- `roll_rms_deg`
- `pitch_rms_deg`
- `min_base_z`
