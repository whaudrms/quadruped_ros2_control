# PerceptiveLeggedReferenceManager

## Summary

`SwitchedModelReferenceManager`를 상속하여 `modifyReferences(initTime, finalTime, initState, targetTrajectories, modeSchedule)` 를 terrain-aware 버전으로 오버라이드. 지형 높이·경사에 맞춰 **base target pitch·z를 수정**하고, foothold projection의 z를 swing trajectory planner의 **lift-off / touch-down 높이로 주입**한다. Baseline `SwitchedModelReferenceManager::modifyReferences`는 `h_terrain = 0`을 하드코딩했으므로, 이 클래스의 도입이 perceptive 모드의 핵심 차이점.

## 상태 변수 (클래스 멤버)

- `convexRegionSelectorPtr_` — 지형 데이터 접근용
- `endEffectorKinematicsPtr_` — 모든 발의 end-effector kinematics clone
- `comHeight_` — `reference.info`에서 읽은 nominal COM 높이
- `enableReferenceModification_` — 런치 플래그
- 발별 **latching 상태** (dev 브랜치 추가):
  - `previousContactFlags_[4]`
  - `hasLatchedContactPosition_[4]`, `lastLiftoffPos_[4]` — stance 진입 시 실제 발 위치를 고정해 future projection을 덮어쓰는 용도
  - `activeSwingHeightLatched_[4]`, `latchedSwingLiftOffHeights_[4]`, `latchedSwingTouchDownHeights_[4]` — swing 중 다른 planar region으로 넘어가 발 높이가 튀는 것을 방지
- `latestReferenceTrajectoriesMutex_` + `latestRawBasePath_` / `latestTerrainAwareBasePath_` / `latestFootPlacementDebugInfo_` — `CtrlComponent`가 시각화/로그용으로 비동기 조회

## modifyReferences (핵심 함수)

### 1. horizon 확장과 mode schedule

$$
T_{\mathrm{horizon}} = t_f - t_0,\quad \mathcal{M} = \mathrm{GaitSchedule}(t_0 - T_{\mathrm{horizon}},\ t_f + T_{\mathrm{horizon}})
$$

(baseline은 단순 `[t_0 - T, t_f + T]`만, 여기선 그대로지만 뒤에서 convex region selector가 쓴다.)

### 2. target trajectory terrain-aware 수정 (if `enableReferenceModification_`)

$n = 11$ nodes를 horizon 상에 균등 배치. dev 브랜치는 blending + clamping으로 거친 경사에서 터지지 않도록 만든 상수들을 hard-coded:

| 상수 | 값 | 역할 |
|---|---|---|
| `nodeNum` | 11 | horizon 샘플 개수 |
| `normalSamplingStep` | 0.3 m | finite diff 반경 |
| `pitchBlend` | 0.6 | 원본 pitch ↔ terrain pitch 블렌드 비율 |
| `heightBlend` | 0.5 | 원본 z ↔ terrain-aware z 블렌드 비율 |
| `maxAbsPitch` | 0.25 rad | node별 절대 pitch 한계 |
| `maxPitchDeltaPerNode` | 0.06 rad | node간 pitch 변화량 한계 |
| `maxHeightDeltaPerNode` | 0.03 m | node간 z 변화량 한계 |
| `downStepHeightThreshold` | 0.03 m | 이 이상 낮아지면 step-down 판정 |
| `downStepCommitDistance` | 0.08 m | init pos에서 이 이상 떨어진 뒤에야 step-down 허용 |

**지형 법선** (`smooth_planar` layer에서 유한차분):
$$
n_x = \frac{h(x-\Delta,y) - h(x+\Delta,y)}{2\Delta},\quad n_y = \frac{h(x,y-\Delta) - h(x,y+\Delta)}{2\Delta},\quad n_z = 1
$$
정규화 후 yaw 회전 전개:
$$
\mathbf{v} = R_z(\psi)^\top \mathbf{n},\quad \theta_{\mathrm{terrain}} = \arctan\!\left(\frac{v_x}{v_z}\right)
$$

**pitch 수정** (blend + clamp):
$$
\theta' = \theta_{\mathrm{raw}} + 0.6\,(\theta_{\mathrm{terrain}} - \theta_{\mathrm{raw}}),\quad \theta' \leftarrow \mathrm{clamp}(\theta',\pm0.25),\quad |\theta' - \theta_{\mathrm{prev}}| \le 0.06
$$

**base z 수정**:
$$
z_{\mathrm{terrain}} = h_{\mathrm{smooth}}(x,y) + \frac{h_{\mathrm{com}}}{\cos \theta'} \quad \text{(safeCos = max(0.9, cos(θ')))}
$$

**step-down 가드**: `z_terrain` 이 `z_raw - 0.03 m` 보다 낮고(descending) 동시에 초기 위치로부터 `0.08 m` 이내면 아직 raw를 유지 — 즉 내려가는 계단은 발이 어느 정도 나아간 뒤에야 target에 반영. 갑자기 COM이 툭 떨어지는 걸 막는다.

$$
z' = z_{\mathrm{raw}} + 0.5\,(z_{\mathrm{terrain}} - z_{\mathrm{raw}}),\quad |z' - z_{\mathrm{prev}}| \le 0.03\ \mathrm{m}
$$

예외 발생 시 `previousPitch`, `previousHeight`를 그대로 유지 (grid map 범위 밖 접근 등).

### 3. Footstep & swing

`rawTargetTrajectories`(수정 전 복사본)를 convex region selector에 넘김:
```cpp
convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, rawTargetTrajectories);
updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
```

### 4. 디버그 스냅샷 저장

`latestRawBasePath_`, `latestTerrainAwareBasePath_`(각 node의 base XYZ), `latestFootPlacementDebugInfo_` (time, contactFlags, footPlacementFlags, 각 polygon vertex 개수, projection z, initStandFinalTimes) 를 mutex로 보호하며 저장 → `CtrlComponent`의 publisher/로거에서 주기적으로 발행.

## updateSwingTrajectoryPlanner(initTime, initState, modeSchedule)

각 발 leg ∈ {0..3}에 대해:

1. `modeSchedule.modeSequence` 로부터 `contactFlagStocks[leg]` (phase별 bool) 추출
2. `initIndex` = lookup(initTime)
3. `ConvexRegionSelector`가 만든 `projections` 를 복제
4. `modifyProjections(...)` — 아래 설명, projection의 z를 lastLiftoff로 덮어쓰기
5. `getHeights(contactFlagStocks, projections)` — lift/touch 높이 배열 생성
6. **현재 swing 중** 이면서 (`!activeSwingHeightLatched_[leg]` 또는 `previousContact`) 인 phase 전환 첫 tick에만 현재 값들을 latch. 이후에는 latch된 값으로 전체 active swing 구간(`findActiveSwingBounds`로 찾은 범위)을 덮어쓴다. → 발이 공중에 있는 동안 target 높이가 변동하지 않도록.
7. `liftOffHeightSequence[leg]`, `touchDownHeightSequence[leg]` 를 planner에 주입.

마지막:
```cpp
swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
```

## modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks, projections)

**목적** : 다리가 지금 stance 중이면, 그 stance 구간 전체의 projection.z를 "실제로 디딘" 위치로 덮어씀. 그러면 swing planner가 touch-down 높이를 이 값으로 가져가서 다음 swing에 이어붙임.

- `enteringContact` (= currently stance and no latch or previousContact false): 현재 발 위치를 저장
  $$
  \mathbf{p}_{\text{liftoff,last}} = \mathbf{p}_{ee}(x_{\text{init}}) + (0,0,-0.02)^\top
  $$
- stance 구간 전체 (initIndex 앞뒤로 확장): `projections[i].positionInWorld = lastLiftoffPos_[leg]`
- `initTime > initStandFinalTime_[leg]` (첫 stand 종료 이후)이면 과거 쪽도 추가 보정 — 여러 stance가 연속되어도 같은 값으로 연결

## getHeights(contactFlagStocks, projections)

각 phase $i$에서 swing 발만 대상:
$$
h_{\mathrm{lift}}[i] = \begin{cases}z_{i-1}, & \neg c[i] \land c[i-1] \\ h_{\mathrm{lift}}[i-1], & \neg c[i] \land \neg c[i-1]\end{cases},\quad
h_{\mathrm{touch}}[i] = \begin{cases}z_{i+1}, & \neg c[i] \land c[i+1] \\ h_{\mathrm{touch}}[i+1], & \neg c[i] \land \neg c[i+1]\end{cases}
$$
즉 각 swing phase의 lift는 바로 직전 stance의 projection.z, touch는 바로 다음 stance의 projection.z.

## getFootPlacementFlags(time)

각 발에 대해 `isActive` 판단용 플래그:
$$
\text{flag}_i = c_i(t) \land (t \ge \text{initStandFinalTime}_i)
$$
즉 **stance 중이면서, 초기 stance 구간이 끝난 후** 에만 foot placement 제약 활성. 초기 stance에 대해 제약이 과도하게 걸리는 것을 피함. `FootPlacementConstraint::isActive`가 이 함수를 읽는다.

## getLatestReferencePaths / getLatestFootPlacementDebugInfo

외부(CtrlComponent)에서 비동기로 snapshot을 얻는 접근자. `latestReferenceTrajectoriesMutex_`로 보호하고, 한 번이라도 `modifyReferences`가 불렸을 때만 `true` 반환.
