# ConvexRegionSelector

## Summary

horizon 내의 각 stance phase에 대해 **(1) 그 stance 구간의 대표 시각(middle time)을 고르고**, **(2) target trajectory 기준 nominal foothold를 계산**, **(3) planarTerrain의 region 중 가장 잘 맞는 것에 projection**, **(4) 그 projection 주변에서 inset boundary 안쪽으로만 성장하는 convex polygon을 만든다**. 이 결과를 발별 버퍼에 저장해서 `FootPlacementConstraint`와 `PerceptiveLeggedPrecomputation`이 조회하게 함.

## 상태 변수

- `info_` — CentroidalModelInfo (발 개수 등)
- `numVertices_` — convex polygon이 몇 변으로 근사될지 (예: 8)
- `planarTerrainPtr_`, `terrainDataMutexPtr_` — `PerceptiveLeggedInterface`와 공유
- `planarTerrain_` — `update()` 진입 시 mutex 내부에서 복사한 로컬 snapshot
- 발별 버퍼 `feet_array_t<std::vector<...>>` 크기 = `numPhases`:
  - `feetProjections_[leg][i]` : `PlanarTerrainProjection` (regionPtr, positionInWorld, positionInTerrainFrame)
  - `convexPolygons_[leg][i]` : `CgalPolygon2d`
  - `nominalFootholds_[leg][i]` : 월드 좌표 `vector3_t`
- `middleTimes_[leg]` : stance phase별 중심 시각 목록
- `timeEvents_[leg]` — 최신 `modeSchedule.eventTimes` 사본 (lookup에 씀)
- `initStandFinalTime_[4]`, `initStandFinalTimeLatched_[4]` — 초기 시각이 포함된 stand phase의 종료 시각. 처음에만 한 번 latch 되며 `PerceptiveLeggedReferenceManager::getFootPlacementFlags`가 활성 시점 판정에 사용.

## 조회 API (getProjection / getConvexPolygon / getNominalFootholds / getMiddleTimes / getInitStandFinalTimes)

모두 `lookup::findIndexInTimeArray(timeEvents_[leg], time)` 로 index를 찾고 그 index의 값을 반환. 현재 MPC horizon이 여러 번 그려놓은 버퍼에서 시점별 값을 뽑는 방식이라, `update()` 이후 버퍼가 완성되어야 의미 있음.

## update(modeSchedule, initTime, initState, targetTrajectories) — 핵심 함수

### 1. terrain snapshot

```cpp
if (terrainDataMutexPtr_) { lock; planarTerrain_ = *planarTerrainPtr_; } else ...
```
외부에서 `PlanarTerrainReceiver`가 업데이트 중이더라도 consistent한 snapshot으로 진행.

### 2. stance phase의 시작/끝 index 찾기

각 발마다 phase별로 `findIndex(i, contactFlagStocks[leg])` 호출:
- 그 phase가 stance면, 이전에 swing이었던 지점(start)과 다음에 swing이 되는 지점 직전(final)을 찾음.
- 즉 하나의 stance 블록에 속한 phase들은 모두 같은 (start, final) index를 가진다.

### 3. 각 phase 처리

```cpp
lastStandMiddleTime = NaN;
for each phase i with c_leg[i]=1:
    standStartTime = eventTimes[startIndices[leg][i]];
    standFinalTime = eventTimes[finalIndices[leg][i]];
    stanceDuration = standFinalTime - standStartTime;
    standSelectionTime = min(standFinalTime,
                             standStartTime + min(0.05, 0.15 * stanceDuration));
```

→ **stance 구간 초반부 (5% 또는 15% 중 작은 쪽, 최대 0.05 s)**를 "대표 시각"으로 쓴다. 이 시각에 target trajectory를 읽어 nominal foothold를 계산.

같은 stance 블록 내 여러 phase는 첫 번째 phase에서만 실제 계산을 수행하고 나머지는 바로 앞 phase의 결과를 복사 (`feetProjections_[leg][i] = feetProjections_[leg][i-1]` 등) — 중복 계산 방지.

대표 시각에서 수행:
```cpp
vector3_t footPos = getNominalFoothold(leg, standSelectionTime, initState, targetTrajectories);
projection = getBestPlanarRegionAtPositionInWorld(footPos, planarTerrain_.planarRegions, penaltyFunction);
convexRegion = growConvexPolygonInsideShape(
        projection.regionPtr->boundaryWithInset.boundary,
        projection.positionInTerrainFrame,
        numVertices_, growthFactor=1.05);
```

- `getBestPlanarRegionAtPositionInWorld` : 월드 점에서 가까운 planar region을 후보로 골라서, penalty function(여기선 항상 0)을 비교해 최적 region + 그 region 좌표계에서의 projection을 반환.
- `growConvexPolygonInsideShape` : 그 region의 **inset boundary** 안쪽에서 projection 주변으로 점점 커지는 convex polygon을 grow. 결과는 2D (`CgalPolygon2d`) with `numVertices_` 개 꼭짓점.

stance 블록이 초기 시각을 포함하면 (`standStartTime < initTime < standFinalTime`) 그 종료 시각을 `initStandFinalTime_[leg]`에 latch (한 번만).

### 4. timeEvents_ 갱신

`timeEvents_[leg] = modeSchedule.eventTimes` — lookup용.

## extractContactFlags(modeSequence)

`modeNumber2StanceLeg(mode)` 로 16가지 mode를 4-bit 으로 해석해 `contactFlagStock[leg][phase]` 배열 만들어 반환. 다른 곳(`PerceptiveLeggedReferenceManager`)에서도 이 헬퍼를 씀.

## findIndex(index, contactFlagStock) — private

주어진 stance index에서 뒤로 스캔해 첫 swing을 만나면 그 index가 `startTimesIndex`, 앞으로 스캔해 첫 swing 직전을 `finalTimesIndex`. contactFlag가 false면 `(0,0)` 반환.

## getNominalFoothold(leg, time, initState, targetTrajectories) — private

**목적**: target trajectory가 원하는 발 위치에 pitch 보정 offset을 빼서 terrain-pitch를 반영한 이상적 foothold 얻기.

```cpp
scalar_t height = 0.4;  // ← hip 위쪽을 의미하는 경험적 상수
vector_t desiredState = targetTrajectories.getDesiredState(time);
vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);
```

Raibert-style **feedback** (계산만 하고 실제로는 쓰지 않음):
$$
\mathbf{f}_{\mathrm{fb}} = \sqrt{h/g}\,(\mathbf{v}_{\mathrm{meas}} - \mathbf{v}_{\mathrm{des}}),\quad g = 9.81
$$

실제로 적용되는 **pitch offset**:
$$
\theta = \text{pitch of desired base pose},\quad \psi = \text{yaw of desired base pose}
$$
$$
\Delta x = \tan(-\theta)\,h,\quad
\mathbf{o} =
\begin{bmatrix}
\Delta x \cos(-\theta) \\ 0 \\ \Delta x \sin(-\theta)
\end{bmatrix},\quad
R_z(\psi) =
\begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1
\end{bmatrix}
$$
$$
\boxed{\mathbf{p}_{\mathrm{nom}} = \mathbf{p}_{ee}^{\mathrm{des}}(t) - R_z(\psi)^\top \mathbf{o}}
$$

`p_ee^des(t)` 는 desired state에서의 end-effector FK. feedback 항은 주석 처리되어 있어 지형 기울기에 대한 pitch-driven offset만 적용된다.

## sampleTerrainHeight(x, y) — dev 추가

`planarTerrainPtr_->gridMap`에서 `smooth_planar` layer(없으면 `elevation`)를 조회해 월드 (x, y)에서의 지면 높이를 반환. `CtrlComponent::alignPerceptiveBaseHeightToTerrain`이 Kalman estimator의 base z를 지형에 붙이려고 호출한다 (linear_kalman 추정기에서 z drift 보정용).
