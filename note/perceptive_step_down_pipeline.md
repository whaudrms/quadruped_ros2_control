# Perceptive MPC Step-Down Scenario Pipeline

이 문서는 로봇이 **높은 평평한 박스 위에서 낮은 평평한 바닥으로 내려가는 상황**을 기준으로, OCS2 perceptive MPC 전체 파이프라인을 정리한 것이다.

핵심 관점:

```text
terrain perception/reference layer
→ centroidal MPC
→ WBC
→ motor torque
```

perceptive mode는 WBC를 바꾸지 않는다. 지형 정보는 MPC 앞단의 reference modification, swing trajectory update, precomputation, state soft constraint를 통해 들어간다.

---

## 1. 전체 계층 구조

평지 baseline:

```text
cmd_vel / gait command
→ TargetTrajectories
→ SwitchedModelReferenceManager
→ SwingTrajectoryPlanner(terrain height = 0)
→ Centroidal MPC
→ WBC
→ motor
```

Perceptive step-down:

```text
box step terrain
→ PlanarTerrain + SDF
→ PerceptiveLeggedReferenceManager
→ ConvexRegionSelector
→ PerceptiveLeggedPrecomputation
→ terrain-aware Centroidal MPC
→ unchanged WBC
→ motor
```

단차를 내려가는 상황에서 perceptive layer가 하는 일은 네 가지다.

1. base pitch와 base z reference를 지형에 맞게 수정한다.
2. swing foot의 lift-off / touch-down height를 높은 박스와 낮은 바닥 높이에 맞춘다.
3. 다음 stance foot 위치가 낮은 바닥의 안전한 convex region 안에 오도록 soft constraint를 추가한다.
4. SDF로 swing foot과 body sphere collision을 피하게 한다.

---

## 2. MPC Horizon과 Contact Mode

OCS2 legged MPC의 horizon은 gait phase에 맞춰 시작하지 않는다. 매 MPC iteration마다 현재 시간 기준으로 고정 길이 horizon을 잡는다.

```math
[t_0,\; t_0 + T]
```

여기서 `t_0`는 현재 observation time이다.

따라서 horizon은 다음과 같은 구간도 자연스럽게 포함할 수 있다.

```text
현재 swing phase 중간
→ touch-down
→ stance phase
→ lift-off
→ 다음 swing phase 중간
```

contact mode sequence는 MPC가 최적화하지 않는다. gait scheduler가 미리 정한 mode schedule을 horizon 안에 잘라 넣는다.

```math
\sigma(t), \quad t \in [t_0,\; t_0+T]
```

각 발의 stance duration은 사용자가 정의한 gait template에서 나온다.

예:

```text
trot period = 0.6 s
phase 0.0 ~ 0.5 : LF + RH stance
phase 0.5 ~ 1.0 : RF + LH stance
```

그러면 diagonal pair의 stance duration은:

```math
0.5 \times 0.6 = 0.3 s
```

이다.

---

## 3. Step Terrain이 PlanarTerrain과 SDF로 변환된다

예시 지형:

```text
높은 박스 윗면: z = 0.20 m
낮은 바닥:     z = 0.00 m
```

`StaticPlanarTerrainPublisher`는 MuJoCo XML의 box geometry를 읽고 각 box의 top face를 planar region으로 만든다.

```text
Region 1: 높은 박스 윗면
  height ≈ 0.20 m
  boundary = box top rectangle

Region 2: 낮은 바닥
  height ≈ 0.00 m
  boundary = floor region
```

동시에 grid map이 만들어진다.

```text
elevation(x, y)
  실제 계단 높이. edge에서 불연속적일 수 있음.

smooth_planar(x, y)
  elevation을 Gaussian smoothing한 layer.
  base pitch와 base z reference 계산에 사용.

SDF
  terrain / obstacle surface까지의 signed distance field.
  foot collision, body collision soft constraint에 사용.
```

`PlanarTerrainReceiver`는 terrain message를 ROS callback에서 버퍼에 저장하고, MPC solver가 시작되기 직전 `preSolverRun`에서 공유 terrain과 SDF를 갱신한다.

```text
terrain topic
→ PlanarTerrainReceiver callback buffer
→ preSolverRun
→ planarTerrainPtr_
→ signedDistanceFieldPtr_
```

이렇게 하면 한 번의 MPC solve 동안 reference modification, convex region selection, SDF constraint가 같은 terrain snapshot을 본다.

---

## 4. Base z Reference가 Step-Down Guard를 거쳐 수정된다

기존 flat-ground target의 base height를 `z_raw`라고 하자.

```math
z_{raw}
```

이 값은 perceptive modification 전의 target trajectory에 들어 있던 base z reference다. 실험 전체의 시작 높이가 아니라, 현재 MPC iteration에서 target trajectory가 가진 raw base height이다.

perceptive layer는 horizon 위의 sample node `(x, y)`에서 smoothed terrain height를 읽는다.

```math
h_s(x,y) = \text{smooth\_planar}(x,y)
```

그리고 terrain-aware candidate height를 만든다.

```math
z_{terrain}
=
h_s(x,y)
+
\frac{h_{com}}{\max(0.9,\cos\theta')}
```

여기서 `z_terrain`은 최종 reference가 아니라, terrain을 반영한 후보 base height이다.

단차를 내려갈 때 높은 박스 위에서 낮은 바닥 쪽 node를 보면:

```text
z_raw      ≈ 0.20 + h_com
z_terrain  ≈ 0.00 + h_com
```

즉 `z_terrain`이 `z_raw`보다 크게 낮아진다.

step-down 판정:

```math
z_{terrain} < z_{raw} - 0.03
```

이 부등식은 "terrain을 반영하면 raw reference보다 3 cm 이상 낮아지는 상황"을 의미한다. 즉 내려가는 단차를 감지하는 조건이다.

여기에 거리 guard가 붙는다.

```math
d((x,y),(x_0,y_0)) < 0.08
```

여기서:

```text
(x, y)
  horizon sample node의 target 위치

(x0, y0)
  현재 MPC iteration의 initial state, 즉 현재 observation의 base 위치
```

따라서 이 조건은 "해당 target node가 현재 로봇 위치에서 8 cm 이내인가?"를 묻는다.

두 조건이 동시에 참이면:

```text
아직 base reference를 낮추지 않는다.
raw height를 유지한다.
```

즉:

```math
z_{candidate} = z_{raw}
```

로 둔다.

의미:

```text
로봇이 아직 박스 edge 근처에 있는데
smooth_planar 때문에 낮은 바닥 높이가 너무 일찍 보이면
COM reference가 갑자기 내려가며 자세가 무너질 수 있다.

따라서 현재 위치에서 너무 가까운 node에서는
step-down height를 바로 반영하지 않는다.
```

node가 현재 위치에서 충분히 멀어지면:

```math
d((x,y),(x_0,y_0)) \ge 0.08
```

그때부터 낮은 terrain height를 reference에 반영한다.

마지막으로 blending과 변화량 제한을 적용한다.

```math
z'
=
z_{raw} + 0.5(z_{candidate} - z_{raw})
```

```math
|z'_k - z'_{k-1}| \le 0.03 \text{ m}
```

결과적으로 base z reference는 낮은 바닥을 따라가지만, 단차 edge에서 갑자기 떨어지지 않는다.

---

## 5. Base Pitch Reference도 Terrain Normal로 수정된다

`smooth_planar` layer에서 finite difference로 terrain normal을 계산한다.

```math
n_x =
\frac{h_s(x-\Delta,y)-h_s(x+\Delta,y)}{2\Delta}
```

```math
n_y =
\frac{h_s(x,y-\Delta)-h_s(x,y+\Delta)}{2\Delta}
```

```math
n_z = 1
```

정규화 후 yaw frame으로 돌린다.

```math
\bm{v} = R_z(\psi)^T \bm{n}
```

terrain pitch:

```math
\theta_{terrain}
=
\arctan\left(\frac{v_x}{v_z}\right)
```

raw pitch와 blend한다.

```math
\theta'
=
\theta_{raw}
+
0.6(\theta_{terrain}-\theta_{raw})
```

그리고 과도한 pitch를 제한한다.

```math
|\theta'| \le 0.25 \text{ rad}
```

node 사이의 변화량도 제한한다.

```math
|\theta'_k-\theta'_{k-1}| \le 0.06 \text{ rad}
```

단차 내려가기에서는 `smooth_planar`가 edge 주변을 완만한 내리막처럼 만들기 때문에, base pitch reference가 약간 내려가는 방향으로 기울어진다.

---

## 6. ConvexRegionSelector가 다음 Stance Foot의 안전 영역을 만든다

단차를 내려갈 때 중요한 것은 다음 발이 낮은 바닥의 안전한 영역에 착지하는 것이다.

`ConvexRegionSelector`는 각 발의 stance block마다 대표시각을 잡는다.

```math
t_{sel}
=
t_{start}
+
\min(0.05,\;0.15(t_{final}-t_{start}))
```

이 시간은 stance 시작 직후의 안정된 시점이다.

이렇게 잡는 이유:

```text
t_start는 contact transition 경계라 불안정할 수 있음.
stance 중간/끝은 이미 착지 후 시간이 많이 지난 상태.
foot placement는 touchdown 직후 위치가 중요.
```

따라서 stance 시작 후 15% 지점, 최대 0.05초 안쪽을 대표시각으로 쓴다.

---

## 7. Desired Foot Position은 Terrain만으로 계산하지 않는다

`p_ee,i^des(t_sel)`은 terrain map에서 직접 나오는 값이 아니다.

`ConvexRegionSelector::update`는 다음 정보를 받는다.

```text
modeSchedule
initTime
initState
targetTrajectories
```

`targetTrajectories.getDesiredState(t_sel)`로 desired centroidal state를 얻는다.

```math
\bm{x}^{des}(t_{sel})
=
\begin{bmatrix}
\dot{\bm{p}}_{com}^{des} \\
\bm{\omega}^{des} \\
\bm{r}_b^{des} \\
\bm{\theta}_b^{des} \\
\bm{q}_j^{des}
\end{bmatrix}
```

여기에는 desired base pose와 nominal/default joint posture가 들어 있다.

따라서 Pinocchio forward kinematics로 desired foot position을 계산할 수 있다.

```math
\bm{p}_{ee,i}^{des}(t_{sel})
=
FK_i
(
\bm{r}_b^{des}(t_{sel}),
\bm{\theta}_b^{des}(t_{sel}),
\bm{q}_j^{des}(t_{sel})
)
```

즉 perceptive layer는 terrain만 보고 footstep을 새로 생성하는 것이 아니다. 기존 target trajectory가 암시하는 desired robot configuration에서 nominal foot 위치를 읽고, 그것을 terrain region 선택의 seed로 쓴다.

---

## 8. Pitch Offset을 빼서 Nominal Foothold를 만든다

base pitch가 있으면 FK로 얻은 foot position이 몸통 기울기에 의해 앞뒤로 치우칠 수 있다.

그래서 pitch-driven offset을 계산한다.

```math
\Delta x = \tan(-\theta) h
```

```math
\bm{o}
=
\begin{bmatrix}
\Delta x \cos(-\theta) \\
0 \\
\Delta x \sin(-\theta)
\end{bmatrix}
```

여기서 `h ≈ 0.4 m`는 경험적 높이 상수다.

이 offset은 robot heading frame 기준이므로 yaw를 반영해서 world frame으로 돌린다.

```math
\bm{p}_{nom,i}
=
\bm{p}_{ee,i}^{des}
-
R_z(\psi)^T \bm{o}
```

의미:

```text
desired FK foot position에서
base pitch 때문에 생긴 앞뒤 offset을 제거해서
terrain region을 고를 seed point를 만든다.
```

---

## 9. Nominal Foothold가 낮은 Floor Region으로 Projection된다

단차 지형에는 여러 planar region이 있다.

```text
Region A: 높은 박스 윗면
Region B: 낮은 바닥
```

다음 stance foot의 `p_nom`이 박스 edge 너머 낮은 바닥 쪽에 있다면, selector는 낮은 floor region을 선택한다.

```math
\Pi_i
=
\operatorname{proj}_{\mathcal{R}_{floor}}(\bm{p}_{nom,i})
```

낮은 floor가 `z=0`인 평면이면 projection은 직관적으로:

```math
\bm{p}_{proj}
\approx
\begin{bmatrix}
p_{nom,x} \\
p_{nom,y} \\
0
\end{bmatrix}
```

이다.

그 다음 projection 주변에서 floor region의 inset boundary 안쪽으로 convex polygon을 grow한다.

```text
projection point on lower floor
→ grow convex polygon inside lower floor boundary
→ safe foot placement region
```

---

## 10. Convex Polygon은 Foot Placement Soft Constraint가 된다

2D polygon은 half-space inequality로 바뀐다.

```math
A_{poly} p_{2d} + b_{poly} \ge 0
```

이를 world foot position에 대한 제약으로 변환한다.

```math
A_i p_{ee,i}(x) + b_i \ge 0
```

perceptive foot placement constraint:

```math
\boxed{
\bm{h}_{fp,i}(\bm{x})
=
\bm{A}_i \bm{p}_{ee,i}(\bm{x})
+
\bm{b}_i
\ge
\bm{0}
}
```

이 제약은 baseline `legged_mpc.tex`에 있던 발 속도 제약과 다른 것이다.

baseline stance constraint:

```math
\dot{\bm{p}}_{ee,i}=0
```

의미:

```text
stance foot이 미끄러지지 않게 함
```

perceptive foot placement constraint:

```math
A_i p_{ee,i}(x)+b_i \ge 0
```

의미:

```text
stance foot 위치가 안전한 terrain polygon 안에 오게 함
```

따라서 단차 내려가기에서는:

```text
낮은 바닥에 착지할 발:
  foot placement soft constraint
  → 낮은 floor polygon 안에 착지하도록 유도

착지 후:
  stance zero-velocity constraint
  → 그 위치에서 미끄러지지 않도록 유지
```

이 제약은 hard constraint가 아니라 `stateSoftConstraintPtr`에 들어가는 relaxed barrier soft constraint다.

```math
\ell_{MPC}
=
\ell_{baseline}
+
p_{barrier}(A_i p_{ee,i}(x)+b_i)
```

---

## 11. Swing Foot Height가 높은 박스와 낮은 바닥 높이에 맞춰진다

기존 swing trajectory planner는 terrain height를 0으로 가정한다.

평지 baseline:

```math
z_{lo}=0,\qquad z_{td}=0
```

perceptive mode에서는 ConvexRegionSelector의 projection.z가 swing planner의 lift-off / touch-down height sequence에 들어간다.

높은 박스에서 낮은 floor로 내려가는 swing이라면:

```text
previous stance projection.z ≈ 0.20 m
next stance projection.z     ≈ 0.00 m
```

따라서:

```math
h_{lift} = z_{\text{previous stance projection}}
```

```math
h_{touch} = z_{\text{next stance projection}}
```

이고, swing planner의 endpoint height로 들어간다.

```math
z_{lo} \leftarrow h_{lift}
```

```math
z_{td} \leftarrow h_{touch}
```

즉:

```text
lift-off node:
  z ≈ 0.20 m

mid-swing node:
  z ≈ min(0.20, 0.00) + swingHeight

touch-down node:
  z ≈ 0.00 m
```

이로부터 swing foot vertical reference가 만들어진다.

```math
z_i^{ref}(t), \qquad \dot{z}_i^{ref}(t)
```

이 reference는 MPC의 swing foot normal velocity constraint에 들어간다.

```math
g_i^{nv}(x,u,t)
=
\dot{z}_{foot,i}
+
\alpha_p(z_{foot,i}-z_i^{ref}(t))
+
\dot{z}_i^{ref}(t)
=0
```

따라서 높은 박스에서 낮은 바닥으로 내려갈 때 swing foot은 높은 곳에서 떠서 낮은 곳으로 착지하는 vertical trajectory를 갖는다.

---

## 12. Active Swing Height Latching

발이 공중에 떠 있는 동안 terrain projection이 바뀌면 swing trajectory endpoint가 갑자기 바뀔 수 있다.

예:

```text
edge 근처에서 projection이
높은 박스 region ↔ 낮은 floor region
사이를 왔다 갔다 할 수 있음
```

이를 막기 위해 active swing 중에는 lift/touch height를 latch한다.

```text
swing 시작 시 h_lift, h_touch 저장
→ active swing interval 동안 같은 값 유지
```

즉:

```math
(h_{lift}^k,h_{touch}^k)
\leftarrow
(\bar{h}_{lift},\bar{h}_{touch})
```

for all active swing phases.

이 덕분에 foot이 공중에 있는 동안 target height가 튀지 않는다.

---

## 13. SDF Collision Constraints

단차를 내려갈 때 발끝이나 종아리 링크가 박스 edge에 부딪힐 수 있다.

이를 막기 위해 SDF 기반 soft constraint가 추가된다.

### 13.1 Swing Foot Collision

```math
h_{fc,i}(x)
=
d_{SDF}(p_{ee,i}(x)) - d_{clr}
\ge 0
```

```math
d_{clr}=0.01 \text{ m}
```

의미:

```text
swing foot은 terrain / obstacle surface에서 최소 1 cm 이상 떨어져야 한다.
```

단, contact transition 근처에서는 꺼진다.

```math
\neg c_i(t)
\land
\neg c_i(t+0.025)
\land
\neg c_i(t-0.05)
```

touch-down 순간에는 발이 지면에 가까워져야 하므로 barrier가 터지지 않게 하기 위함이다.

### 13.2 Body Sphere Collision

calf link들을 sphere로 근사한다.

```text
FL_calf, FR_calf, RL_calf, RR_calf
```

각 sphere center `p_j(x)`에 대해:

```math
h_j(x)
=
d_{SDF}(p_j(x)) - r_j
\ge 0
```

의미:

```text
종아리 sphere가 박스 edge나 지형 내부로 들어가지 않게 한다.
```

---

## 14. Perceptive MPC OCP

동역학은 기존 centroidal MPC와 동일하다.

```math
\dot{x} = f_{SRBD}(x,u,\sigma(t))
```

입력도 동일하다.

```math
u =
\begin{bmatrix}
F_c \\
\dot{q}_j
\end{bmatrix}
```

perceptive mode에서 바뀌는 것은 reference와 soft cost다.

```math
\ell_{perc}
=
\ell_{base}
+
\ell_{footPlacement}
+
\ell_{footCollision}
+
\ell_{bodyCollision}
```

여기서:

```text
ell_base
  terrain-aware base z / pitch reference,
  terrain-aware swing trajectory reference를 사용

ell_footPlacement
  stance foot이 convex polygon 안에 들어가도록 유도

ell_footCollision
  swing foot이 SDF clearance를 유지하도록 유도

ell_bodyCollision
  calf sphere가 terrain과 충돌하지 않도록 유도
```

따라서 perceptive MPC는 새 dynamics를 쓰는 것이 아니다.

```text
기존 centroidal MPC
+ terrain-aware reference
+ terrain-aware state soft constraints
```

이다.

---

## 15. WBC는 Perceptive Layer를 직접 모른다

MPC가 terrain-aware policy를 만들면 WBC는 기존과 동일하게 그 policy를 평가한다.

```text
MPC output:
  x*(t)      optimized centroidal state
  u*(t)      optimized contact force / joint velocity input
  sigma*(t)  planned contact mode
```

WBC는:

```text
x*, u*, sigma*
→ inverse-dynamics QP
→ torque
```

를 수행한다.

WBC는:

```text
왜 base z reference가 낮아졌는지
왜 다음 발이 낮은 floor polygon 안에 가야 하는지
왜 swing height가 0.20 → 0.00으로 바뀌었는지
```

를 알 필요가 없다.

그 결과만 받아 full-body dynamics, contact constraints, torque limits를 만족하는 torque로 변환한다.

---

## 16. Step-Down Scenario End-to-End Summary

높은 박스에서 낮은 바닥으로 내려가는 전체 흐름:

```text
1. StaticPlanarTerrainPublisher가 박스 윗면과 바닥을 planar region으로 발행

2. PlanarTerrainReceiver가 MPC 시작 직전에 terrain과 SDF를 갱신

3. PerceptiveLeggedReferenceManager가 smooth_planar를 샘플링
   → base pitch reference 수정
   → base z candidate 계산
   → step-down guard로 너무 이른 COM 하강 방지

4. ConvexRegionSelector가 다음 stance phase의 대표시각 t_sel을 잡음

5. target trajectory에서 desired base pose + nominal joint posture를 읽음
   → FK로 p_ee_des(t_sel) 계산

6. pitch offset을 빼서 p_nom 생성

7. p_nom이 낮은 바닥 쪽이면 낮은 floor planar region으로 projection

8. projection 주변에서 convex polygon 생성

9. PerceptiveLeggedPrecomputation이 polygon을
   A p_ee + b >= 0 형태의 world-frame half-space로 변환

10. FootPlacementConstraint가 stance foot 위치를 낮은 floor polygon 안으로 유도

11. SwingTrajectoryPlanner는
    z_lo ≈ 높은 박스 height,
    z_td ≈ 낮은 바닥 height
    를 받아 swing vertical reference 생성

12. FootCollisionConstraint와 SphereSdfConstraint가
    발끝/종아리 충돌을 SDF soft barrier로 회피

13. Centroidal MPC가 terrain-aware reference와 soft constraint를 포함해 OCP solve

14. WBC는 unchanged 상태로 MPC policy를 torque로 변환
```

한 줄 요약:

```text
perceptive MPC는 단차를 내려갈 때 base는 너무 빨리 낮추지 않고,
다음 발은 낮은 바닥의 안전한 convex region에 놓이도록 유도하며,
swing foot은 높은 lift-off에서 낮은 touch-down으로 가는 trajectory를 만들고,
SDF로 발/몸체 충돌을 피하게 만든 뒤,
기존 centroidal MPC와 WBC 구조를 그대로 사용한다.
```
