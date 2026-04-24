# MPC ↔ WBC 협업 구조와 OCS2 Whole-Body Control 방법론

## 1. MPC와 WBC의 역할 분리 (왜 두 단계로 나뉘는가)

| 구분 | MPC (SQP, 50 Hz) | WBC (QP, 400 Hz) |
|---|---|---|
| 모델 | Centroidal / SRBD (24-dim state) | Full floating-base rigid-body dyn. |
| 결정변수 | 접촉력 $\mathbf{f}_c$, 관절속도 $\dot{\mathbf{q}}_j$ | 가속도 $\ddot{\mathbf{u}}$, 접촉력 $\mathbf{F}$, 관절토크 $\boldsymbol{\tau}$ |
| 시야 | 1 s horizon (≈67 노드) | 단일 순간 (no horizon) |
| 출력 | 최적 궤적 $(\mathbf{x}^*(t),\mathbf{u}^*(t), m^*(t))$ | 실제 모터에 보낼 $\boldsymbol{\tau}^*$ |
| 목적 | "앞으로 어디로 갈지"를 계획 | "지금 이 순간 토크를 어떻게 쏘아줄지"를 실행 |

MPC는 무겁고 느리지만 미래를 본다. WBC는 가볍고 빠르며 풀 다이나믹스를 본다. 둘을 계층화해 **"MPC가 느리게 계획 → WBC가 빠르게 실체화"** 하는 구조.

## 2. MPC 결과를 WBC가 어떻게 이용하는가 (타이밍)

MPC는 50 Hz (∆t ≈ 20 ms)로 한 번 풀 때마다 **1 초 분량의 궤적 전체와 선형 피드백 게인**을 policy 형태로 내보낸다. WBC는 **400 Hz**(∆t ≈ 2.5 ms)로 돌면서 그 policy를 현재 시각 $t$에 **보간·평가**해서 그 순간의 기준값만 뽑아 쓴다.

```
ctrl_component_->mpc_mrt_interface_->updatePolicy();           // 최신 policy swap
mpc_mrt_interface_->evaluatePolicy(t_now, x_obs,
                                   optimized_state_,           // x*(t_now)
                                   optimized_input_,           // u*(t_now)
                                   planned_mode);              // m*(t_now)
```

즉 한 번의 MPC 해가 20 ms 동안 약 **8 번의 WBC tick에 재활용**되며, 각 tick은 `evaluatePolicy`로 그 순간 기준만 꺼내 쓴다. MPC가 새 policy를 내려보내기 전까지는 같은 policy를 warm-start처럼 계속 사용한다.

WBC가 tick마다 꺼내 쓰는 것:
- $\mathbf{x}^*(t)$ → `q_desired`, $\dot{\mathbf{q}}_j^{\text{des}}$, `p_foot_des`, $\dot{\mathbf{p}}_{\text{foot}}^{\text{des}}$
- $\mathbf{u}^*(t)$ → 접촉력 레퍼런스 $\mathbf{F}^{\text{des}}$ (MPC가 분배한 GRF)
- $m^*(t)$ → 각 발 contact flag $c_i \in \{0,1\}$ → 어떤 제약/태스크를 켤지 결정
- centroidal momentum rate → 원하는 base 가속도 $\ddot{\mathbf{p}}_b^{\text{des}},\ \ddot{\boldsymbol{\theta}}^{\text{des}}$

## 3. OCS2의 WBC 방법론 — QP 기반 Inverse Dynamics

OCS2 WBC는 PID가 **아니고**, 매 tick 한 번의 **Quadratic Programming** 을 qpOASES로 푸는 **Inverse-Dynamics QP** 방식이다. PD는 swing leg의 "원하는 가속도"를 만들 때만 내부적으로 쓰인다.

### 3-1 결정변수 (30-dim)
$$
\mathbf{z} = \begin{bmatrix}\ddot{\mathbf{u}} \\ \mathbf{F} \\ \boldsymbol{\tau}\end{bmatrix} \in \mathbb{R}^{6+12+12}
$$

### 3-2 Hard constraints (반드시 만족)

| 목적 | 해결 방식 | 수식 |
|---|---|---|
| 물리 일관성 | floating-base 운동방정식 등식 | $M\ddot{\mathbf{u}} - J^\top \mathbf{F} - S^\top \boldsymbol{\tau} = -\mathbf{n}$ |
| 하드웨어 한계 | 토크 부등식 박스 | $-\boldsymbol{\tau}_{\max} \le \boldsymbol{\tau} \le \boldsymbol{\tau}_{\max}$ |
| 미끄럼 방지 | friction pyramid (접촉 발) / 힘=0 (유각 발) | $\|f_{xy}\| \le \mu f_z,\ f_z \ge 0$ |
| 접촉 발 고정 | 접촉 발 가속도 = 0 | $J_c \ddot{\mathbf{u}} = -\dot{J}_c \mathbf{v}$ |

### 3-3 Soft tasks (trade-off 대상)

| 목적 | 해결 방식 | 수식 |
|---|---|---|
| 유각 발을 MPC 궤적에 맞추기 | 작업공간 **PD** 로 원하는 가속도를 뽑고 least-squares | $\mathbf{a}^{\text{swing}} = k_p(\mathbf{p}^{\text{des}}-\mathbf{p}) + k_d(\mathbf{v}^{\text{des}}-\mathbf{v})$ |
| 바디를 MPC 계획대로 움직이기 | base 선/각가속도 least-squares | $\|\ddot{\mathbf{p}}_b^{\text{des}} - \ddot{\mathbf{p}}_b\|^2 + \|\ddot{\boldsymbol{\theta}}^{\text{des}} - \ddot{\boldsymbol{\theta}}\|^2$ |
| MPC가 분배한 GRF 존중 | 접촉 발 접촉력 least-squares | $\sum c_i \|\mathbf{f}_i^{\text{des}} - \mathbf{f}_i\|^2$ |

### 3-4 두 가지 QP 통합 방식 (코드베이스에 모두 존재)

1. **[WeightedWbc](wbcWeighted.md)** — 가중합 방식 (default)
   - 모든 soft task를 가중치로 묶어 하나의 least-squares로 합치고, hard constraint만 부등식/등식으로 추가.
   - `qpOASES::QProblem` 한 번으로 풀이 → 가장 빠름, trade-off는 weight tuning.
   - $H = A_w^\top A_w,\ \mathbf{g} = -A_w^\top \mathbf{b}_w$.

2. **[HierarchicalWbc + HoQp](wbcHierarchical.md)** — 계층형 Null-space 방식
   - task를 3계층으로 분리 : $\mathcal{T}_0$(물리/토크/마찰/접촉) $\succ\ \mathcal{T}_1$(base+swing) $\succ\ \mathcal{T}_2$(contact force).
   - 상위 계층 해의 null-space 안에서만 하위 계층을 최적화 → 상위를 절대 깨지 않음.
   - $\mathbf{z}^\star = \mathrm{HoQp}(\mathcal{T}_2,\ \mathrm{HoQp}(\mathcal{T}_1,\ \mathrm{HoQp}(\mathcal{T}_0)))$

### 3-5 해에서 최종 명령으로

QP 해 $\mathbf{z}^\star$에서 끝 12개 원소만 뽑아 모터 토크로 보내고, 관절 위치/속도 기준은 MPC 정책에서 그대로 가져와서 **저레벨 PD**($k_p, k_d$ 고정)와 합쳐 하드웨어에 전송.
```
τ_cmd  = z*.tail(12)
q_cmd  = centroidal_model::getJointAngles(optimized_state_)
dq_cmd = centroidal_model::getJointVelocities(optimized_input_)
→ motor : τ_cmd + kp(q_cmd - q) + kd(dq_cmd - dq)
```

## 4. 한 줄 요약

> **MPC (50 Hz, SQP, horizon 1 s)** 가 centroidal 레벨의 궤적·접촉력·접촉모드를 미리 계획하면, **WBC (400 Hz, QP, 단일 순간)** 가 그 policy를 현재 시각으로 보간해서 풀 다이나믹스·토크 한계·마찰 콘을 모두 지키는 관절 토크로 바꿔 모터에 쏜다. PID는 오직 swing foot의 원하는 가속도 산출과 최저레벨 모터 루프에만 쓰이고, 핵심 배분은 전부 QP 최적화로 이뤄진다.
