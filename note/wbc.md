- 1.  State Estimation
    1. Overview
        
        전체 흐름은 다음과 같음
        
        $$
        \text{Sensor Data} \;\rightarrow\; \text{State Estimation} \;\rightarrow\; x_{\text{rbd}} \;\rightarrow\; x_{\text{centroidal}} \;\rightarrow\; \text{Observation}
        $$
        
    2. RBD 상태 수집
        - RBD 모델 :
        
        $$
        \mathbf{x}_k^{\mathrm{RBD}} =
        \begin{bmatrix}
        \boldsymbol{\theta} \\
        \mathbf{p}_b \\
        \mathbf{q}_j \\
        \boldsymbol{\omega} \\
        \mathbf{v}_b \\
        \dot{\mathbf{q}}_j
        \end{bmatrix}
        \in \mathbb{R}^{2n}
        $$
        
        - 상태 구성:
        
        $$
        \boldsymbol{\theta} \in SO(3), \quad \mathbf{q}_j \in \mathbb{R}^{n-6}, \quad \dot{\mathbf{q}}_j \in \mathbb{R}^{n-6}, \quad \boldsymbol{\omega}, \; \mathbf{v}_b \in \mathbb{R}^3
        
        $$
        
        - 센서 입력:
        
        $$
        \begin{cases}
        \text{IMU:} & \mathbf{q}_{\mathrm{imu}}, \; \boldsymbol{\omega}_{\mathrm{local}}, \; \mathbf{a}_{\mathrm{local}} \\
        \\
        \text{Joint Encoder:} & \mathbf{q}_j, \; \dot{\mathbf{q}}_j \\
        \\
        \text{Foot Force:} & F_i > F_{\mathrm{threshold}} \;\Rightarrow\; c_i = 1
        \end{cases}
        $$
        
    3.  Kalman Filter 상태 추정
        
        $$
        \text{Sensor Data} \;\rightarrow\; \text{State Estimation} \;\rightarrow\; x_{\text{rbd}} 
        $$
        
    4.  Centroidal 상태 변환
        - 추정된 RBD 상태를 Centroidal 상태로 변환
        
        $$
        \mathbf{x}_k^{\mathrm{cent}} =
        T_{\mathrm{RBD} \rightarrow \mathrm{cent}}(\mathbf{x}_k^{\mathrm{RBD}})
        =
        \begin{bmatrix}
        \mathbf{p}_b \\
        \boldsymbol{\theta} \\
        \dot{\mathbf{p}}_b \\
        \boldsymbol{\omega}
        \end{bmatrix}
        $$
        
        $$
        \mathbf{x}_k^{\mathrm{cent}} =
        \begin{bmatrix}
        x_b \\
        y_b \\
        z_b \\
        \phi \\
        \theta \\
        \psi_{\mathrm{unwrap}} \\
        \dot{x}_b \\
        \dot{y}_b \\
        \dot{z}_b \\
        \omega_x \\
        \omega_y \\
        \omega_z
        \end{bmatrix}
        \in \mathbb{R}^{12}
        $$
        
    5. 보행 모드 (Contact Mode)
        - 보행모드 함수
        - Obeservation에 같이 넘겨줌
        
        $$
        \text{mode} = \mathrm{stanceLeg2ModeNumber}(c_1, c_2, c_3, c_4)
        $$
        
    6. 최종 출력: SystemObservation
    
    $$
    \text{SystemObservation} =
    \begin{cases}
    \mathbf{x} = \mathbf{x}_k^{\mathrm{cent}} \in \mathbb{R}^{12} \\
    \mathbf{u} = \mathbf{u}_{\mathrm{cmd}} \in \mathbb{R}^{12} \\
    \text{mode} \in \{0, 1, \ldots, 15\} \\
    t \in \mathbb{R}
    \end{cases}
    $$
    
    1. 주요코드
        
        [Ocs2QuadrupedController.cpp](https://www.notion.so/Ocs2QuadrupedController-cpp-34303cd821238032aafacfdc513aa40c?pvs=21)
        
        ‣ 
        
- 2. Gait Scheduler
    1. Overview
        
        전체 흐름은 다음과 같음
        
        $$
        \text{Gait Schedule Generation} \;\rightarrow\; \text{modifyReference} \;\rightarrow\;  \text{Reference Trajectory} \;\rightarrow\; MPC
        $$
        
    2. Gait 템플릿
        - Gait.info 구성
            
            $$
            \mathcal{G} = \{ G_0, G_1, \dots, G_{N_g} \}
            $$
            
        - 각 Gait template 구성
            
            $$
            G_i = \{ M_i, \mathcal{T}_i \}
            
            $$
            
            - 모드 sequence
            
            $$
            M_i = \{ m_i(0), m_i(1), \dots, m_i(n_i) \}
            $$
            
            - 모드 전환 시간
        
        $$
        \mathcal{T}_i = \{ t_i(0), t_i(1), \dots, t_i(n_i+1) \}
        $$
        
        - 예시 (Tort)
            
            0 ≤ t <0.3 → LF_RH
            0.3 ≤ t <0.6 → RF_LH
            
        
        $$
        G_{\mathrm{trot}} =
        \begin{cases}
        M_{\mathrm{trot}} = [\mathrm{LF\_RH}, \mathrm{RF\_LH}] \\
        \mathcal{T}_{\mathrm{trot}} = [0.0, 0.3, 0.6]
        \end{cases}
        $$
        
    3. Gait 커맨드
        
        $$
        \begin{array}{c|c|c|c}
        \text{Joystick Button} & u_{\text{cmd}} & i_{\text{gait}} & \text{Gait} \\
        \hline
        1 & 0 & - & \text{유지} \\
        2 & 2 & 0 & \text{Stance} \\
        3 & 3 & 1 & \text{Trot} \\
        4 & 4 & 2 & \text{Standing\_Trot} \\
        5 & 5 & 3 & \text{Flying\_Trot}
        \end{array}
        $$
        
    4. Gait schedule 갱신
        - 타이밍 관계
            
            MPC의 한 스텝에서 구간 설정은 다음과 같다.
            
        
        $$
        \textbf{MPC iteration } k:
        \begin{cases}
        t = \mathrm{initTime}^k \\
        T_{\mathrm{horizon}} = \mathrm{finalTime}^k - \mathrm{initTime}^k
        \end{cases}
        $$
        
        - 스케쥴 예시
            
            기존 gait 유지 → 타겟 gait
            
        
        $$
        \tilde{\mathbf{m}}_{\mathrm{schedule}} =\begin{bmatrix}\text{existing schedule} & [t_{\mathrm{now}},\; t_{\mathrm{start}}) \\G_k^{\mathrm{target}} & [t_{\mathrm{start}},\; t_{\mathrm{start}} + T] \\\text{future schedule} & [t_{\mathrm{start}} + T,\; \infty)\end{bmatrix}
        $$
        
    5. 레퍼런스 매니저
        - 매 iteration 마다 Mode Schedule로 Reference Trajectory 생성
            
            $$
            \tau_{\mathrm{ref}}(t)=\mathrm{modifyReference}\big(t,t+T,x,\text{targetTrajectory},\tilde{\mathbf{m}}_{\mathrm{schedule}}(t)\big)
            $$
            
    6. MPC 제어 입력
    7. 주요코드
        
        ‣ 
        
        [Gait.info](https://www.notion.so/Gait-info-34303cd82123807680b0c60d3fe5bc8b?pvs=21)
        
        ‣ 
        
- 3. Reference Manager
    1. Overview
        
        전체 흐름은 다음과 같음
        
        $$
        \text{Target Trajectory} \;\rightarrow\;\text{modifyReference} \;\rightarrow\;  \text{Reference Trajectory} \;\rightarrow\; MPC
        $$
        
    2. /cmd_vel 로 Reference 생성
    3. TargetTrajectories 구성
        - 시간 정의
            
            $$
            t^* = [t_k,\; t_k + T], \quad T = \text{timeHorizon}
            $$
            
        - 상태 궤적 정의
            
            $$
            \mathbf{x}^0 =
            \begin{bmatrix}
            \mathbf{v}_{\mathrm{ref}} \\
            \mathbf{0}_6 \\
            \mathbf{p}_{\mathrm{cur}} \\
            \boldsymbol{\theta}_{\mathrm{cur}} \\
            \mathbf{q}_{j,\mathrm{default}}
            \end{bmatrix},
            \quad
            \mathbf{x}^1 =
            \begin{bmatrix}
            \mathbf{v}_{\mathrm{ref}} \\
            \mathbf{0}_6 \\
            \mathbf{p}_{\mathrm{target}} \\
            \boldsymbol{\theta}_{\mathrm{target}} \\
            \mathbf{q}_{j,\mathrm{default}}
            \end{bmatrix}
            $$
            
        
        ```cpp
        TargetTrajectories{
          timeTrajectory = {observation.time, targetReachingTime},
          stateTrajectory = {x0, x1},
          inputTrajectory = {zero, zero}
        }
        ```
        
    4. 레퍼런스 매니저
        - 매 iteration 마다 Mode Schedule로 Reference Trajectory 생성
        
        $$
        \tau_{\mathrm{ref}}(t)=\mathrm{modifyReference}\big(t,t+T,x,\text{targetTrajectory},\tilde{\mathbf{m}}_{\mathrm{schedule}}(t)\big)
        $$
        
    5. 주요코드
        
        ‣ 
        
        ‣ 
        
- 4. Swing Trajectory Planner
    
    ‣ 
    
    ‣ 
    
- 5. MPC Solver
    
    SQPMPC
    
- 6. Whole-Body Controller
    - 1. Policy Evaluation
        - MPC 에서 (x*,u*,m*) 받음
        
        ```cpp
        // 최신 MPC 정책 가져오기
        ctrl_component_->mpc_mrt_interface_->updatePolicy();
        
        // 현재 시각에 맞는 최적 제어 계산
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(
        										    observation_.time,           // 현재 시각
        										    observation_.state,          // 현재 상태
        										    optimized_state_,           // 출력: 최적화된 상태
        										    optimized_input_,           // 출력: 최적화된 입력
        										    planned_mode);              // 출력: 예정된 모드
        ```
        
        - 정책
        
        $$
        \pi^*(t) = (x^*(t),\; u^*(t),\; m^*(t))
        $$
        
        - 출력
            
            $$
            (x_t^*,\; u_t^*,\; m_t^*)
            = \mathrm{evaluatePolicy}(\pi^*(t),\; t_{\text{obs}},\; x_{\text{obs}})
            $$
            
            $$
            x_t^* \in \mathbb{R}^{12}, \;\;
            
            u_t^* \in \mathbb{R}^{12}, \;\;
            
            m_t^* \in \{0,1,\dots,15\},
            $$
            
            x*_t= centroidal state, (position,velocity,pose,angular velocity)
            u*_t=Ground Reaction Force, (각 발마다 xyz 4개)
            m*_t=contact mode, (4개 발 총 16가지 경우의 수)
            
    - 2. WBC
        - 1. Measured Dynamics (측정 동역학 정보 계산)
            - Obeservation : RBD State → Centroidal State
            - QP Formulation 준비 단계
                
                ```cpp
                void updateMeasured(const vector_t& rbdStateMeasured)
                {
                    // RBD 상태 → Centroidal 형식 변환
                    q_measured_ = [position, orientation, joint_angles]ᵀ
                    v_measured_ = [linear_velocity, angular_velocity, joint_velocities]ᵀ
                    
                    // Forward Kinematics
                    forwardKinematics(model, data, q_measured_, v_measured_);
                    
                    // 질량행렬 M(q) 계산 (CRBA)
                    crba(model, data, q_measured_);  // → data.M
                    
                    // 비선형 항 계산 (RNEA)
                    nonLinearEffects(model, data, q_measured_, v_measured_);  // → data.nle
                    
                    // Jacobian 계산
                    computeJointJacobians(model, data);
                    for (size_t i = 0; i < numFeet; i++) {
                        getFrameJacobian(...) → j_[i]  // 발 위치 Jacobian
                        getFrameJacobianTimeVariation(...) → dj_[i]  // 시간 미분
                    }
                }
                ```
                
            
            $$
            q = q_{\text{obs}}, \quad v = v_{\text{obs}}
            $$
            
            $$
            (q, v) \rightarrow \big( M(q),\; n(q, v),\; J_i(q),\; \dot{J}_i(q, v) \big)
            $$
            
            $$
            M(q)\in \mathbb{R}^{6\times6},\; n(q,\dot{q})\in \mathbb{R}^6,\; J_i(q)\in \mathbb{R}^{3\times6},\; \dot{J}_i(q,\dot{q})\in \mathbb{R}^{3\times6}
            $$
            
        - 2. Desired Dynamics (목표 동역학 정보 계산)
            - MPC → (x*,u*,m*) → (q, qdot)
            - QP Formulation 준비 단계
                
                ```cpp
                void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired)
                {
                    // 최적화된 상태에서 관절 각도 추출
                    q_desired = mapping_.getPinocchioJointPosition(stateDesired);
                    
                    // Forward Kinematics (원하는 상태에서)
                    forwardKinematics(model, data, q_desired);
                    updateCentroidalDynamics(..., q_desired);
                }
                ```
                
            
            $$
            (x^*, u^*) \rightarrow \big(q^*_j, \dot{q}^*_j,p^*_{f,i},\dot{p}^*_{f,i},a^*_{b},F^*_{target})
            $$
            
            i = {1,2,3,4} 다리 , j ={1,2,3 …12} 조인트, f=발이라는 뜻
            
        - 3. QP Formulation
            - 1. Decision Variables
                
                $$
                \mathbf{x} = \begin{bmatrix}
                \ddot{\mathbf{p}}_b \
                \ddot{\boldsymbol{\theta}} \
                \mathbf{F} \
                \boldsymbol{\tau}
                \end{bmatrix} \in \mathbb{R}^{n}
                $$
                
                $$
                
                \ddot{p}_b \in \mathbb{R}^3, \;\;\;\;\ \text{베이스의 선형 가속도}
                $$
                
                $$
                \ddot{\theta} \in \mathbb{R}^3, \;\;\;\;\ \text{베이스의 각가속도}
                $$
                
                $$
                F = [f_1,\; f_2,\; f_3,\; f_4] \in \mathbb{R}^{12}
                , \\ \text{각 발의 Ground Reaction Force (4 × 3D 힘)
                }
                $$
                
                $$
                \tau \in \mathbb{R}^{12}
                , \;\;\;\;\  \text{12개 관절 토크}
                $$
                
                - 전체 크기 n = 3+3+12+12 = 30
            - 2. Constraints
                
                [WbcBase](https://www.notion.so/WbcBase-34403cd8212380108e69d11e78c1e36c?pvs=21) 
                
                1. Floating-Base EoM
                    
                    $$
                    M(q)
                    \begin{bmatrix}
                    \ddot{p}_b \\
                    \ddot{\theta} \\
                    \ddot{q}_j
                    \end{bmatrix}
                    - J^T F - S^T \tau
                    = -\,n(q,\dot{q})
                    $$
                    
                2. Torque Limits
                    
                    각 관절 토크 제한
                    
                    $$
                    -\tau_{\max} \le \tau_i \le \tau_{\max}, \quad \forall i \in \{1,\dots,12\}
                    $$
                    
                3. Friction Cone
                    1. 접촉 시
                        
                        $$
                        \begin{bmatrix}
                        0 & 0 & -1 \\
                        1 & 0 & -\mu \\
                        -1 & 0 & -\mu \\
                        0 & 1 & -\mu \\
                        0 & -1 & -\mu
                        \end{bmatrix}
                        f_i \le 0
                        \quad \text{(if } c_i = 1 \text{)}
                        $$
                        
                        $$
                        \|f_{i,xy}\| \le \mu f_{i,z}, \quad f_{i,z} \ge 0
                        $$
                        
                    2. 비접촉시
                        
                        $$
                        f_i = 0
                        \quad \text{(if } c_i = 0 \text{)}
                        $$
                        
                4. No Contact Motion
                    
                    접촉 발 가속도=0
                    
                    $$
                    a_i = J_i \ddot{x}_b + \dot{J}_i \dot{x}_b = 0 \quad \text{if } c_i = 1
                    $$
                    
            - 3. Cost Function
                1. Swing Leg Tracking
                    
                    Swing 다리 가속도
                    
                    $$
                    J_{\text{swing}} = \sum_{i:\,c_i=0} \| a_i^{\text{des}} - a_i \|^2
                    $$
                    
                2. Base Acceleration Tracking
                    
                    베이스 가속도를 MPC가 계획한 값에 맞춤
                    
                    $$
                    J_{\text{base}} = \| \ddot{p}_b^{\text{des}} - \ddot{p}_b \|^2
                    + \| \ddot{\theta}^{\text{des}} - \ddot{\theta} \|^2
                    $$
                    
                3. Contact Force Tracking
                    
                    contact 다리 중 GRF를 MPC 계획과 일치시키는 cost
                    
                    $$
                    J_{\text{force}} = \sum_{i=1}^{4} c_i \, \| f_i^{\text{des}} - f_i \|^2
                    
                    $$
                    
                4. Total QP Cost
                    
                    Cost 가중치 부여
                    
                    $$
                    J = \| A_{\text{task}} x - b_{\text{task}} \|^2
                    $$
                    
                    $$
                    A_i x \approx b_i
                    $$
                    
                    $$
                    \begin{bmatrix}W_{\text{swing}} A_{\text{swing}} \\W_{\text{base}} A_{\text{base}} \\W_{\text{force}} A_{\text{force}}\end{bmatrix}x\approx\begin{bmatrix}W_{\text{swing}} b_{\text{swing}} \\W_{\text{base}} b_{\text{base}} \\W_{\text{force}} b_{\text{force}}\end{bmatrix}
                    $$
                    
        - 4. QP Solving
            
            ```cpp
            vector_t WeightedWbc::update(...)
            {
                // 행렬 구성
                Eigen::Matrix<double, Dynamic, Dynamic, RowMajor> H = 
                    weighedTask.a_.transpose() * weighedTask.a_;
                vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;
                
                // 제약 구성
                matrix_t A = [constraints.a_; constraints.d_];
                vector_t lbA = [constraints.b_; -∞];
                vector_t ubA = [constraints.b_; constraints.f_];
                
                // qpOASES 해결
                qpOASES::QProblem qp(num_decision_vars, numConstraints);
                qp.init(H.data(), g.data(), A.data(), ..., lbA.data(), ubA.data(), nWsr);
                qp.getPrimalSolution(qpSol.data());
                return qpSol;
            }
            ```
            
            [WeightedWbc](https://www.notion.so/WeightedWbc-34403cd821238023b6f2c7c326b88e76?pvs=21) 
            
        - 5. Solution Extraction
            
            ```cpp
            vector_t x = wbc_->update(...);  // x = [ü; F; τ]ᵀ
            
            // 토크 추출 (마지막 12개 요소)
            vector_t torque = x.tail(12);
            
            // 목표 위치/속도 추출 (MPC 솔루션에서)
            vector_t pos_des = centroidal_model::getJointAngles(optimized_state_, info_);
            vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_, info_);
            ```
            
            - WBC Ouptput
                
                $$
                x^* =\begin{bmatrix}\ddot{p}_b^* \\\ddot{\theta}^* \\F^* \\\tau^*\end{bmatrix}
                $$
                
            - Torque값은 뒤에 12개만 추출
                
                $$
                \tau^* = x^*[\,n_x - 12 : n_x\,]
                $$
                
    - 3. Hardware Command
        - 각 조인트에 구한 토크값 넘겨줌
        
        [HardwareUnitree.cpp](https://www.notion.so/HardwareUnitree-cpp-34303cd821238083aec3eb4656b8861b?pvs=21)
        
        ```cpp
        for (int i = 0; i < 12; i++) {
            joint_torque_command_[i] = torque(i);
            joint_position_command_[i] = pos_des(i);
            joint_velocities_command_[i] = vel_des(i);
            joint_kp_command_[i] = default_kp_;  // 고정값
            joint_kd_command_[i] = default_kd_;  // 고정값
        }
        ```
        
    - 4. 주요코드
        
        [StateOCS2.cpp](https://www.notion.so/StateOCS2-cpp-34303cd8212380deb6fec6c03056b6ee?pvs=21)
        
        [WbcBase](https://www.notion.so/WbcBase-34403cd8212380108e69d11e78c1e36c?pvs=21) 
        
        [WeightedWbc](https://www.notion.so/WeightedWbc-34403cd821238023b6f2c7c326b88e76?pvs=21) 
        
    
    ```
    MPC Iteration k:
      ├─ t = t_k
      ├─ Observation ← State Estimation
      ├─ evaluatePolicy(t_k, x_obs) → x*, u*, m*
      ├─ WBC:
      │   ├─ contact_flag ← modeNumber2StanceLeg(m*)
      │   ├─ Measured Dynamics: q_obs, v_obs → J, dJ, M, nle
      │   ├─ Desired Dynamics: x*, u* → q_des, dq_des, pos_des, vel_des, base_accel_desired, Force_target
      │   ├─ QP Formulation:
      │   │   ├─ Constraints: EoM, τ_limit, friction, no-motion
      │   │   └─ Cost: swing tracking, base accel, force tracking
      │   ├─ Solve QP → x* = [ü*, F*, τ*]ᵀ
      │   └─ Extract τ* ← x*.tail(12)
      │
      ├─ Command Construction:
      │   ├─ q_cmd ← q_des
      │   ├─ dq_cmd ← dq_des
      │   ├─ τ_cmd ← τ*
      │   ├─ kp_cmd ← K_p
      │   └─ kd_cmd ← K_d
      │
      └─ Motor Control:
            ↓
      t = t_{k+1} (다음 사이클)
    ```
    

```
조종신호 (Joystick/cmd_vel)
    ↓
TargetManager::update()
    ├─ 목표 자세 생성: p_target, θ_target
    └─ TargetTrajectories → ReferenceManager
        ↓
ReferenceManager::preSolverRun()
    ├─ Mode schedule 가져오기
    ├─ SwingTrajectoryPlanner::update()
    │   └─ 발 높이 궤적 계산: z_ref(t), ż_ref(t)
    └─ Cost/Constraint 계수 설정
        ↓
MPC Solver (SqpMpc)
    ├─ 입력: 목표 궤적, 발 높이 제약, mode schedule
    ├─ 최적화: 비용 함수 최소화
    └─ 출력: optimized_state*, optimized_input*, mode*
        ↓
WbcBase::updateDesired(optimized_state, optimized_input)
    ├─ FK: Forward Kinematics
    ├─ 동역학 계산
    └─ 목표 관절력/가속도 계산
        ↓
WeightedWbc::update()
    ├─ 측정 동역학과 목표 동역학 비교
    ├─ QP 문제 구성 및 해결
    └─ 관절 토크 명령: τ*
        ↓
Hardware Interface
    └─ 모터에 q, dq, kp, kd, τ 전송
```