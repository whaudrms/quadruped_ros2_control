# Summary

동역학, 토크 제한, 접촉 조건, 스윙 발 tracking, 접촉력 tracking 같은 WBC task를 공통 형식으로 구성해서 상위 WeightedWbc나 HierarchicalWbc가 실제 최적화를 풀 수 있게 함

- measured/desired Pinocchio interface, centroidal model 정보, end-effector kinematics 저장
    
    decision variable 개수 계산
    
    measured joint state 버퍼와 이전 입력 버퍼 초기화
    
    decision variable 구성
    
    $$
    \begin{aligned}\mathbf{z} =\begin{bmatrix}\ddot{\mathbf{u}} \\\mathbf{F} \\\boldsymbol{\tau}\end{bmatrix}\end{aligned}
    $$
    

- update
    
    현재 mode에서 contact flag 계산
    
    접촉 발 수 계산
    
    measured state 갱신
    
    desired state 갱신
    
    기본 클래스에서는 최종 해를 만들지 않고 빈 벡터 반환
    
- updateMeasured
    
    측정된 rbdState를 Pinocchio 좌표계의 q_measured, v_measured로 변환
    
    그 다음 Jacobian, 질량행렬, nonlinear effect, Jacobian time derivative 계산
    
    상태 재배치 구성
    
    $$
    \begin{aligned}\mathbf{q}_{\mathrm{meas}} &=\begin{bmatrix}\mathbf{p}_{\mathrm{base}} \\\mathbf{rpy}_{\mathrm{base}} \\\mathbf{q}_j\end{bmatrix}, \\\mathbf{v}_{\mathrm{meas}} &=\begin{bmatrix}\mathbf{v}_{\mathrm{base}} \\\dot{\mathbf{rpy}}_{\mathrm{base}} \\\dot{\mathbf{q}}_j\end{bmatrix}\end{aligned}
    $$
    
    발 Jacobian과 시간미분 Jacobian
    
    $$
    \begin{aligned}J =\begin{bmatrix}J_1 \\J_2 \\\vdots \\J_n\end{bmatrix},\qquad\dot{J} =\begin{bmatrix}\dot{J}_1 \\\dot{J}_2 \\\vdots \\\dot{J}_n\end{bmatrix}\end{aligned}
    $$
    

- updateDesired
    
    desired state, desired input을 Pinocchio 좌표로 변환
    
    desired model 기준 forward kinematics, Jacobian, centroidal dynamics 갱신
    
- formulateFloatingBaseEomTask
    
    floating-base dynamics equality task 구성
    
    $$
    \begin{aligned}M \ddot{\mathbf{u}} - J^\top \mathbf{F} - S^\top \boldsymbol{\tau} = -\mathbf{n}\end{aligned}
    $$
    
    $$
    \begin{aligned}M &= \text{mass matrix}, \\J &= \text{contact Jacobian}, \\\mathbf{F} &= \text{contact forces}, \\S &= \text{actuation selection matrix}, \\\boldsymbol{\tau} &= \text{joint torques}, \\\mathbf{n} &= \text{nonlinear effects}\end{aligned}
    $$
    

 

- formulateTorqueLimitsTask
    
    토크 상한/하한 부등식 task 구성
    
    $$
    \begin{aligned}-\boldsymbol{\tau}_{\max} \le \boldsymbol{\tau} \le \boldsymbol{\tau}_{\max}\end{aligned}
    $$
    

- formulateNoContactMotionTask
    
    접촉 발에 대해 발 가속도 0 조건 구성
    
    $$
    \begin{aligned}J_c \ddot{\mathbf{u}} = -\dot{J}_c \mathbf{v}\end{aligned}
    $$
    
- formulateFrictionConeTask
    
    비접촉 발은 접촉력을 0으로 두고 접촉 발은 friction pyramid 부등식 적용
    
    $$
    \begin{aligned}-f_z &\le 0, \\f_x - \mu f_z &\le 0, \\-f_x - \mu f_z &\le 0, \\f_y - \mu f_z &\le 0, \\-f_y - \mu f_z &\le 0\end{aligned}
    $$
    

- formulateBaseAccelTask
    
    desired centroidal momentum rate에서 floating base 가속도를 구하는 task 구성
    
    먼저 joint acceleration 근사
    
    $$
    \begin{aligned}\ddot{\mathbf{q}}_j \approx \frac{\dot{\mathbf{q}}_j^{\mathrm{des}} - \dot{\mathbf{q}}_j^{\mathrm{last}}}{\Delta t}\end{aligned}
    $$
    

- centroidal momentum matrix를
    
    A=[Ab  Aj]로 두면 base acceleration 
    
    $$
    \begin{aligned}\dot{\mathbf{h}} &= m \, \dot{\mathbf{h}}_{\mathrm{norm}}, \\\mathbf{b} &= A_b^{-1} \left( \dot{\mathbf{h}} - \dot{A}\mathbf{v} - A_j \ddot{\mathbf{q}}_j \right)\end{aligned}
    $$
    
- formulateSwingLegTask
    
    스윙 발에 대해 PD 기반 목표 가속도 task 구성
    
    $$
    \begin{aligned}\mathbf{a}_{\mathrm{swing}} &=k_p \left( \mathbf{p}^{\mathrm{des}} - \mathbf{p}^{\mathrm{meas}} \right)+k_d \left( \mathbf{v}^{\mathrm{des}} - \mathbf{v}^{\mathrm{meas}} \right), \\J_s \ddot{\mathbf{u}} &= \mathbf{a}_{\mathrm{swing}} - \dot{J}_s \mathbf{v}\end{aligned}
    $$
    
- formulateContactForceTask
    
    desired input의 contact force 부분을 그대로 추종하는 task 구성
    
    $$
    \begin{aligned}\mathbf{F} = \mathbf{F}^{\mathrm{des}}\end{aligned}
    $$