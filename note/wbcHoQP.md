# Summary

WBC task를 우선순위 계층대로 푸는 hierarchical quadratic program 구현 파일

- initVars
    
    현재 task의 equality, inequality 존재 여부 확인
    
    상위 문제가 있으면 이전 해, 이전 null-space, 이전 slack, 이전 task stack 가져옴
    
    없으면 identity null-space와 0 해로 시작
    
    현재 task와 이전 task를 이어붙여 stacked task 구성
    
- formulateProblem
    
    내부적으로 H, c, D, f 구성
    
- buildHMatrix
    
    equality task를 최소제곱 형태의 Hessian으로 만듦
    
    slack 변수에는 identity 가중 부여
    
    $$
    \begin{aligned}H &=\begin{bmatrix}Z^\top A^\top A Z & 0 \\0 & I\end{bmatrix}\end{aligned}
    $$
    
    $$
    \begin{aligned}A &= \text{current equality matrix}, \\Z &= \text{null-space basis from higher priority level}\end{aligned}
    $$
    
- buildCVector
    
    equality task residual 기반 선형항 구성
    
    $$
    \begin{aligned}\mathbf{c} =\begin{bmatrix}(AZ)^\top (A \mathbf{x}_{\mathrm{prev}} - \mathbf{b}) \\\mathbf{0}\end{bmatrix}\end{aligned}
    $$
    

- buildDMatrix
    
    inequality와 slack 관련 제약행렬 구성
    
    현재 level slack, 이전 level stacked slack, 현재 inequality를 한 번에 포함
    
    $$
    \begin{aligned}D =\begin{bmatrix}0 & -I \\D_{\mathrm{prev}} Z_{\mathrm{prev}} & 0 \\D Z_{\mathrm{prev}} & -I\end{bmatrix}\end{aligned}
    $$
    
- buildFVector
    
    inequality 우변 구성
    
    이전 계층 제약 위반량과 현재 계층 제약 위반량을 모두 반영
    
    $$
    \begin{aligned}\mathbf{f} =\begin{bmatrix}\mathbf{0} \\\mathbf{f}_{\mathrm{prev}} - D_{\mathrm{prev}} \mathbf{x}_{\mathrm{prev}} + \mathbf{s}_{\mathrm{prev}} \\\mathbf{f} - D \mathbf{x}_{\mathrm{prev}}\end{bmatrix}\end{aligned}
    $$
    
- buildZMatrix
    
    equality task가 있으면 현재 task의 null-space 계산
    
    없으면 이전 null-space 그대로 유지
    
    $$
    \begin{aligned}Z_{\mathrm{new}} = Z_{\mathrm{prev}} \, \mathrm{null}\!\left(A Z_{\mathrm{prev}}\right)\end{aligned}
    $$
    
- solveProblem
    
    qpOASES로 현재 level QP 풀이
    
    해를 decision variable 부분과 slack 부분으로 분리
    
    $$
    \begin{aligned}\mathbf{z}_{\mathrm{qp}} =\begin{bmatrix}\Delta \mathbf{y} \\\mathbf{s}\end{bmatrix}\end{aligned}
    $$
    
- stackSlackSolutions
    
    상위 계층 slack과 현재 계층 slack 이어붙임
    
- getSolutions
    
    최종 해 복원
    
    상위 해에 null-space 방향 보정량을 더함
    
    $$
    \begin{aligned}\mathbf{x} = \mathbf{x}_{\mathrm{prev}} + Z_{\mathrm{prev}} \, \Delta \mathbf{y}\end{aligned}
    $$