# Summary

hard constraint는 반드시 만족시키고 swing leg tracking, base acceleration tracking, contact force tracking은 가중 least-squares로 절충해서 최종 WBC 해를 qpOASES로 구함

- update
    
    먼저 WbcBase::update 호출
    
    그 다음 hard constraint task와 weighted task를 구성
    
    이를 QP 형태로 바꿔 qpOASES로 풂
    
    최종 primal solution 반환
    
    제약은 equality와 inequality를 한 행렬로 합친다
    
    $$
    \begin{aligned}A &=\begin{bmatrix}A_{\mathrm{eq}} \\A_{\mathrm{ineq}}\end{bmatrix}, \\\ell_A &=\begin{bmatrix}\mathbf{b}_{\mathrm{eq}} \\-\infty\end{bmatrix}, \\u_A &=\begin{bmatrix}\mathbf{b}_{\mathrm{eq}} \\\mathbf{f}_{\mathrm{ineq}}\end{bmatrix}\end{aligned}
    $$
    
    cost는 weighted task의 least-squares를 QP 표준형으로
    
    $$
    \begin{aligned}\min_{\mathbf{z}} \; \frac{1}{2}\mathbf{z}^\top H \mathbf{z} + \mathbf{g}^\top \mathbf{z}, \\H &= A_w^\top A_w, \\\mathbf{g} &= -A_w^\top \mathbf{b}_w\end{aligned}
    $$
    
    $$
    \begin{aligned}\mathbf{z} &= \text{WBC decision variable}, \\A_w \mathbf{z} \approx \mathbf{b}_w &= \text{weighted task stack}\end{aligned}
    $$
    
- formulateConstraints
    
    hard constraint task들을 하나로 합침
    
    $$
    \begin{aligned}\mathcal{T}_{\mathrm{con}} =\mathcal{T}_{\mathrm{fbEom}}+ \mathcal{T}_{\mathrm{torqueLimit}}+ \mathcal{T}_{\mathrm{friction}}+ \mathcal{T}_{\mathrm{noContactMotion}}\end{aligned}
    $$
    

- formulateWeightedTasks
    
    soft하게 맞추고 싶은 task들을 가중합으로 묶음
    
    $$
    \begin{aligned}\mathcal{T}_{w}=w_{\mathrm{swing}} \, \mathcal{T}_{\mathrm{swingLeg}}+w_{\mathrm{base}} \, \mathcal{T}_{\mathrm{baseAccel}}+w_{\mathrm{force}} \, \mathcal{T}_{\mathrm{contactForce}}\end{aligned}
    $$