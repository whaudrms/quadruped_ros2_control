# Summary

WBC task를 WeightedWbc처럼 가중합으로 절충하지 않고 우선순위 계층으로 나눠서 HoQP로 푸는 파일

- update
    
    먼저 WbcBase::update 호출
    
    그 다음 task를 3단계 우선순위로 나눔
    
    마지막에 HoQp를 계층적으로 구성해서 해를 구함
    
    $$
    \begin{aligned}\mathcal{T}_0 &=\mathcal{T}_{\mathrm{fbEom}}+ \mathcal{T}_{\mathrm{torqueLimit}}+ \mathcal{T}_{\mathrm{friction}}+ \mathcal{T}_{\mathrm{noContactMotion}}, \\\mathcal{T}_1 &=\mathcal{T}_{\mathrm{baseAccel}}+ \mathcal{T}_{\mathrm{swingLeg}}, \\\mathcal{T}_2 &=\mathcal{T}_{\mathrm{contactForce}}\end{aligned}
    $$
    
    우선순위 구조
    
    $$
    \begin{aligned}\mathcal{T}_0 \succ \mathcal{T}_1 \succ \mathcal{T}_2\end{aligned}
    $$
    
    계층적 QP 해로 구한 최종해
    
    $$
    \begin{aligned}\mathbf{z}^\star = \mathrm{HoQp}(\mathcal{T}_2,\; \mathrm{HoQp}(\mathcal{T}_1,\; \mathrm{HoQp}(\mathcal{T}_0)))\end{aligned}
    $$