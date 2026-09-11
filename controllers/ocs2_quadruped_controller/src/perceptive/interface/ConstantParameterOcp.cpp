#include "ocs2_quadruped_controller/perceptive/interface/ConstantParameterOcp.h"
#include <stdexcept>

namespace ocs2::legged_robot {
namespace {
class ParameterPreComputation final : public PreComputation {
 public:
    ParameterPreComputation(const PreComputation& source, size_t n) : inner(source.clone()), n_(n) {}
    ParameterPreComputation(const ParameterPreComputation& rhs) : ParameterPreComputation(*rhs.inner, rhs.n_) {}
    ParameterPreComputation* clone() const override { return new ParameterPreComputation(*this); }
    void request(RequestSet r, scalar_t t, const vector_t& x, const vector_t& u) override { inner->request(r,t,x.head(n_),u); }
    void requestPreJump(RequestSet r, scalar_t t, const vector_t& x) override { inner->requestPreJump(r,t,x.head(n_)); }
    void requestFinal(RequestSet r, scalar_t t, const vector_t& x) override { inner->requestFinal(r,t,x.head(n_)); }
    std::unique_ptr<PreComputation> inner;
 private:
    size_t n_;
};
const PreComputation& physicalPreComputation(const PreComputation& pc) {
    return *cast<ParameterPreComputation>(pc).inner;
}
ScalarFunctionQuadraticApproximation extendCost(const ScalarFunctionQuadraticApproximation& a, size_t nx) {
    auto b = ScalarFunctionQuadraticApproximation::Zero(nx, a.dfdu.size());
    b.f=a.f; b.dfdx.head(a.dfdx.size())=a.dfdx; b.dfdu=a.dfdu;
    b.dfdxx.topLeftCorner(a.dfdxx.rows(),a.dfdxx.cols())=a.dfdxx;
    b.dfdux.leftCols(a.dfdux.cols())=a.dfdux; b.dfduu=a.dfduu;
    return b;
}
VectorFunctionLinearApproximation extendJacobian(const VectorFunctionLinearApproximation& a, size_t nx) {
    auto b=VectorFunctionLinearApproximation::Zero(a.f.size(),nx,a.dfdu.cols());
    b.f=a.f; b.dfdx.leftCols(a.dfdx.cols())=a.dfdx; b.dfdu=a.dfdu;
    return b;
}
class PhysicalStateCost final : public StateCost {
 public:
    PhysicalStateCost(std::unique_ptr<StateCost> source, size_t n) : inner_(std::move(source)), n_(n) {}
    PhysicalStateCost(const PhysicalStateCost& rhs) : PhysicalStateCost(std::unique_ptr<StateCost>(rhs.inner_->clone()),rhs.n_) {}
    PhysicalStateCost* clone() const override { return new PhysicalStateCost(*this); }
    bool isActive(scalar_t t) const override { return inner_->isActive(t); }
    scalar_t getValue(scalar_t t, const vector_t& x, const TargetTrajectories& target, const PreComputation& pc) const override {
        return inner_->getValue(t,x.head(n_),target,physicalPreComputation(pc));
    }
    ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const TargetTrajectories& target, const PreComputation& pc) const override {
        return extendCost(inner_->getQuadraticApproximation(t,x.head(n_),target,physicalPreComputation(pc)),x.size());
    }
 private:
    std::unique_ptr<StateCost> inner_;
    size_t n_;
};
class PhysicalStateConstraint final : public StateConstraint {
 public:
    PhysicalStateConstraint(std::unique_ptr<StateConstraint> source, size_t n) : StateConstraint(ConstraintOrder::Linear),inner_(std::move(source)),n_(n) {}
    PhysicalStateConstraint(const PhysicalStateConstraint& rhs) : PhysicalStateConstraint(std::unique_ptr<StateConstraint>(rhs.inner_->clone()),rhs.n_) {}
    PhysicalStateConstraint* clone() const override { return new PhysicalStateConstraint(*this); }
    bool isActive(scalar_t t) const override { return inner_->isActive(t); }
    size_t getNumConstraints(scalar_t t) const override { return inner_->getNumConstraints(t); }
    vector_t getValue(scalar_t t, const vector_t& x, const PreComputation& pc) const override {
        return inner_->getValue(t,x.head(n_),physicalPreComputation(pc));
    }
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const PreComputation& pc) const override {
        return extendJacobian(inner_->getLinearApproximation(t,x.head(n_),physicalPreComputation(pc)),x.size());
    }
 private:
    std::unique_ptr<StateConstraint> inner_;
    size_t n_;
};
class PhysicalStateInputCost final : public StateInputCost {
 public:
    PhysicalStateInputCost(std::unique_ptr<StateInputCost> source, size_t n) : inner_(std::move(source)), n_(n) {}
    PhysicalStateInputCost(const PhysicalStateInputCost& rhs) : PhysicalStateInputCost(std::unique_ptr<StateInputCost>(rhs.inner_->clone()),rhs.n_) {}
    PhysicalStateInputCost* clone() const override { return new PhysicalStateInputCost(*this); }
    bool isActive(scalar_t t) const override { return inner_->isActive(t); }
    scalar_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& target, const PreComputation& pc) const override {
        return inner_->getValue(t,x.head(n_),u,target,physicalPreComputation(pc));
    }
    ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t t, const vector_t& x, const vector_t& u, const TargetTrajectories& target, const PreComputation& pc) const override {
        return extendCost(inner_->getQuadraticApproximation(t,x.head(n_),u,target,physicalPreComputation(pc)),x.size());
    }
 private:
    std::unique_ptr<StateInputCost> inner_;
    size_t n_;
};
class PhysicalStateInputConstraint final : public StateInputConstraint {
 public:
    PhysicalStateInputConstraint(std::unique_ptr<StateInputConstraint> source, size_t n) : StateInputConstraint(ConstraintOrder::Linear),inner_(std::move(source)),n_(n) {}
    PhysicalStateInputConstraint(const PhysicalStateInputConstraint& rhs) : PhysicalStateInputConstraint(std::unique_ptr<StateInputConstraint>(rhs.inner_->clone()),rhs.n_) {}
    PhysicalStateInputConstraint* clone() const override { return new PhysicalStateInputConstraint(*this); }
    bool isActive(scalar_t t) const override { return inner_->isActive(t); }
    size_t getNumConstraints(scalar_t t) const override { return inner_->getNumConstraints(t); }
    vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& pc) const override {
        return inner_->getValue(t,x.head(n_),u,physicalPreComputation(pc));
    }
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& pc) const override {
        return extendJacobian(inner_->getLinearApproximation(t,x.head(n_),u,physicalPreComputation(pc)),x.size());
    }
 private:
    std::unique_ptr<StateInputConstraint> inner_;
    size_t n_;
};
// Preserve collection names and term ordering; cloning still clones each wrapped term.
template <typename Base, typename Wrapper>
class PhysicalCollection final : public Base {
 public:
    PhysicalCollection(const Base& source, size_t n) : Base(source) {
        for(auto& term : this->terms_) term=std::make_unique<Wrapper>(std::move(term),n);
    }
};
class ParameterDynamics final : public SystemDynamicsBase {
 public:
    ParameterDynamics(const SystemDynamicsBase& source, const PreComputation& pc, size_t n, size_t p)
        : SystemDynamicsBase(ParameterPreComputation(pc,n)),inner_(source.clone()),n_(n),p_(p) {}
    ParameterDynamics(const ParameterDynamics& rhs) : SystemDynamicsBase(rhs),inner_(rhs.inner_->clone()),n_(rhs.n_),p_(rhs.p_) {}
    ParameterDynamics* clone() const override { return new ParameterDynamics(*this); }
    vector_t computeFlowMap(scalar_t t,const vector_t& x,const vector_t& u,const PreComputation& pc) override {
        vector_t dx=vector_t::Zero(n_+p_);
        dx.head(n_)=inner_->computeFlowMap(t,x.head(n_),u,physicalPreComputation(pc));
        return dx;
    }
    vector_t computeJumpMap(scalar_t t,const vector_t& x,const PreComputation& pc) override {
        vector_t next=x;
        next.head(n_)=inner_->computeJumpMap(t,x.head(n_),physicalPreComputation(pc));
        return next;
    }
    VectorFunctionLinearApproximation linearApproximation(scalar_t t,const vector_t& x,const vector_t& u,const PreComputation& pc) override {
        const auto a=inner_->linearApproximation(t,x.head(n_),u,physicalPreComputation(pc));
        auto b=VectorFunctionLinearApproximation::Zero(n_+p_,n_+p_,u.size());
        b.f.head(n_)=a.f; b.dfdx.topLeftCorner(n_,n_)=a.dfdx; b.dfdu.topRows(n_)=a.dfdu;
        return b;
    }
    VectorFunctionLinearApproximation jumpMapLinearApproximation(scalar_t t,const vector_t& x,const PreComputation& pc) override {
        const auto a=inner_->jumpMapLinearApproximation(t,x.head(n_),physicalPreComputation(pc));
        auto b=VectorFunctionLinearApproximation::Zero(n_+p_,n_+p_,0);
        b.f=x; b.f.head(n_)=a.f; b.dfdx.topLeftCorner(n_,n_)=a.dfdx;
        b.dfdx.bottomRightCorner(p_,p_).setIdentity();
        return b;
    }
 private:
    std::unique_ptr<SystemDynamicsBase> inner_;
    size_t n_,p_;
};
class ParameterInitializer final : public Initializer {
 public:
    ParameterInitializer(const Initializer& source,size_t n) : inner_(source.clone()),n_(n) {}
    ParameterInitializer(const ParameterInitializer& rhs) : ParameterInitializer(*rhs.inner_,rhs.n_) {}
    ParameterInitializer* clone() const override { return new ParameterInitializer(*this); }
    void compute(scalar_t t,const vector_t& x,scalar_t nextTime,vector_t& u,vector_t& next) override {
        vector_t physicalNext;
        inner_->compute(t,x.head(n_),nextTime,u,physicalNext);
        next=x; next.head(n_)=physicalNext;
    }
 private:
    std::unique_ptr<Initializer> inner_;
    size_t n_;
};
class ParameterKinematics final : public EndEffectorKinematics<scalar_t> {
 public:
    ParameterKinematics(const EndEffectorKinematics<scalar_t>& source,size_t n) : inner_(source.clone()),n_(n) {}
    ParameterKinematics(const ParameterKinematics& rhs) : ParameterKinematics(*rhs.inner_,rhs.n_) {}
    ParameterKinematics* clone() const override { return new ParameterKinematics(*this); }
    const std::vector<std::string>& getIds() const override { return inner_->getIds(); }
    std::vector<vector3_t> getPosition(const vector_t& x) const override { return inner_->getPosition(x.head(n_)); }
    std::vector<vector3_t> getVelocity(const vector_t& x,const vector_t& u) const override { return inner_->getVelocity(x.head(n_),u); }
    std::vector<vector3_t> getOrientationError(const vector_t& x,const std::vector<quaternion_t>& q) const override { return inner_->getOrientationError(x.head(n_),q); }
    std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& x) const override {
        auto a=inner_->getPositionLinearApproximation(x.head(n_));
        for(auto& item:a) item=extendJacobian(item,x.size());
        return a;
    }
    std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& x,const vector_t& u) const override {
        auto a=inner_->getVelocityLinearApproximation(x.head(n_),u);
        for(auto& item:a) item=extendJacobian(item,x.size());
        return a;
    }
    std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(const vector_t& x,const std::vector<quaternion_t>& q) const override {
        auto a=inner_->getOrientationErrorLinearApproximation(x.head(n_),q);
        for(auto& item:a) item=extendJacobian(item,x.size());
        return a;
    }
 private:
    std::unique_ptr<EndEffectorKinematics<scalar_t>> inner_;
    size_t n_;
};
}  // namespace

void appendConstantParameters(OptimalControlProblem& problem,size_t n,size_t p) {
    if (!problem.equalityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.stateEqualityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.inequalityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.stateInequalityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.preJumpEqualityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.preJumpInequalityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.finalEqualityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    if (!problem.finalInequalityLagrangianPtr->empty()) throw std::invalid_argument("Constant parameters: augmented Lagrangians are unsupported");
    problem.costPtr=std::make_unique<PhysicalCollection<StateInputCostCollection,PhysicalStateInputCost>>(*problem.costPtr,n);
    problem.softConstraintPtr=std::make_unique<PhysicalCollection<StateInputCostCollection,PhysicalStateInputCost>>(*problem.softConstraintPtr,n);
    problem.stateCostPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.stateCostPtr,n);
    problem.preJumpCostPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.preJumpCostPtr,n);
    problem.finalCostPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.finalCostPtr,n);
    problem.stateSoftConstraintPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.stateSoftConstraintPtr,n);
    problem.preJumpSoftConstraintPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.preJumpSoftConstraintPtr,n);
    problem.finalSoftConstraintPtr=std::make_unique<PhysicalCollection<StateCostCollection,PhysicalStateCost>>(*problem.finalSoftConstraintPtr,n);
    problem.equalityConstraintPtr=std::make_unique<PhysicalCollection<StateInputConstraintCollection,PhysicalStateInputConstraint>>(*problem.equalityConstraintPtr,n);
    problem.inequalityConstraintPtr=std::make_unique<PhysicalCollection<StateInputConstraintCollection,PhysicalStateInputConstraint>>(*problem.inequalityConstraintPtr,n);
    problem.stateEqualityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.stateEqualityConstraintPtr,n);
    problem.preJumpEqualityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.preJumpEqualityConstraintPtr,n);
    problem.finalEqualityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.finalEqualityConstraintPtr,n);
    problem.stateInequalityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.stateInequalityConstraintPtr,n);
    problem.preJumpInequalityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.preJumpInequalityConstraintPtr,n);
    problem.finalInequalityConstraintPtr=std::make_unique<PhysicalCollection<StateConstraintCollection,PhysicalStateConstraint>>(*problem.finalInequalityConstraintPtr,n);
    problem.dynamicsPtr=std::make_unique<ParameterDynamics>(*problem.dynamicsPtr,*problem.preComputationPtr,n,p);
    problem.preComputationPtr=std::make_unique<ParameterPreComputation>(*problem.preComputationPtr,n);
}
std::unique_ptr<Initializer> parameterInitializer(const Initializer& physical,size_t n) {
    return std::make_unique<ParameterInitializer>(physical,n);
}
std::unique_ptr<EndEffectorKinematics<scalar_t>> parameterKinematics(const EndEffectorKinematics<scalar_t>& physical,size_t n) {
    return std::make_unique<ParameterKinematics>(physical,n);
}
}  // namespace ocs2::legged_robot
