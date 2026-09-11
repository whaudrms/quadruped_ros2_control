#include <cmath>
#include <iostream>
#include <stdexcept>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/test/testProblemsGeneration.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_core/penalties/penalties/QuadraticPenalty.h>
#include <ocs2_sqp/SqpSolver.h>
#include "ocs2_quadruped_controller/perceptive/interface/ConstantParameterOcp.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustGuardBoundaryConstraint.h"
#include "ocs2_quadruped_controller/perceptive/constraint/RobustWidthBounds.h"
#include "ocs2_quadruped_controller/perceptive/cost/RobustWidthReward.h"

using namespace ocs2;
using namespace ocs2::legged_robot;
namespace {
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
class Feet final : public EndEffectorKinematics<scalar_t> {
 public:
    Feet* clone() const override { return new Feet(*this); }
    const std::vector<std::string>& getIds() const override { return ids_; }
    std::vector<vector3_t> getPosition(const vector_t& x) const override {
        require(x.size()==24,"kinematics received auxiliary states"); return {x.head<3>()};
    }
    std::vector<vector3_t> getVelocity(const vector_t& x,const vector_t& u) const override {
        require(x.size()==24 && u.size()==24,"velocity dimensions"); return {u.head<3>()};
    }
    std::vector<vector3_t> getOrientationError(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& x) const override {
        auto a=VectorFunctionLinearApproximation::Zero(3,24,0);
        a.f=getPosition(x).front(); a.dfdx.leftCols(3).setIdentity(); return {a};
    }
    std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& x,const vector_t& u) const override {
        auto a=VectorFunctionLinearApproximation::Zero(3,24,24);
        a.f=getVelocity(x,u).front(); a.dfdu.leftCols(3).setIdentity(); return {a};
    }
    std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
 private:
    std::vector<std::string> ids_{"FL"};
};
class Manager final : public PerceptiveLeggedReferenceManager {
 public:
    explicit Manager(const Feet& feet) : PerceptiveLeggedReferenceManager({},nullptr,nullptr,nullptr,feet,0.0) {}
    void setWindow(const RobustWindowData& window) { robustWindows_[0]=window; }
 protected:
    void modifyReferences(scalar_t,scalar_t,const vector_t&,TargetTrajectories&,ModeSchedule& modes) override {
        modes=ModeSchedule({0.075}, {15, 15});  // exercise constant-parameter jump map
    }
};
}
int main() {
    Feet feet;
    auto manager=std::make_shared<Manager>(feet);
    PerceptiveLeggedReferenceManager::RobustPhaseSettings settings;
    settings.enabled=true; settings.optimize_d=true;
    auto fixedSettings=settings;
    fixedSettings.d_min=fixedSettings.d_max;
    manager->setRobustPhaseSettings(fixedSettings);
    require(!manager->getRobustPhaseSettings().optimize_d,"equal bounds did not select fixed width");
    manager->setRobustPhaseSettings(settings);
    RobustWindowData window;
    window.active=true; window.t_a=0.0; window.t_b=0.1; window.dt_mpc=0.05;
    window.d=0.05; window.d_state_index=24; window.v_max=1.0;
    manager->setWindow(window);
    auto kinematics=parameterKinematics(feet,24);
    PreComputation pc;
    vector_t state=vector_t::Zero(28); state.tail(4).setConstant(0.035);
    for (auto formulation : {RobustGuardBoundaryConstraint::Formulation::EqualityResidual,
                             RobustGuardBoundaryConstraint::Formulation::TraversalInequality}) {
        RobustGuardBoundaryConstraint boundary(*manager,*kinematics,0,formulation);
        for (double time : {0.0,0.1}) {
            const auto a=boundary.getLinearApproximation(time,state,pc);
            for (int j=0;j<28;++j) {
                vector_t plus=state,minus=state; plus(j)+=1e-6; minus(j)-=1e-6;
                const vector_t fd=(boundary.getValue(time,plus,pc)-boundary.getValue(time,minus,pc))/2e-6;
                require((fd-a.dfdx.col(j)).norm()<1e-8,"boundary Jacobian mismatch");
            }
        }
    }
    const TargetTrajectories targets;
    RobustWidthReward reward(*manager,0,10.0);
    require(reward.isActive(0.0) && reward.isActive(0.05),"reward missing inside window");
    require(!reward.isActive(-0.01) && !reward.isActive(0.1),"reward outside window");
    auto rewardApprox=reward.getQuadraticApproximation(0.05,state,targets,pc);
    require(std::abs(rewardApprox.f+0.35)<1e-12,"reward must be linear -w_d*d_i");
    require(rewardApprox.dfdxx.norm()==0.0,"linear reward has nonzero Hessian");
    for (int j=0;j<28;++j) {
        vector_t plus=state,minus=state; plus(j)+=1e-6; minus(j)-=1e-6;
        const auto fd=(reward.getValue(0.05,plus,targets,pc)-reward.getValue(0.05,minus,targets,pc))/2e-6;
        require(std::abs(fd-rewardApprox.dfdx(j))<1e-8,"reward gradient mismatch");
    }
    RobustWidthReward disabled(*manager,0,0.0);
    require(!disabled.isActive(0.05) && disabled.getValue(0.05,state,targets,pc)==0.0,"zero weight not disabled");
    window.active=false;  // Same invalidation performed after a contact splice.
    manager->setWindow(window);
    require(!reward.isActive(0.05) && reward.getValue(0.05,state,targets,pc)==0.0,"removed window rewarded");
    window.active=true; window.d_state_index=-1;
    manager->setWindow(window);
    require(!reward.isActive(0.05) && reward.getValue(0.05,state.head(24),targets,pc)==0.0,"fixed width rewarded");
    window.d_state_index=24;
    manager->setWindow(window);
    for (const auto weight : {-1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        bool rejected=false;
        try { RobustWidthReward invalid(*manager,0,weight); } catch (const std::invalid_argument&) { rejected=true; }
        require(rejected,"invalid reward weight accepted");
    }
    // The complete wrapped OCP exercises physical costs, dynamics, initialization,
    // robust boundary costs, hard bounds and policy feedback.
    OptimalControlProblem problem;
    auto dynamics=VectorFunctionLinearApproximation::Zero(24,24,24);
    dynamics.dfdu.setIdentity(); problem.dynamicsPtr=getOcs2Dynamics(dynamics);
    auto cost=ScalarFunctionQuadraticApproximation::Zero(24,24);
    cost.dfdxx.setIdentity(); cost.dfduu.setIdentity();
    problem.costPtr->add("physical",getOcs2Cost(cost));
    problem.finalCostPtr->add("physical",getOcs2StateCost(cost));
    auto equality=VectorFunctionLinearApproximation::Zero(1,24,24);
    equality.dfdx(0,23)=1.0; equality.dfdu(0,23)=1.0;
    problem.equalityConstraintPtr->add("physicalEquality",getOcs2Constraints(equality));
    appendConstantParameters(problem,24,4);
    DefaultInitializer physicalInitializer(24);
    auto initializer=parameterInitializer(physicalInitializer,24);
    problem.stateSoftConstraintPtr->add("boundary",std::make_unique<StateSoftConstraint>(
        std::make_unique<RobustGuardBoundaryConstraint>(*manager,*kinematics,0),std::make_unique<QuadraticPenalty>(400)));
    problem.stateInequalityConstraintPtr->add("width",std::make_unique<RobustWidthBounds>(*manager,24));
    problem.finalInequalityConstraintPtr->add("width",std::make_unique<RobustWidthBounds>(*manager,24));
    manager->setTargetTrajectories(TargetTrajectories({0.0},{vector_t::Zero(24)},{vector_t::Zero(24)}));
    sqp::Settings solverSettings; solverSettings.dt=0.05; solverSettings.nThreads=1;
    solverSettings.sqpIteration=10; solverSettings.useFeedbackPolicy=true;
    solverSettings.printSolverStatus=true; solverSettings.printLinesearch=true;
    for (const double weight : {0.0,10.0,1000.0}) {
        auto rewardedProblem=problem;
        rewardedProblem.stateCostPtr->add("widthReward",std::make_unique<RobustWidthReward>(*manager,0,weight));
        SqpSolver solver(solverSettings,rewardedProblem,*initializer);
        solver.setReferenceManager(manager); solver.setNumFreeInitialStates(4);
        state.tail(4).setConstant(0.05);
        solver.run(0.0,state,0.2);
        const auto solution=solver.primalSolution(0.2);
        require(solution.stateTrajectory_.front().head(24).norm()<1e-7,"physical initial state changed");
        for (const auto& x:solution.stateTrajectory_) {
            require(x.size()==28,"MPC state dimension");
            if (weight==0.0) require(std::abs(x(24)-0.02)<2e-5,"zero-reward regression");
            else require(x(24)>0.021,"reward did not increase optimized width");
            if (weight==1000.0) require(std::abs(x(24)-0.05)<2e-5,"large reward did not select upper bound");
            require((x.tail(4).array() >= 0.02-1e-7).all() && (x.tail(4).array() <= 0.05+1e-7).all(),
                    "width escaped bounds");
            require((x.tail(4)-solution.stateTrajectory_.front().tail(4)).norm()<1e-7,"width changed during horizon");
        }
        const auto input=solution.controllerPtr_->computeInput(0.0,solution.stateTrajectory_.front());
        require(input.size()==24 && input.allFinite(),"physical control input changed dimension");
        require(manager->getRobustWindow(0).v_max==1.0,"optimization changed speed cap");
        std::cout << "w_d=" << weight << " d_opt=" << solution.stateTrajectory_.front()(24) << "\n";
    }
    std::cout << "Width optimization, bounds, physical adapters and boundary Jacobians passed\n";
}
