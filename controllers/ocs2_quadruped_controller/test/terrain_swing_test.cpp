#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ocs2_oc/test/testProblemsGeneration.h>
#include <ocs2_quadruped_controller/perceptive/foot_planner/TerrainSwing.h>
#include <ocs2_quadruped_controller/perceptive/cost/SwingFootTrackingCost.h>
#include <ocs2_quadruped_controller/perceptive/interface/ConstantParameterOcp.h>
#include <ocs2_core/cost/StateInputCostCollection.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
using namespace ocs2;
using namespace ocs2::legged_robot;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
class Feet final : public EndEffectorKinematics<scalar_t> {
 public:
    Feet* clone() const override { return new Feet(*this); }
    const std::vector<std::string>& getIds() const override { return ids_; }
    std::vector<vector3_t> getPosition(const vector_t& x) const override {
        require(x.size()==24,"kinematics received auxiliary states"); return {x.head<3>()};
    }
    std::vector<vector3_t> getVelocity(const vector_t& x,const vector_t& u) const override {
        require(x.size()==24 && u.size()==24,"velocity dimensions"); return {u.head<3>() + 0.2*x.head<3>()};
    }
    std::vector<vector3_t> getOrientationError(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& x) const override {
        auto a=VectorFunctionLinearApproximation::Zero(3,24,0);
        a.f=getPosition(x).front(); a.dfdx.leftCols(3).setIdentity(); return {a};
    }
    std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& x,const vector_t& u) const override {
        auto a=VectorFunctionLinearApproximation::Zero(3,24,24);
        a.f=getVelocity(x,u).front(); a.dfdu.leftCols(3).setIdentity(); a.dfdx.leftCols(3)=0.2*matrix3_t::Identity(); return {a};
    }
    std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
 private:
    std::vector<std::string> ids_{"FL"};
};

class Manager final : public SwitchedModelReferenceManager {
public:
    Manager(std::shared_ptr<SwingTrajectoryPlanner> planner) : SwitchedModelReferenceManager(nullptr, planner) {}
    RobustWindowData window;
    bool isInRobustWindow(size_t, scalar_t t) const override { return window.active && t>=window.t_a && t<window.t_b; }
    RobustWindowData getRobustWindow(size_t) const override { return window; }
    void schedule(const ModeSchedule& value) {
        ReferenceManager::setModeSchedule(value);
        preSolverRun(0.0, 1.0, vector_t::Zero(24));
    }
protected:
    void modifyReferences(scalar_t, scalar_t, const vector_t&, TargetTrajectories&, ModeSchedule&) override {}
};
class FourFeet final : public EndEffectorKinematics<scalar_t> {
public:
    FourFeet* clone() const override { return new FourFeet(*this); }
    const std::vector<std::string>& getIds() const override { return ids_; }
    std::vector<vector3_t> getPosition(const vector_t& x) const override {
        return {x.head<3>()+vector3_t(.2,.15,0),x.head<3>()+vector3_t(.2,-.15,0),
                x.head<3>()+vector3_t(-.2,.15,0),x.head<3>()+vector3_t(-.2,-.15,0)};
    }
    std::vector<vector3_t> getVelocity(const vector_t&, const vector_t&) const override { return std::vector<vector3_t>(4,vector3_t::Zero()); }
    std::vector<vector3_t> getOrientationError(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t&,const vector_t&) const override { return {}; }
    std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(const vector_t&,const std::vector<quaternion_t>&) const override { return {}; }
private:
    std::vector<std::string> ids_{"FL","FR","RL","RR"};
};
class PerceptiveManager final : public PerceptiveLeggedReferenceManager {
public:
    using PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager;
    using PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner;
};
void referencePipelineTest() {
    using namespace convex_plane_decomposition;
    CentroidalModelInfo info{}; info.numThreeDofContacts=4; info.stateDim=24; info.inputDim=24;
    info.generalizedCoordinatesNum=18; info.actuatedDofNum=12;
    auto terrain=std::make_shared<PlanarTerrain>();
    PlanarRegion region; region.transformPlaneToWorld.setIdentity();
    CgalPolygon2d polygon;
    for (const auto& point : std::vector<CgalPoint2d>{{-2,-2},{2,-2},{2,2},{-2,2}}) polygon.push_back(point);
    region.boundaryWithInset.boundary=CgalPolygonWithHoles2d(polygon);
    region.boundaryWithInset.insets.push_back(CgalPolygonWithHoles2d(polygon)); region.bbox2d=polygon.bbox();
    terrain->planarRegions.push_back(region); terrain->gridMap.setGeometry(grid_map::Length(4,4),.02);
    terrain->gridMap.add("elevation",0.0); terrain->gridMap.add("smooth_planar",0.0);
    // Raw narrow obstacle on the swing path; smooth_planar deliberately has no obstacle.
    const auto size=terrain->gridMap.getSize();
    for (int i=0;i<size.x();++i) for(int j=0;j<size.y();++j) {
        grid_map::Position position; const grid_map::Index index(i,j);terrain->gridMap.getPosition(index,position);
        if (position.x()>.34 && position.x()<.40) terrain->gridMap.at("elevation",index)=.10;
    }
    FourFeet feet;
    auto selector=std::make_shared<ConvexRegionSelector>(info,terrain,std::make_shared<std::mutex>(),feet,16);
    SwingTrajectoryPlanner::Config config;config.terrainAware=true;config.swingHeight=.08;
    auto planner=std::make_shared<SwingTrajectoryPlanner>(config,4);
    PerceptiveManager manager(info,nullptr,planner,selector,feet,.35);
    ModeSchedule schedule({-1,0,1,2},{15,15,0,15,15});
    vector_t initial=vector_t::Zero(24);initial(2)=.023;
    vector_t desired=initial; desired(0)=.3;
    TargetTrajectories target({-1,3},{desired,desired},{vector_t::Zero(24),vector_t::Zero(24)});
    selector->update(schedule,-.1,initial,target);
    manager.updateSwingTrajectoryPlanner(-.1,initial,schedule);
    const auto* first=planner->getTerrainSwing(0,.5);require(first,"no 3D reference generated");
    const vector3_t liftoff=first->startPosition;
    require((liftoff-feet.getPosition(initial)[0]).norm()<1e-9,"3D planner did not restore measured foot-frame anchor");
    require((first->endPosition-vector3_t(.5,.15,.02)).norm()<1e-7,"3D target not selected touchdown XYZ");
    require(first->spline.position(.5).z()>.17,"raw obstacle profile absent from planned spline");
    vector_t moving=initial; moving(0)=.04;moving(2)=.08;
    selector->update(schedule,.2,moving,target);manager.updateSwingTrajectoryPlanner(.2,moving,schedule);
    require((planner->getTerrainSwing(0,.5)->startPosition-liftoff).norm()<1e-9,"liftoff anchor moved with swinging foot");
    desired(0)=.35;target=TargetTrajectories({-1,3},{desired,desired},{vector_t::Zero(24),vector_t::Zero(24)});
    selector->update(schedule,.3,moving,target);manager.updateSwingTrajectoryPlanner(.3,moving,schedule);
    require(std::abs(planner->getTerrainSwing(0,.5)->endPosition.x()-.55)<1e-7,"swing ignored updated selected touchdown");
    require((planner->getTerrainSwing(0,.5)->startPosition-liftoff).norm()<1e-9,"replanning lost measured liftoff anchor");
    // Other legs can insert mode events in the current stance. On a slope the
    // measured anchor must still be restored vertically, not shifted by radius*n.
    terrain->planarRegions[0].transformPlaneToWorld.linear() = Eigen::AngleAxisd(.2, vector3_t::UnitY()).toRotationMatrix();
    const vector3_t normal = terrain->planarRegions[0].transformPlaneToWorld.linear().col(2);
    for (int i=0;i<size.x();++i) for(int j=0;j<size.y();++j) {
        grid_map::Position xy;const grid_map::Index index(i,j);terrain->gridMap.getPosition(index,xy);
        terrain->gridMap.at("elevation",index)=-(normal.x()*xy.x()+normal.y()*xy.y())/normal.z();
    }
    schedule=ModeSchedule({-1,0,.2,.5,1,2},{15,15,15,0,15,15,15});
    PerceptiveManager slopeManager(info,nullptr,planner,selector,feet,.35);
    selector->update(schedule,-.1,initial,target);slopeManager.updateSwingTrajectoryPlanner(-.1,initial,schedule);
    require((planner->getTerrainSwing(0,.3)->startPosition-feet.getPosition(initial)[0]).norm()<1e-9,
            "mode events inside stance shifted measured liftoff on a slope");
}
void geometryTest() {
    const vector3_t from(0,0,0.02), to(0.4,0,0.12), normal=vector3_t::UnitZ();
    const std::vector<std::pair<double,double>> flatProfile;
    const std::vector<std::pair<double,double>> stepProfile{{0.0,0.0},{0.49,0.0},{0.5,0.1},{1.0,0.1}};
    TerrainSwing flat(1,1.4,from,to,normal,normal,.05,-.1,.08,.15,flatProfile);
    TerrainSwing step(1,1.4,from,to,normal,normal,.05,-.1,.08,.15,stepProfile);
    require((step.spline.position(1)-from).norm()<1e-8,"liftoff position");
    require((step.spline.position(1.4)-to).norm()<1e-8,"touchdown position");
    require((step.spline.velocity(1)-vector3_t(0,0,.05)).norm()<1e-8,"liftoff velocity");
    require((step.spline.velocity(1.4)-vector3_t(0,0,-.1)).norm()<1e-8,"touchdown velocity");
    require(step.spline.acceleration(1).norm()<1e-7 && step.spline.acceleration(1.4).norm()<1e-7,"endpoint acceleration");
    require(step.spline.position(1.2).z()>flat.spline.position(1.2).z()+.02,"terrain height profile did not lift swing");
    require((step.spline.velocity(1.2)-vector3_t(2,0,.5)).norm()<1e-8,"original midpoint tangential velocity");
    require((step.spline.acceleration(1.2-1e-9)-step.spline.acceleration(1.2+1e-9)).norm()<1e-4,"acceleration discontinuity");
    require((step.spline.jerk(1.2-1e-9)-step.spline.jerk(1.2+1e-9)).norm()<1e-3,"jerk discontinuity");
    TerrainSwing shortSwing(0,.075,from,from,normal,normal,.05,-.1,.08,.15,{});
    require(std::abs(shortSwing.spline.position(.0375).z()-.06)<1e-9,"short swing height scaling");
    require(std::abs(shortSwing.spline.velocity(0).z()-.025)<1e-9,"short swing velocity scaling");
    const vector3_t tilted = vector3_t(.2,0,1).normalized();
    TerrainSwing slope(0,1,from,to,normal,tilted,.05,-.1,.08,.15,{});
    require((slope.normal(.1)-normal).norm()<1e-9 && (slope.normal(.9)-tilted).norm()<1e-9,"terrain normal interpolation");
    require((slope.spline.velocity(1)+.1*tilted).norm()<1e-8,"normal touchdown velocity");
    TerrainSwing outlier(1,1.4,from,to,normal,normal,.05,-.1,.08,.15,{{.5,100}});
    require((outlier.spline.position(1.2)-(from+.6*(to-from))).norm()<.24000001,"outlier adaptation cap");
}
void costTest() {
    SwingTrajectoryPlanner::Config config; config.terrainAware=true;
    auto planner=std::make_shared<SwingTrajectoryPlanner>(config,4);
    SwingTrajectoryPlanner::TerrainSwings paths;
    paths[0].push_back(std::make_shared<TerrainSwing>(0,1,vector3_t(0,0,.02),vector3_t(.3,.1,.12),
        vector3_t::UnitZ(),vector3_t::UnitZ(),.05,-.1,.08,.15,std::vector<std::pair<double,double>>{}));
    planner->setTerrainSwings(paths);
    Manager manager(planner); manager.schedule(ModeSchedule({0.0,1.0},{15,0,15}));
    Feet feet; SwingFootTrackingCost cost(manager,*planner,feet,0);
    vector_t x=vector_t::Constant(24,.05),u=vector_t::Constant(24,.03);
    TargetTrajectories target; PreComputation precomp;
    require(!cost.isActive(-.1) && cost.isActive(.5) && !cost.isActive(1.1),"swing activation");
    for (bool robust : {false,true}) {
        manager.window.active=robust; manager.window.t_a=.4; manager.window.t_b=.9;
        manager.window.n=vector3_t(.2,.1,1).normalized();
        const auto a=cost.getQuadraticApproximation(.5,x,u,target,precomp);
        require(std::abs(a.f-cost.getValue(.5,x,u,target,precomp))<1e-10,"cost approximation value");
        constexpr double eps=1e-6;
        for (int j=0;j<24;++j) {
            vector_t plus=x,minus=x; plus(j)+=eps; minus(j)-=eps;
            const double fd=(cost.getValue(.5,plus,u,target,precomp)-cost.getValue(.5,minus,u,target,precomp))/(2*eps);
            require(std::abs(fd-a.dfdx(j))<1e-7,"tracking state gradient");
            const auto ap=cost.getQuadraticApproximation(.5,plus,u,target,precomp);
            const auto am=cost.getQuadraticApproximation(.5,minus,u,target,precomp);
            require(((ap.dfdx-am.dfdx)/(2*eps)-a.dfdxx.col(j)).norm()<1e-7,"tracking state Hessian");
            require(((ap.dfdu-am.dfdu)/(2*eps)-a.dfdux.col(j)).norm()<1e-7,"tracking mixed Hessian");
            plus=u; minus=u; plus(j)+=eps; minus(j)-=eps;
            const double fu=(cost.getValue(.5,x,plus,target,precomp)-cost.getValue(.5,x,minus,target,precomp))/(2*eps);
            require(std::abs(fu-a.dfdu(j))<1e-7,"tracking input gradient");
        }
        if (robust) {
            vector_t displaced=x; displaced.head<3>()+=.08*manager.window.n;
            require(std::abs(cost.getValue(.5,displaced,u,target,precomp)-a.f)<1e-9,"robust normal position penalized");
            displaced=u; displaced.head<3>()+=.2*manager.window.n;
            require(std::abs(cost.getValue(.5,x,displaced,target,precomp)-a.f)<1e-9,"robust normal velocity penalized");
        }
        OptimalControlProblem problem;
        problem.dynamicsPtr=getOcs2Dynamics(VectorFunctionLinearApproximation::Zero(24,24,24));
        problem.costPtr->add("swing",std::unique_ptr<StateInputCost>(cost.clone()));
        appendConstantParameters(problem,24,4);
        vector_t augmented=vector_t::Zero(28);augmented.head(24)=x;augmented.tail(4).setConstant(.04);
        const auto padded=problem.costPtr->getQuadraticApproximation(.5,augmented,u,target,*problem.preComputationPtr);
        require(padded.dfdx.size()==28 && padded.dfdx.tail(4).isZero(),"auxiliary d cost gradient");
        require(padded.dfdxx.rightCols(4).isZero() && padded.dfdux.rightCols(4).isZero(),"auxiliary d cost Hessian");
        require(std::abs(padded.f-a.f)<1e-9,"auxiliary state adapter changed tracking cost");
    }
    manager.schedule(ModeSchedule({0.0,.45,1.0},{15,0,15,15}));
    require(!cost.isActive(.5) && cost.getValue(.5,x,u,target,precomp)==0.0,"splice stance still tracks swing");
}
}
int main() {
    try { geometryTest(); costTest(); referencePipelineTest(); std::cout<<"Terrain swing geometry, tracking derivatives, robust/stance gating and d adapter passed\n"; }
    catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
