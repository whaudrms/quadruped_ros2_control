//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <optional>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_quadruped_controller/estimator/FromOdomTopic.h>
#include <ocs2_quadruped_controller/estimator/GroundTruth.h>
#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_quadruped_controller/control/GaitManager.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedInterface.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/perceptive/synchronize/PlanarTerrainReceiver.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces) : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        // Humble throws ParameterAlreadyDeclaredException when controller_manager has
        // already declared parameters via --params-file. Guard each declare with has_parameter.
        if (!node_->has_parameter("robot_pkg"))
            node_->declare_parameter("robot_pkg", robot_pkg_);
        if (!node_->has_parameter("feet"))
            node_->declare_parameter("feet", feet_names_);
        if (!node_->has_parameter("enable_perceptive"))
            node_->declare_parameter("enable_perceptive", enable_perceptive_);
        if (!node_->has_parameter("enable_perceptive_reference_modification"))
            node_->declare_parameter("enable_perceptive_reference_modification", enable_perceptive_reference_modification_);
        if (!node_->has_parameter("enable_perceptive_foot_placement_constraint"))
            node_->declare_parameter("enable_perceptive_foot_placement_constraint", enable_perceptive_foot_placement_constraint_);
        if (!node_->has_parameter("enable_perceptive_foot_collision_constraint"))
            node_->declare_parameter("enable_perceptive_foot_collision_constraint", enable_perceptive_foot_collision_constraint_);
        if (!node_->has_parameter("enable_perceptive_body_collision_constraint"))
            node_->declare_parameter("enable_perceptive_body_collision_constraint", enable_perceptive_body_collision_constraint_);
        if (!node_->has_parameter("perceptive_foot_placement_boundary_margin"))
            node_->declare_parameter("perceptive_foot_placement_boundary_margin", perceptive_foot_placement_boundary_margin_);
        if (!node_->has_parameter("perceptive_foot_collision_clearance"))
            node_->declare_parameter("perceptive_foot_collision_clearance", perceptive_foot_collision_clearance_);
        robot_pkg_ = node_->get_parameter("robot_pkg").as_string();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        feet_names_ = node_->get_parameter("feet").as_string_array();
        enable_perceptive_ = node_->get_parameter("enable_perceptive").as_bool();
        enable_perceptive_reference_modification_ =
            node_->get_parameter("enable_perceptive_reference_modification").as_bool();
        enable_perceptive_foot_placement_constraint_ =
            node_->get_parameter("enable_perceptive_foot_placement_constraint").as_bool();
        enable_perceptive_foot_collision_constraint_ =
            node_->get_parameter("enable_perceptive_foot_collision_constraint").as_bool();
        enable_perceptive_body_collision_constraint_ =
            node_->get_parameter("enable_perceptive_body_collision_constraint").as_bool();
        perceptive_foot_placement_boundary_margin_ =
            node_->get_parameter("perceptive_foot_placement_boundary_margin").as_double();
        perceptive_foot_collision_clearance_ =
            node_->get_parameter("perceptive_foot_collision_clearance").as_double();


        const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg_);
        urdf_file_ = package_share_directory + "/urdf/robot.urdf";
        task_file_ = package_share_directory + "/config/ocs2/task.info";
        reference_file_ = package_share_directory + "/config/ocs2/reference.info";
        gait_file_ = package_share_directory + "/config/ocs2/gait.info";

        loadData::loadCppDataType(task_file_, "legged_robot_interface.verbose", verbose_);
        loadData::loadCppDataType(reference_file_, "comHeight", perceptive_com_height_);

        setupLeggedInterface();
        setupMpc();
        setupMrt();

        CentroidalModelPinocchioMapping pinocchio_mapping(legged_interface_->getCentroidalModelInfo());
        ee_kinematics_ = std::make_unique<PinocchioEndEffectorKinematics>(
            legged_interface_->getPinocchioInterface(), pinocchio_mapping,
            legged_interface_->modelSettings().contactNames3DoF);

        rbd_conversions_ = std::make_unique<CentroidalModelRbdConversions>(legged_interface_->getPinocchioInterface(),
                                                                           legged_interface_->getCentroidalModelInfo());

        // Init visualizer
        visualizer_ = std::make_unique<LeggedRobotVisualizer>(
            legged_interface_->getPinocchioInterface(),
            legged_interface_->getCentroidalModelInfo(),
            *ee_kinematics_,
            node_);

        // Init observation
        observation_.state.setZero(static_cast<long>(legged_interface_->getCentroidalModelInfo().stateDim));
        observation_.input.setZero(
            static_cast<long>(legged_interface_->getCentroidalModelInfo().inputDim));
        observation_.mode = STANCE;

    }

    nav_msgs::msg::Path CtrlComponent::pathFromBasePositions(
        const std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>>& basePositions,
        const rclcpp::Time& stamp) const
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = "odom";
        path.header.stamp = stamp;
        path.poses.reserve(basePositions.size());

        for (const auto& position : basePositions)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = position.x();
            pose.pose.position.y = position.y();
            pose.pose.position.z = position.z();
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(std::move(pose));
        }

        return path;
    }

    // Estimator types: "ground_truth", "linear_kalman", "from_odom_topic"
    void CtrlComponent::setupStateEstimate(const std::string& estimator_type)
    {
        estimator_type_ = estimator_type;
        if (estimator_type == "ground_truth")
        {
            estimator_ = std::make_unique<GroundTruth>(legged_interface_->getCentroidalModelInfo(),
                                                       ctrl_interfaces_,
                                                       node_);
            RCLCPP_INFO(node_->get_logger(), "Using Ground Truth Estimator");
        }
        else if (estimator_type == "linear_kalman")
        {
            estimator_ = std::make_unique<KalmanFilterEstimate>(
                legged_interface_->getPinocchioInterface(),
                legged_interface_->getCentroidalModelInfo(),
                *ee_kinematics_, ctrl_interfaces_,
                node_);
            dynamic_cast<KalmanFilterEstimate&>(*estimator_).loadSettings(task_file_, verbose_);
            if (enable_perceptive_)
            {
                dynamic_cast<KalmanFilterEstimate&>(*estimator_).setTerrainHeightProvider(
                    [this](const scalar_t x, const scalar_t y)
                    {
                        return samplePerceptiveTerrainHeight(x, y);
                    },
                    perceptive_com_height_);
            }
            RCLCPP_INFO(node_->get_logger(), "Using Kalman Filter Estimator");
        }
        else
        {
            estimator_ = std::make_unique<FromOdomTopic>(
                legged_interface_->getCentroidalModelInfo(), ctrl_interfaces_, node_);
            RCLCPP_INFO(node_->get_logger(), "Using Odom Topic Based Estimator");
        }
        observation_.time = 0;
    }

    void CtrlComponent::updateState(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Update State Estimation
        measured_rbd_state_ = estimator_->update(time, period);
        if (enable_perceptive_ && estimator_type_ == "linear_kalman")
        {
            alignPerceptiveBaseHeightToTerrain();
        }
        observation_.time += period.seconds();
        const scalar_t yaw_last = observation_.state(9);
        observation_.state = rbd_conversions_->computeCentroidalStateFromRbdModel(measured_rbd_state_);
        observation_.state(9) = yaw_last + angles::shortest_angular_distance(
            yaw_last, observation_.state(9));
        observation_.mode = estimator_->getMode();

        // Track ② step (a) — detection-only contact-event logging. Reads
        // measured-vs-scheduled contact and emits [robust_event] lines on
        // mismatches inside the robust window; does NOT modify the schedule
        // or the WBC contact flags. (b) will add the splice + override.
        detectAndLogContactEvents();

        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            sphereVisualizationPtr_->update(observation_);
            
            // Publish perceptive reference paths if there are subscribers
            publishPerceptiveReferencePaths();
            
            // Debug logging for perceptive foot placement
            logPerceptiveFootPlacementDebug();
        }

        // Compute target trajectory
        target_manager_->update(observation_);
        // Update the current state of the system
        mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    void CtrlComponent::detectAndLogContactEvents()
    {
        if (legged_interface_ == nullptr) return;
        const auto refMgrPtr = legged_interface_->getSwitchedModelReferenceManagerPtr();
        if (!refMgrPtr) return;
        const auto& refMgr = *refMgrPtr;

        // Splice requests route through the perceptive reference manager
        // (NOT GaitManager) so the actual schedule mutation runs at the start
        // of modifyReferences — BEFORE the line-180 getModeSchedule() read
        // that feeds terrain projection / swing planner / robust windows in
        // the same MPC solve. Falls back gracefully if the reference manager
        // isn't perceptive (no robust phase, no splice).
        auto* perceptiveRefMgr = dynamic_cast<PerceptiveLeggedReferenceManager*>(
            legged_interface_->getReferenceManagerPtr().get());

        const scalar_t t = observation_.time;
        // estimator_->getMode() RETURNS a mode number (size_t) — it's already
        // stanceLeg2ModeNumber(contact_flag_) per StateEstimateBase.h:37. Here we
        // need the inverse: mode number → per-leg bool array.
        const contact_flag_t measured  = modeNumber2StanceLeg(observation_.mode);
        const contact_flag_t scheduled = refMgr.getContactFlags(t);

        for (size_t leg = 0; leg < measured.size(); ++leg)
        {
            const bool m      = measured[leg];
            const bool s      = scheduled[leg];
            const bool prev_s = prev_scheduled_contact_[leg];

            // Liftoff (stance→swing edge in the schedule) opens a new swing
            // cycle for this leg. Reset robust-contact latches/counters so
            // the next swing's robust window is eligible to log and request
            // a fresh splice.
            const bool liftoff = prev_s && !s;
            if (liftoff)
            {
                robust_contact_logged_in_window_[leg] = false;
                splice_requested_in_window_[leg]      = false;
                sustained_robust_contact_ticks_[leg]  = 0;
                candidate_event_time_[leg]            = 0.0;
            }

            // Robust-window contact event: measured contact while the schedule
            // still says swing AND we are inside the robust window [t_a, t_b].
            // Both paper-side "early" (g_event > 0) and "late" (g_event < 0)
            // hits land here.
            const bool robust_contact_now = refMgr.isInRobustWindow(leg, t) && m && !s;

            // Capture the FIRST contact tick's time for use as event_time when
            // the splice is later confirmed at 5 ticks. Without this we'd send
            // the (debounce-confirmed) time ≈ 5 ms after first contact, which
            // is paper-incorrect (the event time should anchor on first
            // contact). Per chat6_eventtrigger.md A3 #1.
            if (robust_contact_now) {
                if (sustained_robust_contact_ticks_[leg] == 0) {
                    candidate_event_time_[leg] = t;  // rising edge
                }
                ++sustained_robust_contact_ticks_[leg];
            } else {
                sustained_robust_contact_ticks_[leg] = 0;
            }

            // Once-per-window log of the first robust-contact tick. We log the
            // window context (t_a, t_b, d, p_plane.z) so post-hoc analysis can
            // determine where in the [-d, +d] band the event landed:
            //   - If t is closer to t_a (early in window) → likely high-side
            //     (foot met ground above the perceived plane; perception was
            //     low; equivalent to paper "early contact").
            //   - If t is closer to t_b (late in window)  → likely low-side
            //     (perception was high; equivalent to paper "late contact").
            // The actual signed g_event = n·(p_foot − p_plane) − foot_frame_offset
            // is computed inside RobustGuardBoundaryConstraint::getValue at
            // every MPC evaluation; if needed for diagnostics, enable
            // robustPhase.verbose_log in task.info to dump per-cycle [robust_phase]
            // lines from the reference manager.
            if (robust_contact_now && !robust_contact_logged_in_window_[leg])
            {
                const auto w = refMgr.getRobustWindow(leg);
                RCLCPP_INFO(node_->get_logger(),
                            "[robust_event] leg=%zu type=robust_contact t=%.3f "
                            "t_a=%.3f t_b=%.3f d=%.3f p_plane.z=%.3f "
                            "(contact inside robust window — same path for "
                            "high-side and low-side hits)",
                            leg, t, w.t_a, w.t_b, w.d, w.p_plane.z());
                robust_contact_logged_in_window_[leg] = true;
            }

            // Robust-contact splice request. After 5 consecutive ticks of the
            // robust-window contact condition, queue a stance splice. We send
            // candidate_event_time_[leg] (the first-contact tick's observation
            // time), NOT the current `t` — anchors the splice on the actual
            // event time, paper-faithful.
            if (robust_contact_now &&
                sustained_robust_contact_ticks_[leg] >= kRobustContactSpliceSustainedTicks &&
                !splice_requested_in_window_[leg] &&
                perceptiveRefMgr != nullptr)
            {
                perceptiveRefMgr->requestRobustContactSplice(leg, candidate_event_time_[leg]);
                splice_requested_in_window_[leg] = true;
            }

            prev_scheduled_contact_[leg] = s;
        }
    }

    std::optional<scalar_t> CtrlComponent::samplePerceptiveTerrainHeight(const scalar_t x, const scalar_t y) const
    {
        if (!enable_perceptive_ || legged_interface_ == nullptr)
        {
            return std::nullopt;
        }

        auto* perceptiveReferenceManager = dynamic_cast<PerceptiveLeggedReferenceManager*>(
            legged_interface_->getReferenceManagerPtr().get());
        if (perceptiveReferenceManager == nullptr)
        {
            return std::nullopt;
        }

        const auto& convexRegionSelectorPtr = perceptiveReferenceManager->getConvexRegionSelectorPtr();
        if (!convexRegionSelectorPtr)
        {
            return std::nullopt;
        }

        return convexRegionSelectorPtr->sampleTerrainHeight(x, y);
    }

    void CtrlComponent::alignPerceptiveBaseHeightToTerrain()
    {
        if (measured_rbd_state_.size() < 6)
        {
            return;
        }

        const scalar_t x = measured_rbd_state_(3);
        const scalar_t y = measured_rbd_state_(4);
        const auto terrainHeight = samplePerceptiveTerrainHeight(x, y);
        if (!terrainHeight.has_value())
        {
            return;
        }

        measured_rbd_state_(5) = *terrainHeight + perceptive_com_height_;
    }

    void CtrlComponent::init()
    {
        if (mpc_running_ == false)
        {
            mpc_mrt_interface_->setCurrentObservation(observation_);

            const TargetTrajectories target_trajectories({observation_.time},
                                                         {observation_.state},
                                                         {observation_.input});

            mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);
            RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
            while (!mpc_mrt_interface_->initialPolicyReceived())
            {
                mpc_mrt_interface_->advanceMpc();
                rclcpp::WallRate(legged_interface_->mpcSettings().mrtDesiredFrequency_).sleep();
            }
            RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");

            mpc_running_ = true;
        }
    }

    void CtrlComponent::setupLeggedInterface()
    {
        if (enable_perceptive_)
        {
            legged_interface_ = std::make_unique<PerceptiveLeggedInterface>(task_file_, urdf_file_, reference_file_);
            dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).setPerceptiveDebugOptions(
                enable_perceptive_reference_modification_,
                enable_perceptive_foot_placement_constraint_,
                enable_perceptive_foot_collision_constraint_,
                enable_perceptive_body_collision_constraint_,
                perceptive_foot_placement_boundary_margin_, perceptive_foot_collision_clearance_);
        }
        else
        {
            legged_interface_ = std::make_unique<LeggedInterface>(task_file_, urdf_file_, reference_file_);
        }

        legged_interface_->setupJointNames(joint_names_, feet_names_);
        legged_interface_->setupOptimalControlProblem(task_file_, urdf_file_, reference_file_, verbose_);

        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_ = std::make_unique<FootPlacementVisualization>(
                *dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr()).
                getConvexRegionSelectorPtr(),
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_, 20.0,
                perceptive_foot_placement_boundary_margin_, enable_perceptive_foot_placement_constraint_);
            auto& perceptiveReferenceManager =
                dynamic_cast<PerceptiveLeggedReferenceManager&>(*legged_interface_->getReferenceManagerPtr());
            const auto swingPlanner = perceptiveReferenceManager.getSwingTrajectoryPlanner();
            perceptiveReferenceManager.setVisualizationCallback(
                [this, swingPlanner](scalar_t initTime, scalar_t finalTime, const ModeSchedule& schedule)
                {
                    footPlacementVisualizationPtr_->update(initTime, finalTime, schedule, *swingPlanner);
                });

            sphereVisualizationPtr_ = std::make_unique<SphereVisualization>(
                legged_interface_->getPinocchioInterface(), legged_interface_->getCentroidalModelInfo(),
                *dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPinocchioSphereInterfacePtr(), node_);

            rawReferencePathPublisherPtr_ =
                node_->create_publisher<nav_msgs::msg::Path>("/perceptive_reference/raw_base_path", 1);
            terrainAwareReferencePathPublisherPtr_ =
                node_->create_publisher<nav_msgs::msg::Path>("/perceptive_reference/terrain_aware_base_path", 1);
        }
    }

    /**
     * Set up the SQP MPC, Gait Manager and Reference Manager
     */
    void CtrlComponent::setupMpc()
    {
        mpc_ = std::make_shared<SqpMpc>(legged_interface_->mpcSettings(),
                                        legged_interface_->sqpSettings(),
                                        legged_interface_->getOptimalControlProblem(),
                                        legged_interface_->getInitializer());

        // Initialize the reference manager
        gait_manager_ptr_ = std::make_shared<GaitManager>(
            ctrl_interfaces_,
            legged_interface_->getSwitchedModelReferenceManagerPtr()->
                               getGaitSchedule());
        gait_manager_ptr_->init(gait_file_);
        mpc_->getSolverPtr()->addSynchronizedModule(gait_manager_ptr_);
        mpc_->getSolverPtr()->setReferenceManager(legged_interface_->getReferenceManagerPtr());

        target_manager_ = std::make_unique<TargetManager>(ctrl_interfaces_,
                                                          node_,
                                                          legged_interface_->getReferenceManagerPtr(),
                                                          task_file_,
                                                          reference_file_);

        if (enable_perceptive_)
        {
            const auto planarTerrainReceiver =
                std::make_shared<PlanarTerrainReceiver>(
                    node_, dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getPlanarTerrainPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getSignedDistanceFieldPtr(),
                    dynamic_cast<PerceptiveLeggedInterface&>(*legged_interface_).getTerrainDataMutexPtr(),
                    "/convex_plane_decomposition_ros/planar_terrain", "elevation");
            mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
        }
    }

    void CtrlComponent::setupMrt()
    {
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);
        mpc_mrt_interface_->initRollout(&legged_interface_->getRollout());
        mpc_timer_.reset();

        controller_running_ = true;
        mpc_thread_ = std::thread([&]
        {
            using timing_clock_t = std::chrono::steady_clock;
            auto timingWindowStart = timing_clock_t::now();
            size_t solvesInWindow = 0;
            scalar_t solveTimeSumMs = 0.0;
            scalar_t solveTimeMaxMs = 0.0;
            size_t sqpIterationsSum = 0;
            size_t sqpIterationsMax = 0;
            size_t deadlineHitsInWindow = 0;
            bool timingWindowStarted = false;

            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                if (!timingWindowStarted)
                                {
                                    timingWindowStart = timing_clock_t::now();
                                    timingWindowStarted = true;
                                }

                                mpc_timer_.startTimer();
                                mpc_mrt_interface_->advanceMpc();
                                mpc_timer_.endTimer();

                                const scalar_t lastSolveMs = mpc_timer_.getLastIntervalInMilliseconds();
                                const scalar_t targetFrequency =
                                    legged_interface_->mpcSettings().mpcDesiredFrequency_;
                                const scalar_t deadlineMs =
                                    targetFrequency > 0.0 ? 1000.0 / targetFrequency : 0.0;
                                const size_t lastSqpIterations =
                                    mpc_->getSolverPtr()->getIterationsLog().size();
                                ++solvesInWindow;
                                solveTimeSumMs += lastSolveMs;
                                solveTimeMaxMs = std::max(solveTimeMaxMs, lastSolveMs);
                                sqpIterationsSum += lastSqpIterations;
                                sqpIterationsMax = std::max(sqpIterationsMax, lastSqpIterations);
                                if (deadlineMs > 0.0 && lastSolveMs <= deadlineMs)
                                {
                                    ++deadlineHitsInWindow;
                                }

                                const auto now = timing_clock_t::now();
                                const scalar_t elapsedSec =
                                    std::chrono::duration<scalar_t>(now - timingWindowStart).count();
                                if (elapsedSec >= 1.0)
                                {
                                    const scalar_t solveHz = static_cast<scalar_t>(solvesInWindow) / elapsedSec;
                                    const scalar_t averageSolveMs =
                                        solveTimeSumMs / static_cast<scalar_t>(solvesInWindow);
                                    const scalar_t capacityHz =
                                        averageSolveMs > 0.0 ? 1000.0 / averageSolveMs : 0.0;
                                    const scalar_t solverLoadPercent = solveHz * averageSolveMs / 10.0;
                                    const scalar_t averageSqpIterations =
                                        static_cast<scalar_t>(sqpIterationsSum) /
                                        static_cast<scalar_t>(solvesInWindow);
                                    const scalar_t deadlineRatePercent =
                                        100.0 * static_cast<scalar_t>(deadlineHitsInWindow) /
                                        static_cast<scalar_t>(solvesInWindow);

                                    RCLCPP_INFO(
                                        node_->get_logger(),
                                        "[MPC timing] target_hz=%.1f solve_hz=%.1f capacity_hz=%.1f "
                                        "solve_ms(last/avg/max)=%.2f/%.2f/%.2f load=%.1f%% "
                                        "sqp_iter(last/avg/max/limit)=%zu/%.2f/%zu/%zu "
                                        "deadline_hit=%zu/%zu deadline_rate=%.1f%%",
                                        targetFrequency,
                                        solveHz, capacityHz, lastSolveMs, averageSolveMs,
                                        solveTimeMaxMs, solverLoadPercent,
                                        lastSqpIterations, averageSqpIterations,
                                        sqpIterationsMax,
                                        legged_interface_->sqpSettings().sqpIteration,
                                        deadlineHitsInWindow, solvesInWindow,
                                        deadlineRatePercent);

                                    timingWindowStart = now;
                                    solvesInWindow = 0;
                                    solveTimeSumMs = 0.0;
                                    solveTimeMaxMs = 0.0;
                                    sqpIterationsSum = 0;
                                    sqpIterationsMax = 0;
                                    deadlineHitsInWindow = 0;
                                }
                            }
                        },
                        legged_interface_->mpcSettings().mpcDesiredFrequency_);
                }
                catch (const std::exception& e)
                {
                    controller_running_ = false;
                    RCLCPP_WARN(node_->get_logger(), "[Ocs2 MPC thread] Error : %s", e.what());
                }
            }
        });
        setThreadPriority(legged_interface_->sqpSettings().threadPriority, mpc_thread_);
        RCLCPP_INFO(node_->get_logger(), "MRT initialized. MPC thread started.");
    }

    void CtrlComponent::publishPerceptiveReferencePaths()
    {
        if (!rawReferencePathPublisherPtr_ || !terrainAwareReferencePathPublisherPtr_)
        {
            return;
        }

        if ((rawReferencePathPublisherPtr_->get_subscription_count() == 0 &&
             rawReferencePathPublisherPtr_->get_intra_process_subscription_count() == 0) &&
            (terrainAwareReferencePathPublisherPtr_->get_subscription_count() == 0 &&
             terrainAwareReferencePathPublisherPtr_->get_intra_process_subscription_count() == 0))
        {
            return;
        }

        if (observation_.time - lastReferencePathPublishTime_ < minReferencePathPublishTimeDifference_)
        {
            return;
        }

        auto* perceptiveReferenceManager = dynamic_cast<PerceptiveLeggedReferenceManager*>(
            legged_interface_->getReferenceManagerPtr().get());
        if (perceptiveReferenceManager == nullptr)
        {
            return;
        }

        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> rawBasePath;
        std::vector<vector3_t, Eigen::aligned_allocator<vector3_t>> terrainAwareBasePath;
        if (!perceptiveReferenceManager->getLatestReferencePaths(rawBasePath, terrainAwareBasePath))
        {
            return;
        }

        const auto stamp = node_->now();
        rawReferencePathPublisherPtr_->publish(pathFromBasePositions(rawBasePath, stamp));
        terrainAwareReferencePathPublisherPtr_->publish(pathFromBasePositions(terrainAwareBasePath, stamp));
        lastReferencePathPublishTime_ = observation_.time;
    }

    void CtrlComponent::logPerceptiveFootPlacementDebug()
    {
        if (!enable_perceptive_foot_placement_constraint_)
        {
            return;
        }

        if (observation_.time - lastFootPlacementDebugLogTime_ < minFootPlacementDebugLogTimeDifference_)
        {
            return;
        }

        auto* perceptiveReferenceManager = dynamic_cast<PerceptiveLeggedReferenceManager*>(
            legged_interface_->getReferenceManagerPtr().get());
        if (perceptiveReferenceManager == nullptr)
        {
            return;
        }

        PerceptiveLeggedReferenceManager::FootPlacementDebugInfo debugInfo;
        if (!perceptiveReferenceManager->getLatestFootPlacementDebugInfo(debugInfo))
        {
            return;
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3);
        oss << "[FootPlacementDebug] t=" << debugInfo.time;
        oss << " contact=[";
        for (size_t i = 0; i < debugInfo.contactFlags.size(); ++i)
        {
            if (i > 0) oss << ",";
            oss << static_cast<int>(debugInfo.contactFlags[i]);
        }
        oss << "] active=[";
        for (size_t i = 0; i < debugInfo.footPlacementFlags.size(); ++i)
        {
            if (i > 0) oss << ",";
            oss << static_cast<int>(debugInfo.footPlacementFlags[i]);
        }
        oss << "] poly=[";
        for (size_t i = 0; i < debugInfo.polygonVertexCounts.size(); ++i)
        {
            if (i > 0) oss << ",";
            oss << debugInfo.polygonVertexCounts[i];
        }
        oss << "] proj_z=[";
        for (size_t i = 0; i < debugInfo.projectionHeights.size(); ++i)
        {
            if (i > 0) oss << ",";
            oss << debugInfo.projectionHeights[i];
        }
        oss << "] init_stand_final=[";
        for (size_t i = 0; i < debugInfo.initStandFinalTimes.size(); ++i)
        {
            if (i > 0) oss << ",";
            oss << debugInfo.initStandFinalTimes[i];
        }
        oss << "]";

        RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
        lastFootPlacementDebugLogTime_ = observation_.time;
    }
}
