//
// Created by biao on 3/15/25.
//

#include "ocs2_quadruped_controller/control/CtrlComponent.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <angles/angles.h>
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
        if (!node_->has_parameter("ocs2_dump_dir"))
            node_->declare_parameter<std::string>("ocs2_dump_dir", "");
        if (!node_->has_parameter("ocs2_dump_max_cycles"))
            node_->declare_parameter<int>("ocs2_dump_max_cycles", 0);
        if (!node_->has_parameter("ocs2_dump_min_interval_sec"))
            node_->declare_parameter<double>("ocs2_dump_min_interval_sec", 0.01);
        // Stage 8 — synchronous robust_refine swap path.
        if (!node_->has_parameter("enable_refiner_swap"))
            node_->declare_parameter<bool>("enable_refiner_swap", false);
        if (!node_->has_parameter("refined_dir"))
            node_->declare_parameter<std::string>("refined_dir", "/dev/shm/robust_refine/out");
        if (!node_->has_parameter("refined_timeout_sec"))
            node_->declare_parameter<double>("refined_timeout_sec", 0.5);
        // Stage 9 — one-shot OCP mode.
        if (!node_->has_parameter("mpc_one_shot"))
            node_->declare_parameter<bool>("mpc_one_shot", false);
        if (!node_->has_parameter("mpc_one_shot_solves"))
            node_->declare_parameter<int>("mpc_one_shot_solves", 10);
        mpc_one_shot_ = node_->get_parameter("mpc_one_shot").as_bool();
        mpc_one_shot_solves_ = node_->get_parameter("mpc_one_shot_solves").as_int();

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

        // Stage 7: optional MPC PrimalSolution dump for robust_refine
        const std::string dump_dir = node_->get_parameter("ocs2_dump_dir").as_string();
        const int dump_max_cycles = node_->get_parameter("ocs2_dump_max_cycles").as_int();
        const double dump_min_interval = node_->get_parameter("ocs2_dump_min_interval_sec").as_double();

        // Stage 8: synchronous robust_refine swap path
        const bool enable_refiner_swap = node_->get_parameter("enable_refiner_swap").as_bool();
        const std::string refined_dir = node_->get_parameter("refined_dir").as_string();
        const double refined_timeout_sec = node_->get_parameter("refined_timeout_sec").as_double();

        if (enable_refiner_swap)
        {
            // The recorder feeds the daemon's input pipe; it MUST be always-on,
            // unbounded, and produce one file per MPC solve.
            const std::string sync_in_dir = "/dev/shm/robust_refine/in";
            mpc_dump_recorder_ = std::make_unique<MpcDumpRecorder>(
                sync_in_dir,
                /*maxCycles=*/0,         // 0 means infinite
                /*minIntervalSec=*/0.0); // dump every solve
            if (mpc_dump_recorder_->isEnabled())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[MpcDumpRecorder] Sync-IPC mode. dir='%s' (infinite, no throttle).",
                            sync_in_dir.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[MpcDumpRecorder] Failed to create '%s' — sync IPC disabled.",
                            sync_in_dir.c_str());
                mpc_dump_recorder_.reset();
            }

            refined_policy_reader_ = std::make_unique<RefinedPolicyReader>(
                refined_dir, refined_timeout_sec, /*blocking=*/true);
            RCLCPP_INFO(node_->get_logger(),
                        "[RefinedPolicyReader] Enabled. dir='%s' timeout=%.3fs",
                        refined_dir.c_str(), refined_timeout_sec);
        }
        else if (!dump_dir.empty() && dump_max_cycles > 0)
        {
            // Legacy Stage 7 path (offline dump only, no IPC wait).
            mpc_dump_recorder_ = std::make_unique<MpcDumpRecorder>(
                dump_dir, static_cast<size_t>(dump_max_cycles), dump_min_interval);
            if (mpc_dump_recorder_->isEnabled())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "[MpcDumpRecorder] Enabled. dir='%s' max_cycles=%d interval=%.3fs",
                            dump_dir.c_str(), dump_max_cycles, dump_min_interval);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[MpcDumpRecorder] Failed to create directory '%s' — disabled.",
                            dump_dir.c_str());
                mpc_dump_recorder_.reset();
            }
        }
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

        visualizer_->update(observation_);
        if (enable_perceptive_)
        {
            footPlacementVisualizationPtr_->update(observation_);
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

            if (mpc_one_shot_)
            {
                // Stage 9 — full-horizon 1-shot OCP.
                // Do NOT override the reference manager with a stationary single-point
                // target. TargetManager has already populated forward-tracking
                // references from the live cmd_vel (set by the scenario before
                // OCS2 entry). We just run the solver enough SQP iterations to
                // converge from cold start, then dump once.
                //
                // GaitManager::preSolverRun normally inserts a newly-selected
                // gait template at finalTime (so it activates AFTER the current
                // horizon), which is harmless in MPC mode but disastrous here
                // because we never re-solve. Prime the gait schedule across
                // [initTime, finalTime] BEFORE the bootstrap loop so the OCP
                // optimizes over the actual walking gait instead of all-stance.
                if (gait_manager_ptr_)
                {
                    const scalar_t initTime = observation_.time;
                    const scalar_t finalTime =
                        initTime + legged_interface_->mpcSettings().timeHorizon_;
                    gait_manager_ptr_->primeForOneShot(initTime, finalTime);
                }

                RCLCPP_INFO(node_->get_logger(),
                            "[mpc_one_shot] Solving full-horizon OCP (%d SQP iters)...",
                            mpc_one_shot_solves_);
                rclcpp::WallRate rate(legged_interface_->mpcSettings().mrtDesiredFrequency_);
                const auto wall_t0 = std::chrono::steady_clock::now();
                for (int i = 0; i < mpc_one_shot_solves_; ++i)
                {
                    mpc_mrt_interface_->advanceMpc();
                    rate.sleep();
                }
                const auto wall_t1 = std::chrono::steady_clock::now();
                const double solve_ms =
                    std::chrono::duration<double, std::milli>(wall_t1 - wall_t0).count();
                RCLCPP_INFO(node_->get_logger(),
                            "[mpc_one_shot] OCP solved (%d iters in %.1f ms).",
                            mpc_one_shot_solves_, solve_ms);

                if (mpc_dump_recorder_ && !mpc_dump_recorder_->isFinished())
                {
                    SystemObservation mpcInitObs = observation_;
                    PrimalSolution dumpSolution;
                    const scalar_t startTime = mpcInitObs.time;
                    const scalar_t finalTime =
                        (mpc_->settings().solutionTimeWindow_ < 0)
                            ? mpc_->getSolverPtr()->getFinalTime()
                            : startTime + mpc_->settings().solutionTimeWindow_;
                    mpc_->getSolverPtr()->getPrimalSolution(finalTime, &dumpSolution);
                    const bool wrote =
                        mpc_dump_recorder_->record(mpcInitObs, dumpSolution);
                    RCLCPP_INFO(node_->get_logger(),
                                "[mpc_one_shot] One-shot policy dump %s "
                                "(t0=%.3f, tf=%.3f, N=%zu).",
                                wrote ? "succeeded" : "FAILED",
                                startTime, finalTime,
                                dumpSolution.timeTrajectory_.size());
                }

                mpc_one_shot_done_.store(true);
            }
            else
            {
                // Original receding-horizon bootstrap.
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
            }

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
                perceptive_foot_placement_boundary_margin_);
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
                legged_interface_->getCentroidalModelInfo().numThreeDofContacts, node_);

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
            while (controller_running_)
            {
                try
                {
                    executeAndSleep(
                        [&]
                        {
                            if (mpc_running_)
                            {
                                // Stage 9 — in one-shot mode, init() already solved
                                // and dumped the OCP. Skip all advanceMpc()/dump
                                // calls here so the cached policy never gets
                                // overwritten. Refined-policy polling still runs
                                // below so an offline-refined plan can replace it.
                                const bool skip_solve =
                                    mpc_one_shot_ && mpc_one_shot_done_.load();

                                if (!skip_solve)
                                {
                                    mpc_timer_.startTimer();
                                    mpc_mrt_interface_->advanceMpc();
                                    mpc_timer_.endTimer();

                                    // Stage 8 — synchronous robust_refine IPC.
                                    // After every MPC solve, dump the just-solved
                                    // PrimalSolution to /dev/shm/robust_refine/in.
                                    if (mpc_dump_recorder_ &&
                                        !mpc_dump_recorder_->isFinished())
                                    {
                                        SystemObservation mpcInitObs;
                                        {
                                            mpcInitObs = observation_;
                                        }

                                        PrimalSolution dumpSolution;
                                        const scalar_t startTime = mpcInitObs.time;
                                        const scalar_t finalTime =
                                            (mpc_->settings().solutionTimeWindow_ < 0)
                                                ? mpc_->getSolverPtr()->getFinalTime()
                                                : startTime + mpc_->settings().solutionTimeWindow_;
                                        mpc_->getSolverPtr()->getPrimalSolution(
                                            finalTime, &dumpSolution);

                                        mpc_dump_recorder_->record(mpcInitObs, dumpSolution);
                                    }
                                }

                                // Stage 8 Option B (async-with-latest): always poll
                                // for the freshest refined plan. In MPC mode this
                                // runs after every solve; in one-shot mode this is
                                // the only thing the thread does after init().
                                if (refined_policy_reader_)
                                {
                                    size_t loadedSeq = std::numeric_limits<size_t>::max();
                                    scalar_array_t tTraj;
                                    vector_array_t xTraj;
                                    vector_array_t uTraj;
                                    const bool ok = refined_policy_reader_->loadLatest(
                                        loadedSeq, tTraj, xTraj, uTraj);
                                    if (ok && !tTraj.empty())
                                    {
                                        const size_t prevSeq =
                                            refined_seq_.load(std::memory_order_acquire);
                                        const bool isNewer =
                                            (prevSeq == std::numeric_limits<size_t>::max()) ||
                                            (loadedSeq > prevSeq);
                                        if (isNewer)
                                        {
                                            std::lock_guard<std::mutex> lk(refined_mtx_);
                                            refined_init_time_ = tTraj.front();
                                            refined_time_traj_ = std::move(tTraj);
                                            refined_state_traj_ = std::move(xTraj);
                                            refined_input_traj_ = std::move(uTraj);
                                            refined_seq_.store(loadedSeq, std::memory_order_release);
                                            RCLCPP_DEBUG(node_->get_logger(),
                                                         "[robust_refine] latest seq=%zu loaded "
                                                         "(N=%zu)",
                                                         loadedSeq,
                                                         refined_time_traj_.size());
                                        }
                                    }
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
