// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// std
#include <iostream>
#include <memory>
#include <algorithm>

// YARP
#include <yarp/eigen/Eigen.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/YARPConversions.h>
#include <iDynTree/YARPEigenConversions.h>
#include <iDynTree/Model.h>

// blf
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>

// walking-controllers
#include <WalkingControllers/WalkingModule/Module.h>
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/StdUtilities/Helper.h>

using namespace WalkingControllers;

void WalkingModule::propagateTime()
{
    // propagate time
    m_time += m_dT;
}

bool WalkingModule::advanceReferenceSignals()
{
    // check if vector is not initialized
    if (m_leftTrajectory.empty() || m_rightTrajectory.empty() || m_leftInContact.empty() || m_rightInContact.empty() || m_DCMPositionDesired.empty() || m_DCMVelocityDesired.empty() || m_comHeightTrajectory.empty())
    {
        yError() << "[WalkingModule::advanceReferenceSignals] Cannot advance empty reference signals.";
        return false;
    }

    m_rightTrajectory.pop_front();
    m_rightTrajectory.push_back(m_rightTrajectory.back());

    m_leftTrajectory.pop_front();
    m_leftTrajectory.push_back(m_leftTrajectory.back());

    m_rightTwistTrajectory.pop_front();
    m_rightTwistTrajectory.push_back(m_rightTwistTrajectory.back());

    m_rightAccelerationTrajectory.pop_front();
    m_rightAccelerationTrajectory.push_back(m_rightAccelerationTrajectory.back());

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_leftAccelerationTrajectory.pop_front();
    m_leftAccelerationTrajectory.push_back(m_leftAccelerationTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    m_isStancePhase.pop_front();
    m_isStancePhase.push_back(m_isStancePhase.back());

    m_desiredZMP.pop_front();
    m_desiredZMP.push_back(m_desiredZMP.back());

    m_weightInLeftDesired.pop_front();
    m_weightInLeftDesired.push_back(m_weightInLeftDesired.back());

    m_weightInRightDesired.pop_front();
    m_weightInRightDesired.push_back(m_weightInRightDesired.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if (!m_mergePoints.empty())
    {
        for (auto &mergePoint : m_mergePoints)
            mergePoint--;

        if (m_mergePoints[0] == 0)
            m_mergePoints.pop_front();
    }
    return true;
}

double WalkingModule::getPeriod()
{
    //  period of the module (seconds)
    return m_dT;
}

bool WalkingModule::setRobotModel(const yarp::os::Searchable &rf)
{
    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model", yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    yInfo() << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if (!m_loader.loadReducedModelFromFile(pathToModel, m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder &rf)
{
    // module name (used as prefix for opened ports)
    m_useMPC = rf.check("use_mpc", yarp::os::Value(false)).asBool();
    m_useQPIK = rf.check("use_QP-IK", yarp::os::Value(false)).asBool();
    m_useTSIDadmittance = rf.check("use_TSID-Admittance", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();
    m_maxInitialCoMVelocity = rf.check("max_initial_com_vel", yarp::os::Value(1.0)).asFloat64();
    std::string goalSuffix = rf.check("goal_port_suffix", yarp::os::Value("/goal:i")).asString();
    m_skipDCMController = rf.check("skip_dcm_controller", yarp::os::Value(false)).asBool();
    m_removeZMPOffset = rf.check("remove_zmp_offset", yarp::os::Value(false)).asBool();
    // m_maxTimeToWaitForGoal = rf.check("max_time_to_wait_for_goal", yarp::os::Value(1.0)).asFloat64();

    m_goalScaling.resize(3);
    if (!YarpUtilities::getVectorFromSearchable(rf, "goal_port_scaling", m_goalScaling))
    {
        yError() << "[WalkingModule::configure] Failed while reading goal_port_scaling.";
        return false;
    }

    yarp::os::Bottle &generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.01)).asFloat64();

    if (m_dT <= 0)
    {
        yError() << "[WalkingModule::configure] sampling_time is supposed to be strictly positive.";
        return false;
    }

    double maxFBDelay = rf.check("max_feedback_delay_in_s", yarp::os::Value(1.0)).asFloat64();
    m_feedbackAttemptDelay = m_dT / 10;
    m_feedbackAttempts = static_cast<size_t>(std::round(maxFBDelay / m_feedbackAttemptDelay));

    double plannerAdvanceTimeInS = rf.check("planner_advance_time_in_s", yarp::os::Value(0.18)).asFloat64();
    m_plannerAdvanceTimeSteps = static_cast<size_t>(std::round(plannerAdvanceTimeInS / m_dT)) + 2; // The additional 2 steps are because the trajectory from the planner is requested two steps in advance wrt the merge point

    std::string name;
    if (!YarpUtilities::getStringFromSearchable(generalOptions, "name", name))
    {
        yError() << "[WalkingModule::configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());

    std::string heightFrame = generalOptions.check("height_reference_frame", yarp::os::Value("com")).asString();
    m_useRootLinkForHeight = heightFrame == "root_link";

    m_robotControlHelper = std::make_unique<RobotInterface>();
    yarp::os::Bottle &robotControlHelperOptions = rf.findGroup("ROBOT_CONTROL");
    robotControlHelperOptions.append(generalOptions);
    if (!m_robotControlHelper->configureRobot(robotControlHelperOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle &forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    forceTorqueSensorsOptions.append(generalOptions);
    if (!m_robotControlHelper->configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    yarp::os::Bottle &globalCoPEvaluatorOptions = rf.findGroup("COP_EVALUATOR");
    globalCoPEvaluatorOptions.append(generalOptions);
    if (!m_globalCoPEvaluator.initialize(std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(globalCoPEvaluatorOptions)))
    {
        yError() << "[WalkingModule::configure] Unable to configure the global CoP Evaluator.";
        return false;
    }

    if (!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if (!m_rpcPort.open(rpcPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    std::string desiredUnyciclePositionPortName = "/" + getName() + goalSuffix;
    if (!m_desiredUnyciclePositionPort.open(desiredUnyciclePositionPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << desiredUnyciclePositionPortName << " port.";
        return false;
    }

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryGenerator>();
    yarp::os::Bottle &trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER");
    yarp::os::Bottle ellipseMangerOptions = rf.findGroup("FREE_SPACE_ELLIPSE_MANAGER");
    trajectoryPlannerOptions.append(generalOptions);
    trajectoryPlannerOptions.append(ellipseMangerOptions);
    if (!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }

    // initialize the Free space ellipse manager
    m_freeSpaceEllipseManager = std::make_unique<FreeSpaceEllipseManager>();
    ellipseMangerOptions.append(generalOptions);
    if (!m_freeSpaceEllipseManager->initialize(ellipseMangerOptions))
    {
        yError() << "[configure] Unable to initialize the free space ellipse manager.";
        return false;
    }

    if (m_useMPC)
    {
        // initialize the MPC controller
        m_walkingController = std::make_unique<WalkingController>();
        yarp::os::Bottle &dcmControllerOptions = rf.findGroup("DCM_MPC_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if (!m_walkingController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize the controller.";
            return false;
        }
    }
    else
    {
        // initialize the MPC controller
        m_walkingDCMReactiveController = std::make_unique<WalkingDCMReactiveController>();
        yarp::os::Bottle &dcmControllerOptions = rf.findGroup("DCM_REACTIVE_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if (!m_walkingDCMReactiveController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize the controller.";
            return false;
        }
    }

    // initialize the ZMP controller
    m_walkingZMPController = std::make_unique<WalkingZMPController>();
    yarp::os::Bottle &zmpControllerOptions = rf.findGroup("ZMP_CONTROLLER");
    zmpControllerOptions.append(generalOptions);
    if (!m_walkingZMPController->initialize(zmpControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the ZMP controller.";
        return false;
    }

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WalkingIK>();
    yarp::os::Bottle &inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if (!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(),
                                m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the ik solver";
        return false;
    }

    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle &forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if (!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the fk solver";
        return false;
    }

    if (m_useQPIK)
    {
        yarp::os::Bottle &inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");
        // TODO check if this is required
        inverseKinematicsQPSolverOptions.append(generalOptions);
        m_BLFIKSolver = std::make_unique<BLFIK>();
        auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
        paramHandler->set(inverseKinematicsQPSolverOptions);
        paramHandler->setParameter("use_root_link_for_height", m_useRootLinkForHeight);

        if (!m_BLFIKSolver->initialize(paramHandler, m_FKSolver->getKinDyn()))
        {
            yError() << "[WalkingModule::configure] Failed to configure the blf ik solver";
            return false;
        }
    }

    // initialize the TSID admittance controller
    if (m_useTSIDadmittance)
    {
        yarp::os::Bottle &TSIDOptions = rf.findGroup("TASK_SPACE_INVERSE_DYNAMICS");
        m_BLFTSIDSolver = std::make_unique<BLFTSID>();
        auto paramHandler = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
        paramHandler->set(TSIDOptions);
        paramHandler->setParameter("use_root_link_for_height", m_useRootLinkForHeight);
        if (!m_BLFTSIDSolver->initialize(paramHandler, m_FKSolver->getKinDyn()))
        {
            yError() << "[WalkingModule::configure] Failed to configure the TSID controller";
            return false;
        }

        yarp::os::Bottle &AdmittanceControllerOptions = rf.findGroup("JOINT_ADMITTANCE_CONTROLLER");
        m_jointAdmittanceController = std::make_unique<AdmittanceController>();
        paramHandler->set(AdmittanceControllerOptions);
        paramHandler->setParameter("number_of_joints", static_cast<int>(m_robotControlHelper->getActuatedDoFs()));
        if (!m_jointAdmittanceController->initialize(paramHandler))
        {
            yError() << "[WalkingModule::configure] Failed to configure the Joint Admittance controller";
            return false;
        }

        m_jointAccelerationIntegrator = std::make_unique<JointAccelerationIntegrator>();
        m_jointAccelerationIntegrator->initialize(m_robotControlHelper->getActuatedDoFs(), m_dT);
    }


    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<StableDCMModel>();
    if (!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the lipm.";
        return false;
    }

    // set PIDs gains
    yarp::os::Bottle &pidOptions = rf.findGroup("PID");
    if (!m_robotControlHelper->configurePIDHandler(pidOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the PIDs.";
        return false;
    }

    // configure the retargeting
    yarp::os::Bottle retargetingOptions = rf.findGroup("RETARGETING");
    retargetingOptions.append(generalOptions);
    m_retargetingClient = std::make_unique<RetargetingClient>();
    if (!m_retargetingClient->initialize(retargetingOptions, getName(), m_dT, m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the retargeting";
        return false;
    }

    // configure the transform helper
    yarp::os::Bottle transformHelperOptions = rf.findGroup("TRANSFORM_HELPER");
    if (!transformHelperOptions.isNull())
    {
        transformHelperOptions.append(generalOptions);
        m_transformHelper = std::make_unique<YarpUtilities::TransformHelper>();
        if (!m_transformHelper->configure(transformHelperOptions))
        {
            yWarning() << "[WalkingModule::configure] Failed to configure the transform helper. Avoiding using it.";
            m_transformHelper.reset(nullptr);
        }
        else {
            for (const std::string& frame : m_transformHelper->getAdditionalFrames())
            {
                iDynTree::FrameIndex frameIndex = m_loader.model().getFrameIndex(frame);
                if (frameIndex != iDynTree::FRAME_INVALID_INDEX)
                {
                    m_framesToStream.push_back({ frameIndex, frame });
                }
                else
                {
                    yWarning() << "[WalkingModule::configure] Frame " << frame << " not found in the model. It will not be published in the transform server.";
                }
            }
        }
    }

    // initialize the logger
    if (m_dumpData)
    {
        auto loggerOption = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(rf)->getGroup("WALKING_LOGGER").lock();
        if (loggerOption == nullptr)
        {
            yError() << "[WalkingModule::configure] Unable to get the group WALKING_LOGGER.";
            return false;
        }

        std::string logPort;
        if(!loggerOption->getParameter("remote", logPort))
        {
            yError() << "[WalkingModule::configure] Unable to get the remote from the group WALKING_LOGGER.";
            return false;
        }

        // prepend the module name to the port name
        logPort = "/" + getName() + logPort;
        loggerOption->setParameter("remote", logPort);
        if (!m_vectorsCollectionServer.initialize(loggerOption))
        {
            yError() << "[WalkingModule::configure] Unable to get the string from searchable.";
            return false;
        }

        m_vectorsCollectionServer.populateMetadata("dcm::position::measured", {"x", "y"});
        m_vectorsCollectionServer.populateMetadata("dcm::position::desired", {"x", "y"});
        m_vectorsCollectionServer.populateMetadata("dcm::velocity::desired", {"x", "y"});

        // ZMP
        m_vectorsCollectionServer.populateMetadata("zmp::measured", {"x", "y"});
        m_vectorsCollectionServer.populateMetadata("zmp::desired", {"x", "y"});

        m_vectorsCollectionServer.populateMetadata("zmp::desired_planner", {"x", "y"});

        // COM
        m_vectorsCollectionServer.populateMetadata("com::position::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("com::position::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("com::position::CoM_ZMP_controller", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("com::position::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("com::velocity::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("com::velocity::tsid", {"x", "y", "z"});

        // Left foot
        m_vectorsCollectionServer.populateMetadata("left_foot::position::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::position::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::position::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::orientation::measured", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("left_foot::orientation::desired", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("left_foot::orientation::tsid", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("left_foot::linear_velocity::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::linear_velocity::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::angular_velocity::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::angular_velocity::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::linear_force::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("left_foot::angular_torque::measured", {"x", "y", "z"});

        m_vectorsCollectionServer.populateMetadata("left_foot::angular_velocity::correction", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::angular_velocity::correction", {"x", "y", "z"});

        // Right foot
        m_vectorsCollectionServer.populateMetadata("right_foot::position::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::position::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::position::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::orientation::measured", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("right_foot::orientation::desired", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("right_foot::orientation::tsid", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("right_foot::linear_velocity::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::linear_velocity::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::angular_velocity::desired", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::angular_velocity::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::linear_force::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("right_foot::angular_torque::measured", {"x", "y", "z"});

        // Joint
        m_vectorsCollectionServer.populateMetadata("joints_state::positions::measured", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::positions::desired::ik", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::positions::retargeting", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::positions::retargeting_raw", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::velocities::measured", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::velocities::desired::ik", m_robotControlHelper->getAxesList());
        m_vectorsCollectionServer.populateMetadata("joints_state::velocities::retargeting", m_robotControlHelper->getAxesList());
        if (m_useTSIDadmittance)
        {
            m_vectorsCollectionServer.populateMetadata("joints_state::positions::desired::tsid", m_robotControlHelper->getAxesList());
            m_vectorsCollectionServer.populateMetadata("joints_state::velocity::desired::tsid", m_robotControlHelper->getAxesList());
            m_vectorsCollectionServer.populateMetadata("joints_state::acceleration::desired", m_robotControlHelper->getAxesList());
            m_vectorsCollectionServer.populateMetadata("joints_state::torque::desired::tsid", m_robotControlHelper->getAxesList());
            m_vectorsCollectionServer.populateMetadata("joints_state::torque::desired::admittance", m_robotControlHelper->getAxesList());
        }

        // root link information
        m_vectorsCollectionServer.populateMetadata("root_link::position::desired::tsid", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("root_link::position::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("root_link::orientation::measured", {"roll", "pitch", "yaw"});
        m_vectorsCollectionServer.populateMetadata("root_link::linear_velocity::measured", {"x", "y", "z"});
        m_vectorsCollectionServer.populateMetadata("root_link::angular_velocity::measured", {"x", "y", "z"});

        // collect the stance foot information
        m_vectorsCollectionServer.populateMetadata("stance_foot::is_left", {"scalar"});

        m_vectorsCollectionServer.finalizeMetadata();
    }

    // time profiler
    m_profiler = std::make_unique<BipedalLocomotion::System::TimeProfiler>();
    m_profiler->setPeriod(static_cast<int>(round(1 / m_dT)));
    if (m_useMPC)
        m_profiler->addTimer("MPC");

    m_profiler->addTimer("IK");
    m_profiler->addTimer("TSID");
    m_profiler->addTimer("Total");
    m_profiler->addTimer("Loop");
    m_profiler->addTimer("Feedback");

    // initialize some variables
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();

    // resize variables
    m_qDesiredIK.resize(m_robotControlHelper->getActuatedDoFs());
    m_dqDesiredIK.resize(m_robotControlHelper->getActuatedDoFs());
    m_qDesiredTSID.resize(m_robotControlHelper->getActuatedDoFs());
    m_dqDesiredTSID.resize(m_robotControlHelper->getActuatedDoFs());
    m_ddqDesiredTSID.resize(m_robotControlHelper->getActuatedDoFs());
    m_desiredJointTorquesTSID.resize(m_robotControlHelper->getActuatedDoFs());
    m_desiredJointTorquesAdmittance.resize(m_robotControlHelper->getActuatedDoFs());

    yInfo() << "[WalkingModule::configure] Ready to play! Please prepare the robot.";

    return true;
}

void WalkingModule::reset()
{
    if (m_useMPC)
        m_walkingController->reset();

    m_trajectoryGenerator->reset();
}

void WalkingModule::applyGoalScaling(yarp::sig::Vector &plannerInput)
{
    for (size_t i = 0; i < std::min(plannerInput.size(), m_goalScaling.size()); ++i)
    {
        plannerInput(i) *= m_goalScaling(i);
    }
}

bool WalkingModule::close()
{
    // restore PID
    m_robotControlHelper->getPIDHandler().restorePIDs();

    // close retargeting ports
    m_retargetingClient->close();

    // close the ports
    m_rpcPort.close();
    m_desiredUnyciclePositionPort.close();

    // close the connection with robot
    if (!m_robotControlHelper->close())
    {
        yError() << "[WalkingModule::close] Unable to close the connection with the robot.";
        return false;
    }

    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_walkingController.reset(nullptr);
    m_walkingZMPController.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);
    m_transformHelper.reset(nullptr);

    return true;
}

bool WalkingModule::solveBLFIK(const iDynTree::Position &desiredCoMPosition,
                               const iDynTree::Vector3 &desiredCoMVelocity,
                               const iDynTree::Rotation &desiredNeckOrientation,
                               iDynTree::VectorDynSize &output)
{
    const std::string phase = m_isStancePhase.front() ? "stance" : "walking";
    bool ok = m_BLFIKSolver->setPhase(phase);
    ok = ok && m_BLFIKSolver->setTorsoSetPoint(desiredNeckOrientation);

    ok = ok && m_BLFIKSolver->setLeftFootSetPoint(m_leftTrajectory.front(),
                                                  m_leftTwistTrajectory.front());
    ok = ok && m_BLFIKSolver->setRightFootSetPoint(m_rightTrajectory.front(),
                                                   m_rightTwistTrajectory.front());
    ok = ok && m_BLFIKSolver->setCoMSetPoint(desiredCoMPosition, desiredCoMVelocity);
    ok = ok && m_BLFIKSolver->setRetargetingJointSetPoint(m_retargetingClient->jointPositions(),
                                                          m_retargetingClient->jointVelocities());

    if (m_useRootLinkForHeight)
    {
        ok = ok && m_BLFIKSolver->setRootSetPoint(desiredCoMPosition, desiredCoMVelocity);
    }

    ok = ok && m_BLFIKSolver->solve();

    if (ok)
    {
        output = m_BLFIKSolver->getDesiredJointVelocity();
    }

    return ok;
}

bool WalkingModule::solveBLFTSID(const iDynTree::Position& desiredCoMPosition,
                                 const iDynTree::Vector3& desiredCoMVelocity,
                                 const iDynTree::Rotation& desiredTorsoRotation)
{
    iDynTree::Vector3 zero3dVector;
    zero3dVector.zero();

    iDynTree::Twist zeroTwist;
    zeroTwist.zero();

    iDynTree::VectorDynSize zeroNdofVector(m_robotControlHelper->getActuatedDoFs());
    zeroNdofVector.zero();

    // compute the desired contact wrenches
    double robotMass = m_FKSolver->getKinDyn()->getRobotModel().getTotalMass();
    iDynTree::Vector3 robotWeightinLeft;
    robotWeightinLeft.zero();
    robotWeightinLeft(2) = m_weightInLeftDesired.front() * robotMass * 9.81;

    iDynTree::Vector3 robotWeightinRight;
    robotWeightinRight.zero();
    robotWeightinRight(2) = m_weightInRightDesired.front() * robotMass * 9.81;

    iDynTree::Wrench leftContactWrench;
    iDynTree::Wrench rightContactWrench;
    leftContactWrench.zero();
    rightContactWrench.zero();
    leftContactWrench.setLinearVec3(robotWeightinLeft);
    rightContactWrench.setLinearVec3(robotWeightinRight);


    // compute ankle strategy
    computeLocalCoPCorrection(m_leftAngularVelocityCorrection, m_rightAngularVelocityCorrection);
    auto leftTwist = m_leftTwistTrajectory.front();
    auto rightTwist = m_rightTwistTrajectory.front();
    iDynTree::Vector3 temp;
    iDynTree::toEigen(temp) = iDynTree::toEigen(leftTwist.getAngularVec3()) + m_leftAngularVelocityCorrection;
    leftTwist.setAngularVec3(temp);
    iDynTree::toEigen(temp) = iDynTree::toEigen(rightTwist.getAngularVec3()) + m_rightAngularVelocityCorrection;
    rightTwist.setAngularVec3(temp);

    // set the desired set points
    bool ok{true};
    ok = ok && m_BLFTSIDSolver->setCoMTrackingSetPoint(desiredCoMPosition, desiredCoMVelocity, zero3dVector);
    ok = ok && m_BLFTSIDSolver->setLeftFootTrackingSetPoint(m_leftTrajectory.front(), leftTwist,
        m_leftAccelerationTrajectory.front());
    ok = ok && m_BLFTSIDSolver->setRightFootTrackingSetPoint(m_rightTrajectory.front(), rightTwist,
        m_rightAccelerationTrajectory.front());
    ok = ok && m_BLFTSIDSolver->setJointTrackingSetPoint(m_qDesiredIK, m_dqDesiredIK, zeroNdofVector);
    ok = ok && m_BLFTSIDSolver->setTorsoTrackingSetPoint(desiredTorsoRotation, zero3dVector, zero3dVector);
    m_BLFTSIDSolver->setLeftContactActive(m_leftInContact.front());
    m_BLFTSIDSolver->setRightContactActive(m_rightInContact.front());
    ok = ok && m_BLFTSIDSolver->setLeftContactWrenchSetPoint(leftContactWrench);
    ok = ok && m_BLFTSIDSolver->setRightContactWrenchSetPoint(rightContactWrench);
    ok = ok && m_BLFTSIDSolver->setTorqueRegularizationSetPoint(m_desiredJointTorquesTSID);

    if (m_useRootLinkForHeight)
    {
        ok = ok && m_BLFTSIDSolver->setRootTrackingSetPoint(desiredCoMPosition, desiredCoMVelocity, zero3dVector);
    }

    if (!ok)
    {
        yError() << "[WalkingModule::solveBLFTSID] Unable to set the TSID set points.";
    }

    // solve the optimization problem
    ok = ok && m_BLFTSIDSolver->solve();

    if (!ok)
    {
        yError() << "[WalkingModule::solveBLFTSID] Unable to solve the optimization problem.";
    }

    return ok;
}

void WalkingModule::getBLFTSIDOutput(iDynTree::VectorDynSize &jointDesiredAcceleration,
                                     iDynTree::VectorDynSize &jointDesiredTorque)
{
    jointDesiredAcceleration = m_BLFTSIDSolver->getDesiredJointAcceleration();
    jointDesiredTorque = m_BLFTSIDSolver->getDesiredJointTorque();
}

bool WalkingModule::storeBLFTSIDTrajectories()
{

    // set robot state to TSID joint values
    if (!m_FKSolver->setInternalRobotState(m_qDesiredTSID, m_dqDesiredTSID))
    {
        yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
        return false;
    }

    m_CoMPositionTSID = m_FKSolver->getCoMPosition();
    m_CoMVelocityTSID = m_FKSolver->getCoMVelocity();

    m_rootLinkPositionTSID = m_FKSolver->getRootLinkToWorldTransform().getPosition();

    m_leftFootPoseTSID = m_FKSolver->getLeftFootToWorldTransform();
    m_rightFootPoseTSID = m_FKSolver->getRightFootToWorldTransform();

    m_leftFootTwistTSID = m_FKSolver->getLeftFootTwist();
    m_rightFootTwistTSID = m_FKSolver->getRightFootTwist();

    return true;
}

bool WalkingModule::advanceJointAdmittanceController(const iDynTree::VectorDynSize & jointTorqueFeedforward,
                                                     const iDynTree::VectorDynSize & jointDesiredPosition,
                                                     const iDynTree::VectorDynSize & jointDesiredVelocity,
                                                     iDynTree::VectorDynSize & jointDesiredTorque)
{
    bool ok = m_jointAdmittanceController->setInput(iDynTree::toEigen(jointTorqueFeedforward),
                                                    iDynTree::toEigen(jointDesiredPosition),
                                                    iDynTree::toEigen(jointDesiredVelocity),
                                                    iDynTree::toEigen(m_robotControlHelper->getJointPosition()),
                                                    iDynTree::toEigen(m_robotControlHelper->getJointVelocity()));

    ok = ok && m_jointAdmittanceController->advance();
    if (ok)
    {
        iDynTree::toEigen(jointDesiredTorque) = m_jointAdmittanceController->getOutput();
    }
    return ok;
}

bool WalkingModule::computeGlobalCoP(Eigen::Ref<Eigen::Vector2d> globalCoP)
{
    BipedalLocomotion::Contacts::ContactWrench leftFootContact, rightFootContact;
    leftFootContact.wrench = iDynTree::toEigen(m_robotControlHelper->getLeftWrench().asVector());
    leftFootContact.pose = BipedalLocomotion::Conversions::toManifPose(m_FKSolver->getLeftFootToWorldTransform());

    rightFootContact.wrench = iDynTree::toEigen(m_robotControlHelper->getRightWrench().asVector());
    rightFootContact.pose = BipedalLocomotion::Conversions::toManifPose(m_FKSolver->getRightFootToWorldTransform());

    if (!m_globalCoPEvaluator.setInput({leftFootContact, rightFootContact}))
    {
        yError() << "[WalkingModule::computeGlobalCoP] Unable to set the contact wrenches to globalCoPEvaluator.";
        return false;
    }

    if (!m_globalCoPEvaluator.advance())
    {
        yError() << "[WalkingModule::computeGlobalCoP] Unable to compute the global CoP.";
        return false;
    }

    globalCoP = m_globalCoPEvaluator.getOutput().head<2>();

    return true;
}

bool WalkingModule::computeLocalCoPCorrection(Eigen::Ref<Eigen::Vector3d> leftCorrection, Eigen::Ref<Eigen::Vector3d> rightCorrection)
{
    BipedalLocomotion::Contacts::ContactWrench leftFootContact, rightFootContact;
    leftFootContact.wrench = iDynTree::toEigen(m_robotControlHelper->getLeftWrench().asVector());
    leftFootContact.pose = BipedalLocomotion::Conversions::toManifPose(m_FKSolver->getLeftFootToWorldTransform());

    rightFootContact.wrench = iDynTree::toEigen(m_robotControlHelper->getRightWrench().asVector());
    rightFootContact.pose = BipedalLocomotion::Conversions::toManifPose(m_FKSolver->getRightFootToWorldTransform());

    Eigen::Vector3d leftCoP, rightCoP;
    leftCoP = leftFootContact.wrench.getLocalCoP();
    rightCoP = rightFootContact.wrench.getLocalCoP();

    Eigen::Vector3d errorCoPLeft;
    Eigen::Vector3d errorCoPRight;

    errorCoPLeft = -leftCoP;
    double gain{0.1};
    Eigen::Vector3d localCorrectionLeft;

    localCorrectionLeft(0) = errorCoPLeft(1) * (-gain);
    localCorrectionLeft(1) = errorCoPLeft(0) * gain;
    localCorrectionLeft(2) = 0.0;

    errorCoPRight = -rightCoP;
    Eigen::Vector3d localCorrectionRight;

    localCorrectionRight(0) = errorCoPRight(1) * (-gain);
    localCorrectionRight(1) = errorCoPRight(0) * gain;
    localCorrectionRight(2) = 0.0;

    // set to 0 if force too small
    if (leftFootContact.wrench.force().z() < 10)
    {
        localCorrectionLeft.setZero();
    }
    if (rightFootContact.wrench.force().z() < 10)
    {
        localCorrectionRight.setZero();
    }

    leftCorrection = leftFootContact.pose.rotation() * localCorrectionLeft;
    leftCorrection(2) = 0.0;
    rightCorrection = rightFootContact.pose.rotation() * localCorrectionRight;
    rightCorrection(2) = 0.0;

    return true;
}


bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_robotState == WalkingFSM::Preparing)
    {
        if (!m_robotControlHelper->getFeedbacksRaw(m_feedbackAttempts, m_feedbackAttemptDelay))
        {
            yError() << "[updateModule] Unable to get the feedback.";
            return false;
        }

        bool motionDone = false;
        if (!m_robotControlHelper->checkMotionDone(motionDone))
        {
            yError() << "[WalkingModule::updateModule] Unable to check if the motion is done";
            yInfo() << "[WalkingModule::updateModule] Try to prepare again";
            reset();
            m_robotState = WalkingFSM::Stopped;
            return true;
        }
        if (motionDone)
        {
            // send the reference again in order to reduce error
            if (!m_robotControlHelper->setDirectPositionReferences(m_qDesiredIK))
            {
                yError() << "[prepareRobot] Error while setting the initial position using "
                         << "POSITION DIRECT mode.";
                yInfo() << "[WalkingModule::updateModule] Try to prepare again";
                reset();
                m_robotState = WalkingFSM::Stopped;
                return true;
            }

            yarp::sig::Vector buffer(m_qDesiredIK.size());
            iDynTree::toYarp(m_qDesiredIK, buffer);
            // instantiate Integrators object

            yarp::sig::Matrix jointLimits(m_robotControlHelper->getActuatedDoFs(), 2);
            for (int i = 0; i < m_robotControlHelper->getActuatedDoFs(); i++)
            {
                jointLimits(i, 0) = m_robotControlHelper->getPositionLowerLimits()(i);
                jointLimits(i, 1) = m_robotControlHelper->getPositionUpperLimits()(i);
            }
            m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer, jointLimits);

            if (m_useTSIDadmittance){
                m_qDesiredTSID = m_qDesiredIK;
                m_dqDesiredTSID.zero();
                m_jointAccelerationIntegrator->setState(iDynTree::toEigen(m_qDesiredTSID),
                iDynTree::toEigen(m_dqDesiredTSID));
            }

            // reset the models
            m_walkingZMPController->reset(m_DCMPositionDesired.front());
            m_stableDCMModel->reset(m_DCMPositionDesired.front());

            // reset the retargeting
            if (!m_robotControlHelper->getFeedbacks(m_feedbackAttempts, m_feedbackAttemptDelay))
            {
                yError() << "[WalkingModule::updateModule] Unable to get the feedback.";
                return false;
            }

            if (!updateFKSolver())
            {
                yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
                return false;
            }

            // reset the retargeting client with the desired robot data
            iDynTree::VectorDynSize zero(m_qDesiredIK.size());
            zero.zero();
            // reset the internal robot state of the kindyn object
            if (!m_FKSolver->setInternalRobotState(m_qDesiredIK, zero))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the robot state before resetting the retargeting client.";
                return false;
            }

            if (!m_retargetingClient->reset(*m_FKSolver))
            {
                yError() << "[WalkingModule::updateModule] Unable to reset the retargeting client.";
                return false;
            }

            // reset the internal robot state of the kindyn object
            if (!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                   m_robotControlHelper->getJointVelocity()))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the robot state after resetting the retargeting client.";
                return false;
            }

            m_firstRun = true;

            if (m_useRootLinkForHeight)
            {
                m_comHeightOffset = m_FKSolver->getRootLinkToWorldTransform().getPosition()(2) - m_FKSolver->getCoMPosition()(2);
                yInfo() << "[WalkingModule::updateModule] rootlink offset " << m_comHeightOffset << ".";
            }
            else
            {
                m_comHeightOffset = 0.0;
            }

            // warm start desired torque for tsid
            iDynTree::VectorDynSize generalizedGravityForces(m_robotControlHelper->getActuatedDoFs()+6);
            m_FKSolver->getKinDyn()->generalizedGravityForces(generalizedGravityForces);
            iDynTree::toEigen(m_desiredJointTorquesTSID) =
                iDynTree::toEigen(generalizedGravityForces).tail(m_robotControlHelper->getActuatedDoFs());

            yInfo() << "[WalkingModule::updateModule] Warm start the desired torque for tsid."
                        << m_desiredJointTorquesTSID.toString();

            m_robotState = WalkingFSM::Prepared;

            yInfo() << "[WalkingModule::updateModule] The robot is prepared.";
        }
    }
    else if (m_robotState == WalkingFSM::Walking)
    {
        iDynTree::Vector2 measuredZMP;

        bool resetTrajectory = false;

        m_profiler->setInitTime("Total");

        // check desired planner input
        yarp::sig::Vector *desiredUnicyclePosition = nullptr;
        desiredUnicyclePosition = m_desiredUnyciclePositionPort.read(false);
        if (desiredUnicyclePosition != nullptr)
        {
            applyGoalScaling(*desiredUnicyclePosition);
            if (!setPlannerInput(*desiredUnicyclePosition))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the planner input";
                return false;
            }
        //     m_lastSetGoalTime = m_time;
        // }
        // else if (!m_firstRun && ((m_time - m_lastSetGoalTime) > m_maxTimeToWaitForGoal))
        // {
        //     yarp::sig::Vector dummy(3, 0.0);
        //     if (!setPlannerInput(dummy))
        //     {
        //         yError() << "[WalkingModule::updateModule] Unable to set the planner input";
        //         return false;
        //     }
        }

        // if a new trajectory is required check if its the time to evaluate the new trajectory or
        // the time to attach new one
        if (m_newTrajectoryRequired)
        {
            // when we are near to the merge point the new trajectory is evaluated
            if (m_newTrajectoryMergeCounter == m_plannerAdvanceTimeSteps)
            {

                double initTimeTrajectory;
                initTimeTrajectory = m_time + m_newTrajectoryMergeCounter * m_dT;

                iDynTree::Transform measuredTransform = m_isLeftFixedFrame.front() ? m_rightTrajectory[m_newTrajectoryMergeCounter] : m_leftTrajectory[m_newTrajectoryMergeCounter];

                // ask for a new trajectory
                if (!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                        measuredTransform, m_newTrajectoryMergeCounter,
                                        m_plannerInput))
                {
                    yError() << "[WalkingModule::updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if (m_newTrajectoryMergeCounter == 2)
            {
                if (!updateTrajectories(m_newTrajectoryMergeCounter))
                {
                    yError() << "[WalkingModule::updateModule] Error while updating trajectories. They were not computed yet.";
                    return false;
                }
                m_newTrajectoryRequired = false;
                resetTrajectory = true;
            }

            m_newTrajectoryMergeCounter--;
        }

        if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
        {
            if (!m_robotControlHelper->getPIDHandler().updatePhases(m_leftInContact, m_rightInContact, m_time))
            {
                yError() << "[WalkingModule::updateModule] Unable to get the update PID.";
                return false;
            }
        }

        m_profiler->setInitTime("Feedback");

        // get feedbacks and evaluate useful quantities
        if (!m_robotControlHelper->getFeedbacks(m_feedbackAttempts, m_feedbackAttemptDelay))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the feedback.";
            return false;
        }

        m_profiler->setEndTime("Feedback");

        auto retargetingPhase = m_isStancePhase.front() ? RetargetingClient::Phase::Stance : RetargetingClient::Phase::Walking;
        m_retargetingClient->setPhase(retargetingPhase);

        if (!m_retargetingClient->getFeedback())
        {
            yError() << "[WalkingModule::updateModule] Unable to get the feedback from the retargeting client.";
            return false;
        }

        if (!updateFKSolver())
        {
            yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
            return false;
        }

        // compute the global CoP
        if (!computeGlobalCoP(iDynTree::toEigen(measuredZMP)))
        {
            yError() << "[WalkingModule::updateModule] Unable to compute the global CoP.";
            return false;
        }

        // remove the ZMP offset if required
        if (m_removeZMPOffset)
        {
            // remove rotated offset
            iDynTree::Rotation yawRotation = this->computeAverageYawRotationFromPlannedFeet();
            m_zmpOffset = yawRotation * m_zmpOffsetLocal;
            iDynTree::toEigen(measuredZMP) += iDynTree::toEigen(m_zmpOffset).head<2>();
        }

        // evaluate 3D-LIPM reference signal
        m_stableDCMModel->setInput(m_DCMPositionDesired.front());
        if (!m_stableDCMModel->integrateModel())
        {
            yError() << "[WalkingModule::updateModule] Unable to propagate the 3D-LIPM.";
            return false;
        }

        // DCM controller
        if (m_useMPC)
        {
            // Model predictive controller
            m_profiler->setInitTime("MPC");
            if (!m_walkingController->setConvexHullConstraint(m_leftTrajectory, m_rightTrajectory,
                                                              m_leftInContact, m_rightInContact))
            {
                yError() << "[WalkingModule::updateModule] unable to evaluate the convex hull.";
                return false;
            }

            if (!m_walkingController->setFeedback(m_FKSolver->getDCM()))
            {
                yError() << "[WalkingModule::updateModule] unable to set the feedback.";
                return false;
            }

            if (!m_walkingController->setReferenceSignal(m_DCMPositionDesired, resetTrajectory))
            {
                yError() << "[WalkingModule::updateModule] unable to set the reference Signal.";
                return false;
            }

            if (!m_walkingController->solve())
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the problem.";
                return false;
            }

            m_profiler->setEndTime("MPC");
        }
        else
        {
            m_walkingDCMReactiveController->setFeedback(m_FKSolver->getDCM());
            m_walkingDCMReactiveController->setReferenceSignal(m_DCMPositionDesired.front(),
                                                               m_DCMVelocityDesired.front());

            if (!m_walkingDCMReactiveController->evaluateControl())
            {
                yError() << "[WalkingModule::updateModule] Unable to evaluate the DCM control output.";
                return false;
            }
        }

        // inner COM-ZMP controller
        // if the the norm of desired DCM velocity is lower than a threshold then the robot
        // is stopped
        m_walkingZMPController->setPhase(m_isStancePhase.front());

        iDynTree::Vector2 desiredZMP;
        if (m_skipDCMController)
        {
            desiredZMP = m_desiredZMP.front();
        }
        else
        {
            if (m_useMPC)
                desiredZMP = m_walkingController->getControllerOutput();
            else
                desiredZMP = m_walkingDCMReactiveController->getControllerOutput();
        }

        // set feedback and the desired signal

        m_walkingZMPController->setFeedback(measuredZMP, m_FKSolver->getCoMPosition());
        m_walkingZMPController->setReferenceSignal(desiredZMP, m_stableDCMModel->getCoMPosition(),
                                                   m_stableDCMModel->getCoMVelocity());

        if (!m_walkingZMPController->evaluateControl())
        {
            yError() << "[WalkingModule::updateModule] Unable to evaluate the ZMP control output.";
            return false;
        }

        iDynTree::Vector2 outputZMPCoMControllerPosition, outputZMPCoMControllerVelocity;
        if (!m_walkingZMPController->getControllerOutput(outputZMPCoMControllerPosition,
                                                         outputZMPCoMControllerVelocity))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the ZMP controller output.";
            return false;
        }

        // inverse kinematics
        m_profiler->setInitTime("IK");

        iDynTree::Position desiredCoMPositionIK;
        desiredCoMPositionIK(0) = m_stableDCMModel->getCoMPosition()(0);
        desiredCoMPositionIK(1) = m_stableDCMModel->getCoMPosition()(1);
        desiredCoMPositionIK(2) = m_retargetingClient->comHeight() + m_comHeightOffset;

        iDynTree::Vector3 desiredCoMVelocityIK;
        desiredCoMVelocityIK(0) = m_stableDCMModel->getCoMVelocity()(0);
        desiredCoMVelocityIK(1) = m_stableDCMModel->getCoMVelocity()(1);
        desiredCoMVelocityIK(2) = m_retargetingClient->comHeightVelocity();

        iDynTree::Position desiredCoMPositionTSID;
        desiredCoMPositionTSID(0) = outputZMPCoMControllerPosition(0);
        desiredCoMPositionTSID(1) = outputZMPCoMControllerPosition(1);
        desiredCoMPositionTSID(2) = m_retargetingClient->comHeight() + m_comHeightOffset;

        iDynTree::Vector3 desiredCoMVelocityTSID;
        desiredCoMVelocityTSID(0) = outputZMPCoMControllerVelocity(0);
        desiredCoMVelocityTSID(1) = outputZMPCoMControllerVelocity(1);
        desiredCoMVelocityTSID(2) = m_retargetingClient->comHeightVelocity();

        if (m_firstRun)
        {
            double comVelocityNorm = iDynTree::toEigen(desiredCoMVelocityTSID).norm();

            if (comVelocityNorm > m_maxInitialCoMVelocity)
            {
                yError() << "[WalkingModule::updateModule] The initial CoM velocity is too high (desired:"
                         << comVelocityNorm << "limit:" << m_maxInitialCoMVelocity << ")."
                         << "ZMP measured: " << measuredZMP.toString()
                         << "Desired ZMP" << desiredZMP.toString()
                         << "measured com position" << m_FKSolver->getCoMPosition().toString()
                         << "desired com position" << m_stableDCMModel->getCoMPosition().toString()
                         << "desired com velocity" << m_stableDCMModel->getCoMVelocity().toString();
                return false;
            }
        }

        // evaluate desired neck transformation
        iDynTree::Rotation modifiedInertial;
        iDynTree::Rotation yawRotation = this->computeAverageYawRotationFromPlannedFeet();

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if (m_useQPIK)
        {
            // integrate dq because velocity control mode seems not available
            yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
            yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());

            if (!m_FKSolver->setInternalRobotState(m_qDesiredIK, m_dqDesiredIK))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

            yawRotation = yawRotation.inverse() * m_trajectoryGenerator->getChestAdditionalRotation();

            if (!solveBLFIK(desiredCoMPositionIK,
                            desiredCoMVelocityIK,
                            yawRotation,
                            m_dqDesiredIK))
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the QP problem with "
                            "blf ik.";
                return false;
            }

            iDynTree::toYarp(m_dqDesiredIK, bufferVelocity);

            bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
            iDynTree::toiDynTree(bufferPosition, m_qDesiredIK);

            // update robot state with IK solution
            if (!m_FKSolver->setInternalRobotState(m_qDesiredIK, m_dqDesiredIK))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

            // restore robot state

            if (!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                   m_robotControlHelper->getJointVelocity()))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }
        }
        else
        {
            if (m_IKSolver->usingAdditionalRotationTarget())
            {
                if (!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
                {
                    yError() << "[WalkingModule::updateModule] Error updating the inertia to world frame rotation.";
                    return false;
                }

                if (!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
                {
                    yError() << "[WalkingModule::updateModule] Error while setting the feedback to the inverse Kinematics.";
                    return false;
                }

                if (!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                           desiredCoMPositionIK, m_qDesiredIK))
                {
                    yError() << "[WalkingModule::updateModule] Error during the inverse Kinematics iteration.";
                    return false;
                }
            }
        }
        m_profiler->setEndTime("IK");

        if (m_transformHelper)
        {
            if (!m_transformHelper->setBaseTransform(m_FKSolver->getRootLinkToWorldTransform()))
            {
                yWarning() << "[WalkingModule::updateModule] Unable to publish the base transform.";
            }

            if (!m_transformHelper->setJoystickTransform(m_trajectoryGenerator->getUnicyclePose()))
            {
                yWarning() << "[WalkingModule::updateModule] Unable to publish the joystick transform.";
            }

            auto kinDynPointer = m_FKSolver->getKinDyn();

            for (const auto& frame : m_framesToStream)
            {
                if (!m_transformHelper->setTransform(frame.second, kinDynPointer->getWorldTransform(frame.first)))
                {
                    yWarning() << "[WalkingModule::updateModule] Unable to publish the transform of" << frame.second;
                }
            }
        }

        // tsid-admittance
        m_profiler->setInitTime("TSID");
        if (m_useTSIDadmittance){

            // set robot state to TSID joint values
            if (!m_FKSolver->setInternalRobotState(m_qDesiredTSID, m_dqDesiredTSID))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

            if(!solveBLFTSID(desiredCoMPositionTSID,
                desiredCoMVelocityTSID, yawRotation)){
                yError() << "[WalkingModule::updateModule] Unable to solve the TSID problem";
                return false;
            }

            getBLFTSIDOutput(m_ddqDesiredTSID, m_desiredJointTorquesTSID);

            // double integration of joint acceleration to get joint position
            m_jointAccelerationIntegrator->setInput(iDynTree::toEigen(m_ddqDesiredTSID));
            m_jointAccelerationIntegrator->oneStepIntegration();
            iDynTree::toEigen(m_dqDesiredTSID) = m_jointAccelerationIntegrator->getJointVelocity();
            iDynTree::toEigen(m_qDesiredTSID) = m_jointAccelerationIntegrator->getJointPosition();

            storeBLFTSIDTrajectories();

            // admittance controller
            iDynTree::VectorDynSize m_desiredJointVelocitiesAdmittance(m_robotControlHelper->getActuatedDoFs());
            m_desiredJointVelocitiesAdmittance.zero();
            advanceJointAdmittanceController(m_desiredJointTorquesTSID,
                                             m_qDesiredTSID,
                                             m_desiredJointVelocitiesAdmittance,
                                             m_desiredJointTorquesAdmittance);

            // restore robot state
            if (!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                   m_robotControlHelper->getJointVelocity()))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }


        }
        m_profiler->setEndTime("TSID");


        if (m_useTSIDadmittance){

            if (!m_robotControlHelper->setTorqueReferences(m_desiredJointTorquesAdmittance))
            {
                yError() << "[WalkingModule::updateModule] Error while setting the reference position to iCub.";
                return false;
            }
            // if (!m_robotControlHelper->setDirectPositionReferences(m_qDesiredTSID))
            // {
            //     yError() << "[WalkingModule::updateModule] Error while setting the reference position to iCub.";
            //     return false;
            // }

        } else {
            if (!m_robotControlHelper->setDirectPositionReferences(m_qDesiredIK))
            {
                yError() << "[WalkingModule::updateModule] Error while setting the reference position to iCub.";
                return false;
            }
        }

        // send data to the logger
        if (m_dumpData)
        {
            iDynTree::Vector2 desiredZMP;
            if (m_useMPC)
                desiredZMP = m_walkingController->getControllerOutput();
            else
                desiredZMP = m_walkingDCMReactiveController->getControllerOutput();

            iDynTree::Vector3 measuredCoM = m_FKSolver->getCoMPosition();

            if (m_useRootLinkForHeight)
            {
                measuredCoM(2) = m_FKSolver->getRootLinkToWorldTransform().getPosition()(2);
            }

            auto leftFoot = m_FKSolver->getLeftFootToWorldTransform();
            auto rightFoot = m_FKSolver->getRightFootToWorldTransform();

            m_vectorsCollectionServer.prepareData();
            m_vectorsCollectionServer.clearData();

            // DCM
            m_vectorsCollectionServer.populateData("dcm::position::measured", m_FKSolver->getDCM());
            m_vectorsCollectionServer.populateData("dcm::position::desired", m_DCMPositionDesired.front());
            m_vectorsCollectionServer.populateData("dcm::velocity::desired", m_DCMVelocityDesired.front());

            // ZMP
            m_vectorsCollectionServer.populateData("zmp::measured", measuredZMP);
            m_vectorsCollectionServer.populateData("zmp::desired", desiredZMP);

            // "zmp_des_planner_x", "zmp_des_planner_y",
            const iDynTree::Vector2 &desiredZMPPlanner = m_desiredZMP.front();
            m_vectorsCollectionServer.populateData("zmp::desired_planner", desiredZMPPlanner);

            // COM
            m_vectorsCollectionServer.populateData("com::position::measured", measuredCoM);

            // Manual definition of this value to add also the planned CoM height
            std::vector<double> CoMPositionDesired(3);
            CoMPositionDesired[0] = m_stableDCMModel->getCoMPosition().data()[0];
            CoMPositionDesired[1] = m_stableDCMModel->getCoMPosition().data()[1];
            CoMPositionDesired[2] = m_retargetingClient->comHeight() + m_comHeightOffset;

            m_vectorsCollectionServer.populateData("com::position::desired", CoMPositionDesired);
            m_vectorsCollectionServer.populateData("com::position::CoM_ZMP_controller", desiredCoMPositionTSID);
            m_vectorsCollectionServer.populateData("com::position::tsid", m_CoMPositionTSID);

            // Manual definition of this value to add also the planned CoM height velocity
            std::vector<double> CoMVelocityDesired(3);
            CoMVelocityDesired[0] = m_stableDCMModel->getCoMVelocity().data()[0];
            CoMVelocityDesired[1] = m_stableDCMModel->getCoMVelocity().data()[1];
            CoMVelocityDesired[2] = m_retargetingClient->comHeightVelocity();

            m_vectorsCollectionServer.populateData("com::velocity::desired", CoMVelocityDesired);
            m_vectorsCollectionServer.populateData("com::velocity::tsid", m_CoMVelocityTSID);

            // Left foot position
            m_vectorsCollectionServer.populateData("left_foot::position::measured", leftFoot.getPosition());
            m_vectorsCollectionServer.populateData("left_foot::position::desired", m_leftTrajectory.front().getPosition());
            m_vectorsCollectionServer.populateData("left_foot::position::tsid", m_leftFootPoseTSID.getPosition());

            // Left foot orientation
            const iDynTree::Vector3 leftFootOrientationMeasured = leftFoot.getRotation().asRPY();
            m_vectorsCollectionServer.populateData("left_foot::orientation::measured", leftFootOrientationMeasured);

            const iDynTree::Vector3 leftFootOrientationDesired = m_leftTrajectory.front().getRotation().asRPY();
            m_vectorsCollectionServer.populateData("left_foot::orientation::desired", leftFootOrientationDesired);
            m_vectorsCollectionServer.populateData("left_foot::orientation::tsid", m_leftFootPoseTSID.getRotation().asRPY());

            // "lf_des_dx", "lf_des_dy", "lf_des_dz",
            // "lf_des_droll", "lf_des_dpitch", "lf_des_dyaw",
            m_vectorsCollectionServer.populateData("left_foot::linear_velocity::desired", m_leftTwistTrajectory.front().getLinearVec3());
            m_vectorsCollectionServer.populateData("left_foot::linear_velocity::tsid", m_leftFootTwistTSID.getLinearVec3());
            m_vectorsCollectionServer.populateData("left_foot::angular_velocity::desired", m_leftTwistTrajectory.front().getAngularVec3());
            m_vectorsCollectionServer.populateData("left_foot::angular_velocity::tsid", m_leftFootTwistTSID.getAngularVec3());

            m_vectorsCollectionServer.populateData("left_foot::angular_velocity::correction", m_leftAngularVelocityCorrection);
            m_vectorsCollectionServer.populateData("right_foot::angular_velocity::correction", m_rightAngularVelocityCorrection);

            // "lf_force_x", "lf_force_y", "lf_force_z",
            // "lf_force_roll", "lf_force_pitch", "lf_force_yaw",
            m_vectorsCollectionServer.populateData("left_foot::linear_force::measured", m_robotControlHelper->getLeftWrench().getLinearVec3());
            m_vectorsCollectionServer.populateData("left_foot::angular_torque::measured", m_robotControlHelper->getLeftWrench().getAngularVec3());

            // Right foot position
            m_vectorsCollectionServer.populateData("right_foot::position::measured", rightFoot.getPosition());
            m_vectorsCollectionServer.populateData("right_foot::position::desired", m_rightTrajectory.front().getPosition());
            m_vectorsCollectionServer.populateData("right_foot::position::tsid", m_rightFootPoseTSID.getPosition());

            // Right foot orientation
            const iDynTree::Vector3 rightFootOrientationMeasured = rightFoot.getRotation().asRPY();
            m_vectorsCollectionServer.populateData("right_foot::orientation::measured", rightFootOrientationMeasured);
            const iDynTree::Vector3 rightFootOrientationDesired = m_rightTrajectory.front().getRotation().asRPY();
            m_vectorsCollectionServer.populateData("right_foot::orientation::desired", rightFootOrientationDesired);
            m_vectorsCollectionServer.populateData("right_foot::orientation::tsid", m_rightFootPoseTSID.getRotation().asRPY());

            // "rf_des_dx", "rf_des_dy", "rf_des_dz",
            // "rf_des_droll", "rf_des_dpitch", "rf_des_dyaw",
            m_vectorsCollectionServer.populateData("right_foot::linear_velocity::desired", m_rightTwistTrajectory.front().getLinearVec3());
            m_vectorsCollectionServer.populateData("right_foot::linear_velocity::tsid", m_rightFootTwistTSID.getLinearVec3());
            m_vectorsCollectionServer.populateData("right_foot::angular_velocity::desired", m_rightTwistTrajectory.front().getAngularVec3());
            m_vectorsCollectionServer.populateData("right_foot::angular_velocity::tsid", m_rightFootTwistTSID.getAngularVec3());

            // "rf_force_x", "rf_force_y", "rf_force_z",
            // "rf_force_roll", "rf_force_pitch", "rf_force_yaw",
            m_vectorsCollectionServer.populateData("right_foot::linear_force::measured", m_robotControlHelper->getRightWrench().getLinearVec3());
            m_vectorsCollectionServer.populateData("right_foot::angular_torque::measured", m_robotControlHelper->getRightWrench().getAngularVec3());

            // Joint
            m_vectorsCollectionServer.populateData("joints_state::positions::measured", m_robotControlHelper->getJointPosition());
            m_vectorsCollectionServer.populateData("joints_state::positions::desired::ik", m_qDesiredIK);
            m_vectorsCollectionServer.populateData("joints_state::positions::retargeting", m_retargetingClient->jointPositions());
            m_vectorsCollectionServer.populateData("joints_state::positions::retargeting_raw", m_retargetingClient->rawJointPositions());
            m_vectorsCollectionServer.populateData("joints_state::velocities::measured", m_robotControlHelper->getJointVelocity());
            m_vectorsCollectionServer.populateData("joints_state::velocities::desired::ik", m_dqDesiredIK);
            m_vectorsCollectionServer.populateData("joints_state::velocities::retargeting", m_retargetingClient->jointVelocities());
            if (m_useTSIDadmittance)
            {
                m_vectorsCollectionServer.populateData("joints_state::positions::desired::tsid", m_qDesiredTSID);
                m_vectorsCollectionServer.populateData("joints_state::velocity::desired::tsid", m_dqDesiredTSID);
                m_vectorsCollectionServer.populateData("joints_state::acceleration::desired", m_ddqDesiredTSID);
                m_vectorsCollectionServer.populateData("joints_state::torque::desired::tsid", m_desiredJointTorquesTSID);
                m_vectorsCollectionServer.populateData("joints_state::torque::desired::admittance", m_desiredJointTorquesAdmittance);
            }

            // root link information
            m_vectorsCollectionServer.populateData("root_link::position::desired::tsid", m_rootLinkPositionTSID);
            m_vectorsCollectionServer.populateData("root_link::position::measured", m_FKSolver->getRootLinkToWorldTransform().getPosition());
            m_vectorsCollectionServer.populateData("root_link::orientation::measured", m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY());
            m_vectorsCollectionServer.populateData("root_link::linear_velocity::measured", m_FKSolver->getRootLinkVelocity().getLinearVec3());
            m_vectorsCollectionServer.populateData("root_link::angular_velocity::measured", m_FKSolver->getRootLinkVelocity().getAngularVec3());

            // collect the stance foot information
            const double isLeftFootFixed = m_isLeftFixedFrame.front() ? 1.0 : 0.0;
            m_vectorsCollectionServer.populateData("stance_foot::is_left", std::array<double, 1>{isLeftFootFixed});

            m_vectorsCollectionServer.sendData();
        }


        propagateTime();

        // advance all the signals
        advanceReferenceSignals();


        m_retargetingClient->setRobotBaseOrientation(yawRotation.inverse());

        if (!m_firstRun)
        {
            m_profiler->setEndTime("Loop");
        }

        m_firstRun = false;

        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();

        m_profiler->setInitTime("Loop");
    }
    return true;
}

iDynTree::Rotation WalkingModule::computeAverageYawRotationFromPlannedFeet() const
{
    const double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
    const double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

    const double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                      std::cos(yawLeft) + std::cos(yawRight));
    return iDynTree::Rotation::RotZ(meanYaw);
}

bool WalkingModule::prepareRobot(bool onTheFly)
{
    if (m_robotState != WalkingFSM::Configured && m_robotState != WalkingFSM::Stopped)
    {
        yError() << "[WalkingModule::prepareRobot] The robot can be prepared only at the "
                 << "beginning or when the controller is stopped.";
        return false;
    }

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    if (!m_robotControlHelper->getFeedbacksRaw(m_feedbackAttempts, m_feedbackAttemptDelay))
    {
        yError() << "[WalkingModule::prepareRobot] Unable to get the feedback.";
        return false;
    }

    if (onTheFly)
    {
        if (!m_FKSolver->setBaseOnTheFly())
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set the onTheFly base.";
            return false;
        }

        if (!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                               m_robotControlHelper->getJointVelocity()))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set joint state.";
            return false;
        }

        // evaluate the left to right transformation, the inertial frame is on the left foot
        iDynTree::Transform leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

        // evaluate the first trajectory. The robot does not move!
        if (!generateFirstTrajectories(leftToRightTransform))
        {
            yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
            return false;
        }
    }
    else
    {
        // evaluate the first trajectory. The robot does not move! So the first trajectory
        if (!generateFirstTrajectories())
        {
            yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
            return false;
        }
    }

    // reset the gains
    if (m_robotControlHelper->getPIDHandler().usingGainScheduling())
    {
        if (!(m_robotControlHelper->getPIDHandler().reset()))
            return false;
    }

    if (!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
    {
        yError() << "[WalkingModule::prepareRobot] Error while setting the feedback to the IK solver.";
        return false;
    }

    iDynTree::Position desiredCoMPosition;
    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0);
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();

    if (m_IKSolver->usingAdditionalRotationTarget())
    {
        // it is important to notice that the inertial frames rotate with the robot
        iDynTree::Rotation modifiedInertial;
        iDynTree::Rotation yawRotation = this->computeAverageYawRotationFromPlannedFeet();

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if (!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[WalkingModule::prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }

    if (!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                               desiredCoMPosition, m_qDesiredIK))
    {
        yError() << "[WalkingModule::prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }

    std::cerr << "q desired IK " << Eigen::VectorXd(iDynTree::toEigen(m_qDesiredIK) * 180 / M_PI).transpose() << std::endl;

    if (!m_robotControlHelper->setPositionReferences(m_qDesiredIK, 5.0))
    {
        yError() << "[WalkingModule::prepareRobot] Error while setting the initial position.";
        return false;
    }

    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_robotState = WalkingFSM::Preparing;
    }

    return true;
}

bool WalkingModule::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform)
{
    if (m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if (!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if (!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::generateFirstTrajectories()
{
    if (m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if (m_robotControlHelper->isExternalRobotBaseUsed())
    {
        if (!m_trajectoryGenerator->generateFirstTrajectories(m_robotControlHelper->getBaseTransform().getPosition()))
        {
            yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
            return false;
        }
    }
    else
    {
        if (!m_trajectoryGenerator->generateFirstTrajectories())
        {
            yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
            return false;
        }
    }

    if (!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::askNewTrajectories(const double &initTime, const bool &isLeftSwinging,
                                       const iDynTree::Transform &measuredTransform,
                                       const size_t &mergePoint, const iDynTree::VectorDynSize &plannerDesiredInput)
{
    if (m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if (!m_freeSpaceEllipseManager)
    {
        yError() << "[WalkingModule::askNewTrajectories] Free space ellipsoid not available.";
        return false;
    }

    if (mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[WalkingModule::askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }

    if (m_freeSpaceEllipseManager->isNewEllipseAvailable())
    {
        auto freeSpaceEllipse = m_freeSpaceEllipseManager->getEllipse();
        if (!m_trajectoryGenerator->setFreeSpaceEllipse(freeSpaceEllipse.imageMatrix, freeSpaceEllipse.centerOffset))
        {
            yError() << "[WalkingModule::askNewTrajectories] Unable to set the free space ellipse.";
            return false;
        }
    }

    if (!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                   m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                   measuredTransform, plannerDesiredInput))
    {
        yError() << "[WalkingModule::askNewTrajectories] Unable to update the trajectory.";
        return false;
    }
    return true;
}

bool WalkingModule::updateTrajectories(const size_t &mergePoint)
{
    if (!(m_trajectoryGenerator->isTrajectoryComputed()))
    {
        yError() << "[updateTrajectories] The trajectory is not computed.";
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<iDynTree::SpatialAcc> leftAccelerationTrajectory;
    std::vector<iDynTree::SpatialAcc> rightAccelerationTrajectory;
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<iDynTree::Vector2> desiredZMP;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;
    std::vector<bool> isStancePhase;
    std::vector<double> weightInLeft;
    std::vector<double> weightInRight;

    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);

    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetAcceleration(leftAccelerationTrajectory, rightAccelerationTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // get stance phase flags
    m_trajectoryGenerator->getIsStancePhase(isStancePhase);

    m_trajectoryGenerator->getDesiredZMPPosition(desiredZMP);

    m_trajectoryGenerator->getWeightPercentage(weightInLeft, weightInRight);

    // append vectors to deques
    StdUtilities::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(leftAccelerationTrajectory, m_leftAccelerationTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(rightAccelerationTrajectory, m_rightAccelerationTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    StdUtilities::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    StdUtilities::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);

    StdUtilities::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    StdUtilities::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    StdUtilities::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    StdUtilities::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    StdUtilities::appendVectorToDeque(isStancePhase, m_isStancePhase, mergePoint);

    StdUtilities::appendVectorToDeque(desiredZMP, m_desiredZMP, mergePoint);

    StdUtilities::appendVectorToDeque(weightInLeft, m_weightInLeftDesired, mergePoint);
    StdUtilities::appendVectorToDeque(weightInRight, m_weightInRightDesired, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    // the first merge point is always equal to 0
    m_mergePoints.pop_front();

    return true;
}

bool WalkingModule::updateFKSolver()
{
    if (!m_robotControlHelper->isExternalRobotBaseUsed())
    {
        if (!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                           m_rightTrajectory.front(),
                                                           m_isLeftFixedFrame.front()))
        {
            yError() << "[WalkingModule::updateFKSolver] Unable to evaluate the world to base transformation.";
            return false;
        }
    }
    else
    {
        m_FKSolver->evaluateWorldToBaseTransformation(m_robotControlHelper->getBaseTransform(),
                                                      m_robotControlHelper->getBaseTwist());
    }

    if (!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                           m_robotControlHelper->getJointVelocity()))
    {
        yError() << "[WalkingModule::updateFKSolver] Unable to set the robot state.";
        return false;
    }

    return true;
}

bool WalkingModule::startWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_robotState != WalkingFSM::Prepared && m_robotState != WalkingFSM::Paused)
    {
        yError() << "[WalkingModule::startWalking] Unable to start walking if the robot is not prepared or paused.";
        return false;
    }

    // if the robot was only prepared the filters has to be reseted
    if (m_robotState == WalkingFSM::Prepared)
    {
        m_robotControlHelper->resetFilters(m_feedbackAttempts, m_feedbackAttemptDelay);

        updateFKSolver();

        if (m_robotControlHelper->isExternalRobotBaseUsed())
        {
            double heightOffset = (m_FKSolver->getLeftFootToWorldTransform().getPosition()(2) + m_FKSolver->getRightFootToWorldTransform().getPosition()(2)) / 2;
            m_robotControlHelper->setHeightOffset(heightOffset);
        }
    }

    if (!m_robotControlHelper->loadCustomInteractionMode())
    {
        yError() << "[WalkingModule::startWalking] Unable to set the intraction mode of the joints";
        return false;
    }

    // we evaluate the offset between the CoM and the ZMP. This quantities is used only if
    // remove_zmp_offset is set to true.
    m_zmpOffset.zero();
    m_zmpOffsetLocal.zero();

    // Adjusting the offset on the ZMP at the begining with respect to CoM in the lateral direction
    iDynTree::Vector2 measuredZMP;
    iDynTree::Vector3 measuredCoM = m_FKSolver->getCoMPosition();

    if (!computeGlobalCoP(iDynTree::toEigen(measuredZMP)))
    {
        yError() << "[WalkingModule::updateModule] Unable to compute the global CoP.";
        return false;
    }

    if (!m_BLFIKSolver->setRegularizationJointSetPoint(m_robotControlHelper->getJointPosition()))
    {
        yError() << "[WalkingModule::startWalking] Unable to set regularization joint value.";
        return false;
    }

    if (m_removeZMPOffset)
    {
        m_zmpOffset(0) = measuredCoM(0) - measuredZMP(0);
        m_zmpOffset(1) = measuredCoM(1) - measuredZMP(1);
    }
    m_zmpOffsetLocal = m_zmpOffset;

    m_robotState = WalkingFSM::Walking;

    yInfo() << "[WalkingModule::startWalking] Started!";

    return true;
}

bool WalkingModule::setPlannerInput(const yarp::sig::Vector &plannerInput)
{
    m_plannerInput = plannerInput;

    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if (m_mergePoints.empty())
    {
        if (!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[WalkingModule::setPlannerInput] The trajectory has already finished but the system is not in double support.";
            return false;
        }

        if (m_newTrajectoryRequired)
            return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged after x cycles
        m_newTrajectoryMergeCounter = m_plannerAdvanceTimeSteps;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        // Searches for the first merge point that is at least m_plannerAdvanceTimeSteps steps away
        auto firstMergePointAvailable = std::find_if(m_mergePoints.begin(), m_mergePoints.end(), [this](size_t input)
                                                     { return input >= m_plannerAdvanceTimeSteps; });

        if (firstMergePointAvailable != m_mergePoints.end())
        {
            if (m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = *firstMergePointAvailable;
        }
        else
        {
            if (m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = m_plannerAdvanceTimeSteps;
        }
    }

    m_newTrajectoryRequired = true;

    return true;
}

bool WalkingModule::setGoal(const yarp::sig::Vector &plannerInput)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_robotState != WalkingFSM::Walking)
        return false;

    return setPlannerInput(plannerInput);
}

bool WalkingModule::pauseWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_robotState != WalkingFSM::Walking)
        return false;

    m_robotState = WalkingFSM::Paused;
    return true;
}

bool WalkingModule::stopWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_robotState != WalkingFSM::Walking)
        return false;

    reset();

    m_robotState = WalkingFSM::Stopped;
    return true;
}
