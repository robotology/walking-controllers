/**
 * @file WalkingModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iostream>
#include <memory>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>

#include <WalkingModule.hpp>
#include <Utils.hpp>
double timeOffset;

double impactTimeNominal, impactTimeAdjusted;
iDynTree::Vector2 zmpNominal, zmpAdjusted;

int indexPush;

void WalkingModule::propagateTime()
{
    // propagate time
    m_time += m_dT;
}

bool WalkingModule::advanceReferenceSignals()
{
    // check if vector is not initialized
    if(m_leftTrajectory.empty()
            || m_rightTrajectory.empty()
            || m_leftInContact.empty()
            || m_rightInContact.empty()
            || m_DCMPositionDesired.empty()
            || m_DCMPositionAdjusted.empty()
            || m_DCMVelocityDesired.empty()
            || m_comHeightTrajectory.empty())
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

    m_leftTwistTrajectory.pop_front();
    m_leftTwistTrajectory.push_back(m_leftTwistTrajectory.back());

    m_rightInContact.pop_front();
    m_rightInContact.push_back(m_rightInContact.back());

    m_leftInContact.pop_front();
    m_leftInContact.push_back(m_leftInContact.back());

    m_isLeftFixedFrame.pop_front();
    m_isLeftFixedFrame.push_back(m_isLeftFixedFrame.back());

    m_ZMPPositionDesired.pop_front();
    m_ZMPPositionDesired.push_back(m_ZMPPositionDesired.back());
    if (m_useStepAdaptation) {
        m_DCMPositionAdjusted.pop_front();
        m_DCMPositionAdjusted.push_back(m_DCMPositionAdjusted.back());

        m_DCMVelocityAdjusted.pop_front();
        m_DCMVelocityAdjusted.push_back(m_DCMVelocityAdjusted.back());
    }

    m_DCMPositionDesired.pop_front();
    m_DCMPositionDesired.push_back(m_DCMPositionDesired.back());

    m_DCMVelocityDesired.pop_front();
    m_DCMVelocityDesired.push_back(m_DCMVelocityDesired.back());

    m_comHeightTrajectory.pop_front();
    m_comHeightTrajectory.push_back(m_comHeightTrajectory.back());

    m_comHeightVelocity.pop_front();
    m_comHeightVelocity.push_back(m_comHeightVelocity.back());

    // at each sampling time the merge points are decreased by one.
    // If the first merge point is equal to 0 it will be dropped.
    // A new trajectory will be merged at the first merge point or if the deque is empty
    // as soon as possible.
    if(!m_mergePoints.empty())
    {
        for(auto& mergePoint : m_mergePoints)
            mergePoint--;

        if(m_mergePoints[0] == 0)
            m_mergePoints.pop_front();
    }
    return true;
}

double WalkingModule::getPeriod()
{
    //  period of the module (seconds)
    return m_dT;
}

bool WalkingModule::setRobotModel(const yarp::os::Searchable& rf)
{
    // load the model in iDynTree::KinDynComputations
    std::string model = rf.check("model",yarp::os::Value("model.urdf")).asString();
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    yInfo() << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel;

    // only the controlled joints are extracted from the URDF file
    if(!m_loader.loadReducedModelFromFile(pathToModel, m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::setRobotModel] Error while loading the model from " << pathToModel;
        return false;
    }
    return true;
}

bool WalkingModule::configure(yarp::os::ResourceFinder& rf)
{

    m_removeMe=1;
    // m_useExternalRobotBase = rf.check("use_external_robot_base", yarp::os::Value("False")).asBool();
    m_useStepAdaptation = rf.check("use_step_adaptation", yarp::os::Value(false)).asBool();
    m_useFloatingBaseEstimator=rf.check("use_floating_base_estimator", yarp::os::Value("False")).asBool();
    // TODO REMOVE ME
    impactTimeNominal = 0;
    impactTimeAdjusted = 0;

    zmpNominal.zero();
    zmpAdjusted.zero();
    m_isPushActive=0;

    m_nominalValuesLeft.zero();
    m_nominalValuesRight.zero();
    m_currentValues.zero();

    m_adaptatedFootLeftTwist.zero();
    m_adaptatedFootRightTwist.zero();
    m_currentFootLeftTwist.zero();
    m_currentFootRightTwist.zero();

    iDynTree::Position tempTemp;
    iDynTree::Rotation tempRot;
    tempRot.Identity();
    //    tempRot.
    tempTemp.zero();
    m_adaptatedFootLeftTransform.setPosition(tempTemp);
    m_currentFootLeftTransform.setPosition(tempTemp);

    m_adaptatedFootRightTransform.setPosition(tempTemp);
    m_currentFootRightTransform.setPosition(tempTemp);

    m_currentFootLeftTransform.setRotation(tempRot);
    m_adaptatedFootLeftTransform.setRotation(tempRot);

    m_currentFootRightTransform.setRotation(tempRot);
    m_adaptatedFootRightTransform.setRotation(tempRot);
    // module name (used as prefix for opened ports)
    m_useMPC = rf.check("use_mpc", yarp::os::Value(false)).asBool();
    m_useQPIK = rf.check("use_QP-IK", yarp::os::Value(false)).asBool();
    m_useOSQP = rf.check("use_osqp", yarp::os::Value(false)).asBool();
    m_dumpData = rf.check("dump_data", yarp::os::Value(false)).asBool();

    leftAdaptedStepParameters(0)=0.0;
    leftAdaptedStepParameters(1)=0.0;
    leftAdaptedStepParameters(2)=0.0;


    yarp::os::Bottle& generalOptions = rf.findGroup("GENERAL");
    m_dT = generalOptions.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    std::string name;
    if(!YarpHelper::getStringFromSearchable(generalOptions, "name", name))
    {
        yError() << "[WalkingModule::configure] Unable to get the string from searchable.";
        return false;
    }
    setName(name.c_str());

    m_robotControlHelper = std::make_unique<RobotHelper>();
    yarp::os::Bottle& robotControlHelperOptions = rf.findGroup("ROBOT_CONTROL");
    robotControlHelperOptions.append(generalOptions);
    if(!m_robotControlHelper->configureRobot(robotControlHelperOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the robot.";
        return false;
    }

    yarp::os::Bottle& forceTorqueSensorsOptions = rf.findGroup("FT_SENSORS");
    forceTorqueSensorsOptions.append(generalOptions);
    if(!m_robotControlHelper->configureForceTorqueSensors(forceTorqueSensorsOptions))
    {
        yError() << "[WalkingModule::configure] Unable to configure the Force Torque sensors.";
        return false;
    }

    if(!setRobotModel(rf))
    {
        yError() << "[configure] Unable to set the robot model.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    std::string desiredUnyciclePositionPortName = "/" + getName() + "/goal:i";
    if(!m_desiredUnyciclePositionPort.open(desiredUnyciclePositionPortName))
    {
        yError() << "[WalkingModule::configure] Could not open" << desiredUnyciclePositionPortName << " port.";
        return false;
    }

    // initialize the trajectory planner
    m_trajectoryGenerator = std::make_unique<TrajectoryGenerator>();
    yarp::os::Bottle& trajectoryPlannerOptions = rf.findGroup("TRAJECTORY_PLANNER");
    trajectoryPlannerOptions.append(generalOptions);
    if(!m_trajectoryGenerator->initialize(trajectoryPlannerOptions))
    {
        yError() << "[configure] Unable to initialize the planner.";
        return false;
    }
    m_stepHeight = trajectoryPlannerOptions.check("stepHeight", yarp::os::Value(0.005)).asDouble();

    if(m_useStepAdaptation)
    {
        // initialize the step adaptation
        m_stepAdaptator = std::make_unique<StepAdaptator>();
        yarp::os::Bottle& stepAdaptatorOptions = rf.findGroup("STEP_ADAPTATOR");
        stepAdaptatorOptions.append(generalOptions);
        if(!m_stepAdaptator->initialize(stepAdaptatorOptions))
        {
            yError() << "[configure] Unable to initialize the step adaptator!";
            return false;
        }
        // yarp::os::Bottle& plannerParameters = rf.findGroup("STEP_ADAPTATOR");
    }


    if(m_useMPC)
    {
        // initialize the MPC controller
        m_walkingController = std::make_unique<WalkingController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_MPC_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize the controller.";
            return false;
        }
    }
    else
    {
        // initialize the MPC controller
        m_walkingDCMReactiveController = std::make_unique<WalkingDCMReactiveController>();
        yarp::os::Bottle& dcmControllerOptions = rf.findGroup("DCM_REACTIVE_CONTROLLER");
        dcmControllerOptions.append(generalOptions);
        if(!m_walkingDCMReactiveController->initialize(dcmControllerOptions))
        {
            yError() << "[WalkingModule::configure] Unable to initialize the controller.";
            return false;
        }
    }

    // initialize the ZMP controller
    m_walkingZMPController = std::make_unique<WalkingZMPController>();
    yarp::os::Bottle& zmpControllerOptions = rf.findGroup("ZMP_CONTROLLER");
    zmpControllerOptions.append(generalOptions);
    if(!m_walkingZMPController->initialize(zmpControllerOptions))
    {
        yError() << "[WalkingModule::configure] Unable to initialize the ZMP controller.";
        return false;
    }

    // initialize the inverse kinematics solver
    m_IKSolver = std::make_unique<WalkingIK>();
    yarp::os::Bottle& inverseKinematicsSolverOptions = rf.findGroup("INVERSE_KINEMATICS_SOLVER");
    if(!m_IKSolver->initialize(inverseKinematicsSolverOptions, m_loader.model(),
                               m_robotControlHelper->getAxesList()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the ik solver";
        return false;
    }

    if(m_useQPIK)
    {
        yarp::os::Bottle& inverseKinematicsQPSolverOptions = rf.findGroup("INVERSE_KINEMATICS_QP_SOLVER");
        inverseKinematicsQPSolverOptions.append(generalOptions);
        if(m_useOSQP)
            m_QPIKSolver = std::make_unique<WalkingQPIK_osqp>();
        else
            m_QPIKSolver = std::make_unique<WalkingQPIK_qpOASES>();

        if(!m_QPIKSolver->initialize(inverseKinematicsQPSolverOptions,
                                     m_robotControlHelper->getActuatedDoFs(),
                                     m_robotControlHelper->getVelocityLimits(),
                                     m_robotControlHelper->getPositionUpperLimits(),
                                     m_robotControlHelper->getPositionLowerLimits()))
        {
            yError() << "[WalkingModule::configure] Failed to configure the QP-IK solver (qpOASES)";
            return false;
        }
    }

    // initialize the forward kinematics solver
    m_FKSolver = std::make_unique<WalkingFK>();
    yarp::os::Bottle& forwardKinematicsSolverOptions = rf.findGroup("FORWARD_KINEMATICS_SOLVER");
    forwardKinematicsSolverOptions.append(generalOptions);
    if(!m_FKSolver->initialize(forwardKinematicsSolverOptions, m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the fk solver";
        return false;
    }

    //initialize the DCM estimator
    m_DCMEstimator=std::make_unique<DCMEstimator>();
    if(!m_DCMEstimator->initialize(generalOptions,m_loader.model()))
    {
        yError() << "[WalkingModule::configure] Failed to configure the DCM estimator.";
        return false;
    }
    // initialize the linear inverted pendulum model
    m_stableDCMModel = std::make_unique<StableDCMModel>();
    if(!m_stableDCMModel->initialize(generalOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the lipm.";
        return false;
    }

    // set PIDs gains
    yarp::os::Bottle& pidOptions = rf.findGroup("PID");
    if (!m_robotControlHelper->configurePIDHandler(pidOptions))
    {
        yError() << "[WalkingModule::configure] Failed to configure the PIDs.";
        return false;
    }

    // configure the retargeting
    yarp::os::Bottle retargetingOptions = rf.findGroup("RETARGETING");
    retargetingOptions.append(generalOptions);
    m_retargetingClient = std::make_unique<RetargetingClient>();
    if (!m_retargetingClient->initialize(retargetingOptions, getName(), m_dT))
    {
        yError() << "[WalkingModule::configure] Failed to configure the retargeting";
        return false;
    }

    // initialize the logger
    if(m_dumpData)
    {
        m_walkingLogger = std::make_unique<LoggerClient>();
        yarp::os::Bottle& loggerOptions = rf.findGroup("WALKING_LOGGER");
        if(!m_walkingLogger->configure(loggerOptions, getName()))
        {
            yError() << "[WalkingModule::configure] Unable to configure the logger.";
            return false;
        }
    }

    // time profiler
    m_profiler = std::make_unique<TimeProfiler>();
    m_profiler->setPeriod(round(0.1 / m_dT));
    if(m_useMPC)
        m_profiler->addTimer("MPC");

    m_profiler->addTimer("IK");
    m_profiler->addTimer("Total");

    // initialize some variables
    m_newTrajectoryRequired = false;
    m_newTrajectoryMergeCounter = -1;
    m_robotState = WalkingFSM::Configured;

    m_inertial_R_worldFrame = iDynTree::Rotation::Identity();
    // TODO move in the config
    std::string portNameBaseEst;
    portNameBaseEst = "/" + name + "/base-est/rpc";
    m_rpcBaseEstPort.open(portNameBaseEst);
    yarp::os::Network::connect(portNameBaseEst, "/base-estimator/rpc");

    // resize variables
    m_qDesired.resize(m_robotControlHelper->getActuatedDoFs());
    m_dqDesired.resize(m_robotControlHelper->getActuatedDoFs());

    yInfo() << "[WalkingModule::configure] Ready to play!";

    return true;
}

void WalkingModule::reset()
{
    if(m_useMPC)
        m_walkingController->reset();

    m_trajectoryGenerator->reset();

    if(m_dumpData)
        m_walkingLogger->quit();
}

bool WalkingModule::close()
{
    if(m_dumpData)
        m_walkingLogger->quit();

    // restore PID
    m_robotControlHelper->getPIDHandler().restorePIDs();

    // close retargeting ports
    m_retargetingClient->close();

    // close the ports
    m_rpcPort.close();
    m_desiredUnyciclePositionPort.close();

    // close the connection with robot
    if(!m_robotControlHelper->close())
    {
        yError() << "[WalkingModule::close] Unable to close the connection with the robot.";
        return false;
    }

    // clear all the pointer
    m_trajectoryGenerator.reset(nullptr);
    m_walkingController.reset(nullptr);
    m_stepAdaptator.reset(nullptr);
    m_walkingZMPController.reset(nullptr);
    m_IKSolver.reset(nullptr);
    m_QPIKSolver.reset(nullptr);
    m_FKSolver.reset(nullptr);
    m_stableDCMModel.reset(nullptr);
    m_DCMEstimator.reset(nullptr);

    return true;
}

bool WalkingModule::solveQPIK(const std::unique_ptr<WalkingQPIK>& solver, const iDynTree::Position& desiredCoMPosition,
                              const iDynTree::Vector3& desiredCoMVelocity,
                              const iDynTree::Rotation& desiredNeckOrientation,
                              iDynTree::VectorDynSize &output)
{
    bool ok = true;
    double threshold = 0.001;
    bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
    solver->setPhase(stancePhase);

    ok &= solver->setRobotState(m_robotControlHelper->getJointPosition(),
                                m_FKSolver->getLeftFootToWorldTransform(),
                                m_FKSolver->getRightFootToWorldTransform(),
                                m_FKSolver->getLeftHandToWorldTransform(),
                                m_FKSolver->getRightHandToWorldTransform(),
                                m_FKSolver->getNeckOrientation(),
                                m_FKSolver->getCoMPosition());

    solver->setDesiredNeckOrientation(desiredNeckOrientation.inverse());

    //    solver->setDesiredFeetTransformation(m_adaptatedFootLeftTransform,
    //                                         m_adaptatedFootRightTransform);

    //    solver->setDesiredFeetTwist(m_adaptatedFootLeftTwist,
    //                                m_adaptatedFootRightTwist);
    if (m_useStepAdaptation) {
        solver->setDesiredFeetTransformation(m_currentFootLeftTransform,
                                             m_currentFootRightTransform);

        solver->setDesiredFeetTwist(m_currentFootLeftTwist,
                                    m_currentFootRightTwist);
    }
    else {
        solver->setDesiredFeetTransformation(m_leftTrajectory.front(),
                                             m_rightTrajectory.front());

        solver->setDesiredFeetTwist(m_leftTwistTrajectory.front(),
                                    m_rightTwistTrajectory.front());
    }



    solver->setDesiredCoMVelocity(desiredCoMVelocity);
    solver->setDesiredCoMPosition(desiredCoMPosition);

    // TODO probably the problem can be written locally w.r.t. the root or the base
    solver->setDesiredHandsTransformation(m_FKSolver->getHeadToWorldTransform() * m_retargetingClient->leftHandTransform(),
                                          m_FKSolver->getHeadToWorldTransform() * m_retargetingClient->rightHandTransform());

    // set jacobians
    iDynTree::MatrixDynSize jacobian, comJacobian;
    jacobian.resize(6, m_robotControlHelper->getActuatedDoFs() + 6);
    comJacobian.resize(3, m_robotControlHelper->getActuatedDoFs() + 6);

    ok &= m_FKSolver->getLeftFootJacobian(jacobian);
    ok &= solver->setLeftFootJacobian(jacobian);

    ok &= m_FKSolver->getRightFootJacobian(jacobian);
    ok &= solver->setRightFootJacobian(jacobian);

    ok &= m_FKSolver->getNeckJacobian(jacobian);
    ok &= solver->setNeckJacobian(jacobian);

    ok &= m_FKSolver->getCoMJacobian(comJacobian);
    solver->setCoMJacobian(comJacobian);

    ok &= m_FKSolver->getLeftHandJacobian(jacobian);
    ok &= solver->setLeftHandJacobian(jacobian);

    ok &= m_FKSolver->getRightHandJacobian(jacobian);
    ok &= solver->setRightHandJacobian(jacobian);

    if(!ok)
    {
        yError() << "[WalkingModule::solveQPIK] Error while setting the jacobians.";
        return false;
    }

    if(!solver->solve())
    {
        yError() << "[WalkingModule::solveQPIK] Unable to solve the QP-IK problem.";
        return false;
    }

    output = solver->getDesiredJointVelocities();

    return true;
}

bool WalkingModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

//    iDynTree::Rotation imuRotation=m_robotControlHelper->getIMUOreintation();
//    iDynTree::Vector3 imurolpitchyaw= imuRotation.asRPY();
//    yInfo() <<"imuuu"<<imurolpitchyaw(0)<<imurolpitchyaw(1)<<imurolpitchyaw(2);
    if(m_robotState == WalkingFSM::Preparing)
    {

        if(!m_robotControlHelper->getFeedbacksRaw(10))
        {
            yError() << "[updateModule] Unable to get the feedback.";
            return false;
        }

        bool motionDone = false;
        if(!m_robotControlHelper->checkMotionDone(motionDone))
        {
            yError() << "[WalkingModule::updateModule] Unable to check if the motion is done";
            yInfo() << "[WalkingModule::updateModule] Try to prepare again";
            reset();
            m_robotState = WalkingFSM::Stopped;
            return true;
        }
        if(motionDone)
        {
            //            // send the reference again in order to reduce error
            //            if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
            //            {
            //                yError() << "[prepareRobot] Error while setting the initial position using "
            //                         << "POSITION DIRECT mode.";
            //                yInfo() << "[WalkingModule::updateModule] Try to prepare again";
            //                reset();
            //                m_robotState = WalkingFSM::Stopped;
            //                return true;
            //            }

            yarp::sig::Vector buffer(m_qDesired.size());
            iDynTree::toYarp(m_qDesired, buffer);
            // instantiate Integrator object

            yarp::sig::Matrix jointLimits(m_robotControlHelper->getActuatedDoFs(), 2);
            for(int i = 0; i < m_robotControlHelper->getActuatedDoFs(); i++)
            {
                jointLimits(i, 0) = m_robotControlHelper->getPositionLowerLimits()(i);
                jointLimits(i, 1) = m_robotControlHelper->getPositionUpperLimits()(i);
            }
            m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer, jointLimits);

            // reset the models
            m_walkingZMPController->reset(m_DCMPositionDesired.front());
            m_stableDCMModel->reset(m_DCMPositionDesired.front());
            m_DCMEstimator->reset(m_DCMPositionAdjusted.front());
            // reset the retargeting
            if(!m_robotControlHelper->getFeedbacks(20))
            {
                yError() << "[WalkingModule::updateModule] Unable to get the feedback.";
                return false;
            }

            if(!updateFKSolver())
            {
                yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
                return false;
            }

            m_retargetingClient->reset(m_FKSolver->getHeadToWorldTransform().inverse()
                                       * m_FKSolver->getLeftHandToWorldTransform(),
                                       m_FKSolver->getHeadToWorldTransform().inverse()
                                       * m_FKSolver->getRightHandToWorldTransform());


            //   m_robotState = WalkingFSM::Prepared;

            iDynTree::VectorDynSize dummy(m_robotControlHelper->getActuatedDoFs());
            dummy.zero();
            //  m_walkingAdmittanceController->setDesiredJointTrajectory( m_qDesired, dummy, dummy);

                   // iDynTree::toEigen(m_pelvisRotationOffset.);
          //  iDynTree::toEigen(m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY());
                            m_robotState = WalkingFSM::Prepared;
            yInfo() << "[WalkingModule::updateModule] The robot is prepared.";
        }
    }
    else if(m_robotState == WalkingFSM::Walking)
    {
        iDynTree::Vector2 measuredZMP;

        bool resetTrajectory = false;

        m_profiler->setInitTime("Total");

        // check desired planner input
        yarp::sig::Vector* desiredUnicyclePosition = nullptr;
        desiredUnicyclePosition = m_desiredUnyciclePositionPort.read(false);
        if(desiredUnicyclePosition != nullptr){
            if(!setPlannerInput((*desiredUnicyclePosition)(0), (*desiredUnicyclePosition)(1)))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the planner input";
                return false;
            }
        }

        if (m_mergePoints.front() == 11 && desiredUnicyclePosition == nullptr) {

            if(!setPlannerInput(m_desiredPosition(0) ,m_desiredPosition(1)))
            {
                yError() << "[updateModule] Unable to recall the setplannerInput (when terminal (SetGoal) instead of JoyStick is used)";
                return false;
            }
        }

        // if a new trajectory is required check if its the time to evaluate the new trajectory or
        // the time to attach new one
        if(m_newTrajectoryRequired)
        {
            // when we are near to the merge point the new trajectory is evaluated
            if(m_newTrajectoryMergeCounter == 10)
            {

                double initTimeTrajectory;
                initTimeTrajectory = m_time + m_newTrajectoryMergeCounter * m_dT;

                m_startOfWalkingTime=initTimeTrajectory;
                iDynTree::Transform TempRightFoot;
                iDynTree::Transform TempLeftFoot;

                iDynTree::Transform measuredTransform ;
                if(!m_useStepAdaptation){
                    measuredTransform = m_isLeftFixedFrame.front() ?
                                m_rightTrajectory[m_newTrajectoryMergeCounter] :
                                m_leftTrajectory[m_newTrajectoryMergeCounter];
                }
                else {
                    measuredTransform = m_isLeftFixedFrame.front() ?
                                m_currentFootRightTransform : m_currentFootLeftTransform;
                    //m_currentFootRightTransform : m_currentFootLeftTransform;
                }


                // ask for a new trajectory
                if(!askNewTrajectories(initTimeTrajectory, !m_isLeftFixedFrame.front(),
                                       measuredTransform, m_newTrajectoryMergeCounter,
                                       m_desiredPosition))
                {
                    yError() << "[WalkingModule::updateModule] Unable to ask for a new trajectory.";
                    return false;
                }
            }

            if(m_newTrajectoryMergeCounter == 2)
            {
                if(!updateTrajectories(m_newTrajectoryMergeCounter))
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

        // get feedbacks and evaluate useful quantities
        if(!m_robotControlHelper->getFeedbacks(20))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the feedback.";
            return false;
        }

        m_retargetingClient->getFeedback();

        if(!updateFKSolver())
        {
            yError() << "[WalkingModule::updateModule] Unable to update the FK solver.";
            return false;
        }

        if(!evaluateZMP(measuredZMP))
        {
            yError() << "[WalkingModule::updateModule] Unable to evaluate the ZMP.";
            return false;
        }

        // evaluate 3D-LIPM reference signal
        m_stableDCMModel->setInput(m_DCMPositionDesired.front());
        if(!m_stableDCMModel->integrateModel())
        {
            yError() << "[WalkingModule::updateModule] Unable to propagate the 3D-LIPM.";
            return false;
        }

//        m_DCMEstimator->reset(m_DCMPositionAdjusted.front());
//        if(!m_DCMEstimator->integrateDCMVelocity(measuredZMP,m_DCMPositionAdjusted.front()))
//        {
//            yError() << "[WalkingModule::updateModule] Unable to integrate DCM Velocity.";
//            return false;
//        }

        iDynTree::Rotation imuRotation=m_robotControlHelper->getIMUOreintation();
        iDynTree::Vector3 imuRPY= imuRotation.asRPY();
        //yInfo() <<"imuuu"<<imurolpitchyaw(0)<<imurolpitchyaw(1)<<imurolpitchyaw(2);
        m_DCMEstimator->reset(m_DCMPositionAdjusted.front());
        iDynTree::Rotation pelvisOrientation;
        pelvisOrientation=iDynTree::Rotation::Identity();
        double miladTempP;
         double miladTempR;
        if ((abs(m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY()(1)-imuRPY(1)))>0.005  ) {
            miladTempP=m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY()(1)-imuRPY(1);

            //miladTemp=0;
        }
        else {
            miladTempP=0;
        }

        if ( (abs(m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY()(0)-imuRPY(0)))>0.015) {

            miladTempR=m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY()(0)-imuRPY(0);
            //miladTemp=0;
        }
        else {
            miladTempR=0;
        }


       pelvisOrientation=iDynTree::Rotation::RPY (miladTempR,miladTempP,0); 	//=m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY();
 //      yInfo()<<pelvisOrientation.toString();
//yInfo()<<pelvisOrientation.toString();
/*m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY()(1)-imuRPY(1)*/


//       int numberOfSubTrajectoriesTemp = m_DCMSubTrajectories.size();
//       yInfo()<<"MMIIL001222222"<<numberOfSubTrajectoriesTemp;
//       auto firstSSTemp = m_DCMSubTrajectories[numberOfSubTrajectoriesTemp-2];
//       auto secondSSTemp = m_DCMSubTrajectories[numberOfSubTrajectoriesTemp-4];
//       yInfo()<<"MMIIL00444444222"<<numberOfSubTrajectoriesTemp;
//       iDynTree::Vector2  currentZmpPositionTemp;
//       bool checkFeasibility = false;
//yInfo()<<"MMIIL1";

//       firstSSTemp->getZMPPosition(0, currentZmpPositionTemp, checkFeasibility);
        //pelvisOrientation
        //iDynTree::Vector3 ZMP3d;

        iDynTree::Vector3 ZMP3d;
        ZMP3d(0)=measuredZMP(0);
        ZMP3d(1)=measuredZMP(1);
        ZMP3d(2)=0;

        iDynTree::Vector3 CoM3d;
        //iDynTree::Vector3 ZMP3d;
        CoM3d(0)=m_stableDCMModel->getCoMPosition()(0);
        CoM3d(1)=m_stableDCMModel->getCoMPosition()(1);
        CoM3d(2)=m_comHeightTrajectory.front();

        iDynTree::Vector3 CoMVelocity3d;
        //iDynTree::Vector3 ZMP3d;
        CoMVelocity3d(0)=m_stableDCMModel->getCoMVelocity()(0);
        CoMVelocity3d(1)=m_stableDCMModel->getCoMVelocity()(1);
        CoMVelocity3d(2)=0;

        if(!m_DCMEstimator->pendulumEstimator(pelvisOrientation,ZMP3d,CoM3d,CoMVelocity3d))
        {
            yError() << "[WalkingModule::updateModule] Unable to to recieve DCM from pendulumEstimator";
            return false;
        }

        iDynTree::Vector2 m_dcmEstimatedI= m_DCMEstimator->getDCMPosition();

        if(m_useStepAdaptation){

            // integration of DCM velocity for push recovery
            //               yarp::sig::Vector bufferDCMVelocity;
            //               bufferDCMVelocity.resize(2, 0.0);

            //               // instantiate Integrator object
            //               m_DCMIntegrator = std::make_unique<iCub::ctrl::Integrator>(m_dT, bufferDCMVelocity);



            //               // evaluate the velocity of the CoM
            //               yarp::sig::Vector DCMVelocityYarp(2);
            //               iDynTree::toEigen(DCMVelocityYarp) ;// -m_omega * (iDynTree::toEigen(m_DCMPosition) -
            //                                                                //iDynTree::toEigen(m_DCMPosition));
            //               yarp::sig::Vector DCMPositionYarp(2);
            //                   DCMPositionYarp = m_DCMIntegrator->integrate(DCMVelocityYarp);

            //                   // convert YARP vector into iDynTree vector
            //                   iDynTree::toiDynTree(DCMVelocityYarp, m_dcmVelocityPush);
            //                   iDynTree::toiDynTree(DCMPositionYarp, m_dcmPositionPush);

            //  step adjustment
            double comHeight;
            double omega;

            if(!m_trajectoryGenerator->getNominalCoMHeight(comHeight)){
                yError() << "[updateModule] Unable to get the nominal CoM height!";
                return false;
            }

            omega = sqrt(9.81 / comHeight);

            if (!m_leftInContact.front() || !m_rightInContact.front())
            {

                indexPush++;

                int numberOfSubTrajectories = m_DCMSubTrajectories.size();
                auto firstSS = m_DCMSubTrajectories[numberOfSubTrajectories-2];
                auto secondSS = m_DCMSubTrajectories[numberOfSubTrajectories-4];

                auto secondDS = m_DCMSubTrajectories[numberOfSubTrajectories-3];
                auto firstDS = m_DCMSubTrajectories[numberOfSubTrajectories-1];

                iDynTree::Vector2 nextZmpPosition, currentZmpPosition;
                bool checkFeasibility = false;
                secondSS->getZMPPosition(0, nextZmpPosition, checkFeasibility);
                double angle = !m_leftInContact.front()? m_jleftFootprints->getSteps()[1].angle : m_jRightFootprints->getSteps()[1].angle;
                m_stepAdaptator->setNominalNextStepPosition(nextZmpPosition, angle);

                firstSS->getZMPPosition(0, currentZmpPosition, checkFeasibility);
                m_stepAdaptator->setCurrentZmpPosition(currentZmpPosition);

                // TODO this is a test
                // if(!m_DCMSubTrajectories[numberOfSubTrajectories-2]->getDCMPosition(m_time - timeOffset, dcmCurrentDesired, false))
                // {
                //     yError() << " strange " << m_DCMSubTrajectories[numberOfSubTrajectories - 2]->getTrajectoryDomain().first << " " << m_DCMSubTrajectories[numberOfSubTrajectories - 2]->getTrajectoryDomain().second;
                //     return false;
                // }


                // if((indexPush > 5) &&  (indexPush < 10))
                // {
                //     yInfo() << "push";
                //     m_DCMPositionAdjusted.front()(0) += 0.05;
                // }

                iDynTree::Vector2 dcmMeasured2D;
                dcmMeasured2D(0) = m_FKSolver->getDCM()(0);
                dcmMeasured2D(1) = m_FKSolver->getDCM()(1);
                m_isPushActive=0;

                if((m_DCMPositionAdjusted.front()(0) - m_DCMEstimator->getDCMPosition()(0)) > m_stepAdaptator->getDCMErrorThreshold()(0) ||(m_DCMPositionAdjusted.front()(1) - m_DCMEstimator->getDCMPosition()(1))> m_stepAdaptator->getDCMErrorThreshold()(1) )
                {
                    m_isPushActive=1;
                    yInfo()<<"triggering the push recovery";
                    // std::cerr << "adj " << (iDynTree::toEigen(m_DCMPositionAdjusted.front()) - iDynTree::toEigen(dcmMeasured2D)).norm() << std::endl;
                    m_stepAdaptator->setCurrentDcmPosition(m_DCMEstimator->getDCMPosition());
                }
                else{
                    m_stepAdaptator->setCurrentDcmPosition(m_DCMPositionAdjusted.front());
                }

                iDynTree::Vector2 dcmAtTimeAlpha;
                double timeAlpha = (secondDS->getTrajectoryDomain().second + secondDS->getTrajectoryDomain().first) / 2;
                m_DCMSubTrajectories[numberOfSubTrajectories-2]->getDCMPosition(timeAlpha, dcmAtTimeAlpha, checkFeasibility);

                iDynTree::Vector2 nominalDcmOffset;
                iDynTree::toEigen(nominalDcmOffset) = iDynTree::toEigen(dcmAtTimeAlpha) - iDynTree::toEigen(nextZmpPosition);
                m_stepAdaptator->setNominalDcmOffset(nominalDcmOffset);

                m_stepAdaptator->setTimings(omega, m_time - timeOffset, firstSS->getTrajectoryDomain().second,
                                            secondDS->getTrajectoryDomain().second - secondDS->getTrajectoryDomain().first);
                //            yInfo()<<"m_DCMPositionAdjusted"<<m_DCMPositionAdjusted.front()(0);
                //            yInfo()<<"dcmMeasured2D"<< dcmMeasured2D(0);
                //            yInfo()<<m_DCMPositionDesired.front()(0);
                //yInfo()<<nextZmpPosition(0);
                //yInfo()<<currentZmpPosition(0);
                if(!m_stepAdaptator->solve(!m_leftInContact.front()))
                {
                    yError() << "unable to solve the problem step adjustment";
                    return false;
                }

                impactTimeNominal = firstSS->getTrajectoryDomain().second + timeOffset;
                impactTimeAdjusted = m_stepAdaptator->getDesiredImpactTime() + timeOffset;

                zmpNominal = nextZmpPosition;
                zmpAdjusted = m_stepAdaptator->getDesiredZmp();

                if (!m_leftInContact.front())
                {
                    // TODO REMOVE MAGIC NUMBERS
                    iDynTree::Vector2 zmpOffset;
                    zmpOffset.zero();
                    zmpOffset(0) = 0.03;

                    m_currentFootLeftTransform = m_adaptatedFootLeftTransform;
                    m_currentFootLeftTwist = m_adaptatedFootLeftTwist;
                    m_currentFootLeftAcceleration = m_adaptatedFootLeftAcceleration;
                    if(!m_stepAdaptator->getAdaptatedFootTrajectory(m_stepHeight, m_dT, firstSS->getTrajectoryDomain().first,
                                                                    m_jLeftstepList.at(1).angle,
                                                                    zmpOffset, m_currentFootLeftTransform, m_currentFootLeftTwist,
                                                                    m_adaptatedFootLeftTransform, m_adaptatedFootLeftTwist, m_adaptatedFootLeftAcceleration ))
                    {
                        yError() << "error write something usefull";
                        return false;
                    }
                }
                else
                {
                    // TODO REMOVE MAGIC NUMBERS
                    iDynTree::Vector2 zmpOffset;
                    zmpOffset.zero();
                    zmpOffset(0) = 0.03;

                    m_currentFootRightTransform = m_adaptatedFootRightTransform;
                    m_currentFootRightTwist = m_adaptatedFootRightTwist;
                    m_currentFootRightAcceleration = m_adaptatedFootRightAcceleration;
                    if(!m_stepAdaptator->getAdaptatedFootTrajectory(m_stepHeight, m_dT, firstSS->getTrajectoryDomain().first,
                                                                    m_jRightstepList.at(1).angle,
                                                                    zmpOffset, m_currentFootRightTransform, m_currentFootRightTwist,
                                                                    m_adaptatedFootRightTransform, m_adaptatedFootRightTwist, m_adaptatedFootRightAcceleration ))
                    {
                        yError() << "error write something usefull right";
                        return false;
                    }
                }

                // adapted dcm trajectory

                // add the offset on the zmp evaluated by the step adjustment for each footprint in the trajectory
                // the same approach is used also for the impact time since the step adjustment change the impact time
                iDynTree::Vector2 adaptedZMPOffset;
                iDynTree::toEigen(adaptedZMPOffset) = iDynTree::toEigen(m_stepAdaptator->getDesiredZmp()) - iDynTree::toEigen(nextZmpPosition);
                double adaptedTimeOffset;
                adaptedTimeOffset = m_stepAdaptator->getDesiredImpactTime() - firstSS->getTrajectoryDomain().second;

                // TODO REMOVE MAGIC NUMBERS
                iDynTree::Vector2 zmpOffset;
                zmpOffset.zero();
                zmpOffset(0) = 0.00;

                std::shared_ptr<FootPrint> leftTemp = std::make_unique<FootPrint>();
                leftTemp->setFootName("left");
                leftTemp->addStep(m_jleftFootprints->getSteps()[0]);
                for(int i = 1; i < m_jleftFootprints->getSteps().size(); i++)
                {
                    iDynTree::Vector2 position;
                    iDynTree::toEigen(position) =  iDynTree::toEigen(m_jleftFootprints->getSteps()[i].position) + iDynTree::toEigen(adaptedZMPOffset)
                            + iDynTree::toEigen(zmpOffset);

                    leftTemp->addStep(position, m_jleftFootprints->getSteps()[i].angle, m_jleftFootprints->getSteps()[i].impactTime + adaptedTimeOffset);
                }

                std::shared_ptr<FootPrint> rightTemp = std::make_unique<FootPrint>();
                rightTemp->setFootName("right");
                rightTemp->addStep(m_jRightFootprints->getSteps()[0]);
                for(int i = 1; i < m_jRightFootprints->getSteps().size(); i++)
                {
                    iDynTree::Vector2 position;
                    iDynTree::toEigen(position) =  iDynTree::toEigen(m_jRightFootprints->getSteps()[i].position) + iDynTree::toEigen(adaptedZMPOffset)
                            + iDynTree::toEigen(zmpOffset);

                    rightTemp->addStep(position, m_jRightFootprints->getSteps()[i].angle, m_jRightFootprints->getSteps()[i].impactTime + adaptedTimeOffset);
                }

                // generate the DCM trajectory
                if(!m_trajectoryGenerator->generateTrajectoriesFromFootprints(leftTemp, rightTemp, timeOffset))
                {
                    yError() << "[WalkingModule::updateModule] unable to generate new trajectorie after step adjustment.";
                    return false;
                }

                std::vector<iDynTree::Vector2> DCMPositionAdjusted;
                std::vector<iDynTree::Vector2> DCMVelocityAdjusted;
                m_trajectoryGenerator->getDCMPositionTrajectoryAdj(DCMPositionAdjusted);
                m_trajectoryGenerator->getDCMVelocityTrajectoryAdj(DCMVelocityAdjusted);

                size_t startIndexOfDCMAdjusted = (size_t)round((m_time - timeOffset) / m_dT);

                // yInfo() << "startIndexOfDCMAdjusted " << startIndexOfDCMAdjusted;
                // for(int i = 0; i < DCMPositionAdjusted.size(); i++ )
                // {
                //     if ((iDynTree::toEigen(DCMPositionAdjusted[i]) - iDynTree::toEigen(m_DCMPositionDesired.front())).norm() < 0.01)
                //     {
                //         std::cerr << "should be 0 " << iDynTree::toEigen(m_DCMPositionAdjusted.front()) - iDynTree::toEigen(m_DCMPositionDesired.front()) << std::endl;
                //         std::cerr <<"index " << i << std::endl;
                //     }
                // }

                // std::cerr << "should be 0 out" << iDynTree::toEigen(m_DCMPositionAdjusted.front()) - iDynTree::toEigen(m_DCMPositionDesired.front()) << std::endl;
                // if(indexPush == 6)
                //     yInfo() << "startIndexOfDCMAdjusted " << startIndexOfDCMAdjusted << "[ " << m_DCMPositionAdjusted.front().toString() << " ]";

                m_DCMPositionAdjusted.resize(DCMPositionAdjusted.size() - startIndexOfDCMAdjusted);
                for(int i = 0; i < m_DCMPositionAdjusted.size(); i++)
                    m_DCMPositionAdjusted[i] = DCMPositionAdjusted[i + startIndexOfDCMAdjusted];


                m_DCMVelocityAdjusted.resize(DCMVelocityAdjusted.size() - startIndexOfDCMAdjusted);
                for(int i = 0; i < m_DCMVelocityAdjusted.size(); i++)
                    m_DCMVelocityAdjusted[i] = DCMVelocityAdjusted[i + startIndexOfDCMAdjusted];


                // StdHelper::appendSubVectorToDeque(DCMAdjusted, startIndexOfDCMAdjusted, m_DCMPositionAdjusted, 0);
            }

            else
            {
                m_currentFootLeftAcceleration=m_adaptatedFootLeftAcceleration;
                m_currentFootLeftTwist=m_adaptatedFootLeftTwist;
                m_currentFootLeftTransform=m_adaptatedFootLeftTransform;

                m_currentFootRightAcceleration=m_adaptatedFootRightAcceleration;
                m_currentFootRightTwist=m_adaptatedFootRightTwist;
                m_currentFootRightTransform=m_adaptatedFootRightTransform;
            }
        }




        // DCM controller
        if(m_useMPC)
        {
            // Model predictive controller
            m_profiler->setInitTime("MPC");
            if(!m_walkingController->setConvexHullConstraint(m_leftTrajectory, m_rightTrajectory,
                                                             m_leftInContact, m_rightInContact))
            {
                yError() << "[WalkingModule::updateModule] unable to evaluate the convex hull.";
                return false;
            }

            if(!m_walkingController->setFeedback(m_FKSolver->getDCM()))
            {
                yError() << "[WalkingModule::updateModule] unable to set the feedback.";
                return false;
            }

            if(!m_walkingController->setReferenceSignal(m_DCMPositionDesired, resetTrajectory))
            {
                yError() << "[WalkingModule::updateModule] unable to set the reference Signal.";
                return false;
            }

            if(!m_walkingController->solve())
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the problem.";
                return false;
            }

            m_profiler->setEndTime("MPC");
        }
        else
        {

            if (!m_useStepAdaptation) {
                m_walkingDCMReactiveController->setFeedback(m_FKSolver->getDCM());
                m_walkingDCMReactiveController->setReferenceSignal(m_DCMPositionDesired.front(),
                                                                   m_DCMVelocityDesired.front());
            }
            else {
                iDynTree::Vector2 DCMPositionDesiredAdjusted;
                DCMPositionDesiredAdjusted(0) = m_DCMPositionAdjusted.front()(0);
                DCMPositionDesiredAdjusted(1) = m_DCMPositionAdjusted.front()(1);

                iDynTree::Vector2 DCMVelocityDesiredAdjusted;
                DCMVelocityDesiredAdjusted(0) = m_DCMVelocityAdjusted.front()(0);
                DCMVelocityDesiredAdjusted(1) = m_DCMVelocityAdjusted.front()(1);

                m_walkingDCMReactiveController->setFeedback(m_FKSolver->getDCM());
                m_walkingDCMReactiveController->setReferenceSignal(DCMPositionDesiredAdjusted,
                                                                   DCMVelocityDesiredAdjusted);
            }



            if(!m_walkingDCMReactiveController->evaluateControl())
            {
                yError() << "[WalkingModule::updateModule] Unable to evaluate the DCM control output.";
                return false;
            }
        }

        // inner COM-ZMP controller
        // if the the norm of desired DCM velocity is lower than a threshold then the robot
        // is stopped
        double threshold = 0.001;
        bool stancePhase = iDynTree::toEigen(m_DCMVelocityDesired.front()).norm() < threshold;
        m_walkingZMPController->setPhase(stancePhase);

        iDynTree::Vector2 desiredZMP;
        if(m_useMPC)
            desiredZMP = m_walkingController->getControllerOutput();
        else
            desiredZMP = m_walkingDCMReactiveController->getControllerOutput();

        // set feedback and the desired signal
        m_walkingZMPController->setFeedback(measuredZMP, m_FKSolver->getCoMPosition());
        m_walkingZMPController->setReferenceSignal(desiredZMP, m_stableDCMModel->getCoMPosition(),
                                                   m_stableDCMModel->getCoMVelocity());

        if(!m_walkingZMPController->evaluateControl())
        {
            yError() << "[WalkingModule::updateModule] Unable to evaluate the ZMP control output.";
            return false;
        }

        iDynTree::Vector2 outputZMPCoMControllerPosition, outputZMPCoMControllerVelocity;
        if(!m_walkingZMPController->getControllerOutput(outputZMPCoMControllerPosition,
                                                        outputZMPCoMControllerVelocity))
        {
            yError() << "[WalkingModule::updateModule] Unable to get the ZMP controller output.";
            return false;
        }

        // inverse kinematics
        m_profiler->setInitTime("IK");

        iDynTree::Position desiredCoMPosition;
        desiredCoMPosition(0) = outputZMPCoMControllerPosition(0);
        desiredCoMPosition(1) = outputZMPCoMControllerPosition(1);
        desiredCoMPosition(2) = m_comHeightTrajectory.front();


        iDynTree::Vector3 desiredCoMVelocity;
        desiredCoMVelocity(0) = outputZMPCoMControllerVelocity(0);
        desiredCoMVelocity(1) = outputZMPCoMControllerVelocity(1);
        desiredCoMVelocity(2) = m_comHeightVelocity.front();

        // evaluate desired neck transformation
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        yawRotation = iDynTree::Rotation::RotZ(meanYaw);
        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(m_useQPIK)
        {
            // integrate dq because velocity control mode seems not available
            yarp::sig::Vector bufferVelocity(m_robotControlHelper->getActuatedDoFs());
            yarp::sig::Vector bufferPosition(m_robotControlHelper->getActuatedDoFs());

            if(!m_FKSolver->setInternalRobotState(m_qDesired, m_dqDesired))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

            if(!solveQPIK(m_QPIKSolver, desiredCoMPosition,
                          desiredCoMVelocity,
                          yawRotation, m_dqDesired))
            {
                yError() << "[WalkingModule::updateModule] Unable to solve the QP problem with osqp.";
                return false;
            }

            iDynTree::toYarp(m_dqDesired, bufferVelocity);

            bufferPosition = m_velocityIntegral->integrate(bufferVelocity);
            iDynTree::toiDynTree(bufferPosition, m_qDesired);

            if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                                  m_robotControlHelper->getJointVelocity()))
            {
                yError() << "[WalkingModule::updateModule] Unable to set the internal robot state.";
                return false;
            }

        }
        else
        {
            if(m_IKSolver->usingAdditionalRotationTarget())
            {
                if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
                {
                    yError() << "[WalkingModule::updateModule] Error updating the inertia to world frame rotation.";
                    return false;
                }

                if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
                {
                    yError() << "[WalkingModule::updateModule] Error while setting the feedback to the inverse Kinematics.";
                    return false;
                }

                if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                                          desiredCoMPosition, m_qDesired))
                {
                    yError() << "[WalkingModule::updateModule] Error during the inverse Kinematics iteration.";
                    return false;
                }
            }
        }
        m_profiler->setEndTime("IK");

        if(!m_robotControlHelper->setDirectPositionReferences(m_qDesired))
        {
            yError() << "[WalkingModule::updateModule] Error while setting the reference position to iCub.";
            return false;
        }

        m_profiler->setEndTime("Total");

        // print timings
        m_profiler->profiling();

        iDynTree::VectorDynSize errorL(6), errorR(6);
        if(m_useQPIK)
        {
            errorR = m_QPIKSolver->getRightFootError();
            errorL = m_QPIKSolver->getLeftFootError();
        }

        // send data to the WalkingLogger
        if(m_dumpData)
        {
            iDynTree::Vector2 desiredZMP;
            if(m_useMPC)
                desiredZMP = m_walkingController->getControllerOutput();
            else
                desiredZMP = m_walkingDCMReactiveController->getControllerOutput();

            auto leftFoot = m_FKSolver->getLeftFootToWorldTransform();
            auto rightFoot = m_FKSolver->getRightFootToWorldTransform();
            iDynTree::Vector2 DCMError;
            iDynTree::toEigen( DCMError)=iDynTree::toEigen(m_FKSolver->getDCM())- iDynTree::toEigen( m_DCMPositionDesired.front());

            iDynTree::Vector2 LfootAdaptedX;
            LfootAdaptedX(0)=m_adaptatedFootLeftTransform.getPosition()(0);
            LfootAdaptedX(1)=m_adaptatedFootLeftTransform.getPosition()(1);
            //iDynTree::Lfoot_adaptedX=
            iDynTree::Vector3 estimatedBasePose =m_robotControlHelper->getEstimatedBaseTransform().getPosition();
            iDynTree::Vector2 m_isPushActiveVec;
            m_isPushActiveVec(0)=m_isPushActive;
            m_walkingLogger->sendData(m_FKSolver->getDCM(), m_DCMPositionDesired.front(),DCMError, m_DCMVelocityDesired.front(),
                                      measuredZMP, desiredZMP, m_FKSolver->getCoMPosition(),
                                      m_stableDCMModel->getCoMPosition(),
                                      m_stableDCMModel->getCoMVelocity(),
                                      leftFoot.getPosition(), leftFoot.getRotation().asRPY(),
                                      rightFoot.getPosition(), rightFoot.getRotation().asRPY(),
                                      m_leftTrajectory.front().getPosition(), m_leftTrajectory.front().getRotation().asRPY(),
                                      m_rightTrajectory.front().getPosition(), m_rightTrajectory.front().getRotation().asRPY(),
                                      errorL, errorR,LfootAdaptedX,m_FKSolver->getRootLinkToWorldTransform().getPosition(),m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY(),
                                      estimatedBasePose,m_dcmEstimatedI,m_isPushActiveVec,m_robotControlHelper->getIMUOreintation().asRPY(),m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY());
        }

        propagateTime();

        // advance all the signals
        advanceReferenceSignals();

        m_retargetingClient->setRobotBaseOrientation(yawRotation.inverse());
    }
    return true;
}

bool WalkingModule::evaluateZMP(iDynTree::Vector2& zmp)
{
    if(m_FKSolver == nullptr)
    {
        yError() << "[evaluateZMP] The FK solver is not ready.";
        return false;
    }

    iDynTree::Position zmpLeft, zmpRight, zmpWorld;
    zmpLeft.zero();
    zmpRight.zero();
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;

    const iDynTree::Wrench& rightWrench = m_robotControlHelper->getRightWrench();
    if(rightWrench.getLinearVec3()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.getAngularVec3()(1) / rightWrench.getLinearVec3()(2);
        zmpRight(1) = rightWrench.getAngularVec3()(0) / rightWrench.getLinearVec3()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
    }

    const iDynTree::Wrench& leftWrench = m_robotControlHelper->getLeftWrench();
    if(leftWrench.getLinearVec3()(2) < 0.001)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.getAngularVec3()(1) / leftWrench.getLinearVec3()(2);
        zmpLeft(1) = leftWrench.getAngularVec3()(0) / leftWrench.getLinearVec3()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
    }

    double totalZ = rightWrench.getLinearVec3()(2) + leftWrench.getLinearVec3()(2);
    if(totalZ < 0.1)
    {
        yError() << "[evaluateZMP] The total z-component of contact wrenches is too low.";
        return false;
    }

    zmpLeft = m_FKSolver->getLeftFootToWorldTransform() * zmpLeft;
    zmpRight = m_FKSolver->getRightFootToWorldTransform() * zmpRight;

    // the global zmp is given by a weighted average
    iDynTree::toEigen(zmpWorld) = ((leftWrench.getLinearVec3()(2) * zmpLeftDefined) / totalZ)
            * iDynTree::toEigen(zmpLeft) +
            ((rightWrench.getLinearVec3()(2) * zmpRightDefined)/totalZ) * iDynTree::toEigen(zmpRight);

    zmp(0) = zmpWorld(0);
    zmp(1) = zmpWorld(1);

    return true;
}

bool WalkingModule::prepareRobot(bool onTheFly)
{
    if(m_robotState != WalkingFSM::Configured && m_robotState != WalkingFSM::Stopped)
    {
        yError() << "[WalkingModule::prepareRobot] The robot can be prepared only at the "
                 << "beginning or when the controller is stopped.";
        return false;
    }

    // get the current state of the robot
    // this is necessary because the trajectories for the joints, CoM height and neck orientation
    // depend on the current state of the robot
    bool getExternalRobotBase = true;
    if(!m_robotControlHelper->getFeedbacksRaw(10, getExternalRobotBase))
    {
        yError() << "[WalkingModule::prepareRobot] Unable to get the feedback.";
        return false;
    }

    if(onTheFly)
    {
        if(!m_FKSolver->setBaseOnTheFly())
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set the onTheFly base.";
            return false;
        }

        if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
                                              m_robotControlHelper->getJointVelocity()))
        {
            yError() << "[WalkingModule::prepareRobot] Unable to set joint state.";
            return false;
        }

        // evaluate the left to right transformation, the inertial frame is on the left foot
        iDynTree::Transform leftToRightTransform = m_FKSolver->getRightFootToWorldTransform();

        // evaluate the first trajectory. The robot does not move!
        if(!generateFirstTrajectories(leftToRightTransform))
        {
            yError() << "[WalkingModule::prepareRobot] Failed to evaluate the first trajectories.";
            return false;
        }
    }
    else
    {
        // evaluate the first trajectory. The robot does not move! So the first trajectory
        if(!generateFirstTrajectories())
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

    if(!m_IKSolver->setFullModelFeedBack(m_robotControlHelper->getJointPosition()))
    {
        yError() << "[WalkingModule::prepareRobot] Error while setting the feedback to the IK solver.";
        return false;
    }

    iDynTree::Position desiredCoMPosition;
    desiredCoMPosition(0) = m_DCMPositionDesired.front()(0);
    desiredCoMPosition(1) = m_DCMPositionDesired.front()(1);
    desiredCoMPosition(2) = m_comHeightTrajectory.front();

    if(m_IKSolver->usingAdditionalRotationTarget())
    {
        // get the yow angle of both feet
        double yawLeft = m_leftTrajectory.front().getRotation().asRPY()(2);
        double yawRight = m_rightTrajectory.front().getRotation().asRPY()(2);

        // evaluate the mean of the angles
        double meanYaw = std::atan2(std::sin(yawLeft) + std::sin(yawRight),
                                    std::cos(yawLeft) + std::cos(yawRight));
        iDynTree::Rotation yawRotation, modifiedInertial;

        // it is important to notice that the inertial frames rotate with the robot
        yawRotation = iDynTree::Rotation::RotZ(meanYaw);

        yawRotation = yawRotation.inverse();
        modifiedInertial = yawRotation * m_inertial_R_worldFrame;

        if(!m_IKSolver->updateIntertiaToWorldFrameRotation(modifiedInertial))
        {
            yError() << "[WalkingModule::prepareRobot] Error updating the inertia to world frame rotation.";
            return false;
        }
    }

    if(!m_IKSolver->computeIK(m_leftTrajectory.front(), m_rightTrajectory.front(),
                              desiredCoMPosition, m_qDesired))
    {
        yError() << "[WalkingModule::prepareRobot] Inverse Kinematics failed while computing the initial position.";
        return false;
    }

    if(!m_robotControlHelper->setPositionReferences(m_qDesired, 5.0))
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
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(!m_trajectoryGenerator->generateFirstTrajectories(leftToRightTransform))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
        return false;
    }

    if(!updateTrajectories(0))
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
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unicycle planner not available.";
        return false;
    }

    if(m_robotControlHelper->isExternalRobotBaseUsed())
    {
        if(!m_trajectoryGenerator->generateFirstTrajectories(m_robotControlHelper->getBaseTransform().getPosition()))
        {
            yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
            return false;
        }
    }
    else if (m_robotControlHelper->isFloatingBaseEstimatorUsed()) {
        if(!m_trajectoryGenerator->generateFirstTrajectories(m_robotControlHelper->getEstimatedBaseTransform().getPosition()))
        {
            yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
            return false;
        }
    }
    else
    {
        if(!m_trajectoryGenerator->generateFirstTrajectories())
        {
            yError() << "[WalkingModule::generateFirstTrajectories] Failed while retrieving new trajectories from the unicycle";
            return false;
        }
    }

    if(!updateTrajectories(0))
    {
        yError() << "[WalkingModule::generateFirstTrajectories] Unable to update the trajectory.";
        return false;
    }

    // reset the time
    m_time = 0.0;

    return true;
}

bool WalkingModule::askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                       const iDynTree::Transform& measuredTransform,
                                       const size_t& mergePoint, const iDynTree::Vector2& desiredPosition)
{
    if(m_trajectoryGenerator == nullptr)
    {
        yError() << "[WalkingModule::askNewTrajectories] Unicycle planner not available.";
        return false;
    }

    if(mergePoint >= m_DCMPositionDesired.size())
    {
        yError() << "[WalkingModule::askNewTrajectories] The mergePoint has to be lower than the trajectory size.";
        return false;
    }
    if (!m_useStepAdaptation) {
        if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionDesired[mergePoint],
                                                      m_DCMVelocityDesired[mergePoint], isLeftSwinging,
                                                      measuredTransform, desiredPosition))
        {
            yError() << "[WalkingModule::askNewTrajectories] Unable to update the trajectory.";
            return false;
        }
    }
    else {
        if(!m_trajectoryGenerator->updateTrajectories(initTime, m_DCMPositionAdjusted[mergePoint],
                                                      m_DCMVelocityAdjusted[mergePoint], isLeftSwinging,
                                                      measuredTransform, desiredPosition)) {
            yError() << "[WalkingModule::askNewTrajectories] Unable to update the trajectory.";
            return false;
        }
    }

    return true;
}

bool WalkingModule::updateTrajectories(const size_t& mergePoint)
{
    if(!(m_trajectoryGenerator->isTrajectoryComputed()))
    {
        yError() << "[updateTrajectories] The trajectory is not computed.";
        return false;
    }

    std::vector<iDynTree::Transform> leftTrajectory;
    std::vector<iDynTree::Transform> rightTrajectory;
    std::vector<iDynTree::Twist> leftTwistTrajectory;
    std::vector<iDynTree::Twist> rightTwistTrajectory;
    std::vector<iDynTree::Vector2> DCMPositionDesired;
    std::vector<iDynTree::Vector2> DCMVelocityDesired;
    std::vector<bool> rightInContact;
    std::vector<bool> leftInContact;
    std::vector<double> comHeightTrajectory;
    std::vector<double> comHeightVelocity;
    std::vector<size_t> mergePoints;
    std::vector<bool> isLeftFixedFrame;
    std::vector<iDynTree::Vector2> ZMPPositionDesired;


    timeOffset = m_time + mergePoint * m_dT;
    // get dcm position and velocity
    m_trajectoryGenerator->getDCMPositionTrajectory(DCMPositionDesired);
    m_trajectoryGenerator->getDCMVelocityTrajectory(DCMVelocityDesired);
    m_trajectoryGenerator->getZMPPositionTrajectory(ZMPPositionDesired);
    // get feet trajectories
    m_trajectoryGenerator->getFeetTrajectories(leftTrajectory, rightTrajectory);
    m_trajectoryGenerator->getFeetTwist(leftTwistTrajectory, rightTwistTrajectory);
    m_trajectoryGenerator->getFeetStandingPeriods(leftInContact, rightInContact);
    m_trajectoryGenerator->getWhenUseLeftAsFixed(isLeftFixedFrame);

    // get com height trajectory
    m_trajectoryGenerator->getCoMHeightTrajectory(comHeightTrajectory);
    m_trajectoryGenerator->getCoMHeightVelocity(comHeightVelocity);

    // get merge points
    m_trajectoryGenerator->getMergePoints(mergePoints);

    // append vectors to deques
    StdHelper::appendVectorToDeque(leftTrajectory, m_leftTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTrajectory, m_rightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(leftTwistTrajectory, m_leftTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(rightTwistTrajectory, m_rightTwistTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(isLeftFixedFrame, m_isLeftFixedFrame, mergePoint);

    StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionDesired, mergePoint);
    StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityDesired, mergePoint);

//    if (m_useStepAdaptation) {
//        StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionAdjusted, mergePoint);
//        StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityAdjusted, mergePoint);
//    }


    StdHelper::appendVectorToDeque(DCMPositionDesired, m_DCMPositionAdjusted, mergePoint);
    StdHelper::appendVectorToDeque(DCMVelocityDesired, m_DCMVelocityAdjusted, mergePoint);

    StdHelper::appendVectorToDeque(ZMPPositionDesired, m_ZMPPositionDesired, mergePoint);
    StdHelper::appendVectorToDeque(leftInContact, m_leftInContact, mergePoint);
    StdHelper::appendVectorToDeque(rightInContact, m_rightInContact, mergePoint);

    StdHelper::appendVectorToDeque(comHeightTrajectory, m_comHeightTrajectory, mergePoint);
    StdHelper::appendVectorToDeque(comHeightVelocity, m_comHeightVelocity, mergePoint);

    m_mergePoints.assign(mergePoints.begin(), mergePoints.end());

    if (m_useStepAdaptation) {
        m_DCMSubTrajectories.clear();
        m_trajectoryGenerator->getDCMSubTrajectory(m_DCMSubTrajectories);

        std::shared_ptr<FootPrint> tempLeft;
        m_trajectoryGenerator->getLeftFootprint(tempLeft);
        m_jleftFootprints = std::make_shared<FootPrint>();
        m_jleftFootprints->setFootName("left");
        for(auto step: tempLeft->getSteps())
            m_jleftFootprints->addStep(step);

        // StepList jLeftstepList=jleftFootprints->getSteps();
        m_jLeftstepList=m_jleftFootprints->getSteps();
        std::shared_ptr<FootPrint> tempRight;
        m_trajectoryGenerator->getRightFootprint(tempRight);
        m_jRightFootprints = std::make_shared<FootPrint>();
        m_jRightFootprints->setFootName("right");
        for(auto step: tempRight->getSteps())
            m_jRightFootprints->addStep(step);


        m_jRightstepList=m_jRightFootprints->getSteps();
    }




    // the first merge point is always equal to 0
    m_mergePoints.pop_front();


    if (m_useStepAdaptation) {
        m_mergePoints.size();


        m_adaptatedFootLeftTwist.zero();
        m_adaptatedFootRightTwist.zero();
        m_adaptatedFootLeftAcceleration.zero();
        m_adaptatedFootRightAcceleration.zero();

        m_adaptatedFootLeftTransform = leftTrajectory.front();
        m_adaptatedFootRightTransform = rightTrajectory.front();
        m_adaptatedFootRightTwist.zero();

        // int numberOfSubTrajectories = m_DCMSubTrajectories.size();
        // if(numberOfSubTrajectories  >=4)
        // {
        //     auto firstSS = m_DCMSubTrajectories[numberOfSubTrajectories-2];
        //     auto secondSS = m_DCMSubTrajectories[numberOfSubTrajectories-4];

        //     iDynTree::Vector2 zmp;
        //     secondSS->getZMPPosition(0, zmp, false);
        //     m_stepAdaptator->reset(zmp, firstSS->getTrajectoryDomain().second);
        // }
    }
    return true;
}

bool WalkingModule::updateFKSolver()
{
    if(m_robotControlHelper->isFloatingBaseEstimatorUsed())
    {
        m_FKSolver->evaluateWorldToBaseTransformation(m_robotControlHelper->getEstimatedBaseTransform(),
                                                      m_robotControlHelper->getEstimatedBaseTwist());
    }
    else if (m_robotControlHelper->isExternalRobotBaseUsed())
    {
        m_FKSolver->evaluateWorldToBaseTransformation(m_robotControlHelper->getBaseTransform(),
                                                      m_robotControlHelper->getBaseTwist());
    }
    else {
        if(!m_FKSolver->evaluateWorldToBaseTransformation(m_leftTrajectory.front(),
                                                          m_rightTrajectory.front(),
                                                          m_isLeftFixedFrame.front()))
        {
            yError() << "[WalkingModule::updateFKSolver] Unable to evaluate the world to base transformation.";
            return false;
        }
    }

    if(!m_FKSolver->setInternalRobotState(m_robotControlHelper->getJointPosition(),
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

    if(m_robotState != WalkingFSM::Prepared && m_robotState != WalkingFSM::Paused)
    {
        yError() << "[WalkingModule::startWalking] Unable to start walking if the robot is not prepared or paused.";
        return false;
    }

    if(m_dumpData)
    {
        m_walkingLogger->startRecord({"record","dcm_x", "dcm_y",
                                      "dcm_des_x", "dcm_des_y","errorx","errory",
                                      "dcm_des_dx", "dcm_des_dy",
                                      "zmp_x", "zmp_y",
                                      "zmp_des_x", "zmp_des_y",
                                      "com_x", "com_y", "com_z",
                                      "com_des_x", "com_des_y",
                                      "com_des_dx", "com_des_dy",
                                      "lf_x", "lf_y", "lf_z",
                                      "lf_roll", "lf_pitch", "lf_yaw",
                                      "rf_x", "rf_y", "rf_z",
                                      "rf_roll", "rf_pitch", "rf_yaw",
                                      "lf_des_x", "lf_des_y", "lf_des_z",
                                      "lf_des_roll", "lf_des_pitch", "lf_des_yaw",
                                      "rf_des_x", "rf_des_y", "rf_des_z",
                                      "rf_des_roll", "rf_des_pitch", "rf_des_yaw",
                                      "lf_err_x", "lf_err_y", "lf_err_z",
                                      "lf_err_roll", "lf_err_pitch", "lf_err_yaw",
                                      "rf_err_x", "rf_err_y", "rf_err_z",
                                      "rf_err_roll", "rf_err_pitch", "rf_err_yaw","Lfoot_adaptedX","Lfoot_adaptedY",
                                      "base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw","estimate_base_x", "estimate_base_y", "estimate_base_z",
                                      "dcm_estimated_x","dcm_estimated_y","IsPushActivex","IsPushActivey","imu_roll","imu_pitch","imu_yaw","roll_des","pitch_des","yaw_des"});
    }

    if(m_robotState == WalkingFSM::Prepared)
    {
        m_robotControlHelper->resetFilters();

        updateFKSolver();

        // TODO this is useful for the simulation
        double heightOffset = (m_FKSolver->getLeftFootToWorldTransform().getPosition()(2)
                               + m_FKSolver->getRightFootToWorldTransform().getPosition()(2)) / 2;

        m_robotControlHelper->setHeightOffset(heightOffset);
    }

    iDynTree::Transform stanceFoot_T_world = m_trajectoryGenerator->swingLeft() ?
                m_rightTrajectory.front().inverse() : m_leftTrajectory.front().inverse();

    std::string frameName = m_trajectoryGenerator->swingLeft() ? "r_sole" : "l_sole";

    if (m_useFloatingBaseEstimator) {
        yarp::os::Bottle cmd, outcome;
        cmd.addString("resetLeggedOdometryWithRefFrame");
        cmd.addString(frameName);
        cmd.addDouble(stanceFoot_T_world.getPosition()(0));
        cmd.addDouble(stanceFoot_T_world.getPosition()(1));
        cmd.addDouble(stanceFoot_T_world.getPosition()(2));
        cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(0));
        cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(1));
        cmd.addDouble(stanceFoot_T_world.getRotation().asRPY()(2));
        m_rpcBaseEstPort.write(cmd,outcome);

        if(!outcome.get(0).asBool())
        {
            yError() << "[startWalking] Unable reset the odometry.";
            return false;
        }
    }


    m_robotState = WalkingFSM::Walking;

    return true;
}

bool WalkingModule::setPlannerInput(double x, double y)
{
    // the trajectory was already finished the new trajectory will be attached as soon as possible
    if(m_mergePoints.empty())
    {
        if(!(m_leftInContact.front() && m_rightInContact.front()))
        {
            yError() << "[WalkingModule::setPlannerInput] The trajectory has already finished but the system is not in double support.";
            return false;
        }

        if(m_newTrajectoryRequired)
            return true;

        // Since the evaluation of a new trajectory takes time the new trajectory will be merged after x cycles
        m_newTrajectoryMergeCounter = 10;
    }

    // the trajectory was not finished the new trajectory will be attached at the next merge point
    else
    {
        if(m_mergePoints.front() > 10)
            m_newTrajectoryMergeCounter = m_mergePoints.front();
        else if(m_mergePoints.size() > 1)
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = m_mergePoints[1];
        }
        else
        {
            if(m_newTrajectoryRequired)
                return true;

            m_newTrajectoryMergeCounter = 10;
        }
    }

    m_desiredPosition(0) = x;
    m_desiredPosition(1) = y;

    m_newTrajectoryRequired = true;

    return true;
}

bool WalkingModule::setGoal(double x, double y)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
        return false;

    return setPlannerInput(x, y);
}

bool WalkingModule::pauseWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
        return false;

    // close the logger
    if(m_dumpData)
        m_walkingLogger->quit();

    m_robotState = WalkingFSM::Paused;
    return true;
}

bool WalkingModule::stopWalking()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if(m_robotState != WalkingFSM::Walking)
        return false;

    reset();

    m_robotState = WalkingFSM::Stopped;
    return true;
}
