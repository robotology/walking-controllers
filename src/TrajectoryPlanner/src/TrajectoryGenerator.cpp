// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/YARPConfigurationsLoader.h>

#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

TrajectoryGenerator::~TrajectoryGenerator()
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_generatorState = GeneratorState::Closing;
        m_conditionVariable.notify_one();
    }

    if(m_generatorThread.joinable())
    {
        m_generatorThread.join();
        m_generatorThread = std::thread();
    }
}

bool TrajectoryGenerator::initialize(const yarp::os::Searchable& config)
{
    if(!configurePlanner(config))
    {
        yError() << "[initialize] Failed to configure the unicycle trajectory generator.";
        return false;
    }
    return true;
}

bool TrajectoryGenerator::configurePlanner(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[configurePlanner] Empty configuration for unicycle planner.";
        return false;
    }


    m_dT = config.check("sampling_time", yarp::os::Value(0.016)).asFloat64();
    m_plannerHorizon = config.check("plannerHorizon", yarp::os::Value(20.0)).asFloat64();
    double unicycleGain = config.check("unicycleGain", yarp::os::Value(10.0)).asFloat64();
    double stancePhaseDelaySeconds = config.check("stance_phase_delay",yarp::os::Value(0.0)).asFloat64();

    m_stancePhaseDelay = (std::size_t) std::round(stancePhaseDelaySeconds / m_dT);

    if(!YarpUtilities::getVectorFromSearchable(config, "referencePosition", m_referencePointDistance))
    {
        yError() << "[configurePlanner] Initialization failed while reading referencePosition vector.";
        return false;
    }

    // get left and right ZMP delta
    iDynTree::Vector2 leftZMPDelta;
    if(!YarpUtilities::getVectorFromSearchable(config, "leftZMPDelta", leftZMPDelta))
    {
        yError() << "[configurePlanner] Initialization failed while reading leftZMPDelta vector.";
        return false;
    }

    iDynTree::Vector2 rightZMPDelta;
    if(!YarpUtilities::getVectorFromSearchable(config, "rightZMPDelta", rightZMPDelta))
    {
        yError() << "[configurePlanner] Initialization failed while reading rightZMPDelta vector.";
        return false;
    }

    iDynTree::Vector2 saturationFactors;
    if(!YarpUtilities::getVectorFromSearchable(config, "saturationFactors", saturationFactors))
    {
        yError() << "[configurePlanner] Initialization failed while reading saturationFactors vector.";
        return false;
    }

    std::string unicycleController = config.check("controlType", yarp::os::Value("personFollowing")).asString();

    if (unicycleController == "personFollowing")
    {
        m_unicycleController = UnicycleController::PERSON_FOLLOWING;
    }
    else if (unicycleController == "direct")
    {
        m_unicycleController = UnicycleController::DIRECT;
    }
    else
    {
        yError() << "[configurePlanner] Initialization failed while reading controlType vector. Cannot use"
                 << unicycleController << "as controllerType. Only personFollowing and direct are available.";
        return false;
    }

    double timeWeight = config.check("timeWeight", yarp::os::Value(2.5)).asFloat64();
    double positionWeight = config.check("positionWeight", yarp::os::Value(1.0)).asFloat64();
    double slowWhenTurningGain = config.check("slowWhenTurningGain", yarp::os::Value(0.0)).asFloat64();
    double slowWhenBackwardFactor = config.check("slowWhenBackwardFactor", yarp::os::Value(1.0)).asFloat64();
    double slowWhenSidewaysFactor = config.check("slowWhenSidewaysFactor", yarp::os::Value(1.0)).asFloat64();
    double maxStepLength = config.check("maxStepLength", yarp::os::Value(0.05)).asFloat64();
    double maxStepLengthBackwardMultiplier = config.check("maxLengthBackwardFactor", yarp::os::Value(1.0)).asFloat64();
    double minStepLength = config.check("minStepLength", yarp::os::Value(0.005)).asFloat64();
    double minWidth = config.check("minWidth", yarp::os::Value(0.03)).asFloat64();
    double maxAngleVariation = iDynTree::deg2rad(config.check("maxAngleVariation",
                                                              yarp::os::Value(40.0)).asFloat64());
    double minAngleVariation = iDynTree::deg2rad(config.check("minAngleVariation",
                                                              yarp::os::Value(5.0)).asFloat64());
    double maxStepDuration = config.check("maxStepDuration", yarp::os::Value(8.0)).asFloat64();
    double minStepDuration = config.check("minStepDuration", yarp::os::Value(2.9)).asFloat64();
    double stepHeight = config.check("stepHeight", yarp::os::Value(0.005)).asFloat64();
    double landingVelocity = config.check("stepLandingVelocity", yarp::os::Value(0.0)).asFloat64();
    double apexTime = config.check("footApexTime", yarp::os::Value(0.5)).asFloat64();
    double comHeight = config.check("com_height", yarp::os::Value(0.49)).asFloat64();
    double comHeightDelta = config.check("comHeightDelta", yarp::os::Value(0.01)).asFloat64();
    double nominalDuration = config.check("nominalDuration", yarp::os::Value(4.0)).asFloat64();
    double lastStepSwitchTime = config.check("lastStepSwitchTime", yarp::os::Value(0.5)).asFloat64();
    double switchOverSwingRatio = config.check("switchOverSwingRatio",
                                               yarp::os::Value(0.4)).asFloat64();
    double lastStepDCMOffset = config.check("lastStepDCMOffset", yarp::os::Value(0.0)).asFloat64();
    double lastStepDCMStillPercentage = config.check("lastStepDCMStillPercentage", yarp::os::Value(0.1)).asFloat64();
    m_leftYawDeltaInRad = iDynTree::deg2rad(config.check("leftYawDeltaInDeg", yarp::os::Value(0.0)).asFloat64());
    m_rightYawDeltaInRad = iDynTree::deg2rad(config.check("rightYawDeltaInDeg", yarp::os::Value(0.0)).asFloat64());

    m_nominalWidth = config.check("nominalWidth", yarp::os::Value(0.04)).asFloat64();

    m_swingLeft = config.check("swingLeft", yarp::os::Value(true)).asBool();
    bool startWithSameFoot = config.check("startAlwaysSameFoot", yarp::os::Value(false)).asBool();
    m_useMinimumJerk = config.check("useMinimumJerkFootTrajectory",
                                    yarp::os::Value(false)).asBool();
    double pitchDelta = config.check("pitchDelta", yarp::os::Value(0.0)).asFloat64();

    bool isPauseActive = config.check("isPauseActive", yarp::os::Value(true)).asBool();

    iDynTree::Vector2 mergePointRatios;
    if(!YarpUtilities::getVectorFromSearchable(config, "mergePointRatios", mergePointRatios))
    {
        yError() << "[configurePlanner] Initialization failed while reading mergePointRatios vector.";
        return false;
    }

    yarp::os::Bottle ellipseMethodGroup = config.findGroup("ELLIPSE_METHOD_SETTINGS");
    double freeSpaceConservativeFactor = ellipseMethodGroup.check("conservative_factor", yarp::os::Value(2.0)).asFloat64();
    double innerEllipseSemiMajorOffset = ellipseMethodGroup.check("inner_offset_major", yarp::os::Value(0.0)).asFloat64();
    double innerEllipseSemiMinorOffset = ellipseMethodGroup.check("inner_offset_minor", yarp::os::Value(0.0)).asFloat64();


    // try to configure the planner
    std::shared_ptr<UnicyclePlanner> unicyclePlanner = m_trajectoryGenerator.unicyclePlanner();
    bool ok = true;
    ok = ok && unicyclePlanner->setDesiredPersonDistance(m_referencePointDistance(0),
                                                              m_referencePointDistance(1));
    ok = ok && unicyclePlanner->setPersonFollowingControllerGain(unicycleGain);
    ok = ok && unicyclePlanner->setMaximumIntegratorStepSize(m_dT);
    ok = ok && unicyclePlanner->setMaxStepLength(maxStepLength, maxStepLengthBackwardMultiplier);
    ok = ok && unicyclePlanner->setWidthSetting(minWidth, m_nominalWidth);
    ok = ok && unicyclePlanner->setMaxAngleVariation(maxAngleVariation);
    ok = ok && unicyclePlanner->setCostWeights(positionWeight, timeWeight);
    ok = ok && unicyclePlanner->setStepTimings(minStepDuration,
                                               maxStepDuration, nominalDuration);
    ok = ok && unicyclePlanner->setPlannerPeriod(m_dT);
    ok = ok && unicyclePlanner->setMinimumAngleForNewSteps(minAngleVariation);
    ok = ok && unicyclePlanner->setMinimumStepLength(minStepLength);
    ok = ok && unicyclePlanner->setSlowWhenTurnGain(slowWhenTurningGain);
    ok = ok && unicyclePlanner->setSlowWhenBackwardFactor(slowWhenBackwardFactor);
    ok = ok && unicyclePlanner->setSlowWhenSidewaysFactor(slowWhenSidewaysFactor);
    ok = ok && unicyclePlanner->setSaturationsConservativeFactors(saturationFactors(0), saturationFactors(1));
    unicyclePlanner->setLeftFootYawOffsetInRadians(m_leftYawDeltaInRad);
    unicyclePlanner->setRightFootYawOffsetInRadians(m_rightYawDeltaInRad);
    unicyclePlanner->addTerminalStep(true);
    unicyclePlanner->startWithLeft(m_swingLeft);
    unicyclePlanner->resetStartingFootIfStill(startWithSameFoot);
    ok = ok && unicyclePlanner->setFreeSpaceEllipseConservativeFactor(freeSpaceConservativeFactor);
    ok = ok && unicyclePlanner->setInnerFreeSpaceEllipseOffsets(innerEllipseSemiMajorOffset, innerEllipseSemiMinorOffset);
    ok = ok && unicyclePlanner->setUnicycleController(m_unicycleController);
    ok = ok && m_trajectoryGenerator.setSwitchOverSwingRatio(switchOverSwingRatio);
    ok = ok && m_trajectoryGenerator.setTerminalHalfSwitchTime(lastStepSwitchTime);
    ok = ok && m_trajectoryGenerator.setPauseConditions(maxStepDuration, nominalDuration);

    m_trajectoryGenerator.setPauseActive(isPauseActive);

    if (m_useMinimumJerk) {
        m_feetGenerator = m_trajectoryGenerator.addFeetMinimumJerkGenerator();
    } else {
        m_feetGenerator = m_trajectoryGenerator.addFeetCubicSplineGenerator();
    }
    ok = ok && m_feetGenerator->setStepHeight(stepHeight);
    ok = ok && m_feetGenerator->setFootLandingVelocity(landingVelocity);
    ok = ok && m_feetGenerator->setFootApexTime(apexTime);
    ok = ok && m_feetGenerator->setPitchDelta(pitchDelta);

    m_heightGenerator = m_trajectoryGenerator.addCoMHeightTrajectoryGenerator();
    ok = ok && m_heightGenerator->setCoMHeightSettings(comHeight, comHeightDelta);
    ok = ok && m_trajectoryGenerator.setMergePointRatio(mergePointRatios[0], mergePointRatios[1]);

    m_dcmGenerator = m_trajectoryGenerator.addDCMTrajectoryGenerator();
    m_dcmGenerator->setFootOriginOffset(leftZMPDelta, rightZMPDelta);
    m_dcmGenerator->setOmega(sqrt(9.81/comHeight));
    m_dcmGenerator->setFirstDCMTrajectoryMode(FirstDCMTrajectoryMode::FifthOrderPoly);
    ok = ok && m_dcmGenerator->setLastStepDCMOffsetPercentage(lastStepDCMOffset);
    ok = ok && m_dcmGenerator->setStillnessPercentage(lastStepDCMStillPercentage);

    m_correctLeft = true;

    m_newFreeSpaceEllipse = false;

    if(ok)
    {
        // the mutex is automatically released when lock_guard goes out of its scope
        std::lock_guard<std::mutex> guard(m_mutex);

        // change the state of the generator
        m_generatorState = GeneratorState::FirstStep;

        // start the thread
        m_generatorThread = std::thread(&TrajectoryGenerator::computeThread, this);
    }

    if(!iDynTree::parseRotationMatrix(config, "additional_chest_rotation", m_chestAdditionalRotation))
    {
        yError() << "[initialize] Unable to set the additional chest rotation.";
        return false;
    }

    m_unicyclePose = iDynTree::Transform::Identity();

    return ok;
}

void TrajectoryGenerator::addTerminalStep(bool terminalStep)
{
    m_trajectoryGenerator.unicyclePlanner()->addTerminalStep(terminalStep);
}

void TrajectoryGenerator::computeThread()
{
    while (true)
    {
        double initTime;
        double endTime;
        double dT;

        double nominalWidth;

        bool correctLeft;

        iDynTree::Vector2 desiredPointInRelativeFrame, desiredPointInAbsoluteFrame;
        iDynTree::Vector2 measuredPositionLeft, measuredPositionRight;
        iDynTree::Vector3 desiredDirectControl;
        double measuredAngleLeft, measuredAngleRight;
        double leftYawDeltaInRad, rightYawDeltaInRad;

        iDynTree::Vector2 DCMBoundaryConditionAtMergePointPosition;
        iDynTree::Vector2 DCMBoundaryConditionAtMergePointVelocity;

        bool shouldUpdateEllipsoid;
        FreeSpaceEllipse freeSpaceEllipse;

        iDynTree::Vector2 measuredPosition;
        double measuredAngle;
        DCMInitialState initialState;
        Eigen::Vector2d unicyclePositionFromStanceFoot, footPosition, unicyclePosition;
        unicyclePositionFromStanceFoot(0) = 0.0;

        Eigen::Matrix2d unicycleRotation;
        double unicycleAngle;

        // wait until a new trajectory has to be evaluated.
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_conditionVariable.wait(lock, [&]{return ((m_generatorState == GeneratorState::Called)
                                                       || (m_generatorState == GeneratorState::Closing));});

            if(m_generatorState == GeneratorState::Closing)
                break;

            // set timings
            dT = m_dT ;
            initTime = m_initTime;
            endTime = initTime + m_plannerHorizon;

            // set desired point
            desiredPointInRelativeFrame = m_personFollowingDesiredPoint;

            desiredDirectControl = m_desiredDirectControl;

            // dcm boundary conditions
            DCMBoundaryConditionAtMergePointPosition = m_DCMBoundaryConditionAtMergePointPosition;
            DCMBoundaryConditionAtMergePointVelocity = m_DCMBoundaryConditionAtMergePointVelocity;

            // left foot
            measuredPositionLeft(0) = m_measuredTransformLeft.getPosition()(0);
            measuredPositionLeft(1) = m_measuredTransformLeft.getPosition()(1);
            measuredAngleLeft = m_measuredTransformLeft.getRotation().asRPY()(2);
            leftYawDeltaInRad = m_leftYawDeltaInRad;

            // right foot
            measuredPositionRight(0) = m_measuredTransformRight.getPosition()(0);
            measuredPositionRight(1) = m_measuredTransformRight.getPosition()(1);
            measuredAngleRight = m_measuredTransformRight.getRotation().asRPY()(2);
            rightYawDeltaInRad = m_rightYawDeltaInRad;

            correctLeft = m_correctLeft;

            freeSpaceEllipse = m_freeSpaceEllipse;
            shouldUpdateEllipsoid = m_newFreeSpaceEllipse;
            m_newFreeSpaceEllipse = false;
            nominalWidth = m_nominalWidth;

            if (shouldUpdateEllipsoid)
            {
                yInfo() << "[TrajectoryGenerator_Thread] Setting ellipsoid: " << freeSpaceEllipse.printInfo();
            }

            measuredPosition = correctLeft ? measuredPositionLeft : measuredPositionRight;
            measuredAngle = correctLeft ? measuredAngleLeft : measuredAngleRight;

            initialState.initialPosition = DCMBoundaryConditionAtMergePointPosition;
            initialState.initialVelocity = DCMBoundaryConditionAtMergePointVelocity;

            if (correctLeft)
            {
                unicyclePositionFromStanceFoot(1) = -nominalWidth/2;
                unicycleAngle = measuredAngleLeft - leftYawDeltaInRad;
                footPosition = iDynTree::toEigen(measuredPositionLeft);
            }
            else
            {
                unicyclePositionFromStanceFoot(1) = nominalWidth/2;
                unicycleAngle = measuredAngleRight - rightYawDeltaInRad;
                footPosition = iDynTree::toEigen(measuredPositionRight);
            }

            double s_theta = std::sin(unicycleAngle);
            double c_theta = std::cos(unicycleAngle);

            unicycleRotation(0,0) = c_theta;
            unicycleRotation(0,1) = -s_theta;
            unicycleRotation(1,0) = s_theta;
            unicycleRotation(1,1) = c_theta;

            unicyclePosition = unicycleRotation * unicyclePositionFromStanceFoot + footPosition;

            iDynTree::Position unicyclePosition3D(unicyclePosition(0), unicyclePosition(1), 0.0);
            iDynTree::Rotation unicycleRotation3D = iDynTree::Rotation::RotZ(unicycleAngle);
            m_unicyclePose = iDynTree::Transform(unicycleRotation3D, unicyclePosition3D);
        }

        // apply the homogeneous transformation w_H_{unicycle}
        iDynTree::toEigen(desiredPointInAbsoluteFrame) = unicycleRotation * (iDynTree::toEigen(m_referencePointDistance) +
                                                                             iDynTree::toEigen(desiredPointInRelativeFrame))
                + unicyclePosition;

        // clear the old trajectory
        std::shared_ptr<UnicyclePlanner> unicyclePlanner = m_trajectoryGenerator.unicyclePlanner();
        unicyclePlanner->clearPersonFollowingDesiredTrajectory();

        // add new point
        if(!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(endTime, desiredPointInAbsoluteFrame))
        {
            // something goes wrong
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Configured;
            yError() << "[TrajectoryGenerator_Thread] Error while setting the new reference.";
            break;
        }

        unicyclePlanner->setDesiredDirectControl(desiredDirectControl(0), desiredDirectControl(1), desiredDirectControl(2));

        if (!m_dcmGenerator->setDCMInitialState(initialState)) {
            // something goes wrong
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Configured;
            yError() << "[TrajectoryGenerator_Thread] Failed to set the initial state.";
            break;
        }

        if (shouldUpdateEllipsoid)
        {
            iDynTree::MatrixFixSize<2,2> ellipseImage = freeSpaceEllipse.imageMatrix(), newEllipseImage;
            iDynTree::VectorFixSize<2> centerOffset = freeSpaceEllipse.centerOffset(), newCenterOffset;

            iDynTree::toEigen(newEllipseImage) = unicycleRotation * iDynTree::toEigen(ellipseImage);

            iDynTree::toEigen(newCenterOffset) = unicycleRotation * iDynTree::toEigen(centerOffset) + unicyclePosition;

            if (!freeSpaceEllipse.setEllipse(newEllipseImage, newCenterOffset))
            {
                std::lock_guard<std::mutex> guard(m_mutex);
                m_generatorState = GeneratorState::Configured;
                yError() << "[TrajectoryGenerator_Thread] Failed in setting the free space ellipsoid to a world frame.";
                continue;
            }

            if (!m_trajectoryGenerator.unicyclePlanner()->setFreeSpaceEllipse(freeSpaceEllipse))
            {
                std::lock_guard<std::mutex> guard(m_mutex);
                m_generatorState = GeneratorState::Configured;
                yError() << "[TrajectoryGenerator_Thread] Failed in setting free space ellipsoid.";
                continue;
            }
        }

        if(m_trajectoryGenerator.reGenerate(initTime, dT, endTime,
                                            correctLeft, measuredPosition, measuredAngle))
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Returned;
            continue;
        }
        else
        {
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Configured;
            yError() << "[TrajectoryGenerator_Thread] Failed in computing new trajectory.";
            continue;
        }
    }
}

bool TrajectoryGenerator::generateFirstTrajectories(const iDynTree::Position& initialBasePosition)
{
    // check if this step is the first one
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if(m_generatorState != GeneratorState::FirstStep)
        {
            yError() << "[generateFirstTrajectories] This is not the first step! How can I generate the first step?";
            return false;
        }
    }

    // clear the all trajectory
    std::shared_ptr<UnicyclePlanner> unicyclePlanner = m_trajectoryGenerator.unicyclePlanner();
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();
    unicyclePlanner->setDesiredDirectControl(0.0, 0.0, 0.0);

    // clear left and right footsteps
    m_trajectoryGenerator.getLeftFootPrint()->clearSteps();
    m_trajectoryGenerator.getRightFootPrint()->clearSteps();

    // set initial and final times
    double initTime = 0;
    double endTime = initTime + m_plannerHorizon;

    // at the beginning iCub has to stop
    m_personFollowingDesiredPoint(0) = m_referencePointDistance(0) + initialBasePosition(0);
    m_personFollowingDesiredPoint(1) = m_referencePointDistance(1) + initialBasePosition(1);

    // add the initial point
    if(!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime, m_personFollowingDesiredPoint))
    {
        yError() << "[generateFirstTrajectories] Error while setting the first reference.";
        return false;
    }

    // add the final point
    if(!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(endTime, m_personFollowingDesiredPoint))
    {
        yError() << "[generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    // generate the first trajectories
    if(!m_trajectoryGenerator.generate(initTime, m_dT, endTime))
    {
        yError() << "[generateFirstTrajectories] Error while computing the first trajectories.";
        return false;
    }

    m_generatorState = GeneratorState::Returned;
    return true;
}

bool TrajectoryGenerator::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform)
                                                    // const iDynTree::Position &initialCOMPosition)
{
    // check if this step is the first one
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if(m_generatorState != GeneratorState::FirstStep)
        {
            yError() << "[generateFirstTrajectories] This is not the first step! How can I generate the first step?";
            return false;
        }
    }

    // clear the all trajectory
    std::shared_ptr<UnicyclePlanner> unicyclePlanner = m_trajectoryGenerator.unicyclePlanner();
    unicyclePlanner->clearPersonFollowingDesiredTrajectory();
    unicyclePlanner->setDesiredDirectControl(0.0, 0.0, 0.0);

    // clear left and right footsteps
    m_trajectoryGenerator.getLeftFootPrint()->clearSteps();
    m_trajectoryGenerator.getRightFootPrint()->clearSteps();

    // set initial and final times
    double initTime = 0;
    double endTime = initTime + m_plannerHorizon;

    // at the beginning iCub has to stop
    m_personFollowingDesiredPoint(0) = m_referencePointDistance(0);
    m_personFollowingDesiredPoint(1) = m_referencePointDistance(1);

    // add the initial point
    if(!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(initTime, m_referencePointDistance))
    {
        yError() << "[generateFirstTrajectories] Error while setting the first reference.";
        return false;
    }

    // add the final point
    if(!unicyclePlanner->addPersonFollowingDesiredTrajectoryPoint(endTime, m_personFollowingDesiredPoint))
    {
        yError() << "[generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    // add real position of the feet
    std::shared_ptr<FootPrint> left, right;

    left = m_trajectoryGenerator.getLeftFootPrint();
    left->clearSteps();
    right = m_trajectoryGenerator.getRightFootPrint();
    right->clearSteps();

    iDynTree::Vector2 leftPosition, rightPosition;
    double leftAngle, rightAngle;

    // if the standing foot is the right -> the unicycle starts parallel to the right foot
    if(m_swingLeft)
    {
        rightPosition(0) = 0.0;
        rightPosition(1) = -leftToRightTransform.inverse().getPosition()(1)/2;
        rightAngle = 0;
        right->addStep(rightPosition, rightAngle, 0.0);

        leftPosition(0) = leftToRightTransform.inverse().getPosition()(0);
        leftPosition(1) = leftToRightTransform.inverse().getPosition()(1)/2;
        leftAngle = leftToRightTransform.inverse().getRotation().asRPY()(2);
        left->addStep(leftPosition, leftAngle, 0.0);
    }
    else
    {
        leftPosition(0) = 0.0;
        leftPosition(1) = -leftToRightTransform.getPosition()(1)/2;
        leftAngle = 0;
        left->addStep(leftPosition, leftAngle, 0.0);

        rightPosition(0) = leftToRightTransform.getPosition()(0);
        rightPosition(1) = leftToRightTransform.getPosition()(1)/2;
        rightAngle = leftToRightTransform.getRotation().asRPY()(2);
        right->addStep(rightPosition, rightAngle, 0.0);
    }

    // generate the first trajectories
    if(!m_trajectoryGenerator.generate(initTime, m_dT, endTime))
    {
        yError() << "[generateFirstTrajectories] Error while computing the first trajectories.";
        return false;
    }

    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_generatorState = GeneratorState::Returned;
    }
    return true;
}

bool TrajectoryGenerator::updateTrajectories(double initTime, const iDynTree::Vector2& DCMBoundaryConditionAtMergePointPosition,
                                             const iDynTree::Vector2& DCMBoundaryConditionAtMergePointVelocity, bool correctLeft,
                                             const iDynTree::Transform& measured, const iDynTree::VectorDynSize &plannerDesiredInput)
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);

        if(m_generatorState == GeneratorState::Called)
        {
            yError() << "[updateTrajectories] Cannot launch the generator twice. "
                     << "Please wait until the trajectory has be evaluated .";
            return false;
        }

        if(m_generatorState != GeneratorState::Returned)
        {
            yError() << "[updateTrajectories] The trajectory generator has not computed any trajectory yet. "
                     << "Please call 'generateFirstTrajectories()' method.";
            return false;
        }

        m_personFollowingDesiredPoint.zero();
        m_desiredDirectControl.zero();

        if (m_unicycleController == UnicycleController::PERSON_FOLLOWING)
        {
            if (plannerDesiredInput.size() < 2)
            {
                yErrorThrottle(1.0) << "[updateTrajectories] The plannerDesiredInput is supposed to have dimension 2, while it has dimension" << plannerDesiredInput.size()
                                    << ". Using zero input.";
            }
            else
            {
                if (plannerDesiredInput.size() > 2)
                {
                    yWarningOnce() << "[updateTrajectories] The plannerDesiredInput is supposed to have dimension 2, while it has dimension" << plannerDesiredInput.size()
                                        << ". Using only the first two inputs. This warning will be showed only once.";
                }

                m_personFollowingDesiredPoint(0) = plannerDesiredInput(0);
                m_personFollowingDesiredPoint(1) = plannerDesiredInput(1);
            }
        }
        else if (m_unicycleController == UnicycleController::DIRECT)
        {
            if (plannerDesiredInput.size() != 3)
            {
                yErrorThrottle(1.0) << "[updateTrajectories] The plannerDesiredInput is supposed to have dimension 3, while it has dimension" << plannerDesiredInput.size()
                                    << ". Using zero input.";
            }
            else
            {
                m_desiredDirectControl(0) = plannerDesiredInput(0);
                m_desiredDirectControl(1) = plannerDesiredInput(1);
                m_desiredDirectControl(2) = plannerDesiredInput(2);
            }
        }

        m_initTime = initTime;

        // Boundary condition
        m_DCMBoundaryConditionAtMergePointPosition = DCMBoundaryConditionAtMergePointPosition;
        m_DCMBoundaryConditionAtMergePointVelocity = DCMBoundaryConditionAtMergePointVelocity;

        m_correctLeft = correctLeft;

        if(correctLeft)
            m_measuredTransformLeft = measured;
        else
            m_measuredTransformRight = measured;

        m_generatorState = GeneratorState::Called;
    }

    m_conditionVariable.notify_one();

    return true;
}

bool TrajectoryGenerator::setFreeSpaceEllipse(const iDynTree::MatrixFixSize<2, 2> &imageMatrix, const iDynTree::VectorFixSize<2> &centerOffset)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_newFreeSpaceEllipse = m_freeSpaceEllipse.setEllipse(imageMatrix, centerOffset);
    return m_newFreeSpaceEllipse;
}

bool TrajectoryGenerator::isTrajectoryComputed()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_generatorState == GeneratorState::Returned;
}

bool TrajectoryGenerator::isTrajectoryAsked()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    return m_generatorState == GeneratorState::Called;
}

bool TrajectoryGenerator::getDCMPositionTrajectory(std::vector<iDynTree::Vector2>& DCMPositionTrajectory)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getDCMPositionTrajectory] No trajectories are available";
        return false;
    }

    DCMPositionTrajectory = m_dcmGenerator->getDCMPosition();
    return true;
}

bool TrajectoryGenerator::getDCMVelocityTrajectory(std::vector<iDynTree::Vector2>& DCMVelocityTrajectory)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getDCMVelocityTrajectory] No trajectories are available";
        return false;
    }

    DCMVelocityTrajectory = m_dcmGenerator->getDCMVelocity();
    return true;
}

bool TrajectoryGenerator::getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory,
                                              std::vector<iDynTree::Transform>& rFootTrajectory)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getFeetTrajectories] No trajectories are available";
        return false;
    }

    m_feetGenerator->getFeetTrajectories(lFootTrajectory, rFootTrajectory);

    return true;
}

bool TrajectoryGenerator::getFeetTwist(std::vector<iDynTree::Twist>& lFootTwist,
                                       std::vector<iDynTree::Twist>& rFootTwist)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getFeetTwist] No trajectories are available";
        return false;
    }

    m_feetGenerator->getFeetTwistsInMixedRepresentation(lFootTwist, rFootTwist);

    return true;
}

bool TrajectoryGenerator::getWhenUseLeftAsFixed(std::vector<bool>& isLeftFixedFrame)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getWhenUseLeftAsFixed] No trajectories are available";
        return false;
    }

    m_trajectoryGenerator.getWhenUseLeftAsFixed(isLeftFixedFrame);
    return true;
}


bool TrajectoryGenerator::getFeetStandingPeriods(std::vector<bool>& lFootContacts,
                                                 std::vector<bool>& rFootContacts)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getFeetStandingPeriods] No trajectories are available";
        return false;
    }

    m_trajectoryGenerator.getFeetStandingPeriods(lFootContacts, rFootContacts);
    return true;
}

bool TrajectoryGenerator::getCoMHeightTrajectory(std::vector<double>& CoMHeightTrajectory)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getCoMHeightTrajectory] No trajectories are available";
        return false;
    }

    m_heightGenerator->getCoMHeightTrajectory(CoMHeightTrajectory);
    return true;
}

bool TrajectoryGenerator::getCoMHeightVelocity(std::vector<double>& CoMHeightVelocity)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getCoMHeightVelocity] No trajectories are available";
        return false;
    }

    m_heightGenerator->getCoMHeightVelocity(CoMHeightVelocity);
    return true;
}

bool TrajectoryGenerator::getMergePoints(std::vector<size_t>& mergePoints)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getMergePoints] No trajectories are available";
        return false;
    }

    m_trajectoryGenerator.getMergePoints(mergePoints);
    return true;
}

bool TrajectoryGenerator::getWeightPercentage(std::vector<double> &weightInLeft,
                                              std::vector<double> &weightInRight)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getWeightPercentage] No trajectories are available";
        return false;
    }

    m_dcmGenerator->getWeightPercentage(weightInLeft, weightInRight);
    return true;
}

void TrajectoryGenerator::reset()
{
    // the mutex is automatically released when lock_guard goes out of its scope
    std::lock_guard<std::mutex> guard(m_mutex);

    // change the state of the generator
    m_generatorState = GeneratorState::FirstStep;
}

bool TrajectoryGenerator::getIsStancePhase(std::vector<bool>& isStancePhase)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getDCMVelocityTrajectory] No trajectories are available";
        return false;
    }

    const auto & DCMVelocityTrajectory = m_dcmGenerator->getDCMVelocity();
    isStancePhase.resize(DCMVelocityTrajectory.size());

    double threshold = 0.001;

    // here there is the assumption that each trajectory begins with a stance phase
    std::size_t stancePhaseDelayCounter = 0;
    for(std::size_t i = 0; i < DCMVelocityTrajectory.size(); i++)
    {
        // in this case the robot is moving
        if(iDynTree::toEigen(DCMVelocityTrajectory[i]).norm() > threshold)
        {
            isStancePhase[i] = false;
            // reset the counter for the beginning of the next stance phase.
            // If m_stancePhaseDelay is equal to zero, the stance phase will not be delayed
            stancePhaseDelayCounter = m_stancePhaseDelay;
        }
        else
        {
            // decreased the counter only if it is different from zero.
            // it is required to add a delay in the beginning of the stance phase
            stancePhaseDelayCounter = (stancePhaseDelayCounter == 0)
                                          ? 0
                                          : (stancePhaseDelayCounter - 1);

            // the delay expired the robot can be considered stance
            if(stancePhaseDelayCounter == 0)
                isStancePhase[i] = true;
            else
                isStancePhase[i] = false;
        }
    }

    return true;
}

bool TrajectoryGenerator::getDesiredZMPPosition(std::vector<iDynTree::Vector2> &desiredZMP)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getDesiredZMP] No trajectories are available";
        return false;
    }

    desiredZMP = m_dcmGenerator->getZMPPosition();
    return true;
}

const iDynTree::Rotation& TrajectoryGenerator::getChestAdditionalRotation() const
{
    return m_chestAdditionalRotation;
}

const iDynTree::Transform& WalkingControllers::TrajectoryGenerator::getUnicyclePose() const
{
    return m_unicyclePose;
}
