/**
 * @file TrajectoryGenerator.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <TrajectoryGenerator.hpp>
#include <Utils.hpp>

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

    yarp::os::Value tempValue;

    m_dT = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();
    m_plannerHorizon = config.check("plannerHorizon", yarp::os::Value(20.0)).asDouble();
    double unicycleGain = config.check("unicycleGain", yarp::os::Value(10.0)).asDouble();

    tempValue = config.find("referencePosition");
    if(!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_referencePointDistance))
    {
        yError() << "[configurePlanner] Initialization failed while reading referencePosition vector.";
        return false;
    }

    // get left and right ZMP delta
    iDynTree::Vector2 leftZMPDelta;
    tempValue = config.find("leftZMPDelta");
    if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, leftZMPDelta))
    {
        yError() << "[configurePlanner] Initialization failed while reading rStancePosition vector.";
        return false;
    }

    iDynTree::Vector2 rightZMPDelta;
    tempValue = config.find("rightZMPDelta");
    if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, rightZMPDelta))
    {
        yError() << "[configurePlanner] Initialization failed while reading rStancePosition vector.";
        return false;
    }

    double timeWeight = config.check("timeWeight", yarp::os::Value(2.5)).asDouble();
    double positionWeight = config.check("positionWeight", yarp::os::Value(1.0)).asDouble();
    double slowWhenTurningGain = config.check("slowWhenTurningGain", yarp::os::Value(0.0)).asDouble();
    double maxStepLength = config.check("maxStepLength", yarp::os::Value(0.05)).asDouble();
    double minStepLength = config.check("minStepLength", yarp::os::Value(0.005)).asDouble();
    double minWidth = config.check("minWidth", yarp::os::Value(0.03)).asDouble();
    double maxAngleVariation = iDynTree::deg2rad(config.check("maxAngleVariation",
                                                              yarp::os::Value(40.0)).asDouble());
    double minAngleVariation = iDynTree::deg2rad(config.check("minAngleVariation",
                                                              yarp::os::Value(5.0)).asDouble());
    double maxStepDuration = config.check("maxStepDuration", yarp::os::Value(8.0)).asDouble();
    double minStepDuration = config.check("minStepDuration", yarp::os::Value(2.9)).asDouble();
    double stepHeight = config.check("stepHeight", yarp::os::Value(0.005)).asDouble();
    double landingVelocity = config.check("stepLandingVelocity", yarp::os::Value(0.0)).asDouble();
    double apexTime = config.check("footApexTime", yarp::os::Value(0.5)).asDouble();
    double comHeight = config.check("com_height", yarp::os::Value(0.49)).asDouble();
    double comHeightDelta = config.check("comHeightDelta", yarp::os::Value(0.01)).asDouble();
    double nominalDuration = config.check("nominalDuration", yarp::os::Value(4.0)).asDouble();
    double lastStepSwitchTime = config.check("lastStepSwitchTime", yarp::os::Value(0.5)).asDouble();
    double switchOverSwingRatio = config.check("switchOverSwingRatio",
                                               yarp::os::Value(0.4)).asDouble();
    double mergePointRatio = config.check("mergePointRatio", yarp::os::Value(0.5)).asDouble();

    m_nominalWidth = config.check("nominalWidth", yarp::os::Value(0.04)).asDouble();

    m_swingLeft = config.check("swingLeft", yarp::os::Value(true)).asBool();
    bool startWithSameFoot = config.check("startAlwaysSameFoot", yarp::os::Value(false)).asBool();
    bool useMinimumJerkFootTrajectory = config.check("useMinimumJerkFootTrajectory",
                                                     yarp::os::Value(false)).asBool();
    double pitchDelta = config.check("pitchDelta", yarp::os::Value(0.0)).asDouble();

    // try to configure the planner
    bool ok = true;
    ok = ok && m_trajectoryGenerator.setDesiredPersonDistance(m_referencePointDistance(0),
                                                              m_referencePointDistance(1));
    ok = ok && m_trajectoryGenerator.setControllerGain(unicycleGain);
    ok = ok && m_trajectoryGenerator.setMaximumIntegratorStepSize(m_dT);
    ok = ok && m_trajectoryGenerator.setMaxStepLength(maxStepLength);
    ok = ok && m_trajectoryGenerator.setWidthSetting(minWidth, m_nominalWidth);
    ok = ok && m_trajectoryGenerator.setMaxAngleVariation(maxAngleVariation);
    ok = ok && m_trajectoryGenerator.setCostWeights(positionWeight, timeWeight);
    ok = ok && m_trajectoryGenerator.setStepTimings(minStepDuration,
                                                    maxStepDuration, nominalDuration);
    ok = ok && m_trajectoryGenerator.setPlannerPeriod(m_dT);
    ok = ok && m_trajectoryGenerator.setMinimumAngleForNewSteps(minAngleVariation);
    ok = ok && m_trajectoryGenerator.setMinimumStepLength(minStepLength);
    ok = ok && m_trajectoryGenerator.setSwitchOverSwingRatio(switchOverSwingRatio);
    ok = ok && m_trajectoryGenerator.setTerminalHalfSwitchTime(lastStepSwitchTime);
    ok = ok && m_trajectoryGenerator.setStepHeight(stepHeight);
    ok = ok && m_trajectoryGenerator.setFootLandingVelocity(landingVelocity);
    ok = ok && m_trajectoryGenerator.setFootApexTime(apexTime);
    ok = ok && m_trajectoryGenerator.setPauseConditions(maxStepDuration, nominalDuration);
    ok = ok && m_trajectoryGenerator.setCoMHeightSettings(comHeight, comHeightDelta);
    ok = ok && m_trajectoryGenerator.setSlowWhenTurnGain(slowWhenTurningGain);
    ok = ok && m_trajectoryGenerator.setMergePointRatio(mergePointRatio);
    ok = ok && m_trajectoryGenerator.setPitchDelta(pitchDelta);

    m_trajectoryGenerator.setStanceZMPDelta(leftZMPDelta, rightZMPDelta);
    m_trajectoryGenerator.addTerminalStep(false);
    m_trajectoryGenerator.startWithLeft(m_swingLeft);
    m_trajectoryGenerator.resetTimingsIfStill(startWithSameFoot);



    m_trajectoryGenerator.useMinimumJerkFootTrajectory(useMinimumJerkFootTrajectory);

    m_correctLeft = true;

    if(ok)
    {
        // the mutex is automatically released when lock_guard goes out of its scope
        std::lock_guard<std::mutex> guard(m_mutex);

        // change the state of the generator
        m_generatorState = GeneratorState::FirstStep;

        // start the thread
        m_generatorThread = std::thread(&TrajectoryGenerator::computeThread, this);
    }

    return ok;
}

void TrajectoryGenerator::addTerminalStep(bool terminalStep)
{
    m_trajectoryGenerator.addTerminalStep(terminalStep);
}

void TrajectoryGenerator::computeThread()
{
    while (true)
    {
        double initTime;
        double endTime;
        double dT;

        bool correctLeft;

        iDynTree::Vector2 desiredPoint;
        iDynTree::Vector2 measuredPositionLeft, measuredPositionRight;
        double measuredAngleLeft, measuredAngleRight;

        iDynTree::Vector2 DCMBoundaryConditionAtMergePointPosition;
        iDynTree::Vector2 DCMBoundaryConditionAtMergePointVelocity;

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
            desiredPoint = m_desiredPoint;

            // dcm boundary conditions
            DCMBoundaryConditionAtMergePointPosition = m_DCMBoundaryConditionAtMergePointPosition;
            DCMBoundaryConditionAtMergePointVelocity = m_DCMBoundaryConditionAtMergePointVelocity;

            // left foot
            measuredPositionLeft(0) = m_measuredTransformLeft.getPosition()(0);
            measuredPositionLeft(1) = m_measuredTransformLeft.getPosition()(1);
            measuredAngleLeft = m_measuredTransformLeft.getRotation().asRPY()(2);

            // right foot
            measuredPositionRight(0) = m_measuredTransformRight.getPosition()(0);
            measuredPositionRight(1) = m_measuredTransformRight.getPosition()(1);
            measuredAngleRight = m_measuredTransformRight.getRotation().asRPY()(2);

            correctLeft = m_correctLeft;
        }

        // clear the old trajectory
        m_trajectoryGenerator.clearDesiredTrajectory();

        // add new point
        if(!m_trajectoryGenerator.addDesiredTrajectoryPoint(endTime, desiredPoint))
        {
            // something goes wrong
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Configured;
            yError() << "[TrajectoryGenerator_Thread] Error while setting the new reference.";
            break;
        }

        iDynTree::Vector2 measuredPosition;
        double measuredAngle;
        measuredPosition = correctLeft ? measuredPositionLeft : measuredPositionRight;
        measuredAngle = correctLeft ? measuredAngleLeft : measuredAngleRight;

        if(m_trajectoryGenerator.reGenerateDCM(initTime, dT, endTime,
                                               DCMBoundaryConditionAtMergePointPosition,
                                               DCMBoundaryConditionAtMergePointVelocity,
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

bool TrajectoryGenerator::generateFirstTrajectories()
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
    m_trajectoryGenerator.clearDesiredTrajectory();

    // set initial and final times
    double initTime = 0;
    double endTime = initTime + m_plannerHorizon;

    // at the beginning iCub has to stop
    m_desiredPoint(0) = m_referencePointDistance(0);
    m_desiredPoint(1) = m_referencePointDistance(1);

    // add the initial point
    if(!m_trajectoryGenerator.addDesiredTrajectoryPoint(initTime, m_referencePointDistance))
    {
        yError() << "[generateFirstTrajectories] Error while setting the first reference.";
        return false;
    }

    // add the final point
    if(!m_trajectoryGenerator.addDesiredTrajectoryPoint(endTime, m_desiredPoint))
    {
        yError() << "[generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    // generate the first trajectories
    if(!m_trajectoryGenerator.generateAndInterpolateDCM(initTime, m_dT, endTime))
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
    m_trajectoryGenerator.clearDesiredTrajectory();

    // set initial and final times
    double initTime = 0;
    double endTime = initTime + m_plannerHorizon;

    // at the beginning iCub has to stop
    m_desiredPoint(0) = m_referencePointDistance(0);
    m_desiredPoint(1) = m_referencePointDistance(1);

    // add the initial point
    if(!m_trajectoryGenerator.addDesiredTrajectoryPoint(initTime, m_referencePointDistance))
    {
        yError() << "[generateFirstTrajectories] Error while setting the first reference.";
        return false;
    }

    // add the final point
    if(!m_trajectoryGenerator.addDesiredTrajectoryPoint(endTime, m_desiredPoint))
    {
        yError() << "[generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    // add real position of the feet
    std::shared_ptr<FootPrint> left, right;

    left = std::make_shared<FootPrint>();
    right = std::make_shared<FootPrint>();

    left->setFootName("left");
    right->setFootName("right");

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

    // iDynTree::Vector2 initialCOMPositionXY;
    // initialCOMPositionXY(0) = initialCOMPosition(0);
    // initialCOMPositionXY(1) = initialCOMPosition(1);

    // generate the first trajectories
    if(!m_trajectoryGenerator.generateAndInterpolateDCM(left, right, initTime, m_dT, endTime))
    {
        yError() << "[generateFirstTrajectories] Error while computing the first trajectories.";
        return false;
    }

    m_generatorState = GeneratorState::Returned;
    return true;
}

bool TrajectoryGenerator::updateTrajectories(double initTime, const iDynTree::Vector2& DCMBoundaryConditionAtMergePointPosition,
                                             const iDynTree::Vector2& DCMBoundaryConditionAtMergePointVelocity, bool correctLeft,
                                             const iDynTree::Transform& measured, const iDynTree::Vector2& desiredPosition)
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);

        if(m_generatorState == GeneratorState::Called)
        {
            yError() << "[updateTrajectories_one correction] Cannot launch the generator twice. "
                     << "Please wait until the trajectory has be evaluated .";
            return false;
        }

        if(m_generatorState != GeneratorState::Returned)
        {
            yError() << "[updateTrajectories_one correction] The trajectory generator has not computed any trajectory yet. "
                     << "Please call 'generateFirstTrajectories()' method.";
            return false;
        }
    }
    // if correctLeft is true the stance foot is the true.
    // The vector (expressed in the unicycle reference frame from the left foot to the center of the
    // unicycle is [0, width/2]')
    iDynTree::Vector2 unicyclePositionFromStanceFoot;
    unicyclePositionFromStanceFoot(0) = 0.0;
    unicyclePositionFromStanceFoot(1) = correctLeft ? -m_nominalWidth/2 : m_nominalWidth/2;

    iDynTree::Vector2 desredPositionFromStanceFoot;
    iDynTree::toEigen(desredPositionFromStanceFoot) = iDynTree::toEigen(unicyclePositionFromStanceFoot)
        + iDynTree::toEigen(m_referencePointDistance) + iDynTree::toEigen(desiredPosition);

    // prepare the rotation matrix w_R_{unicycle}
    double theta = measured.getRotation().asRPY()(2);
    double s_theta = std::sin(theta);
    double c_theta = std::cos(theta);

    // save the data
    {
        std::lock_guard<std::mutex> guard(m_mutex);

        // apply the homogeneous transformation w_H_{unicycle}
        m_desiredPoint(0) = c_theta * desredPositionFromStanceFoot(0)
            - s_theta * desredPositionFromStanceFoot(1) + measured.getPosition()(0);
        m_desiredPoint(1) = s_theta * desredPositionFromStanceFoot(0)
            + c_theta * desredPositionFromStanceFoot(1) + measured.getPosition()(1);

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

    DCMPositionTrajectory = m_trajectoryGenerator.getDCMPosition();
    return true;
}

bool TrajectoryGenerator::getDCMVelocityTrajectory(std::vector<iDynTree::Vector2>& DCMVelocityTrajectory)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getDCMVelocityTrajectory] No trajectories are available";
        return false;
    }

    DCMVelocityTrajectory = m_trajectoryGenerator.getDCMVelocity();
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

    m_trajectoryGenerator.getFeetTrajectories(lFootTrajectory, rFootTrajectory);
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

    m_trajectoryGenerator.getFeetTwist(lFootTwist, rFootTwist);
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

    m_trajectoryGenerator.getCoMHeightTrajectory(CoMHeightTrajectory);
    return true;
}

bool TrajectoryGenerator::getCoMHeightVelocity(std::vector<double>& CoMHeightVelocity)
{
    if(!isTrajectoryComputed())
    {
        yError() << "[getCoMHeightVelocity] No trajectories are available";
        return false;
    }

    m_trajectoryGenerator.getCoMHeightVelocity(CoMHeightVelocity);
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
