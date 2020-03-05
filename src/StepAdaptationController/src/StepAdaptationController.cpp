
// std
#define NOMINMAX
#include <algorithm>

// yarp
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Direction.h>

#include <WalkingControllers/StepAdaptationController/StepAdaptationController.hpp>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

StepAdaptationController::StepAdaptationController()
    : m_xPositionsBuffer(2)
    , m_yPositionsBuffer(2)
    , m_zPositionsBuffer(3)
    , m_zzPositionsBuffer(2)
    , m_yawsBuffer(2)
    , m_timesBuffer(2)
    , m_zTimesBuffer(3)
    , m_zzTimesBuffer(2)
{}

bool StepAdaptationController::initialize(const yarp::os::Searchable &config)
{
    // inputs are zmp position, sigma dcm offset
    m_inputSize = 5;

    // constraints are dynamics (2) zmp position (2) impact time(1)
    m_numberOfConstraint = 7;

    m_currentQPSolver = std::make_shared<StepAdaptationQPSolver>(m_inputSize, m_numberOfConstraint);

    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_position_weight",  m_zmpPositionWeight))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "next_dcm_offset_weight",  m_dcmOffsetWeight))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "sigma_weight", m_sigmaWeight))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the number";
        return false;
    }

    m_feetExtendedPolygon.resize(2);
    iDynTree::Polygon foot;
    iDynTree::Vector4 zmpOffsetLeftFoot;
    if(!YarpUtilities::getVectorFromSearchable(config, "zmp_offset_leftFoot",  zmpOffsetLeftFoot))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_dcm_error",  m_dcm_ErrorThreshold))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector of DCM error threshold";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_roll_pitch_error",  m_roll_pitch_ErrorThreshold))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector of roll pitch imu error threshold";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(zmpOffsetLeftFoot(0), zmpOffsetLeftFoot(1),
                                                     zmpOffsetLeftFoot(2), zmpOffsetLeftFoot(3));
    m_feetExtendedPolygon[0] = foot;

    iDynTree::Vector4 zmpOffsetRightFoot;
    if(!YarpUtilities::getVectorFromSearchable(config, "zmp_offset_rightFoot", zmpOffsetRightFoot))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(zmpOffsetRightFoot(0), zmpOffsetRightFoot(1),
                                                     zmpOffsetRightFoot(2), zmpOffsetRightFoot(3));

    m_feetExtendedPolygon[1] = foot;

    if(!YarpUtilities::getNumberFromSearchable(config, "step_duration_tolerance", m_stepDurationTolerance))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the number";
        return false;
    }

    if(!m_currentQPSolver->setHessianMatrix(m_zmpPositionWeight, m_dcmOffsetWeight, m_sigmaWeight))
    {
        yError() << "[StepAdaptationController::initialize] Unable set the hessian";
        return false;
    }

    // reset the solver
    reset();

    return true;

}

void StepAdaptationController::setNominalNextStepPosition(const iDynTree::Vector2& nominalZmpPosition, const double& angle)
{
    m_zmpPositionNominal = nominalZmpPosition;
    m_footTransform.setPosition(iDynTree::Position(nominalZmpPosition(0), nominalZmpPosition(1), 0));
    m_footTransform.setRotation(iDynTree::Rotation::RPY(0, 0, angle));
}

void StepAdaptationController::setTimings(const double & omega, const double & currentTime, const double& nextImpactTime,
                               const double &nextDoubleSupportDuration)
{
    m_nextDoubleSupportDuration = nextDoubleSupportDuration;
    m_currentTime = currentTime;

    m_stepTiming = nextImpactTime + nextDoubleSupportDuration / 2 - currentTime;
    m_remainingSingleSupportDuration = nextImpactTime - currentTime;

    m_sigmaNominal = std::exp(omega * m_stepTiming);
    m_omega = omega;
}

void StepAdaptationController::setNominalDcmOffset(const iDynTree::Vector2& nominalDcmOffset)
{
    m_dcmOffsetNominal = nominalDcmOffset;
}

void StepAdaptationController::setCurrentZmpPosition(const iDynTree::Vector2& currentZmpPosition)
{
    m_currentZmpPosition = currentZmpPosition;
}

void StepAdaptationController::setCurrentDcmPosition(const iDynTree::Vector2& currentDcmPosition)
{
    m_currentDcmPosition = currentDcmPosition;
}

bool StepAdaptationController::solve(bool isLeft)
{

    // generate the convex hull
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;
    yAxis.zero();
    yAxis(1) = 1;

    // initilize plane origin
    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    std::vector<iDynTree::Transform> feetTransforms;
    feetTransforms.push_back(m_footTransform);

    if(isLeft)
    {

        if(!m_convexHullComputer.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                 std::vector<iDynTree::Polygon>(1, m_feetExtendedPolygon[0]),
                                                 feetTransforms))
        {
            yInfo() << "unable to build the convex hull (left)";
            return false;
        }
    }
    else
    {

        if(!m_convexHullComputer.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                 std::vector<iDynTree::Polygon>(1, m_feetExtendedPolygon[1]),
                                                 feetTransforms))
        {
            yInfo() << "unable to build the convex hull (right)";
            return false;
        }
    }

    if (!m_currentQPSolver->setGradientVector(m_zmpPositionWeight, m_dcmOffsetWeight, m_sigmaWeight,
                                              m_zmpPositionNominal, m_dcmOffsetNominal, m_sigmaNominal))
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the Gradient Vector";
        return false;
    }

    if(!m_currentQPSolver->setConstraintsMatrix(m_currentDcmPosition, m_currentZmpPosition, m_convexHullComputer.A))
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the constraint matrix";
        return false;
    }

    if(!m_currentQPSolver->setBoundsVectorOfConstraints(m_currentZmpPosition, m_convexHullComputer.b,
                                                        m_stepTiming, m_stepDurationTolerance, m_remainingSingleSupportDuration,
                                                        m_omega))
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the bounds";
        return false;
    }

    m_isSolutionEvaluated = false;

    if(!m_currentQPSolver->solve())
    {
        yError() << "[StepAdaptationController::solve] Unable to solve the step adaptation problem.";
        return false;
    }

    m_isSolutionEvaluated = true;
    return true;
}

double StepAdaptationController::getDesiredImpactTime()
{
    double optimalStepDuration = std::log(m_currentQPSolver->getSolution()(2)) / m_omega;
    return m_currentTime + optimalStepDuration - m_nextDoubleSupportDuration / 2;
}

iDynTree::Vector2 StepAdaptationController::getDesiredZmp()
{
    iDynTree::Vector2 desiredZmp;
    desiredZmp(0) = m_currentQPSolver->getSolution()(0);
    desiredZmp(1) = m_currentQPSolver->getSolution()(1);
    return desiredZmp;
}

bool StepAdaptationController::getAdaptatedFootTrajectory(double maxFootHeight, double dt, double takeOffTime, double yawAngleAtImpact, iDynTree::Vector2 zmpOffset,
                                               const iDynTree::Transform& currentFootTransform, const iDynTree::Twist& currentFootTwist,
                                               iDynTree::Transform& adaptatedFootTransform, iDynTree::Twist& adaptedFootTwist, iDynTree::SpatialAcc& adaptedFootAcceleration)
{
    iDynTree::CubicSpline xSpline, ySpline, zSpline, yawSpline;

    if(m_currentTime >= getDesiredImpactTime())
    {
        adaptedFootTwist.zero();
        iDynTree::Position newPosition;
        newPosition(0)=getDesiredZmp()(0) - (cos(yawAngleAtImpact) * zmpOffset(0) - sin(yawAngleAtImpact) * zmpOffset(1));
        newPosition(1)=getDesiredZmp()(1) - (cos(yawAngleAtImpact) * zmpOffset(1) + sin(yawAngleAtImpact) * zmpOffset(0));
        newPosition(2)= 0;
        adaptatedFootTransform.setPosition(newPosition);

        adaptatedFootTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, yawAngleAtImpact));

        return true;
    }

    double maxFootHeightTime = (getDesiredImpactTime() - takeOffTime) * 0.8 + takeOffTime;
    if (m_currentTime < maxFootHeightTime)
    {
        m_zTimesBuffer(0)= m_currentTime;
        m_zTimesBuffer(1)= maxFootHeightTime;
        m_zTimesBuffer(2)= getDesiredImpactTime();
        m_zPositionsBuffer(0)= currentFootTransform.getPosition()(2);
        m_zPositionsBuffer(1)= maxFootHeight;
        m_zPositionsBuffer(2)= 0;

        zSpline.setInitialConditions(currentFootTwist.getLinearVec3()(2), 0.0);
        zSpline.setFinalConditions(0.0,0.0);

        if (!zSpline.setData(m_zTimesBuffer, m_zPositionsBuffer))
        {
            std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the z-dimension spline." << std::endl;
            return false;
        }
    }
    else
    {
        m_zzTimesBuffer(0)= m_currentTime;
        m_zzTimesBuffer(1)= getDesiredImpactTime();

        iDynTree::Position PositionsBuffer=currentFootTransform.getPosition();
        m_zzPositionsBuffer(0)=PositionsBuffer(2);
        m_zzPositionsBuffer(1)= 0;
        zSpline.setInitialConditions(currentFootTwist.getLinearVec3()(2), 0.0);
        zSpline.setFinalConditions(0.0,0.0);

        if (!zSpline.setData(m_zzTimesBuffer, m_zzPositionsBuffer))
        {
            std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the z-dimension spline." << std::endl;
            return false;
        }
    }

    m_xPositionsBuffer(0)= currentFootTransform.getPosition()(0);
    m_yPositionsBuffer(0)= currentFootTransform.getPosition()(1);

    m_yawsBuffer(0) = currentFootTransform.getRotation().asRPY()(2);

    m_xPositionsBuffer(1)= getDesiredZmp()(0) - (cos(yawAngleAtImpact) * zmpOffset(0) - sin(yawAngleAtImpact) * zmpOffset(1));
    m_yPositionsBuffer(1)= getDesiredZmp()(1) - (cos(yawAngleAtImpact) * zmpOffset(1) + sin(yawAngleAtImpact) * zmpOffset(0));

    m_yawsBuffer(1) = yawAngleAtImpact;

    m_timesBuffer(0) = m_currentTime;
    m_timesBuffer(1) = getDesiredImpactTime();

    double yawAngle;

    iDynTree::AngularMotionVector3 rightTrivializedAngVelocity;
    iDynTree::Vector3 rpyDerivativeCurrent;
    iDynTree::Vector3 rpyDerivative;
    iDynTree::toEigen(rpyDerivativeCurrent) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(0.0, 0.0, m_yawsBuffer(0))) * iDynTree::toEigen(currentFootTwist.getAngularVec3());

    xSpline.setInitialConditions(currentFootTwist.getLinearVec3()(0), 0.0);
    ySpline.setInitialConditions(currentFootTwist.getLinearVec3()(1), 0.0);
    yawSpline.setInitialConditions(rpyDerivativeCurrent(2), 0.0);

    xSpline.setFinalConditions(0.0,0.0);
    ySpline.setFinalConditions(0.0,0.0);
    yawSpline.setFinalConditions(0.0, 0.0);

    if (!xSpline.setData(m_timesBuffer, m_xPositionsBuffer)){
        std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the x-dimension spline." << std::endl;
        return false;
    }
    if (!ySpline.setData(m_timesBuffer, m_yPositionsBuffer)){
        std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the y-dimension spline." << std::endl;
        return false;
    }

    if (!yawSpline.setData(m_timesBuffer, m_yawsBuffer)){
        std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the yaw-dimension spline." << std::endl;
        return false;
    }

    iDynTree::Transform newTransform;
    iDynTree::Position newPosition;
    iDynTree::Vector3 linearVelocity;
    iDynTree::Vector3 linearAcceleration;
    iDynTree::Vector3 rpySecondDerivative;

    newPosition(0) = xSpline.evaluatePoint(m_currentTime + dt, linearVelocity(0), linearAcceleration(0));
    newPosition(1) = ySpline.evaluatePoint(m_currentTime + dt, linearVelocity(1), linearAcceleration(1));
    newPosition(2) = zSpline.evaluatePoint(m_currentTime + dt, linearVelocity(2), linearAcceleration(2));

    yawAngle = yawSpline.evaluatePoint(m_currentTime + dt, rpyDerivative(2), rpySecondDerivative(2));

    adaptatedFootTransform.setPosition(newPosition);
    adaptatedFootTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, yawAngle));

    rpyDerivative(0)=0;
    rpyDerivative(1)=0;

    iDynTree::toEigen(rightTrivializedAngVelocity) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivative(0.0, 0.0, yawAngle)) *iDynTree::toEigen(rpyDerivative);
    adaptedFootTwist.setLinearVec3(linearVelocity);
    adaptedFootTwist.setAngularVec3(rightTrivializedAngVelocity);
    adaptedFootAcceleration.setLinearVec3(linearAcceleration);
    iDynTree::Vector3 vector;
    vector.zero();
    adaptedFootAcceleration.setAngularVec3(vector);

    return true;
}

iDynTree::Vector2 StepAdaptationController::getDCMErrorThreshold(){
    return m_dcm_ErrorThreshold;
}
iDynTree::Vector2 StepAdaptationController::getRollPitchErrorThreshold(){
    return m_roll_pitch_ErrorThreshold;
}

void StepAdaptationController::reset(){
    m_isSolutionEvaluated = false;
}
