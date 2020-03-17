
// std
#define NOMINMAX
#include <algorithm>

// yarp
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/Twist.h>

#include <WalkingControllers/StepAdaptationController/StepAdaptationController.hpp>
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>

// eigen
#include <Eigen/Sparse>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
using namespace WalkingControllers;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
using namespace WalkingControllers;

StepAdaptationController::StepAdaptationController()
    : m_inputSize(5)
    , m_numberOfConstraints(7)
    , m_xPositionsBuffer(2)
    , m_yPositionsBuffer(2)
    , m_zFirstPiecePositionsBuffer(3)
    , m_zSecondPiecePositionsBuffer(2)
    , m_yawsBuffer(2)
    , m_timesBuffer(2)
    , m_zFirstPieceTimesBuffer(3)
    , m_zSecondPieceTimesBuffer(2)
{

    m_constraintsMatrix.resize(m_numberOfConstraints, m_inputSize);
    m_upperBound.resize(m_numberOfConstraints);
    m_lowerBound.resize(m_numberOfConstraints);
    m_gradient.resize(m_inputSize);

    // set the constant elements of the constraint matrix
    m_constraintsMatrix(0, 0) = 1;
    m_constraintsMatrix(1, 1) = 1;
    m_constraintsMatrix(0, 3) = 1;
    m_constraintsMatrix(1, 4) = 1;
    m_constraintsMatrix(6, 2) = 1;

    m_hessianMatrix.resize(m_inputSize, m_numberOfConstraints);
    m_solution.resize(m_inputSize);

    // qpoases
    m_QPSolver_qpOASES = std::make_unique<qpOASES::SQProblem>(m_inputSize,
                                                              m_numberOfConstraints);

    m_QPSolver_qpOASES->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
}

bool StepAdaptationController::initialize(const yarp::os::Searchable &config)
{

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
    iDynTree::Vector4 nextZmpConstraintBoundLeftFoot  ;
    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_constraint_bound_left_foot  ",  nextZmpConstraintBoundLeftFoot))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_dcm_error",  m_dcmErrorThreshold))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector of DCM error threshold";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_roll_pitch_error",  m_rollPitchErrorThreshold))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector of roll pitch imu error threshold";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(nextZmpConstraintBoundLeftFoot(0), nextZmpConstraintBoundLeftFoot(1),
                                                     nextZmpConstraintBoundLeftFoot(2), nextZmpConstraintBoundLeftFoot(3));
    m_feetExtendedPolygon[0] = foot;

    iDynTree::Vector4 nextZmpConstraintBoundRightFoot;
    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_constraint_bound_right_foot  ", nextZmpConstraintBoundRightFoot))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the vector";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(nextZmpConstraintBoundRightFoot(0), nextZmpConstraintBoundRightFoot(1),
                                                     nextZmpConstraintBoundRightFoot(2), nextZmpConstraintBoundRightFoot(3));

    m_feetExtendedPolygon[1] = foot;

    if(!YarpUtilities::getNumberFromSearchable(config, "step_duration_tolerance", m_stepDurationTolerance))
    {
        yError() << "[StepAdaptationController::initialize] Unable to get the number";
        return false;
    }

    if(!computeHessianMatrix())
    {
        yError() << "[StepAdaptationController::initialize] Unable set the hessian";
        return false;
    }

    // reset the solver
    reset();

    return true;
}

bool StepAdaptationController::computeHessianMatrix()
{
    m_hessianMatrix(0,0) = m_zmpPositionWeight(0);
    m_hessianMatrix(1,1) = m_zmpPositionWeight(1);

    m_hessianMatrix(2,2) = m_sigmaWeight;

    m_hessianMatrix(3,3) = m_dcmOffsetWeight(0);
    m_hessianMatrix(4,4) = m_dcmOffsetWeight(1);

    return true;
}

bool StepAdaptationController::computeGradientVector()
{
    iDynTree::toEigen(m_gradient).segment(0, 2) = -(iDynTree::toEigen(m_zmpPositionWeight).asDiagonal() * iDynTree::toEigen(m_zmpPositionNominal));
    m_gradient(2) = -m_sigmaWeight * m_sigmaNominal;
    iDynTree::toEigen(m_gradient).segment(3, 2)  = -(iDynTree::toEigen(m_dcmOffsetWeight).asDiagonal() * iDynTree::toEigen(m_dcmOffsetNominal));

    if(m_QPSolver->isInitialized())
    {
        if(!m_QPSolver->updateGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to update the Gradient Vector";
            return false;
        }
    }
    else
    {
        if(!m_QPSolver->data()->setGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to set the Gradient Vector for the first time";
            return false;
        }
    }
    return true;
}

bool StepAdaptationController::computeConstraintsMatrix()
{
    if(m_convexHullComputer.A.rows() != 4 || m_convexHullComputer.A.cols() != 2)
    {
        yError() << "QPSolver::setConstraintsMatrix the convex hull matrix size is strange " << m_convexHullComputer.A.toString();
        return false;
    }

    iDynTree::Vector2 temp;
    iDynTree::toEigen(temp) = iDynTree::toEigen(m_currentZmpPosition) - iDynTree::toEigen(m_currentDcmPosition);

    m_constraintsMatrix(0, 2) = temp(0);
    m_constraintsMatrix(1, 2) = temp(1);
    m_constraintsMatrix(2, 0) = m_convexHullComputer.A(0, 0);
    m_constraintsMatrix(2, 1) = m_convexHullComputer.A(0, 1);
    m_constraintsMatrix(3, 0) = m_convexHullComputer.A(1, 0);
    m_constraintsMatrix(3, 1) = m_convexHullComputer.A(1, 1);
    m_constraintsMatrix(4, 0) = m_convexHullComputer.A(2, 0);
    m_constraintsMatrix(4, 1) = m_convexHullComputer.A(2, 1);
    m_constraintsMatrix(5, 0) = m_convexHullComputer.A(3, 0);
    m_constraintsMatrix(5, 1) = m_convexHullComputer.A(3, 1);

    return true;
}

bool StepAdaptationController::computeBoundsVectorOfConstraints()
{
    if(m_convexHullComputer.b.size() != 4)
    {
        yError() << "QPSolver::setConstraintsVector the convex hull vector size is strange " << m_convexHullComputer.b.toString();
        return  false;
    }

    iDynTree::toEigen(m_upperBound).segment(0, 2) = iDynTree::toEigen(m_currentZmpPosition);
    iDynTree::toEigen(m_lowerBound).segment(0, 2) = iDynTree::toEigen(m_currentZmpPosition);
    iDynTree::toEigen(m_upperBound).segment(2, 4) = iDynTree::toEigen(m_convexHullComputer.b);

    m_lowerBound(2) = -qpOASES::INFTY;
    m_lowerBound(3) = -qpOASES::INFTY;
    m_lowerBound(4) = -qpOASES::INFTY;
    m_lowerBound(5) = -qpOASES::INFTY;

    m_upperBound(6) = std::exp((m_stepTiming + m_stepDurationTolerance) * m_omega);
    m_lowerBound(6) = std::exp((m_stepTiming - std::min(m_stepDurationTolerance, m_remainingSingleSupportDuration)) * m_omega);

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

    if (!computeGradientVector())
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the Gradient Vector";
        return false;
    }

    if(!computeConstraintsMatrix())
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the constraint matrix";
        return false;
    }

    if(!computeBoundsVectorOfConstraints())
    {
        yError() << "[StepAdaptationController::RunStepAdaptationController] Unable to set the bounds";
        return false;
    }

    m_isSolutionEvaluated = false;

    if(!solve())
    {
        yError() << "[StepAdaptationController::solve] Unable to solve the step adaptation problem.";
        return false;
    }

    MatrixXd constraintMatrix = MatrixXd(iDynTree::toEigen(m_constraintsMatrix));
    MatrixXd hessianMatrix = MatrixXd(iDynTree::toEigen(m_hessianMatrix));

    int nWSR = 100;
    if(!m_isFirstTime)
    {
        if(m_QPSolver_qpOASES->hotstart(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                                        nullptr, nullptr,m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the optimization problem.";
            return false;
        }
    }
    else
    {
        if(m_QPSolver_qpOASES->init(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                                    nullptr, nullptr,m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the optimization problem.";
            return false;
        }
        m_isFirstTime = false;
    }

    m_QPSolver_qpOASES->getPrimalSolution(m_solution.data());

    m_isSolutionEvaluated = true;
    return true;
}

double StepAdaptationController::getDesiredImpactTime()
{
    double optimalStepDuration = std::log(getSolution()(2)) / m_omega;
    return m_currentTime + optimalStepDuration - m_nextDoubleSupportDuration / 2;
}

iDynTree::Vector2 StepAdaptationController::getDesiredZmp()
{
    iDynTree::Vector2 desiredZmp;
    desiredZmp(0) = getSolution()(0);
    desiredZmp(1) = getSolution()(1);
    return desiredZmp;
}

bool StepAdaptationController::getAdaptatedFootTrajectory(const footTrajectoryGenerationInput& input,
                                                          iDynTree::Transform& adaptatedFootTransform, iDynTree::Twist& adaptedFootTwist,
                                                          iDynTree::SpatialAcc& adaptedFootAcceleration)
{

    iDynTree::CubicSpline xSpline, ySpline, zSpline, yawSpline;

    if(m_currentTime >= getDesiredImpactTime())
    {
        adaptedFootTwist.zero();
        iDynTree::Position newPosition;
        newPosition(0)=getDesiredZmp()(0) - (cos(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(0) - sin(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(1));
        newPosition(1)=getDesiredZmp()(1) - (cos(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(1) + sin(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(0));
        newPosition(2)= 0;
        adaptatedFootTransform.setPosition(newPosition);

        adaptatedFootTransform.setRotation(iDynTree::Rotation::RPY(0.0, 0.0, input.yawAngleAtImpact));

        return true;
    }

    double maxFootHeightTime = (getDesiredImpactTime() - input.takeOffTime) * 0.8 + input.takeOffTime;
    if (m_currentTime < maxFootHeightTime)
    {
        m_zFirstPieceTimesBuffer(0)= m_currentTime;
        m_zFirstPieceTimesBuffer(1)= maxFootHeightTime;
        m_zFirstPieceTimesBuffer(2)= getDesiredImpactTime();
        m_zFirstPiecePositionsBuffer(0)= input.currentFootTransform.getPosition()(2);
        m_zFirstPiecePositionsBuffer(1)= input.maxFootHeight;
        m_zFirstPiecePositionsBuffer(2)= 0;

        zSpline.setInitialConditions(input.currentFootTwist.getLinearVec3()(2), 0.0);
        zSpline.setFinalConditions(0.0,0.0);

        if (!zSpline.setData(m_zFirstPieceTimesBuffer, m_zFirstPiecePositionsBuffer))
        {
            std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the z-dimension spline." << std::endl;
            return false;
        }
    }
    else
    {
        m_zSecondPieceTimesBuffer(0)= m_currentTime;
        m_zSecondPieceTimesBuffer(1)= getDesiredImpactTime();

        iDynTree::Position PositionsBuffer=input.currentFootTransform.getPosition();
        m_zSecondPiecePositionsBuffer(0)=PositionsBuffer(2);
        m_zSecondPiecePositionsBuffer(1)= 0;
        zSpline.setInitialConditions(input.currentFootTwist.getLinearVec3()(2), 0.0);
        zSpline.setFinalConditions(0.0,0.0);

        if (!zSpline.setData(m_zSecondPieceTimesBuffer, m_zSecondPiecePositionsBuffer))
        {
            std::cerr << "[StepAdaptationController::getAdaptatedFootTrajectory] Failed to initialize the z-dimension spline." << std::endl;
            return false;
        }
    }

    m_xPositionsBuffer(0)= input.currentFootTransform.getPosition()(0);
    m_yPositionsBuffer(0)= input.currentFootTransform.getPosition()(1);

    m_yawsBuffer(0) = input.currentFootTransform.getRotation().asRPY()(2);

    m_xPositionsBuffer(1)= getDesiredZmp()(0) - (cos(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(0) - sin(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(1));
    m_yPositionsBuffer(1)= getDesiredZmp()(1) - (cos(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(1) + sin(input.yawAngleAtImpact) * input.zmpToCenterOfFootPosition(0));

    m_yawsBuffer(1) = input.yawAngleAtImpact;

    m_timesBuffer(0) = m_currentTime;
    m_timesBuffer(1) = getDesiredImpactTime();

    double yawAngle;

    iDynTree::AngularMotionVector3 rightTrivializedAngVelocity;
    iDynTree::Vector3 rpyDerivativeCurrent;
    iDynTree::Vector3 rpyDerivative;
    iDynTree::toEigen(rpyDerivativeCurrent) = iDynTree::toEigen(iDynTree::Rotation::RPYRightTrivializedDerivativeInverse(0.0, 0.0, m_yawsBuffer(0))) * iDynTree::toEigen(input.currentFootTwist.getAngularVec3());

    xSpline.setInitialConditions(input.currentFootTwist.getLinearVec3()(0), 0.0);
    ySpline.setInitialConditions(input.currentFootTwist.getLinearVec3()(1), 0.0);
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

    newPosition(0) = xSpline.evaluatePoint(m_currentTime + input.discretizationTime, linearVelocity(0), linearAcceleration(0));
    newPosition(1) = ySpline.evaluatePoint(m_currentTime + input.discretizationTime, linearVelocity(1), linearAcceleration(1));
    newPosition(2) = zSpline.evaluatePoint(m_currentTime + input.discretizationTime, linearVelocity(2), linearAcceleration(2));

    yawAngle = yawSpline.evaluatePoint(m_currentTime + input.discretizationTime, rpyDerivative(2), rpySecondDerivative(2));

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
    return m_dcmErrorThreshold;
}
iDynTree::Vector2 StepAdaptationController::getRollPitchErrorThreshold(){
    return m_rollPitchErrorThreshold;
}

void StepAdaptationController::reset(){
    m_isSolutionEvaluated = false;
}

const iDynTree::VectorDynSize& StepAdaptationController::getSolution() const
{
    return m_solution;
}

bool StepAdaptationController::isInitialized()
{
    return m_QPSolver->isInitialized();
}

bool StepAdaptationController::initialize()
{
    return m_QPSolver->initSolver();
}
