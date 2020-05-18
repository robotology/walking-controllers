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

    m_hessianMatrix.resize(m_inputSize, m_inputSize);
    m_solution.resize(m_inputSize);

    // qpoases
    m_QPSolver_qpOASES = std::make_unique<qpOASES::SQProblem>(m_inputSize,
                                                              m_numberOfConstraints);

    m_QPSolver_qpOASES->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
}

bool StepAdaptationController::configure(const yarp::os::Searchable &config)
{
    if(!YarpUtilities::getNumberFromSearchable(config, "stepHeight", m_stepHeight))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the number";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_position_weight", m_zmpPositionWeight))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "next_dcm_offset_weight", m_dcmOffsetWeight))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "sigma_weight", m_sigmaWeight))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the number";
        return false;
    }

    if(!YarpUtilities::getNumberFromSearchable(config, "push_recovery_activation_index", m_pushRecoveryActivationIndex))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the number";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "leftZMPDelta", m_zmpToCenterOfFootPositionLeft))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector of leftZMPDelta";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "rightZMPDelta", m_zmpToCenterOfFootPositionRight))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector of rightZMPDelta";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "offset_roll_pitch_arm_error", m_armRollPitchErrorOffset))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector of offset_roll_pitch_arm_error";
        return false;
    }

    yarp::os::Value *jointsListForPushDetectionYarpRX;
    if(!config.check("joints_list_push_detection_right_arm_x", jointsListForPushDetectionYarpRX))
    {
        yError() << "[StepAdaptationController::Configure] Unable to find joints_list into config file.";
        return false;
    }

    if(!YarpUtilities::yarpListToStringVector(jointsListForPushDetectionYarpRX, m_pushDetectionListRightArmX))
    {
        yError() << "[StepAdaptationController::Configure] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    yarp::os::Value *jointsListForPushDetectionYarpRY;
    if(!config.check("joints_list_push_detection_right_arm_y", jointsListForPushDetectionYarpRY))
    {
        yError() << "[StepAdaptationController::Configure] Unable to find joints_list into config file.";
        return false;
    }

    if(!YarpUtilities::yarpListToStringVector(jointsListForPushDetectionYarpRY, m_pushDetectionListRightArmY))
    {
        yError() << "[StepAdaptationController::Configure] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    yarp::os::Value *jointsListForPushDetectionYarpLX;
    if(!config.check("joints_list_push_detection_left_arm_x", jointsListForPushDetectionYarpLX))
    {
        yError() << "[StepAdaptationController::Configure] Unable to find joints_list into config file.";
        return false;
    }

    if(!YarpUtilities::yarpListToStringVector(jointsListForPushDetectionYarpLX, m_pushDetectionListLeftArmX))
    {
        yError() << "[StepAdaptationController::Configure] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    yarp::os::Value *jointsListForPushDetectionYarpLY;
    if(!config.check("joints_list_push_detection_left_arm_y", jointsListForPushDetectionYarpLY))
    {
        yError() << "[StepAdaptationController::Configure] Unable to find joints_list into config file.";
        return false;
    }

    if(!YarpUtilities::yarpListToStringVector(jointsListForPushDetectionYarpLY, m_pushDetectionListLeftArmY))
    {
        yError() << "[StepAdaptationController::Configure] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    m_feetExtendedPolygon.resize(2);
    iDynTree::Polygon foot;
    iDynTree::Vector4 nextZmpConstraintBoundLeftFoot;
    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_constraint_bound_left_foot", nextZmpConstraintBoundLeftFoot))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_dcm_error",  m_dcmErrorThreshold))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector of DCM error threshold";
        return false;
    }

    if(!YarpUtilities::getVectorFromSearchable(config, "threshold_roll_pitch_error",  m_rollPitchErrorThreshold))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector of roll pitch imu error threshold";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(nextZmpConstraintBoundLeftFoot(0), nextZmpConstraintBoundLeftFoot(1),
                                                     nextZmpConstraintBoundLeftFoot(2), nextZmpConstraintBoundLeftFoot(3));
    m_feetExtendedPolygon[0] = foot;

    iDynTree::Vector4 nextZmpConstraintBoundRightFoot;
    if(!YarpUtilities::getVectorFromSearchable(config, "next_zmp_constraint_bound_right_foot", nextZmpConstraintBoundRightFoot))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the vector";
        return false;
    }

    foot = iDynTree::Polygon::XYRectangleFromOffsets(nextZmpConstraintBoundRightFoot(0), nextZmpConstraintBoundRightFoot(1),
                                                     nextZmpConstraintBoundRightFoot(2), nextZmpConstraintBoundRightFoot(3));

    m_feetExtendedPolygon[1] = foot;

    if(!YarpUtilities::getNumberFromSearchable(config, "step_duration_tolerance", m_stepDurationTolerance))
    {
        yError() << "[StepAdaptationController::Configure] Unable to get the number";
        return false;
    }

    if(!computeHessianMatrix())
    {
        yError() << "[StepAdaptationController::Configure] Unable set the hessian";
        return false;
    }

    //initialize the DCM simple estimator
    m_DCMEstimator=std::make_unique<DCMSimpleEstimator>();
    if(!m_DCMEstimator->configure(config))
    {
        yError() << "[StepAdaptationController::Configure] Failed to configure the DCM pendulum estimator.";
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

bool StepAdaptationController::solve(SwingFoot swingFoot)
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

    if(swingFoot==SwingFoot::Left)
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
    double optimalStepDuration = std::log(m_solution(2)) / m_omega;
    return m_currentTime + optimalStepDuration - m_nextDoubleSupportDuration / 2;
}

iDynTree::Vector2 StepAdaptationController::getDesiredZmp()
{
    iDynTree::Vector2 desiredZmp;
    desiredZmp(0) = m_solution(0);
    desiredZmp(1) = m_solution(1);
    return desiredZmp;
}

iDynTree::Vector2 StepAdaptationController::getDesiredDCMOffset()
{
    iDynTree::Vector2 desiredDCMOffset;
    desiredDCMOffset(0) = m_solution(3);
    desiredDCMOffset(1) = m_solution(4);
    return desiredDCMOffset;
}

bool StepAdaptationController::getAdaptatedFootTrajectory(const FootTrajectoryGenerationInput& input,
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

bool StepAdaptationController::triggerStepAdapterByArmCompliant(const double &numberOfActuatedDof, const iDynTree::VectorDynSize &qDesired, const iDynTree::VectorDynSize &qActual,
                                                                const std::deque<bool>& leftInContact,const std::deque<bool>& rightInContact,std::vector<std::string> jointsListVector)
{
    double leftArmPitchError=0;
    double rightArmPitchError=0;
    double leftArmRollError=0;
    double rightArmRollError=0;

    for (int var=0;var<m_pushDetectionListRightArmX.size();++var)
    {
        std::vector<std::string>::iterator it = std::find(jointsListVector.begin(), jointsListVector.end(), m_pushDetectionListRightArmX.at(var));
        if(it == jointsListVector.end())
        {
            yError() << "[StepAdaptationController::triggerStepAdapterByArmCompliant] Unable to to find"<< m_pushDetectionListRightArmX.at(var)<<"inside the controlled joints of the robot";
            return false;
        }

        rightArmPitchError=rightArmPitchError+abs(qDesired(std::distance(jointsListVector.begin(), it))-qActual(std::distance(jointsListVector.begin(), it)));
    }

    for (int var=0;var<m_pushDetectionListRightArmY.size();++var)
    {
        std::vector<std::string>::iterator it = std::find(jointsListVector.begin(), jointsListVector.end(), m_pushDetectionListRightArmY.at(var));
        if(it == jointsListVector.end())
        {
            yError() << "[StepAdaptationController::triggerStepAdapterByArmCompliant] Unable to to find"<< m_pushDetectionListRightArmY.at(var)<<"inside the controlled joints of the robot";
            return false;
        }

        rightArmRollError=rightArmRollError+abs(qDesired(std::distance(jointsListVector.begin(), it))-qActual(std::distance(jointsListVector.begin(), it)));
    }

    for (int var=0;var<m_pushDetectionListLeftArmX.size();++var)
    {
        std::vector<std::string>::iterator it = std::find(jointsListVector.begin(), jointsListVector.end(), m_pushDetectionListLeftArmX.at(var));
        if(it == jointsListVector.end())
        {
            yError() << "[StepAdaptationController::triggerStepAdapterByArmCompliant] Unable to to find"<< m_pushDetectionListLeftArmX.at(var)<<"inside the controlled joints of the robot";
            return false;
        }

        rightArmPitchError=rightArmPitchError+abs(qDesired(std::distance(jointsListVector.begin(), it))-qActual(std::distance(jointsListVector.begin(), it)));
    }

    for (int var=0;var<m_pushDetectionListLeftArmY.size();++var)
    {
        std::vector<std::string>::iterator it = std::find(jointsListVector.begin(), jointsListVector.end(), m_pushDetectionListLeftArmY.at(var));
        if(it == jointsListVector.end())
        {
            yError() << "[StepAdaptationController::triggerStepAdapterByArmCompliant] Unable to to find"<< m_pushDetectionListLeftArmY.at(var)<<"inside the controlled joints of the robot";
            return false;
        }

        leftArmRollError=leftArmRollError+abs(qDesired(std::distance(jointsListVector.begin(), it))-qActual(std::distance(jointsListVector.begin(), it)));
    }

    if (leftArmPitchError>getRollPitchErrorThreshold()(1) )
    {
        m_armPitchError=leftArmPitchError;
        m_isPitchActive=1;
    }
    else if( rightArmPitchError>getRollPitchErrorThreshold()(1))
    {
        m_armPitchError=rightArmPitchError;
        m_isPitchActive=1;
    }
    else
    {
        m_armPitchError=0;
    }

    if (leftArmRollError>getRollPitchErrorThreshold()(0) )
    {
        if (leftInContact.front())
        {
            m_armRollError=1*leftArmRollError;
        }
        if (rightInContact.front())
        {
            m_armRollError=-1*leftArmRollError;
        }
        m_isRollActive=1;
    }
    else if( rightArmRollError>getRollPitchErrorThreshold()(0) )
    {
        if (rightInContact.front())
        {
            m_armRollError=-1*rightArmRollError;
        }
        if (leftInContact.front())
        {
            m_armRollError=+1*rightArmRollError;
        }
        m_isRollActive=1;
    }
    else
    {
        m_armRollError=0;
    }

    return true;
}

const double& StepAdaptationController::getArmRollError() const
{
    return m_armRollError;
}

const double& StepAdaptationController::getArmPitchError() const
{
    return m_armPitchError;
}

const int& StepAdaptationController::getPushRecoveryActivationIndex() const
{
    return m_pushRecoveryActivationIndex;
}

bool StepAdaptationController::UpdateDCMEstimator(const iDynTree::Vector2& CoM2DPosition,const iDynTree::Vector2& CoMVelocity,const iDynTree::Vector2& measuredZMP,const double& CoMHeight)
{
    iDynTree::Rotation imuRotation;
    iDynTree::Vector3 imuRPY;
    iDynTree::Rotation StanceFootOrientation;
    StanceFootOrientation=iDynTree::Rotation::Identity();

    if (m_isRollActive)
    {
        m_armRollError= m_armRollError+m_armRollPitchErrorOffset(0);
    }
    if (m_isPitchActive)
    {
        m_armPitchError= m_armPitchError+m_armRollPitchErrorOffset(1);
    }

    StanceFootOrientation=iDynTree::Rotation::RPY (m_armRollError,m_armPitchError,0); 	//=m_FKSolver->getRootLinkToWorldTransform().getRotation().asRPY();

    iDynTree::Vector3 ZMP3d;
    ZMP3d(0)=measuredZMP(0);
    ZMP3d(1)=measuredZMP(1);
    ZMP3d(2)=0;

    iDynTree::Vector3 CoM3d;
    //iDynTree::Vector3 ZMP3d;
    CoM3d(0)=CoM2DPosition(0);
    CoM3d(1)=CoM2DPosition(1);
    CoM3d(2)=CoMHeight;

    iDynTree::Vector3 CoMVelocity3d;
    //iDynTree::Vector3 ZMP3d;
    CoMVelocity3d(0)=CoMVelocity(0);
    CoMVelocity3d(1)=CoMVelocity(1);
    CoMVelocity3d(2)=0;

    if(!m_DCMEstimator->update(StanceFootOrientation,ZMP3d,CoM3d,CoMVelocity3d))
    {
        yError() << "[StepAdaptationController::UpdateDCMEstimator] Unable to to recieve DCM from pendulumEstimator";
        return false;
    }

    return true;
}

bool StepAdaptationController::runStepAdaptation(const StepAdapterInput &input, StepAdapterOutput &output)
{
    if (!input.leftInContact.front() || !input.rightInContact.front())
    {
        output.indexPush=output.indexPush+1;

        int numberOfSubTrajectories = input.dcmSubTrajectories.size();

        if(numberOfSubTrajectories<4)
        {
            yError() << "[StepAdaptationController::runStepAdaptation] the number of sub-trajectories should be equal or greater than 4";
            return false;
        }

        auto firstSS = input.dcmSubTrajectories[numberOfSubTrajectories-2];
        auto secondSS = input.dcmSubTrajectories[numberOfSubTrajectories-4];

        auto secondDS = input.dcmSubTrajectories[numberOfSubTrajectories-3];
        auto firstDS = input.dcmSubTrajectories[numberOfSubTrajectories-1];

        iDynTree::Vector2 nextZmpPosition, currentZmpPosition;
        bool checkFeasibility = false;

        if(!secondSS->getZMPPosition(0, nextZmpPosition, checkFeasibility))
        {
            yError() << "[StepAdaptationController::runStepAdaptation] unable to get ZMP Position for second single support";
            return false;
        }

        double angle = !input.leftInContact.front()? input.leftFootprints->getSteps()[1].angle : input.rightFootprints->getSteps()[1].angle;
        setNominalNextStepPosition(nextZmpPosition, angle);

        if(!firstSS->getZMPPosition(0, currentZmpPosition, checkFeasibility))
        {
            yError() << "[StepAdaptationController::runStepAdaptation] unable to get ZMP Position for first single support";
            return false;
        }

        setCurrentZmpPosition(currentZmpPosition);

        output.isPushActive=0;

        if((abs(input.dcmPositionSmoothed(0) - getEstimatedDCM()(0))) > getDCMErrorThreshold()(0) ||(abs(input.dcmPositionSmoothed(1) - getEstimatedDCM()(1)))> getDCMErrorThreshold()(1) )
        {
            if (output.pushRecoveryActiveIndex==m_pushRecoveryActivationIndex )
            {
                output.isPushActive=1;
                iDynTree::Vector2 tempDCMError;
                tempDCMError(1)=0.00;
                tempDCMError(0)=0.00;
                output.pushRecoveryActiveIndex++;
                yInfo()<<"triggering the push recovery";
                if((abs(input.dcmPositionSmoothed(0) - getEstimatedDCM()(0))) > getDCMErrorThreshold()(0))
                {
                    tempDCMError(0)=getEstimatedDCM()(0);
                }
                if((abs(input.dcmPositionSmoothed(1) - getEstimatedDCM()(1))) > getDCMErrorThreshold()(1))
                {
                    tempDCMError(1)=getEstimatedDCM()(1);
                }
                setCurrentDcmPosition(tempDCMError);
            }
            else
            {
                output.pushRecoveryActiveIndex++;
                setCurrentDcmPosition(output.dcmPositionAdjusted.front());
            }

        }
        else
        {
            if (output.pushRecoveryActiveIndex<=m_pushRecoveryActivationIndex)
            {
                output.pushRecoveryActiveIndex=0;
                setCurrentDcmPosition(output.dcmPositionAdjusted.front());
            }
            else if(output.pushRecoveryActiveIndex==(m_pushRecoveryActivationIndex+1))
            {
                setCurrentDcmPosition(output.dcmPositionAdjusted.front());
                output.pushRecoveryActiveIndex++;
            }
            else
            {
                setCurrentDcmPosition(output.dcmPositionAdjusted.front());
            }
        }

        iDynTree::Vector2 dcmAtTimeAlpha;
        double timeAlpha = (secondDS->getTrajectoryDomain().second + secondDS->getTrajectoryDomain().first) / 2;

        if(!input.dcmSubTrajectories[numberOfSubTrajectories-2]->getDCMPosition(timeAlpha, dcmAtTimeAlpha, checkFeasibility))
        {
            yError() << "[StepAdaptationController::runStepAdaptation] unable to get DCM Position ";
            return false;
        }

        iDynTree::Vector2 nominalDcmOffset;
        iDynTree::toEigen(nominalDcmOffset) = iDynTree::toEigen(dcmAtTimeAlpha) - iDynTree::toEigen(nextZmpPosition);
        setNominalDcmOffset(nominalDcmOffset);

        //timeOffset is the time of start of this step(that will be updated in updateTrajectory function at starting point of each step )
        setTimings(input.omega, input.time - input.timeOffset, firstSS->getTrajectoryDomain().second,
                                    secondDS->getTrajectoryDomain().second - secondDS->getTrajectoryDomain().first);

        SwingFoot swingFoot;
        if (!input.leftInContact.front())
        {
            swingFoot=SwingFoot::Left;
        }
        else
        {
            swingFoot=SwingFoot::Right;
        }

        if(!solve(swingFoot))
        {
            yError() << "[StepAdaptationController::runStepAdaptation] unable to solve the step adjustment optimization problem";
            return false;
        }

        output.impactTimeNominal = firstSS->getTrajectoryDomain().second + input.timeOffset;
        if(output.pushRecoveryActiveIndex==(m_pushRecoveryActivationIndex+1))
        {
            double timeOfSmoothing=(secondDS->getTrajectoryDomain().second-secondDS->getTrajectoryDomain().first)/2 +getDesiredImpactTime()-(input.time - input.timeOffset);
            output.indexSmoother=timeOfSmoothing/input.dT;
            output.kDCMSmoother=0;
        }
        if(output.pushRecoveryActiveIndex==(m_pushRecoveryActivationIndex+1))
        {
            double timeOfSmoothing=getDesiredImpactTime()-(input.time - input.timeOffset);
            output.indexFootSmoother=timeOfSmoothing/input.dT;
            output.kFootSmoother=0;
        }
        output.impactTimeAdjusted = getDesiredImpactTime() + input.timeOffset;

        output.zmpNominal = nextZmpPosition;
        output.zmpAdjusted = getDesiredZmp();

        iDynTree::Vector2 zmpOffset;
        if (!input.leftInContact.front())
        {
            zmpOffset=m_zmpToCenterOfFootPositionLeft;
        }
        if (!input.rightInContact.front())
        {
            zmpOffset=m_zmpToCenterOfFootPositionRight;
        }

        if (!input.leftInContact.front())
        {
            output.currentFootLeftTransform = output.adaptedFootLeftTransform;
            output.currentFootLeftTwist = output.adaptedFootLeftTwist;
            output.currentFootLeftAcceleration = output.adaptedFootLeftAcceleration;

            FootTrajectoryGenerationInput inputLeftFootTrajectory;
            inputLeftFootTrajectory.maxFootHeight=m_stepHeight;
            inputLeftFootTrajectory.discretizationTime=input.dT;
            inputLeftFootTrajectory.takeOffTime =firstSS->getTrajectoryDomain().first;
            inputLeftFootTrajectory.yawAngleAtImpact=input.leftStepList.at(1).angle;
            inputLeftFootTrajectory.zmpToCenterOfFootPosition=zmpOffset;
            inputLeftFootTrajectory.currentFootTransform=output.currentFootLeftTransform;
            inputLeftFootTrajectory.currentFootTwist=output.currentFootLeftTwist;

            if(!getAdaptatedFootTrajectory(inputLeftFootTrajectory,
                                                          output.adaptedFootLeftTransform, output.adaptedFootLeftTwist, output.adaptedFootLeftAcceleration ))
            {
                yError() << "[StepAdaptationController::runStepAdaptation] unable to get get adaptated left foot trajectory";
                return false;
            }
        }
        else
        {

            output.currentFootRightTransform = output.adaptedFootRightTransform;
            output.currentFootRightTwist = output.adaptedFootRightTwist;
            output.currentFootRightAcceleration = output.adaptedFootRightAcceleration;

            FootTrajectoryGenerationInput inputRightFootTrajectory;
            inputRightFootTrajectory.maxFootHeight=m_stepHeight;
            inputRightFootTrajectory.discretizationTime=input.dT;
            inputRightFootTrajectory.takeOffTime =firstSS->getTrajectoryDomain().first;
            inputRightFootTrajectory.yawAngleAtImpact=input.rightStepList.at(1).angle;
            inputRightFootTrajectory.zmpToCenterOfFootPosition=zmpOffset;
            inputRightFootTrajectory.currentFootTransform=output.currentFootRightTransform;
            inputRightFootTrajectory.currentFootTwist=output.currentFootRightTwist;

            if(!getAdaptatedFootTrajectory(inputRightFootTrajectory,
                                                            output.adaptedFootRightTransform, output.adaptedFootRightTwist, output.adaptedFootRightAcceleration ))
            {
                yError() << "[StepAdaptationController::runStepAdaptation] unable to get the adaptated right foot trajectory";
                return false;
            }
        }


    }

    else
    {
        output.currentFootLeftAcceleration=output.adaptedFootLeftAcceleration;
        output.currentFootLeftTwist=output.adaptedFootLeftTwist;
        output.currentFootLeftTransform=output.adaptedFootLeftTransform;

        output.currentFootRightAcceleration=output.adaptedFootRightAcceleration;
        output.currentFootRightTwist=output.adaptedFootRightTwist;
        output.currentFootRightTransform=output.adaptedFootRightTransform;
    }
    return true;
}

const iDynTree::Vector2& StepAdaptationController::getEstimatedDCM() const
{
    return m_DCMEstimator->getDCMPosition();
}

bool StepAdaptationController::isArmRollActive()
{
    return m_isRollActive;
}

bool StepAdaptationController::isArmPitchActive()
{
    return m_isPitchActive;
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
