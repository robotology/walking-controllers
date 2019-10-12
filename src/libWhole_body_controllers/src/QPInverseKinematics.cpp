/**
 * @file WalkingQPInverseKinematics.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>, Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingControllers/YarpHelper/Helper.h>
#include <WalkingControllers/iDynTreeHelper/Helper.h>

#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics.h>

using namespace WalkingControllers;

bool WalkingQPIK::initialize(const yarp::os::Searchable &config,
                             const int &actuatedDOFs,
                             const iDynTree::VectorDynSize& maxJointsVelocity,
                             const iDynTree::VectorDynSize& maxJointsPosition,
                             const iDynTree::VectorDynSize& minJointsPosition)
{
    m_actuatedDOFs = actuatedDOFs;
    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for QP-IK solver.";
        return false;
    }

    m_useCoMAsConstraint = config.check("use_com_as_constraint", yarp::os::Value(false)).asBool();
    m_enableHandRetargeting = config.check("use_hand_retargeting", yarp::os::Value(false)).asBool();
    m_useJointsLimitsConstraint = config.check("use_joint_limits_constraint", yarp::os::Value(false)).asBool();

    // TODO in the future the number of constraints should be added inside
    // the configuration file
    // set the number of variables and the number of constraints
    // the number of variables is equal to the number of joints plus
    // 6 (position + attitude) degree of freedom related to the base.
    m_numberOfVariables = m_actuatedDOFs + 6;


    // the number of constraints is equal to the number of joints plus
    // 12 (position + attitude) of the left and right feet
    setNumberOfConstraints();

    // resize vectors
    m_gradient.resize(m_numberOfVariables);
    m_gradient.zero();
    m_lowerBound.resize(m_numberOfConstraints);
    m_lowerBound.zero();
    m_upperBound.resize(m_numberOfConstraints);
    m_upperBound.zero();
    m_solution.resize(m_numberOfVariables);
    m_desiredJointVelocitiesOutput.resize(m_actuatedDOFs);

    m_hessianDense.resize(m_numberOfVariables, m_numberOfVariables);
    m_constraintsMatrixSparse.resize(m_numberOfConstraints, m_numberOfVariables);

    // resize Jacobians matrices
    m_comJacobian.resize(3, m_numberOfVariables);
    m_neckJacobian.resize(3, m_numberOfVariables);
    m_leftFootJacobian.resize(6, m_numberOfVariables);
    m_rightFootJacobian.resize(6, m_numberOfVariables);
    m_leftHandJacobian.resize(6, m_numberOfVariables);
    m_rightHandJacobian.resize(6, m_numberOfVariables);

    m_regularizationTerm.resize(m_actuatedDOFs);
    m_jointPosition.resize(m_actuatedDOFs);

    // preprare constant matrix necessary for the QP problem
    if(!initializeMatrices(config))
    {
        yError() << "[initialize] Unable to Initialize the constant matrix.";
        return false;
    }

    initializeSolverSpecificMatrices();

    if(m_enableHandRetargeting)
        if(!initializeHandRetargeting(config))
        {
            yError() << "[initialize] Unable to Initialize the hand retargeting.";
            return false;
        }

    if(!setJointsBounds(maxJointsVelocity, maxJointsPosition, minJointsPosition))
    {
        yError() << "[initialize] Unable to set the joint bounds.";
        return false;
    }

    instantiateSolver();

    return true;
}

bool WalkingQPIK::setJointsBounds(const iDynTree::VectorDynSize& jointVelocitiesBounds,
                                  const iDynTree::VectorDynSize& jointPositionsUpperBounds,
                                  const iDynTree::VectorDynSize& jointPositionsLowerBounds)
{
    if(jointVelocitiesBounds.size() != jointPositionsUpperBounds.size() ||
       jointPositionsLowerBounds.size() != jointPositionsUpperBounds.size())
    {
        yError() << "[setJointsBounds] The size of the vector limits has to be equal.";
        return false;
    }
    if(jointVelocitiesBounds.size() != m_actuatedDOFs)
    {
        yError() << "[setJointsBounds] The size of the vector limits has to be equal to ."
                 << "the number of the joint";
        return false;
    }
    m_jointVelocitiesBounds = jointVelocitiesBounds;
    m_jointPositionsUpperBounds = jointPositionsUpperBounds;
    m_jointPositionsLowerBounds = jointPositionsLowerBounds;
    return true;
}

bool WalkingQPIK::initializeMatrices(const yarp::os::Searchable& config)
{
    // evaluate constant sub-matrix of the hessian matrix
    // get the CoM weight
    if(!m_useCoMAsConstraint)
    {
        if(!YarpHelper::getVectorFromSearchable(config, "com_weight", m_comWeight))
        {
            yError() << "Initialization failed while reading com_weight vector.";
            return false;
        }
    }

    if(!YarpHelper::getNumberFromSearchable(config, "neck_weight", m_neckWeight))
    {
        yError() << "Initialization failed while reading neck_weight vector.";
        return false;
    }

    // set the matrix related to the joint regularization
    iDynTree::VectorDynSize jointRegularizationWeights(m_actuatedDOFs);
    if(!YarpHelper::getVectorFromSearchable(config, "joint_regularization_weights", jointRegularizationWeights))
    {
        yError() << "Initialization failed while reading jointRegularizationWeights vector.";
        return false;
    }

    //  m_jointRegulatizationHessian = H' \lamda H
    m_jointRegularizationHessian.resize(m_numberOfVariables, m_numberOfVariables);
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_jointRegularizationHessian(i + 6, i + 6) = jointRegularizationWeights(i);

    // evaluate constant sub-matrix of the gradient matrix
    m_jointRegularizationGradient.resize(m_numberOfVariables, m_actuatedDOFs);
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_jointRegularizationGradient(i + 6, i) = jointRegularizationWeights(i);

    m_jointRegularizationGains.resize(m_actuatedDOFs);
    if(!YarpHelper::getVectorFromSearchable(config, "joint_regularization_gains",
                                            m_jointRegularizationGains))
    {
        yError() << "Initialization failed while reading jointRegularizationGains vector.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_posFoot", m_kPosFoot))
    {
        yError() << "Initialization failed while reading k_posFoot.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_attFoot", m_kAttFoot))
    {
        yError() << "Initialization failed while reading k_attFoot.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_neck", m_kNeck))
    {
        yError() << "Initialization failed while reading k_neck.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_posCom", m_kCom))
    {
        yError() << "Initialization failed while reading k_posCom.";
        return false;
    }

    // get the regularization term
    if(!YarpHelper::getVectorFromSearchable(config, "joint_regularization", m_regularizationTerm))
    {
        yError() << "[initialize] Unable to get the iDynTreeVector from searchable.";
        return false;
    }

    iDynTree::toEigen(m_regularizationTerm) = iDynTree::toEigen(m_regularizationTerm) *
        iDynTree::deg2rad(1);

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[initialize] Unable to set the additional rotation.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_joint_limit_lower_bound", m_kJointLimitsLowerBound))
    {
        yError() << "Initialization failed while reading a double.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_joint_limit_upper_bound", m_kJointLimitsUpperBound))
    {
        yError() << "Initialization failed while reading a double.";
        return false;
    }

    return true;
}

bool WalkingQPIK::initializeHandRetargeting(const yarp::os::Searchable& config)
{
    if(!YarpHelper::getNumberFromSearchable(config, "k_posHand", m_kPosHand))
    {
        yError() << "Initialization failed while reading k_posHand.";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(config, "k_attHand", m_kAttHand))
    {
        yError() << "Initialization failed while reading k_attHand.";
        return false;
    }

    m_handWeightWalkingVector.resize(6);
    if(!YarpHelper::getVectorFromSearchable(config, "hand_weight_walking", m_handWeightWalkingVector))
    {
        yError() << "Initialization failed while reading the yarp vector.";
        return false;
    }


    m_handWeightStanceVector.resize(6);
    if(!YarpHelper::getVectorFromSearchable(config, "hand_weight_stance", m_handWeightStanceVector))
    {
        yError() << "Initialization failed while reading the yarp vector.";
        return false;
    }

    double smoothingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "smoothing_time", smoothingTime))
    {
        yError() << "Initialization failed while reading smoothingTime.";
        return false;
    }

    double dT;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", dT))
    {
        yError() << "Initialization failed while a double.";
        return false;
    }


    m_handWeightSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, dT, smoothingTime);
    m_handWeightSmoother->init(m_handWeightStanceVector);

    return true;
}

bool WalkingQPIK::setRobotState(const iDynTree::VectorDynSize& jointPosition,
                                const iDynTree::Transform& leftFootToWorldTransform,
                                const iDynTree::Transform& rightFootToWorldTransform,
                                const iDynTree::Transform& leftHandToWorldTransform,
                                const iDynTree::Transform& rightHandToWorldTransform,
                                const iDynTree::Rotation& neckOrientation,
                                const iDynTree::Position& comPosition)
{
    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setRobotState] The size of the jointPosition vector is not coherent with the "
                 << "number of the actuated Joint";
        return false;
    }

    // avoid to copy the vector if the application is not ran in retargeting mode
    if(m_enableHandRetargeting)
    {
        m_leftHandToWorldTransform = leftHandToWorldTransform;
        m_rightHandToWorldTransform = rightHandToWorldTransform;
    }

    m_jointPosition = jointPosition;
    m_leftFootToWorldTransform = leftFootToWorldTransform;
    m_rightFootToWorldTransform = rightFootToWorldTransform;
    m_neckOrientation = neckOrientation;
    m_comPosition = comPosition;

    return true;
}

void WalkingQPIK::setPhase(const bool& isStancePhase)
{
    if(!m_enableHandRetargeting)
        return;

    if(isStancePhase)
        m_handWeightSmoother->computeNextValues(m_handWeightStanceVector);
    else
        m_handWeightSmoother->computeNextValues(m_handWeightWalkingVector);
}

void WalkingQPIK::setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation)
{
    m_desiredNeckOrientation =  desiredNeckOrientation * m_additionalRotation;
}

bool WalkingQPIK::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
{
    if(comJacobian.rows() != 3)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 3.";
        return false;
    }
    if(comJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }
    m_comJacobian = comJacobian;

    return true;
}

bool WalkingQPIK::setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian)
{
    if(leftFootJacobian.rows() != 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(leftFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_leftFootJacobian = leftFootJacobian;

    return true;
}

bool WalkingQPIK::setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian)
{
    if(rightFootJacobian.rows() != 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightFootJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setCoMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_rightFootJacobian = rightFootJacobian;

    return true;
}

bool WalkingQPIK::setLeftHandJacobian(const iDynTree::MatrixDynSize& leftHandJacobian)
{
    if(!m_enableHandRetargeting)
        return true;

    if(leftHandJacobian.rows() != 6)
    {
        yError() << "[setLeftHandJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(leftHandJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setLeftHandJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_leftHandJacobian = leftHandJacobian;

    return true;
}

bool WalkingQPIK::setRightHandJacobian(const iDynTree::MatrixDynSize& rightHandJacobian)
{
    if(!m_enableHandRetargeting)
        return true;

    if(rightHandJacobian.rows() != 6)
    {
        yError() << "[setRightHandJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(rightHandJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setRightHandJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    m_rightHandJacobian = rightHandJacobian;

    return true;
}

bool WalkingQPIK::setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian)
{
    if(neckJacobian.rows() != 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to 6.";
        return false;
    }
    if(neckJacobian.cols() != m_actuatedDOFs + 6)
    {
        yError() << "[setNeckMJacobian] the number of rows has to be equal to" << m_actuatedDOFs + 6;
        return false;
    }

    iDynTree::toEigen(m_neckJacobian) = iDynTree::toEigen(neckJacobian).block(3, 0, 3,
                                                                              m_actuatedDOFs + 6);

    return true;
}

bool WalkingQPIK::setDesiredJointPosition(const iDynTree::VectorDynSize& regularizationTerm)
{
    if(regularizationTerm.size() != m_actuatedDOFs)
    {
        yError() << "[setDesiredJointPosition] The number of the desired joint position has to be "
                 << "equal to the number of actuated joints";
        return false;
    }

    m_regularizationTerm = regularizationTerm;

    return true;
}

void WalkingQPIK::setDesiredFeetTransformation(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                               const iDynTree::Transform& desiredRightFootToWorldTransform)
{
    m_desiredLeftFootToWorldTransform = desiredLeftFootToWorldTransform;
    m_desiredRightFootToWorldTransform = desiredRightFootToWorldTransform;
}

void WalkingQPIK::setDesiredFeetTwist(const iDynTree::Twist& leftFootTwist,
                                      const iDynTree::Twist& rightFootTwist)
{
    m_desiredLeftFootTwist = leftFootTwist;
    m_desiredRightFootTwist = rightFootTwist;
}

void WalkingQPIK::setDesiredHandsTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform,
                                                const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    if(!m_enableHandRetargeting)
        return;

    m_desiredLeftHandToWorldTransform = desiredLeftHandToWorldTransform;
    m_desiredRightHandToWorldTransform = desiredRightHandToWorldTransform;
}

void WalkingQPIK::setDesiredHandsTwist(const iDynTree::Twist& leftHandTwist,
                                       const iDynTree::Twist& rightHandTwist)
{
    m_desiredLeftHandTwist = leftHandTwist;
    m_desiredRightHandTwist = rightHandTwist;
}

void WalkingQPIK::setDesiredCoMVelocity(const iDynTree::Vector3& comVelocity)
{
    m_desiredComVelocity = comVelocity;
}

void WalkingQPIK::setDesiredCoMPosition(const iDynTree::Position& desiredComPosition)
{
    m_desiredComPosition = desiredComPosition;
}

void WalkingQPIK::evaluateHessianMatrix()
{
    // in that case the hessian matrix is related only to neck orientation and to
    // the joint angle
    auto hessianDense(iDynTree::toEigen(m_hessianDense));

    hessianDense = iDynTree::toEigen(m_neckJacobian).transpose() *
        m_neckWeight *
        iDynTree::toEigen(m_neckJacobian);

    if(!m_enableHandRetargeting)
        hessianDense += iDynTree::toEigen(m_jointRegularizationHessian);
    else
    {
        // think about the possibility to project in the null space the joint regularization
        hessianDense += iDynTree::toEigen(m_jointRegularizationHessian)
            + iDynTree::toEigen(m_leftHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal()
            * iDynTree::toEigen(m_leftHandJacobian)
            + iDynTree::toEigen(m_rightHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal()
            * iDynTree::toEigen(m_rightHandJacobian);
    }

    if(!m_useCoMAsConstraint)
    {
        hessianDense += iDynTree::toEigen(m_comJacobian).transpose() *
            iDynTree::toEigen(m_comWeight).asDiagonal() * iDynTree::toEigen(m_comJacobian);
    }
}

void WalkingQPIK::evaluateGradientVector()
{
    auto gradient(iDynTree::toEigen(m_gradient));

    auto neckJacobian(iDynTree::toEigen(m_neckJacobian));

    auto jointRegularizationGradient(iDynTree::toEigen(m_jointRegularizationGradient));
    auto jointRegularizationGains(iDynTree::toEigen(m_jointRegularizationGains));
    auto jointPosition(iDynTree::toEigen(m_jointPosition));
    auto regularizationTerm(iDynTree::toEigen(m_regularizationTerm));

    auto comJacobian(iDynTree::toEigen(m_comJacobian));
    auto comWeight(iDynTree::toEigen(m_comWeight));
    auto comPosition(iDynTree::toEigen(m_comPosition));
    auto desiredComPosition(iDynTree::toEigen(m_desiredComPosition));
    auto desiredComVelocity(iDynTree::toEigen(m_desiredComVelocity));

    // Neck orientation
    iDynTree::Matrix3x3 errorNeckAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_neckOrientation * m_desiredNeckOrientation.inverse());
    gradient = -neckJacobian.transpose() * m_neckWeight * (-m_kNeck
                                                           * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)));

    // if the hand retargeting is not enable the regularization term should be used as usual
    if(!m_enableHandRetargeting)
       gradient += -jointRegularizationGradient * jointRegularizationGains.asDiagonal()
           * (regularizationTerm - jointPosition);
    else
    {
        auto leftHandCorrectionLinear(iDynTree::toEigen(m_leftHandCorrection.getLinearVec3()));
        auto leftHandCorrectionAngular(iDynTree::toEigen(m_leftHandCorrection.getAngularVec3()));
        auto rightHandCorrectionLinear(iDynTree::toEigen(m_rightHandCorrection.getLinearVec3()));
        auto rightHandCorrectionAngular(iDynTree::toEigen(m_rightHandCorrection.getAngularVec3()));

        //  left hand
        leftHandCorrectionLinear = m_kPosHand * iDynTree::toEigen(m_leftHandToWorldTransform.getPosition()
                                                                  - m_desiredLeftHandToWorldTransform.getPosition());

        iDynTree::Matrix3x3 leftHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_leftHandToWorldTransform.getRotation() *
                                                                                            m_desiredLeftHandToWorldTransform.getRotation().inverse());

        leftHandCorrectionAngular = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(leftHandAttitudeError)));

        //  right hand
        rightHandCorrectionLinear = m_kPosHand * iDynTree::toEigen(m_rightHandToWorldTransform.getPosition()
                                                                   - m_desiredRightHandToWorldTransform.getPosition());

        iDynTree::Matrix3x3 rightHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_rightHandToWorldTransform.getRotation() *
                                                                                             m_desiredRightHandToWorldTransform.getRotation().inverse());

        rightHandCorrectionAngular = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(rightHandAttitudeError)));


        gradient +=
            -jointRegularizationGradient * jointRegularizationGains.asDiagonal()
            * (regularizationTerm - jointPosition)
            - iDynTree::toEigen(m_leftHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal() * (-iDynTree::toEigen(m_leftHandCorrection))
            - iDynTree::toEigen(m_rightHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightSmoother->getPos()).asDiagonal() * (-iDynTree::toEigen(m_rightHandCorrection));
    }

    if(!m_useCoMAsConstraint)
    {
        gradient += -comJacobian.transpose() * comWeight.asDiagonal() *
            (desiredComVelocity - m_kCom * (comPosition - desiredComPosition));
    }
}

void WalkingQPIK::evaluateLinearConstraintMatrix()
{
    copyDenseIntoSparse(m_leftFootJacobian, 0, 0, m_constraintsMatrixSparse);
    copyDenseIntoSparse(m_rightFootJacobian, 6, 0, m_constraintsMatrixSparse);

    if(m_useCoMAsConstraint)
        copyDenseIntoSparse(m_comJacobian, 6 + 6, 0, m_constraintsMatrixSparse);
}

void WalkingQPIK::evaluateBounds()
{
    auto lowerBound(iDynTree::toEigen(m_lowerBound));
    auto upperBound(iDynTree::toEigen(m_upperBound));
    auto leftFootCorrectionLinear(iDynTree::toEigen(m_leftFootCorrection.getLinearVec3()));
    auto leftFootCorrectionAngular(iDynTree::toEigen(m_leftFootCorrection.getAngularVec3()));
    auto rightFootCorrectionLinear(iDynTree::toEigen(m_rightFootCorrection.getLinearVec3()));
    auto rightFootCorrectionAngular(iDynTree::toEigen(m_rightFootCorrection.getAngularVec3()));

    // left foot
    leftFootCorrectionLinear = m_kPosFoot * iDynTree::toEigen((m_leftFootToWorldTransform.getPosition() -
                                                               m_desiredLeftFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorLeftAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_leftFootToWorldTransform.getRotation() *
                                                                                    m_desiredLeftFootToWorldTransform.getRotation().inverse());

    leftFootCorrectionAngular = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorLeftAttitude)));

    // right foot
    rightFootCorrectionLinear = m_kPosFoot * iDynTree::toEigen((m_rightFootToWorldTransform.getPosition() -
                                                                         m_desiredRightFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorRightAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_rightFootToWorldTransform.getRotation() *
                                                                                     m_desiredRightFootToWorldTransform.getRotation().inverse());

    rightFootCorrectionAngular = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorRightAttitude)));

    if((m_desiredLeftFootTwist(0) == m_desiredLeftFootTwist(1)) && (m_desiredLeftFootTwist(0) == 0))
    {
        lowerBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist);
        upperBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist);
    }
    else
    {
        lowerBound.segment<6>(0) = iDynTree::toEigen(m_desiredLeftFootTwist) - iDynTree::toEigen(m_leftFootCorrection);
        upperBound.segment<6>(0) = lowerBound.segment<6>(0);
    }

    if((m_desiredRightFootTwist(0) == m_desiredRightFootTwist(1)) && (m_desiredRightFootTwist(0) == 0))
    {
        lowerBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist);
        upperBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist);
    }
    else
    {
        lowerBound.segment<6>(6) = iDynTree::toEigen(m_desiredRightFootTwist) - iDynTree::toEigen(m_rightFootCorrection);
        upperBound.segment<6>(6) = lowerBound.segment<6>(6);
    }
    if(m_useCoMAsConstraint)
    {
        lowerBound.segment<3>(12) = iDynTree::toEigen(m_desiredComVelocity)
            - m_kCom * (iDynTree::toEigen(m_comPosition) -  iDynTree::toEigen(m_desiredComPosition));
        upperBound.segment<3>(12) = lowerBound.segment<3>(12);
    }

    setJointVelocitiesBounds();
}

const iDynTree::VectorDynSize& WalkingQPIK::getSolution() const
{
    return m_solution;
}

const iDynTree::VectorDynSize& WalkingQPIK::getDesiredJointVelocities() const
{
    return m_desiredJointVelocitiesOutput;
}

iDynTree::VectorDynSize WalkingQPIK::getLeftFootError()
{
    iDynTree::VectorDynSize output(6);
    iDynTree::toEigen(output) = iDynTree::toEigen(m_lowerBound).block(0, 0, 6, 1)
        - iDynTree::toEigen(m_leftFootJacobian) * iDynTree::toEigen(m_solution);
    return output;
}

iDynTree::VectorDynSize WalkingQPIK::getRightFootError()
{
    iDynTree::VectorDynSize output(6);
    iDynTree::toEigen(output) = iDynTree::toEigen(m_lowerBound).block(6, 0, 6, 1)
        - iDynTree::toEigen(m_rightFootJacobian) * iDynTree::toEigen(m_solution);
    return output;
}

const iDynTree::MatrixDynSize& WalkingQPIK::getHessianMatrix() const
{
    return m_hessianDense;
}

const iDynSparseMatrix& WalkingQPIK::getConstraintMatrix() const
{
    return m_constraintsMatrixSparse;
}

const iDynTree::VectorDynSize& WalkingQPIK::getUpperBound() const
{
    return m_upperBound;
}

const iDynTree::VectorDynSize& WalkingQPIK::getLowerBound() const
{
    return m_lowerBound;
}

const iDynTree::VectorDynSize& WalkingQPIK::getGradient() const
{
    return m_gradient;
}
