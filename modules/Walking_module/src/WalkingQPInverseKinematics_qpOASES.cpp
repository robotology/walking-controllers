/**
 * @file WalkingQPInverseKinematics_qpOASES.cpp
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
#include <iDynTree/Core/EigenSparseHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include "WalkingQPInverseKinematics_qpOASES.hpp"
#include "Utils.hpp"

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

bool WalkingQPIK_qpOASES::initializeMatrices(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    // evaluate constant sub-matrix of the hessian matrix
    // get the CoM weight
    if(!m_useCoMAsConstraint)
    {
        tempValue = config.find("comWeightTriplets");
        iDynTree::Triplets comWeightMatrix;
        if(!iDynTreeHelper::Triplets::getTripletsFromValues(tempValue, 3, comWeightMatrix))
        {
            yError() << "Initialization failed while reading comWeightTriplets vector.";
            return false;
        }

        m_comWeightMatrix.resize(3, 3);
        m_comWeightMatrix.setFromConstTriplets(comWeightMatrix);
    }

    if(m_useLeftHand || m_useRightHand)
    {
        tempValue = config.find("handWeightTriplets");
        iDynTree::Triplets handWeightMatrix;
        if(!iDynTreeHelper::Triplets::getTripletsFromValues(tempValue, 6, handWeightMatrix))
        {
            yError() << "Initialization failed while reading handWeightTriplets vector.";
            return false;
        }

        m_handWeightMatrix.resize(6, 6);
        m_handWeightMatrix.setFromConstTriplets(handWeightMatrix);

        if(!YarpHelper::getDoubleFromSearchable(config, "k_posHand", m_kPosHand))
        {
            yError() << "Initialization failed while reading k_posHand.";
            return false;
        }

        if(!YarpHelper::getDoubleFromSearchable(config, "k_attHand", m_kAttHand))
        {
            yError() << "Initialization failed while reading k_attHand.";
            return false;
        }
    }

    // get the CoM weight
    tempValue = config.find("neckWeightTriplets");
    iDynTree::Triplets neckWeightMatrix;
    if(!iDynTreeHelper::Triplets::getTripletsFromValues(tempValue, 3, neckWeightMatrix))
    {
        yError() << "Initialization failed while reading neckWeightTriplets vector.";
        return false;
    }
    m_neckWeightMatrix.resize(3, 3);
    m_neckWeightMatrix.setFromConstTriplets(neckWeightMatrix);

    // set the matrix related to the joint regularization
    tempValue = config.find("jointRegularizationWeights");
    iDynTree::VectorDynSize jointRegularizationWeights(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, jointRegularizationWeights))
    {
        yError() << "Initialization failed while reading neckWeightTriplets vector.";
        return false;
    }

    //  m_jointRegulatizationHessian = H' \lamda H
    m_jointRegulatizationHessian.resize(m_numberOfVariables, m_numberOfVariables);
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_jointRegulatizationHessian(i + 6, i + 6) = jointRegularizationWeights(i);

    // evaluate constant sub-matrix of the gradient matrix
    m_jointRegulatizationGradient.resize(m_numberOfVariables, m_actuatedDOFs);
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_jointRegulatizationGradient(i + 6, i) = jointRegularizationWeights(i);

    // resize matrices
    m_comJacobian.resize(3, m_numberOfVariables);
    m_neckJacobian.resize(3, m_numberOfVariables);
    m_leftFootJacobian.resize(6, m_numberOfVariables);
    m_rightFootJacobian.resize(6, m_numberOfVariables);
    m_leftHandJacobian.resize(6, m_numberOfVariables);
    m_rightHandJacobian.resize(6, m_numberOfVariables);

    tempValue = config.find("jointRegularizationGains");
    iDynTree::VectorDynSize jointRegularizationGains(m_actuatedDOFs);
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, jointRegularizationGains))
    {
        yError() << "Initialization failed while reading neckWeightTriplets vector.";
        return false;
    }
    m_jointRegulatizationGains.resize(m_actuatedDOFs, m_actuatedDOFs);
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_jointRegulatizationGains(i, i) = jointRegularizationGains(i);

    if(!YarpHelper::getDoubleFromSearchable(config, "k_posFoot", m_kPosFoot))
    {
        yError() << "Initialization failed while reading k_posFoot.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "k_intPosFoot", m_kIPosFoot))
    {
        yError() << "Initialization failed while reading k_intPosFoot.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "k_attFoot", m_kAttFoot))
    {
        yError() << "Initialization failed while reading k_attFoot.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "k_neck", m_kNeck))
    {
        yError() << "Initialization failed while reading k_neck.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "k_posCom", m_kCom))
    {
        yError() << "Initialization failed while reading k_posCom.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "k_intPosCom", m_kICom))
    {
        yError() << "Initialization failed while reading k_intPosCom.";
        return false;
    }

    // the first six values are related to the velocity (linear and angular) of the base
    for(int i = 0; i < 6; i++)
    {
        m_minJointLimit[i] = -std::numeric_limits<double>::max();
        m_maxJointLimit[i] = std::numeric_limits<double>::max();
    }

    return true;
}

bool WalkingQPIK_qpOASES::setJointBounds(const iDynTree::VectorDynSize& minJointsPosition,
                                         const iDynTree::VectorDynSize& maxJointsPosition,
                                         const iDynTree::VectorDynSize& minJointsVelocity,
                                         const iDynTree::VectorDynSize& maxJointsVelocity)
{
    if(minJointsPosition.size() != maxJointsPosition.size() ||
       minJointsVelocity.size() != maxJointsVelocity.size() ||
       minJointsVelocity.size() != maxJointsPosition.size())
    {
        yError() << "[setJointBounds] The size of the vector limits has to be equal.";
        return false;
    }
    if(minJointsVelocity.size() != m_actuatedDOFs)
    {
        yError() << "[setJointBounds] The size of the vector limits has to be equal to ."
                 << "the number of the joint";
        return false;
    }

    Eigen::Map<MatrixXd>(m_minJointPosition.data(), m_actuatedDOFs, 1) = iDynTree::toEigen(minJointsPosition);
    Eigen::Map<MatrixXd>(m_maxJointPosition.data(), m_actuatedDOFs, 1) = iDynTree::toEigen(maxJointsPosition);

    Eigen::Map<MatrixXd>(m_minJointVelocity.data(), m_actuatedDOFs, 1) = iDynTree::toEigen(minJointsVelocity);
    Eigen::Map<MatrixXd>(m_maxJointVelocity.data(), m_actuatedDOFs, 1) = iDynTree::toEigen(maxJointsVelocity);

    return true;
}

bool WalkingQPIK_qpOASES::initialize(const yarp::os::Searchable& config,
                                     const int& actuatedDOFs,
                                     const iDynTree::VectorDynSize& minJointsPosition,
                                     const iDynTree::VectorDynSize& maxJointsPosition,
                                     const iDynTree::VectorDynSize& minJointsVelocity,
                                     const iDynTree::VectorDynSize& maxJointsVelocity)
{
    m_actuatedDOFs = actuatedDOFs;
    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for QP-IK solver.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "sampling_time", m_dT))
    {
        yError() << "Initialization failed while reading sampling_time.";
        return false;
    }

    m_useCoMAsConstraint = config.check("useCoMAsConstraint", yarp::os::Value(false)).asBool();
    m_useHandsAsConstraint = config.check("useHandsAsConstraint", yarp::os::Value(false)).asBool();
    m_useLeftHand = config.check("use_left_hand", yarp::os::Value(false)).asBool();
    m_useRightHand = config.check("use_right_hand", yarp::os::Value(false)).asBool();

    // TODO in the future the number of constraints should be added inside
    // the configuration file
    // set the number of variables and the number of constraints
    // the number of variables is equal to the number of joints plus
    // 6 (position + attitude) degree of freedom related to the base.
    m_numberOfVariables = m_actuatedDOFs + 6;

    // the number of constraints is equal to the number of joints plus
    // 12 (position + attitude) of the left and right feet
    m_numberOfConstraints = 6 + 6;

    if(m_useCoMAsConstraint)
        m_numberOfConstraints += 3;

    if(m_useHandsAsConstraint)
    {
        if(m_useLeftHand)
            m_numberOfConstraints += 6;
        if(m_useRightHand)
            m_numberOfConstraints += 6;
    }

    // resize all vectors (matrices)
    m_hessian.resize(m_numberOfVariables * m_numberOfVariables);
    m_gradient.resize(m_numberOfVariables);
    m_constraintMatrix.resize(m_numberOfVariables * m_numberOfConstraints);
    m_upperBound.resize(m_numberOfConstraints);
    m_lowerBound.resize(m_numberOfConstraints);

    m_minJointLimit.resize(m_numberOfVariables);
    m_maxJointLimit.resize(m_numberOfVariables);

    m_minJointPosition.resize(m_actuatedDOFs);
    m_maxJointPosition.resize(m_actuatedDOFs);

    m_minJointVelocity.resize(m_actuatedDOFs);
    m_maxJointVelocity.resize(m_actuatedDOFs);

    m_regularizationTerm.resize(m_actuatedDOFs);
    m_jointPosition.resize(m_actuatedDOFs);

    // get the regularization term
    yarp::os::Value jointRegularization = config.find("jointRegularization");
    if(!YarpHelper::yarpListToiDynTreeVectorDynSize(jointRegularization, m_regularizationTerm))
    {
        yError() << "[initialize] Unable to convert a YARP list to an iDynTree::VectorDynSize, "
                 << "joint regularization";
        return false;
    }

    iDynTree::toEigen(m_regularizationTerm) = iDynTree::toEigen(m_regularizationTerm) *
        iDynTree::deg2rad(1);

    // preprare constant matrix necessary for the QP problem
    if(!initializeMatrices(config))
    {
        yError() << "[initialize] Unable to Initialize the constant matrix.";
        return false;
    }

    if(!setJointBounds(minJointsPosition, maxJointsPosition,
                       minJointsVelocity, maxJointsVelocity))
    {
        yError() << "[initialize] Unable to set the joint bounds.";
        return false;
    }

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[initialize] Unable to set the additional rotation.";
        return false;
    }

    // set integral
    double samplingTime;
    if(!YarpHelper::getDoubleFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(m_useHandsAsConstraint)
        if(!YarpHelper::getDoubleFromSearchable(config, "handTolerance", m_handTolerance))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

    yarp::sig::Vector buffer(3, 0.0);
    m_leftFootErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);
    m_rightFootErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);
    m_comErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    m_optimizer = std::make_shared<qpOASES::SQProblem>(m_numberOfVariables,
                                                       m_numberOfConstraints);

    m_optimizer->setPrintLevel(qpOASES::PL_LOW);

    m_isFirstTime = true;
    return true;
}

void WalkingQPIK_qpOASES::setHandsState(const iDynTree::Transform& leftHandToWorldTransform,
                                        const iDynTree::Transform& rightHandToWorldTransform)
{
    // in the future we should save only the measured hand
    // transformation only for the left or right
    m_leftHandToWorldTransform = leftHandToWorldTransform;
    m_rightHandToWorldTransform = rightHandToWorldTransform;
}

bool WalkingQPIK_qpOASES::setRobotState(const iDynTree::VectorDynSize& jointPosition,
                                        const iDynTree::Transform& leftFootToWorldTransform,
                                        const iDynTree::Transform& rightFootToWorldTransform,
                                        const iDynTree::Rotation& neckOrientation,
                                        const iDynTree::Position& comPosition)
{
    if(jointPosition.size() != m_actuatedDOFs)
    {
        yError() << "[setRobotState] The size of the jointPosition vector is not coherent with the "
                 << "number of the actuated Joint";
        return false;
    }

    m_jointPosition = jointPosition;

    m_leftFootToWorldTransform = leftFootToWorldTransform;
    m_rightFootToWorldTransform = rightFootToWorldTransform;
    m_neckOrientation = neckOrientation;
    m_comPosition = comPosition;

    return true;
}

void WalkingQPIK_qpOASES::setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation)
{
    m_desiredNeckOrientation =  desiredNeckOrientation * m_additionalRotation;
}

bool WalkingQPIK_qpOASES::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
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

bool WalkingQPIK_qpOASES::setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian)
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

bool WalkingQPIK_qpOASES::setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian)
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

bool WalkingQPIK_qpOASES::setLeftHandJacobian(const iDynTree::MatrixDynSize& leftHandJacobian)
{
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

bool WalkingQPIK_qpOASES::setRightHandJacobian(const iDynTree::MatrixDynSize& rightHandJacobian)
{
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

bool WalkingQPIK_qpOASES::setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian)
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

void WalkingQPIK_qpOASES::setJointLimits()
{
    double maxPos, minPos;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        minPos = (m_minJointPosition[i] - m_jointPosition(i)) / m_dT;
        maxPos = (m_maxJointPosition[i] - m_jointPosition(i)) / m_dT;

        if(minPos > m_minJointVelocity[i])
            m_minJointLimit[i + 6] = minPos;
        else
            m_minJointLimit[i + 6] = m_minJointVelocity[i];

        if(maxPos < m_maxJointVelocity[i])
            m_maxJointLimit[i + 6] = maxPos;
        else
            m_maxJointLimit[i + 6] = m_maxJointVelocity[i];
    }
}

bool WalkingQPIK_qpOASES::setHessianMatrix()
{
    // evaluate the hessian matrix
    Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) =
        MatrixXd(iDynTree::toEigen(m_jointRegulatizationHessian)) +
        iDynTree::toEigen(m_neckJacobian).transpose() *
        iDynTree::toEigen(m_neckWeightMatrix) *
        iDynTree::toEigen(m_neckJacobian);

    if(!m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) =
            Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) +
            iDynTree::toEigen(m_comJacobian).transpose() *
            iDynTree::toEigen(m_comWeightMatrix) * iDynTree::toEigen(m_comJacobian);
    }

    if(!m_useHandsAsConstraint)
    {
        if(m_useLeftHand)
        {
            Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) =
                Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) +
                iDynTree::toEigen(m_leftHandJacobian).transpose() *
                iDynTree::toEigen(m_handWeightMatrix) * iDynTree::toEigen(m_leftHandJacobian);
        }

        if(m_useRightHand)
        {
            Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) =
                Eigen::Map<MatrixXd>(m_hessian.data(), m_numberOfVariables, m_numberOfVariables) +
                iDynTree::toEigen(m_rightHandJacobian).transpose() *
                iDynTree::toEigen(m_handWeightMatrix) * iDynTree::toEigen(m_rightHandJacobian);
        }
    }

    return true;
}

bool WalkingQPIK_qpOASES::setGradientVector()
{
    iDynTree::Matrix3x3 errorNeckAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_neckOrientation * m_desiredNeckOrientation.inverse());

    Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
        -iDynTree::toEigen(m_neckJacobian).transpose()
        * iDynTree::toEigen(m_neckWeightMatrix)
        * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)))
        - iDynTree::toEigen(m_jointRegulatizationGradient) *
        (iDynTree::toEigen(m_jointRegulatizationGains) * (iDynTree::toEigen(m_regularizationTerm)
                                                          - iDynTree::toEigen(m_jointPosition)));

    if(!m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
            Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1)
            - iDynTree::toEigen(m_comJacobian).transpose()
            * iDynTree::toEigen(m_comWeightMatrix) * iDynTree::toEigen(m_comVelocity);
    }
    if(!m_useHandsAsConstraint)
    {
        if(m_useLeftHand)
        {
            //  left hand
            Eigen::VectorXd leftHandCorrection(6);
            iDynTree::Position leftHandPositionError = m_leftHandToWorldTransform.getPosition()
                - m_desiredLeftHandToWorldTransform.getPosition();
            leftHandCorrection.block(0,0,3,1) = m_kPosHand * iDynTree::toEigen(leftHandPositionError);

            auto tmp = m_leftHandToWorldTransform.getRotation() *
                m_desiredLeftHandToWorldTransform.getRotation().inverse();
            yInfo() << "hand error left" << leftHandPositionError.toString() <<" " << tmp.asRPY().toString();
            yInfo() << "desired left" << m_desiredLeftHandToWorldTransform.getRotation().asRPY().toString();

            iDynTree::Matrix3x3 leftHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_leftHandToWorldTransform.getRotation() *
                                                                                                m_desiredLeftHandToWorldTransform.getRotation().inverse());

            leftHandCorrection.block(3,0,3,1) = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(leftHandAttitudeError)));

            Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
                Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1)
                - iDynTree::toEigen(m_leftHandJacobian).transpose()
                * iDynTree::toEigen(m_handWeightMatrix) * (-leftHandCorrection);
        }

        if(m_useRightHand)
        {
            //  right hand
            Eigen::VectorXd rightHandCorrection(6);
            iDynTree::Position rightHandPositionError = m_rightHandToWorldTransform.getPosition()
                - m_desiredRightHandToWorldTransform.getPosition();

            auto tmp = m_rightHandToWorldTransform.getRotation() *
                m_desiredRightHandToWorldTransform.getRotation().inverse();

            yInfo() << "hand error right" << rightHandPositionError.toString() <<" " << tmp.asRPY().toString();
            yInfo() << "desired right" << m_desiredRightHandToWorldTransform.getRotation().asRPY().toString();

            rightHandCorrection.block(0,0,3,1) = m_kPosHand * iDynTree::toEigen(rightHandPositionError);

            iDynTree::Matrix3x3 rightHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_rightHandToWorldTransform.getRotation() *
                                                                                                 m_desiredRightHandToWorldTransform.getRotation().inverse());

            rightHandCorrection.block(3,0,3,1) = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(rightHandAttitudeError)));

            Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
                Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1)
                - iDynTree::toEigen(m_rightHandJacobian).transpose()
                * iDynTree::toEigen(m_handWeightMatrix) * (-rightHandCorrection);
        }
    }

    return true;
}

bool WalkingQPIK_qpOASES::setLinearConstraintMatrix()
{
    int subMatrixRowPosition = 0;
    // add left foot
    Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                         m_numberOfConstraints,
                         m_numberOfVariables).block(subMatrixRowPosition, 0, m_leftFootJacobian.rows(),
                                                    m_leftFootJacobian.cols())=
        iDynTree::toEigen(m_leftFootJacobian);
    subMatrixRowPosition += m_leftFootJacobian.rows();

    // add right foot
    Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                          m_numberOfConstraints,
                          m_numberOfVariables).block(subMatrixRowPosition,
                                                     0, m_rightFootJacobian.rows(),
                                                     m_rightFootJacobian.cols())=
        iDynTree::toEigen(m_rightFootJacobian);
    subMatrixRowPosition += m_rightFootJacobian.rows();

    // add com as constraint
    if(m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                             m_numberOfConstraints,
                             m_numberOfVariables).block(subMatrixRowPosition,
                                                        0, m_comJacobian.rows(),
                                                        m_comJacobian.cols())=
            iDynTree::toEigen(m_comJacobian);
        subMatrixRowPosition += m_comJacobian.rows();
    }

    if(m_useHandsAsConstraint)
    {
        if(m_useLeftHand)
        {
            Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                                 m_numberOfConstraints,
                                 m_numberOfVariables).block(subMatrixRowPosition,
                                                            0, m_leftHandJacobian.rows(),
                                                            m_leftHandJacobian.cols())=
                iDynTree::toEigen(m_leftHandJacobian);
            subMatrixRowPosition += m_leftHandJacobian.rows();
        }

        if(m_useRightHand)
        {
            Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                                 m_numberOfConstraints,
                                 m_numberOfVariables).block(subMatrixRowPosition,
                                                            0, m_rightHandJacobian.rows(),
                                                            m_rightHandJacobian.cols())=
                iDynTree::toEigen(m_rightHandJacobian);
            subMatrixRowPosition += m_rightHandJacobian.rows();
        }
    }

    return true;
}

iDynTree::Position WalkingQPIK_qpOASES::evaluateIntegralError(std::unique_ptr<iCub::ctrl::Integrator>& integral,
                                                              const iDynTree::Position& error)
{
    yarp::sig::Vector buffer(error.size());
    yarp::sig::Vector integralErrorYarp(error.size());
    iDynTree::toYarp(error, buffer);
    integralErrorYarp = integral->integrate(buffer);

    // transform into iDynTree Vector
    iDynTree::Position integralError;
    iDynTree::toiDynTree(integralErrorYarp, integralError);

    return integralError;
}

bool WalkingQPIK_qpOASES::setBounds()
{
    int subVectorPosition = 0;
    //  left foot
    Eigen::VectorXd leftFootCorrection(6);
    iDynTree::Position leftFootError = m_leftFootToWorldTransform.getPosition()
        - m_desiredLeftFootToWorldTransform.getPosition();
    leftFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen(leftFootError)
        + m_kIPosFoot * iDynTree::toEigen(evaluateIntegralError(m_leftFootErrorIntegral, leftFootError));

    iDynTree::Matrix3x3 errorLeftAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_leftFootToWorldTransform.getRotation() *
                                                                                    m_desiredLeftFootToWorldTransform.getRotation().inverse());

    leftFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorLeftAttitude)));

    Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
      - leftFootCorrection;
    Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
      - leftFootCorrection;

    subVectorPosition += leftFootCorrection.size();

    // right foot
    Eigen::VectorXd rightFootCorrection(6);
    iDynTree::Position rightFootError = m_rightFootToWorldTransform.getPosition()
        - m_desiredRightFootToWorldTransform.getPosition();

    rightFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen(rightFootError)
        + m_kIPosFoot * iDynTree::toEigen(evaluateIntegralError(m_rightFootErrorIntegral, rightFootError));

    iDynTree::Matrix3x3 errorRightAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_rightFootToWorldTransform.getRotation() *
                                                                                     m_desiredRightFootToWorldTransform.getRotation().inverse());

    rightFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorRightAttitude)));

    Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
      - rightFootCorrection;
    Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
      - rightFootCorrection;

    subVectorPosition += rightFootCorrection.rows();

    if(m_useCoMAsConstraint)
    {
        Eigen::VectorXd comCorrection(3);
        iDynTree::Position comPositionError = m_desiredComPosition - m_comPosition;

        comCorrection = m_kCom * iDynTree::toEigen(comPositionError) +
            m_kICom * iDynTree::toEigen(evaluateIntegralError(m_comErrorIntegral, comPositionError));

        Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 3, 1) = iDynTree::toEigen(m_comVelocity)
            + comCorrection;
        Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 3, 1) = iDynTree::toEigen(m_comVelocity)
            + comCorrection;

        subVectorPosition += comCorrection.rows();
    }

    if(m_useHandsAsConstraint)
    {
        if(m_useLeftHand)
        {
            //  left hand
            Eigen::VectorXd leftHandCorrection(6);

            // linear position error
            iDynTree::Position leftHandPositionError = m_leftHandToWorldTransform.getPosition()
                - m_desiredLeftHandToWorldTransform.getPosition();
            leftHandCorrection.block(0,0,3,1) = m_kPosHand * iDynTree::toEigen(leftHandPositionError);

            // angular position error
            iDynTree::Matrix3x3 leftHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_leftHandToWorldTransform.getRotation() *
                                                                                                m_desiredLeftHandToWorldTransform.getRotation().inverse());
            leftHandCorrection.block(3,0,3,1) = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(leftHandAttitudeError)));

            //
            Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) =
                - leftHandCorrection - Eigen::VectorXd::Ones(6) * m_handTolerance;
            Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) =
                - leftHandCorrection + Eigen::VectorXd::Ones(6) * m_handTolerance;

            subVectorPosition += leftHandCorrection.rows();
        }

        if(m_useRightHand)
        {
            //  right hand
            Eigen::VectorXd rightHandCorrection(6);

            iDynTree::Position rightHandPositionError = m_rightHandToWorldTransform.getPosition()
                - m_desiredRightHandToWorldTransform.getPosition();
            rightHandCorrection.block(0,0,3,1) = m_kPosHand * iDynTree::toEigen(rightHandPositionError);

            iDynTree::Matrix3x3 rightHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_rightHandToWorldTransform.getRotation() *
                                                                                                 m_desiredRightHandToWorldTransform.getRotation().inverse());
            rightHandCorrection.block(3,0,3,1) = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(rightHandAttitudeError)));

            Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) =
                - rightHandCorrection - Eigen::VectorXd::Ones(6) * m_handTolerance;
            Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(subVectorPosition, 0, 6, 1) =
                - rightHandCorrection + Eigen::VectorXd::Ones(6) * m_handTolerance;

            subVectorPosition += rightHandCorrection.rows();
        }
    }
    return true;
}

bool WalkingQPIK_qpOASES::setDesiredJointPosition(const iDynTree::VectorDynSize& regularizationTerm)
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

void WalkingQPIK_qpOASES::setDesiredFeetTransformation(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                                       const iDynTree::Transform& desiredRightFootToWorldTransform)
{
    m_desiredLeftFootToWorldTransform = desiredLeftFootToWorldTransform;
    m_desiredRightFootToWorldTransform = desiredRightFootToWorldTransform;
}

void WalkingQPIK_qpOASES::setDesiredFeetTwist(const iDynTree::Twist& leftFootTwist,
                                              const iDynTree::Twist& rightFootTwist)
{
    m_leftFootTwist = leftFootTwist;
    m_rightFootTwist = rightFootTwist;
}

void WalkingQPIK_qpOASES::setDesiredLeftHandTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform)
{
    m_desiredLeftHandToWorldTransform = desiredLeftHandToWorldTransform;
}

void WalkingQPIK_qpOASES::setDesiredRightHandTransformation(const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    m_desiredRightHandToWorldTransform = desiredRightHandToWorldTransform;
}

// in the future they will be iDynTree::Transform
void WalkingQPIK_qpOASES::setDesiredHandsTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform,
                                                        const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    setDesiredRightHandTransformation(desiredRightHandToWorldTransform);
    setDesiredLeftHandTransformation(desiredLeftHandToWorldTransform);
}

void WalkingQPIK_qpOASES::setDesiredCoMVelocity(const iDynTree::Vector3& comVelocity)
{
    m_comVelocity = comVelocity;
}

void WalkingQPIK_qpOASES::setDesiredCoMPosition(const iDynTree::Position& desiredComPosition)
{
    m_desiredComPosition = desiredComPosition;
}

bool WalkingQPIK_qpOASES::solve()
{
    m_isSolutionEvaluated = false;

    if(!setHessianMatrix())
    {
        yError() << "[solve] Unable to set the hessian matrix.";
        return false;
    }

    if(!setGradientVector())
    {
        yError() << "[solve] Unable to set the gradient vector matrix.";
        return false;
    }

    if(!setLinearConstraintMatrix())
    {
        yError() << "[solve] Unable to set the linear constraint matrix.";
        return false;
    }

    if(!setBounds())
    {
        yError() << "[solve] Unable to set the bounds.";
        return false;
    }

    // the joint limits are set tacking into account the position and velocity limits
    setJointLimits();

    int nWSR = 100;

    if(!m_isFirstTime)
    {
        if(m_optimizer->hotstart(m_hessian.data(), m_gradient.data(), m_constraintMatrix.data(),
                                 m_minJointLimit.data(), m_maxJointLimit.data(),
                                 m_upperBound.data(), m_lowerBound.data(), nWSR, 0) != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }
    }
    else
    {
        if(m_optimizer->init(m_hessian.data(), m_gradient.data(), m_constraintMatrix.data(),
                          m_minJointLimit.data(), m_maxJointLimit.data(),
                          m_upperBound.data(), m_lowerBound.data(), nWSR, 0) != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }

        m_isFirstTime = false;
    }
    m_isSolutionEvaluated = true;

    return true;
}

bool WalkingQPIK_qpOASES::getSolution(iDynTree::VectorDynSize& output)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[getSolution] The solution is not evaluated. "
                 << "Please call 'solve()' method.";
        return false;
    }

    if(output.size() != m_actuatedDOFs)
        output.resize(m_actuatedDOFs);

    std::vector<double> result(m_numberOfVariables);

    m_optimizer->getPrimalSolution(result.data());

    for(int i = 0; i < output.size(); i++)
        output(i) = result[i + 6];

    m_isSolutionEvaluated = false;
    return true;
}

bool WalkingQPIK_qpOASES::getLeftFootError(iDynTree::VectorDynSize& output)
{
    // if(!m_isSolutionEvaluated)
    // {
    //     yError() << "[getLeftFootError] The solution is not evaluated. "
    //              << "Please call 'solve()' method.";
    //     return false;
    // }

    std::vector<double> result(m_numberOfVariables);
    m_optimizer->getPrimalSolution(result.data());

    iDynTree::toEigen(output) = Eigen::Map<Eigen::MatrixXd>(m_lowerBound.data(),
                                                            m_numberOfConstraints, 1).block(0, 0, 6, 1)
        - iDynTree::toEigen(m_leftFootJacobian) * Eigen::Map<Eigen::MatrixXd>(result.data(),
                                                                              m_numberOfVariables,
                                                                              1);
    return true;
}

bool WalkingQPIK_qpOASES::getRightFootError(iDynTree::VectorDynSize& output)
{
    // if(!m_isSolutionEvaluated)
    // {
    //     yError() << "[getRightFootError] The solution is not evaluated. "
    //              << "Please call 'solve()' method.";
    //     return false;
    // }
    std::vector<double> result(m_numberOfVariables);
    m_optimizer->getPrimalSolution(result.data());

    iDynTree::toEigen(output) = Eigen::Map<Eigen::MatrixXd>(m_lowerBound.data(),
                                                            m_numberOfConstraints, 1).block(6, 0, 6, 1)
        - iDynTree::toEigen(m_rightFootJacobian) * Eigen::Map<Eigen::MatrixXd>(result.data(),
                                                                               m_numberOfVariables,
                                                                               1);
    return true;
}


bool WalkingQPIK_qpOASES::getNeckOrientationError(iDynTree::Vector3& output)
{
    // if(!m_isSolutionEvaluated)
    // {
    //     yError() << "[getRightFootError] The solution is not evaluated. "
    //              << "Please call 'solve()' method.";
    //     return false;
    // }

    auto error = m_neckOrientation * m_desiredNeckOrientation.inverse();
    output = error.asRPY();

    return true;
}

bool WalkingQPIK_qpOASES::getDesiredNeckOrientation(iDynTree::Vector3& output)
{
    // if(!m_isSolutionEvaluated)
    // {
    //     yError() << "[getRightFootError] The solution is not evaluated. "
    //              << "Please call 'solve()' method.";
    //     return false;
    // }

    auto error =  m_desiredNeckOrientation;
    output = error.asRPY();

    return true;
}
