/**
 * @file WalkingQPInverseKinematics_osqp.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
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
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Model/Model.h>
#include "iDynTree/yarp/YARPConfigurationsLoader.h"

#include "WalkingQPInverseKinematics_osqp.hpp"
#include "Utils.hpp"

bool WalkingQPIK_osqp::initializeMatrices(const yarp::os::Searchable& config)
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
        yError() << "Initialization failed while reading jointRegularizationWeights vector.";
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
        yError() << "Initialization failed while reading jointRegularizationGains vector.";
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

    return true;
}

bool WalkingQPIK_osqp::setJointBounds(const iDynTree::VectorDynSize& minJointsPosition,
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

    m_minJointsPosition = minJointsPosition;
    m_maxJointsPosition = maxJointsPosition;
    m_minJointsVelocity = minJointsVelocity;
    m_maxJointsVelocity = maxJointsVelocity;

    return true;
}

bool WalkingQPIK_osqp::initialize(const yarp::os::Searchable& config,
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
        yError() << "[initialize] Empty configuration for QP-IKg solver.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "sampling_time", m_dT))
    {
        yError() << "Initialization failed while reading sampling_time.";
        return false;
    }

    m_useCoMAsConstraint = config.check("useCoMAsConstraint", yarp::os::Value(false)).asBool();
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
    m_numberOfConstraints = m_actuatedDOFs + 6 + 6;

    if(m_useCoMAsConstraint)
        m_numberOfConstraints += 3;

    // resize vectors
    m_gradient = Eigen::VectorXd::Zero(m_numberOfVariables);
    m_lowerBound = Eigen::VectorXd::Zero(m_numberOfConstraints);
    m_upperBound = Eigen::VectorXd::Zero(m_numberOfConstraints);

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
    yarp::sig::Vector buffer(3, 0.0);
    m_leftFootErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);
    m_rightFootErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);
    m_comErrorIntegral = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);

    // instantiate the solver
    m_optimizerSolver = std::make_unique<OsqpEigen::Solver>();
    m_optimizerSolver->data()->setNumberOfVariables(m_numberOfVariables);
    m_optimizerSolver->data()->setNumberOfConstraints(m_numberOfConstraints);

    m_optimizerSolver->settings()->setVerbosity(false);
    m_optimizerSolver->settings()->setLinearSystemSolver(0);

    return true;
}

void WalkingQPIK_osqp::setHandsState(const iDynTree::Transform& leftHandToWorldTransform,
                                     const iDynTree::Transform& rightHandToWorldTransform)
{
    // in the future we should save only the measured hand
    // transformation only for the left or right
    m_leftHandToWorldTransform = leftHandToWorldTransform;
    m_rightHandToWorldTransform = rightHandToWorldTransform;
}


bool WalkingQPIK_osqp::setRobotState(const iDynTree::VectorDynSize& jointPosition,
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

void WalkingQPIK_osqp::setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation)
{
    m_desiredNeckOrientation =  desiredNeckOrientation * m_additionalRotation;
}

bool WalkingQPIK_osqp::setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian)
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

bool WalkingQPIK_osqp::setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian)
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

bool WalkingQPIK_osqp::setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian)
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

bool WalkingQPIK_osqp::setLeftHandJacobian(const iDynTree::MatrixDynSize& leftHandJacobian)
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

bool WalkingQPIK_osqp::setRightHandJacobian(const iDynTree::MatrixDynSize& rightHandJacobian)
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

bool WalkingQPIK_osqp::setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian)
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

void WalkingQPIK_osqp::setJointLimits()
{
    int numberOfTaskConstraints = 6 + 6;
    double maxPos, minPos;

    if(m_useCoMAsConstraint)
        numberOfTaskConstraints += 3;

    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        minPos = (m_minJointsPosition(i) - m_jointPosition(i)) / m_dT;
        maxPos = (m_maxJointsPosition(i) - m_jointPosition(i)) / m_dT;

        if(minPos > m_minJointsVelocity(i))
            m_lowerBound(i + numberOfTaskConstraints) = minPos;
        else
            m_lowerBound(i + numberOfTaskConstraints) = m_minJointsVelocity(i);

        if(maxPos < m_maxJointsVelocity(i))
            m_upperBound(i + numberOfTaskConstraints) = maxPos;
        else
            m_upperBound(i + numberOfTaskConstraints) = m_maxJointsVelocity(i);
    }
}

bool WalkingQPIK_osqp::setHessianMatrix()
{
    // evaluate the hessian matrix
    Eigen::SparseMatrix<double> hessianEigenSparse;

    // in that case the hessian matrix is related only to neck orientation and to
    // the joint angle
    m_hessianEigenDense = Eigen::MatrixXd(iDynTree::toEigen(m_jointRegulatizationHessian)) +
        iDynTree::toEigen(m_neckJacobian).transpose() *
        iDynTree::toEigen(m_neckWeightMatrix) *
        iDynTree::toEigen(m_neckJacobian);

    if(!m_useCoMAsConstraint)
    {
        m_hessianEigenDense =  m_hessianEigenDense + iDynTree::toEigen(m_comJacobian).transpose() *
            iDynTree::toEigen(m_comWeightMatrix) * iDynTree::toEigen(m_comJacobian);
    }

    if(m_useLeftHand)
    {
        m_hessianEigenDense =  m_hessianEigenDense +
            iDynTree::toEigen(m_leftHandJacobian).transpose() *
            iDynTree::toEigen(m_handWeightMatrix) * iDynTree::toEigen(m_leftHandJacobian);
    }

    if(m_useRightHand)
    {
        m_hessianEigenDense =  m_hessianEigenDense +
            iDynTree::toEigen(m_rightHandJacobian).transpose() *
            iDynTree::toEigen(m_handWeightMatrix) * iDynTree::toEigen(m_rightHandJacobian);
    }

    Eigen::SparseMatrix<double> hessianEigen = m_hessianEigenDense.sparseView();

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to update the hessian matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setHessianMatrix(hessianEigen))
        {
            yError() << "[setHessianMatrix] Unable to set first time the hessian matrix.";
            return false;
        }
    }
    return true;
}

bool WalkingQPIK_osqp::setGradientVector()
{
    iDynTree::Matrix3x3 errorNeckAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_neckOrientation * m_desiredNeckOrientation.inverse());

    m_gradient = -iDynTree::toEigen(m_neckJacobian).transpose()
        * iDynTree::toEigen(m_neckWeightMatrix) *
        m_kAttFoot * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)))
        - iDynTree::toEigen(m_jointRegulatizationGradient) *
        (iDynTree::toEigen(m_jointRegulatizationGains) * (iDynTree::toEigen(m_regularizationTerm)
                                                          - iDynTree::toEigen(m_jointPosition)));

    // todo
    if(!m_useCoMAsConstraint)
    {
        m_gradient = m_gradient -iDynTree::toEigen(m_comJacobian).transpose()
            * iDynTree::toEigen(m_comWeightMatrix) * iDynTree::toEigen(m_comVelocity);
    }

    if(m_useLeftHand)
    {
        //  left hand
        Eigen::VectorXd leftHandCorrection(6);
        iDynTree::Position leftHandPositionError = m_leftHandToWorldTransform.getPosition()
            - m_desiredLeftHandToWorldTransform.getPosition();
        leftHandCorrection.block(0,0,3,1) = m_kPosHand * iDynTree::toEigen(leftHandPositionError);

        iDynTree::Matrix3x3 leftHandAttitudeError = iDynTreeHelper::Rotation::skewSymmetric(m_leftHandToWorldTransform.getRotation() *
                                                                                            m_desiredLeftHandToWorldTransform.getRotation().inverse());

        leftHandCorrection.block(3,0,3,1) = m_kAttHand * (iDynTree::unskew(iDynTree::toEigen(leftHandAttitudeError)));

        m_gradient = m_gradient - iDynTree::toEigen(m_leftHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightMatrix) * (-leftHandCorrection);
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

        m_gradient = m_gradient - iDynTree::toEigen(m_rightHandJacobian).transpose()
            * iDynTree::toEigen(m_handWeightMatrix) * (-rightHandCorrection);
    }


    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to update the gradient.";
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setGradient(m_gradient))
        {
            yError() << "[setGradient] Unable to set first time the gradient.";
            return false;
        }
    }
    return true;
}

bool WalkingQPIK_osqp::setLinearConstraintMatrix()
{
    iDynTree::Triplets constraintsTriplets;
    constraintsTriplets.addSubMatrix(0, 0, m_leftFootJacobian);
    constraintsTriplets.addSubMatrix(6, 0, m_rightFootJacobian);

    if(m_useCoMAsConstraint)
    {
        constraintsTriplets.addSubMatrix(12, 0, m_comJacobian);
        iDynTreeHelper::Triplets::pushTripletsAsSubMatrix(15, 0,
                                                          m_jointRegularizationLinearConstraintTriplets,
                                                          constraintsTriplets);
    }
    else
    {
        iDynTreeHelper::Triplets::pushTripletsAsSubMatrix(12, 0,
                                                          m_jointRegularizationLinearConstraintTriplets,
                                                          constraintsTriplets);
    }

    iDynSparseMatrix constraintsMatrix(m_numberOfConstraints, m_numberOfVariables);
    constraintsMatrix.setFromConstTriplets(constraintsTriplets);

    // convert iDynTree sparse matrix in eigen sparse matrix
    // it is required by the osqp library
    Eigen::SparseMatrix<double> constraintsMatrixEigen = iDynTree::toEigen(constraintsMatrix);

    m_constraintsMatrixEigenDense = Eigen::MatrixXd(constraintsMatrixEigen);

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateLinearConstraintsMatrix(constraintsMatrixEigen))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to update the constraints matrix.";
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setLinearConstraintsMatrix(constraintsMatrixEigen))
        {
            yError() << "[setLinearConstraintsMatrix] Unable to set the constraints matrix.";
            return false;
        }
    }
    return true;
}

iDynTree::Position WalkingQPIK_osqp::evaluateIntegralError(std::unique_ptr<iCub::ctrl::Integrator>& integral,
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


bool WalkingQPIK_osqp::setBounds()
{
    // update joints limits (they depend on the velocity and on the position)
    setJointLimits();

    // evaluate the left foot error
    Eigen::VectorXd leftFootCorrection(6);
    iDynTree::Position leftFootError = m_leftFootToWorldTransform.getPosition()
        - m_desiredLeftFootToWorldTransform.getPosition();

    leftFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen(leftFootError) +
        + m_kIPosFoot * iDynTree::toEigen(evaluateIntegralError(m_leftFootErrorIntegral, leftFootError));
    iDynTree::Matrix3x3 errorLeftAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_leftFootToWorldTransform.getRotation() *
                                                                                    m_desiredLeftFootToWorldTransform.getRotation().inverse());
    leftFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorLeftAttitude)));

    // evaluate the right foot error
    Eigen::VectorXd rightFootCorrection(6);
    iDynTree::Position rightFootError = m_rightFootToWorldTransform.getPosition()
        - m_desiredRightFootToWorldTransform.getPosition();

    rightFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen(rightFootError)
        + m_kIPosFoot * iDynTree::toEigen(evaluateIntegralError(m_rightFootErrorIntegral, rightFootError));
    iDynTree::Matrix3x3 errorRightAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_rightFootToWorldTransform.getRotation() *
                                                                                     m_desiredRightFootToWorldTransform.getRotation().inverse());

    rightFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorRightAttitude)));

    // update lower and upper bound
    m_lowerBound.block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist) - leftFootCorrection;
    m_upperBound.block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist) - leftFootCorrection;

    m_lowerBound.block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist) - rightFootCorrection;
    m_upperBound.block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist) - rightFootCorrection;

    if(m_useCoMAsConstraint)
    {
        Eigen::VectorXd comCorrection(3);
        iDynTree::Position comPositionError = m_desiredComPosition - m_comPosition;
        comCorrection = m_kCom * iDynTree::toEigen(comPositionError) +
            m_kICom * iDynTree::toEigen(evaluateIntegralError(m_comErrorIntegral, comPositionError));

        m_lowerBound.block(12, 0, 3, 1) = iDynTree::toEigen(m_comVelocity) + comCorrection;
        m_upperBound.block(12, 0, 3, 1) = iDynTree::toEigen(m_comVelocity) + comCorrection;
    }

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateBounds(m_lowerBound, m_upperBound))
        {
            yError() << "[setBounds] Unable to update the bounds.";
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setLowerBound(m_lowerBound))
        {
            yError() << "[setBounds] Unable to set the first time the lower bound.";
            return false;
        }

        if(!m_optimizerSolver->data()->setUpperBound(m_upperBound))
        {
            yError() << "[setBounds] Unable to set the first time the upper bound.";
            return false;
        }
    }
    return true;
}

bool WalkingQPIK_osqp::setDesiredJointPosition(const iDynTree::VectorDynSize& regularizationTerm)
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

void WalkingQPIK_osqp::setDesiredFeetTransformation(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                                    const iDynTree::Transform& desiredRightFootToWorldTransform)
{
    m_desiredLeftFootToWorldTransform = desiredLeftFootToWorldTransform;
    m_desiredRightFootToWorldTransform = desiredRightFootToWorldTransform;
}

void WalkingQPIK_osqp::setDesiredFeetTwist(const iDynTree::Twist& leftFootTwist,
                                           const iDynTree::Twist& rightFootTwist)
{
    m_leftFootTwist = leftFootTwist;
    m_rightFootTwist = rightFootTwist;
}

void WalkingQPIK_osqp::setDesiredLeftHandTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform)
{
    m_desiredLeftHandToWorldTransform = desiredLeftHandToWorldTransform;
}

void WalkingQPIK_osqp::setDesiredRightHandTransformation(const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    m_desiredRightHandToWorldTransform = desiredRightHandToWorldTransform;
}

void WalkingQPIK_osqp::setDesiredHandsTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform,
                                                     const iDynTree::Transform& desiredRightHandToWorldTransform)
{
    setDesiredRightHandTransformation(desiredRightHandToWorldTransform);
    setDesiredLeftHandTransformation(desiredLeftHandToWorldTransform);
}

void WalkingQPIK_osqp::setDesiredCoMVelocity(const iDynTree::Vector3& comVelocity)
{
    m_comVelocity = comVelocity;
}

void WalkingQPIK_osqp::setDesiredCoMPosition(const iDynTree::Position& desiredComPosition)
{
    m_desiredComPosition = desiredComPosition;
}

bool WalkingQPIK_osqp::solve()
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

    if(!m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->initSolver())
        {
            yError() << "[solve] Unable to initialize the solver";
            return false;
        }
    }

    if(!m_optimizerSolver->solve())
    {
        yError() << "[solve] Unable to solve the problem.";
        return false;
    }

    // check equality constraints
    if(!isSolutionFeasible())
    {
        yError() << "[solve] The solution is not feasible.";
        return false;
    }

    m_isSolutionEvaluated = true;

    return true;
}

bool WalkingQPIK_osqp::isSolutionFeasible()
{
    double tolerance = 1;

    Eigen::VectorXd constrainedOutput = m_constraintsMatrixEigenDense
        * m_optimizerSolver->getSolution();

    if(((constrainedOutput - m_upperBound).maxCoeff() < tolerance)
       ||((constrainedOutput - m_lowerBound).maxCoeff() > -tolerance))
        return true;

    yError() << "[isSolutionFeasible] The constraints are not satisfied.";
    return false;
}

bool WalkingQPIK_osqp::getSolution(iDynTree::VectorDynSize& output)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[getSolution] The solution is not evaluated. "
                 << "Please call 'solve()' method.";
        return false;
    }

    if(output.size() != m_actuatedDOFs)
        output.resize(m_actuatedDOFs);

    Eigen::VectorXd solutionEigen = m_optimizerSolver->getSolution();

    for(int i = 0; i < output.size(); i++)
        output(i) = solutionEigen(i + 6);

    return true;
}

bool WalkingQPIK_osqp::getLeftFootError(iDynTree::VectorDynSize& output)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[getLeftFootError] The solution is not evaluated. "
                 << "Please call 'solve()' method.";
        return false;
    }

    iDynTree::toEigen(output) = m_lowerBound.block(0, 0, 6, 1) - iDynTree::toEigen(m_leftFootJacobian) *  m_optimizerSolver->getSolution();
    return true;
}

bool WalkingQPIK_osqp::getRightFootError(iDynTree::VectorDynSize& output)
{
    if(!m_isSolutionEvaluated)
    {
        yError() << "[getRightFootError] The solution is not evaluated. "
                 << "Please call 'solve()' method.";
        return false;
    }

    iDynTree::toEigen(output) = m_lowerBound.block(6, 0, 6, 1) - iDynTree::toEigen(m_rightFootJacobian) *  m_optimizerSolver->getSolution();
    return true;
}

const Eigen::MatrixXd& WalkingQPIK_osqp::getHessianMatrix() const
{
    return m_hessianEigenDense;
}

const Eigen::MatrixXd& WalkingQPIK_osqp::getConstraintMatrix() const
{
    return m_constraintsMatrixEigenDense;
}

const Eigen::VectorXd& WalkingQPIK_osqp::getUpperBound() const
{
    return m_upperBound;
}

const Eigen::VectorXd& WalkingQPIK_osqp::getLowerBound() const
{
    return m_lowerBound;
}

const Eigen::VectorXd& WalkingQPIK_osqp::getGradient() const
{
    return m_gradient;
}

bool WalkingQPIK_osqp::getNeckOrientationError(iDynTree::Vector3& output)
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

bool WalkingQPIK_osqp::getDesiredNeckOrientation(iDynTree::Vector3& output)
{
    // if(!m_isSolutionEvaluated)
    // {
    //     yError() << "[getRightFootError] The solution is not evaluated. "
    //              << "Please call 'solve()' method.";
    //     return false;
    // }

    auto error = m_desiredNeckOrientation;
    output = error.asRPY();

    return true;
}
