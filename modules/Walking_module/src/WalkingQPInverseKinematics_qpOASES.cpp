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
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingQPInverseKinematics_qpOASES.hpp>
#include <Utils.hpp>

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

    return true;
}

bool WalkingQPIK_qpOASES::setVelocityBounds(const iDynTree::VectorDynSize& minJointsLimit,
                                            const iDynTree::VectorDynSize& maxJointsLimit)
{
    if(minJointsLimit.size() != maxJointsLimit.size())
    {
        yError() << "[setVelocityBounds] The size of the vector limits has to be equal.";
        return false;
    }
    if(minJointsLimit.size() != m_actuatedDOFs)
    {
        yError() << "[setVelocityBounds] The size of the vector limits has to be equal to ."
                 << "the number of the joint";
        return false;
    }

    // the first six values are related to the velocity (linear and angular) of the base
    for(int i = 0; i < 6; i++)
    {
        m_minJointLimit[i] = -std::numeric_limits<double>::max();
        m_maxJointLimit[i] = std::numeric_limits<double>::max();
    }

    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        m_minJointLimit[i + 6] = minJointsLimit(i);
        m_maxJointLimit[i + 6] = maxJointsLimit(i);
    }
    return true;
}

bool WalkingQPIK_qpOASES::initialize(const yarp::os::Searchable& config,
                                     const int& actuatedDOFs,
                                     const iDynTree::VectorDynSize& minJointsLimit,
                                     const iDynTree::VectorDynSize& maxJointsLimit)
{
    m_actuatedDOFs = actuatedDOFs;
    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for QP-IK solver.";
        return false;
    }

    m_useCoMAsConstraint = config.check("useCoMAsConstraint", yarp::os::Value(false)).asBool();

    // TODO in the future the number of constraints should be added inside
    // the configuration file
    // set the number of variables and the number of constraints
    // the number of variables is equal to the number of joints plus
    // 6 (position + attitude) degree of freedom related to the base.
    m_numberOfVariables = m_actuatedDOFs + 6;

    // the number of constraints is equal to the number of joints plus
    // 12 (position + attitude) of the left and right feet
    if(m_useCoMAsConstraint)
        m_numberOfConstraints = 6 + 6 + 3;
    else
        m_numberOfConstraints = 6 + 6;

    // resize all vectors (matrices)
    m_hessian.resize(m_numberOfVariables * m_numberOfVariables);
    m_gradient.resize(m_numberOfVariables);
    m_constraintMatrix.resize(m_numberOfVariables * m_numberOfConstraints);
    m_upperBound.resize(m_numberOfConstraints);
    m_lowerBound.resize(m_numberOfConstraints);
    m_minJointLimit.resize(m_numberOfVariables);
    m_maxJointLimit.resize(m_numberOfVariables);
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

    if(!setVelocityBounds(minJointsLimit, maxJointsLimit))
    {
        yError() << "[initialize] Unable to set the velocity bounds.";
        return false;
    }

    if(!iDynTree::parseRotationMatrix(config, "additional_rotation", m_additionalRotation))
    {
        yError() << "[initialize] Unable to set the additional rotation.";
        return false;
    }

    //

    m_optimizer = std::make_shared<qpOASES::SQProblem>(m_numberOfVariables,
                                                       m_numberOfConstraints);

    m_optimizer->setPrintLevel(qpOASES::PL_LOW);

    m_isFirstTime = true;
    return true;
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

    return true;
}

bool WalkingQPIK_qpOASES::setGradientVector()
{
    iDynTree::Matrix3x3 errorNeckAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_neckOrientation * m_desiredNeckOrientation.inverse());

    if(m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1) =
            -iDynTree::toEigen(m_neckJacobian).transpose()
            * iDynTree::toEigen(m_neckWeightMatrix)
            * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)))
            - iDynTree::toEigen(m_jointRegulatizationGradient) *
            (iDynTree::toEigen(m_jointRegulatizationGains) * (iDynTree::toEigen(m_regularizationTerm)
                                                              - iDynTree::toEigen(m_jointPosition)));
    }
    else
    {
        Eigen::Map<MatrixXd>(m_gradient.data(), m_numberOfVariables, 1)
            = -iDynTree::toEigen(m_comJacobian).transpose()
            * iDynTree::toEigen(m_comWeightMatrix) * iDynTree::toEigen(m_comVelocity)
            - iDynTree::toEigen(m_neckJacobian).transpose() * iDynTree::toEigen(m_neckWeightMatrix)
            * (-m_kNeck * iDynTree::unskew(iDynTree::toEigen(errorNeckAttitude)))
            - iDynTree::toEigen(m_jointRegulatizationGradient) *
            (iDynTree::toEigen(m_jointRegulatizationGains) * (iDynTree::toEigen(m_regularizationTerm)
                                                              - iDynTree::toEigen(m_jointPosition)));
    }

    return true;
}

bool WalkingQPIK_qpOASES::setLinearConstraintMatrix()
{
    // add left foot
    Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                         m_numberOfConstraints,
                         m_numberOfVariables).block(0, 0, m_leftFootJacobian.rows(),
                                                    m_leftFootJacobian.cols())=
        iDynTree::toEigen(m_leftFootJacobian);

    // add right foot
    Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                          m_numberOfConstraints,
                          m_numberOfVariables).block(m_leftFootJacobian.rows(),
                                                     0, m_rightFootJacobian.rows(),
                                                     m_rightFootJacobian.cols())=
        iDynTree::toEigen(m_rightFootJacobian);


    // add com as constraint
    if(m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_constraintMatrix.data(),
                             m_numberOfConstraints,
                             m_numberOfVariables).block(m_leftFootJacobian.rows() + m_leftFootJacobian.rows(),
                                                        0, m_comJacobian.rows(),
                                                        m_comJacobian.cols())=
            iDynTree::toEigen(m_comJacobian);
    }

    return true;
}

bool WalkingQPIK_qpOASES::setBounds()
{
    Eigen::VectorXd leftFootCorrection(6);
    leftFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen((m_leftFootToWorldTransform.getPosition() -
                                                                        m_desiredLeftFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorLeftAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_leftFootToWorldTransform.getRotation() *
                                                                                    m_desiredLeftFootToWorldTransform.getRotation().inverse());

    leftFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorLeftAttitude)));

    Eigen::VectorXd rightFootCorrection(6);
    rightFootCorrection.block(0,0,3,1) = m_kPosFoot * iDynTree::toEigen((m_rightFootToWorldTransform.getPosition() -
                                                                         m_desiredRightFootToWorldTransform.getPosition()));

    iDynTree::Matrix3x3 errorRightAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_rightFootToWorldTransform.getRotation() *
                                                                                     m_desiredRightFootToWorldTransform.getRotation().inverse());

    rightFootCorrection.block(3,0,3,1) = m_kAttFoot * (iDynTree::unskew(iDynTree::toEigen(errorRightAttitude)));

    // if((m_leftFootTwist(0) == m_leftFootTwist(1)) && (m_leftFootTwist(0) == 0))
    // {
    //     Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist);
    //     Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist);
    // }
    // else
    // {
    //     Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
    //         - leftFootCorrection;
    //     Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
    //         - leftFootCorrection;
    // }

    Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
      - leftFootCorrection;
    Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(0, 0, 6, 1) = iDynTree::toEigen(m_leftFootTwist)
      - leftFootCorrection;

    // if((m_rightFootTwist(0) == m_rightFootTwist(1)) && (m_rightFootTwist(0) == 0))
    // {
    //     Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist);
    //     Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist);
    // }
    // else
    // {
    //     Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
    //         - rightFootCorrection;
    //     Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
    //         - rightFootCorrection;
    // }


    Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
      - rightFootCorrection;
    Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(6, 0, 6, 1) = iDynTree::toEigen(m_rightFootTwist)
      - rightFootCorrection;

    if(m_useCoMAsConstraint)
    {
        Eigen::Map<MatrixXd>(m_lowerBound.data(), m_numberOfConstraints, 1).block(12, 0, 3, 1) = iDynTree::toEigen(m_comVelocity)
            - m_kCom * (iDynTree::toEigen(m_comPosition) -  iDynTree::toEigen(m_desiredComPosition));
        Eigen::Map<MatrixXd>(m_upperBound.data(), m_numberOfConstraints, 1).block(12, 0, 3, 1) = iDynTree::toEigen(m_comVelocity)
            - m_kCom * (iDynTree::toEigen(m_comPosition) -  iDynTree::toEigen(m_desiredComPosition));
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
