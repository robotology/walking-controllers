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

#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics_qpOASES.h>

using namespace WalkingControllers;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

void WalkingQPIK_qpOASES::setNumberOfConstraints()
{
    if(m_useCoMAsConstraint)
        m_numberOfConstraints = 6 + 6 + 3;
    else
        m_numberOfConstraints = 6 + 6;
}

void WalkingQPIK_qpOASES::initializeSolverSpecificMatrices()
{
    m_minJointLimit.resize(m_numberOfVariables);
    m_maxJointLimit.resize(m_numberOfVariables);

    for(int i = 0; i < m_numberOfVariables; i++)
    {
        m_minJointLimit(i) = -qpOASES::INFTY;
        m_maxJointLimit(i) = qpOASES::INFTY;
    }
}

void WalkingQPIK_qpOASES::setJointVelocitiesBounds()
{
    if(m_useJointsLimitsConstraint)
        for(int i = 0; i < m_actuatedDOFs; i++)
        {
            m_minJointLimit(i + 6) = std::tanh(m_kJointLimitsLowerBound
                                               * (m_jointPosition(i) - m_jointPositionsLowerBounds(i)))
                * (-m_jointVelocitiesBounds(i));

            m_maxJointLimit(i + 6) = std::tanh(m_kJointLimitsUpperBound
                                               * (m_jointPositionsUpperBounds(i) - m_jointPosition(i)))
                * m_jointVelocitiesBounds(i);
        }

    return;
}

void WalkingQPIK_qpOASES::instantiateSolver()
{
    m_optimizer = std::make_unique<qpOASES::SQProblem>(m_numberOfVariables,
                                                       m_numberOfConstraints);

    m_optimizer->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
}

bool WalkingQPIK_qpOASES::solve()
{
    evaluateHessianMatrix();
    evaluateGradientVector();
    evaluateLinearConstraintMatrix();
    evaluateBounds();

    // convert sparse matrix into a dense matrix
    MatrixXd constraintMatrix = MatrixXd(iDynTree::toEigen(m_constraintsMatrixSparse));

    int nWSR = 100;
    if(!m_isFirstTime)
    {
        if(m_optimizer->hotstart(m_hessianDense.data(), m_gradient.data(), constraintMatrix.data(),
                                 m_minJointLimit.data(), m_maxJointLimit.data(),
                                 m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }
    }
    else
    {
        if(m_optimizer->init(m_hessianDense.data(), m_gradient.data(), constraintMatrix.data(),
                             m_minJointLimit.data(), m_maxJointLimit.data(),
                             m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }

        m_isFirstTime = false;
    }

    m_optimizer->getPrimalSolution(m_solution.data());

    for(int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointVelocitiesOutput(i) = m_solution(i + 6);

    return true;
}
