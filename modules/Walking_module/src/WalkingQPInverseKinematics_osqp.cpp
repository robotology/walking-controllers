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
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingQPInverseKinematics_osqp.hpp>

void WalkingQPIK_osqp::setNumberOfConstraints()
{

    m_numberOfConstraints = 6 + 6;

    if(m_useCoMAsConstraint)
        m_numberOfConstraints += 3;

    if(m_useJointsLimitsConstraint)
        m_numberOfConstraints += m_actuatedDOFs;
}

void WalkingQPIK_osqp::initializeSolverSpecificMatrices()
{
    // add constraint for the maximum velocity.
    // In the following we suppose that the constraints are saved in the following order
    // (lf, rf com (if it is present))
    int startingRow = 6 + 6;
    if(m_useCoMAsConstraint)
        startingRow += 3;

    if(m_useJointsLimitsConstraint)
        for(int i = 0; i < m_actuatedDOFs; i++)
            m_constraintsMatrixSparse(startingRow + i, i + 6) = 1;
}

void WalkingQPIK_osqp::setJointVelocitiesBounds()
{
    if(m_useJointsLimitsConstraint)
    {
        int numberOfTaskConstraints;
        if(m_useCoMAsConstraint)
            numberOfTaskConstraints = 6 + 6 + 3;
        else
            numberOfTaskConstraints = 6 + 6;

        for(int i = 0; i < m_actuatedDOFs; i++)
        {
            m_lowerBound(i + numberOfTaskConstraints) = m_kJointLimitsLowerBound *
                std::tanh(m_jointPosition(i) - m_jointPositionsLowerBounds(i))
                * (-m_jointVelocitiesBounds(i));

            m_upperBound(i + numberOfTaskConstraints) = m_kJointLimitsUpperBound *
                std::tanh(m_jointPositionsUpperBounds(i) - m_jointPosition(i))
                * m_jointVelocitiesBounds(i);
        }
    }
    return;
}

void WalkingQPIK_osqp::instantiateSolver()
{
    // instantiate the solver
    m_optimizerSolver = std::make_unique<OsqpEigen::Solver>();
    m_optimizerSolver->data()->setNumberOfVariables(m_numberOfVariables);
    m_optimizerSolver->data()->setNumberOfConstraints(m_numberOfConstraints);

    m_optimizerSolver->settings()->setVerbosity(false);
    m_optimizerSolver->settings()->setLinearSystemSolver(0);
}


bool WalkingQPIK_osqp::initializeSolver()
{
    // Hessian matrix
    auto hessianDense(iDynTree::toEigen(m_hessianDense));
    Eigen::SparseMatrix<double> hessianSparse = hessianDense.sparseView();
    if(!m_optimizerSolver->data()->setHessianMatrix(hessianSparse))
    {
        yError() << "[initializeSolver] Unable to set the hessian matrix.";
        return false;
    }

    // gradient
    auto gradient(iDynTree::toEigen(m_gradient));
    if(!m_optimizerSolver->data()->setGradient(gradient))
    {
        yError() << "[initializeSolver] Unable to set first time the gradient.";
        return false;
    }


    Eigen::SparseMatrix<double> constraintsMatrixSparse = iDynTree::toEigen(m_constraintsMatrixSparse);
    // auto constraintsMatrixSparse(iDynTree::toEigen(m_constraintsMatrixSparse));
    if(!m_optimizerSolver->data()->setLinearConstraintsMatrix(constraintsMatrixSparse))
    {
        yError() << "[initializeSolver] Unable to set the constraints matrix.";
        return false;
    }

    auto lowerBound(iDynTree::toEigen(m_lowerBound));
    if(!m_optimizerSolver->data()->setLowerBound(lowerBound))
    {
        yError() << "[initializeSolver] Unable to set the first time the lower bound.";
        return false;
    }

    auto upperBound(iDynTree::toEigen(m_upperBound));
    if(!m_optimizerSolver->data()->setUpperBound(upperBound))
    {
        yError() << "[initializeSolver] Unable to set the first time the upper bound.";
        return false;
    }

    if(!m_optimizerSolver->initSolver())
    {
        yError() << "[initializeSolver] Unable to initialize the solver";
        return false;
    }

    return true;
}

bool WalkingQPIK_osqp::updateSolver()
{
    // Hessian matrix
    auto hessianDense(iDynTree::toEigen(m_hessianDense));
    Eigen::SparseMatrix<double> hessianSparse = hessianDense.sparseView();
    if(!m_optimizerSolver->updateHessianMatrix(hessianSparse))
    {
        yError() << "[updateSolver] Unable to set the hessian matrix.";
        return false;
    }

    // gradient
    auto gradient(iDynTree::toEigen(m_gradient));
    if(!m_optimizerSolver->updateGradient(gradient))
    {
        yError() << "[updateSolver] Unable to set first time the gradient.";
        return false;
    }

    auto constraintsMatrixSparse(iDynTree::toEigen(m_constraintsMatrixSparse));
    if(!m_optimizerSolver->updateLinearConstraintsMatrix(constraintsMatrixSparse))
    {
        yError() << "[updateSolver] Unable to set the constraints matrix.";
        return false;
    }

    auto lowerBound(iDynTree::toEigen(m_lowerBound));
    auto upperBound(iDynTree::toEigen(m_upperBound));
    if(!m_optimizerSolver->updateBounds(lowerBound, upperBound))
    {
        yError() << "[updateSolver] Unable to set the first time the lower bound.";
        return false;
    }

    return true;
}

bool WalkingQPIK_osqp::solve()
{
    evaluateHessianMatrix();
    evaluateGradientVector();
    evaluateLinearConstraintMatrix();
    evaluateBounds();

    if(!m_optimizerSolver->isInitialized())
    {
        if(!initializeSolver())
        {
            yError() << "[solve] Unable to initialize the solver";
            return false;
        }
    }
    else
    {
        if(!updateSolver())
        {
            yError() << "[solve] Unable to update the solver";
            return false;
        }
    }

    if(!m_optimizerSolver->solve())
    {
        yError() << "[solve] Unable to solve the problem.";
        return false;
    }

    iDynTree::toEigen(m_solution) = m_optimizerSolver->getSolution();
    for(int i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointVelocitiesOutput(i) = m_solution(i + 6);

    return true;
}
