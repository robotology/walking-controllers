/**
 * @file MPCSolver.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <WalkingControllers/iDynTreeHelper/Helper.h>
#include <WalkingControllers/SimplifiedModelControllers/MPCSolver.h>

using namespace WalkingControllers;

MPCSolver::MPCSolver(const int& stateSize, const int& inputSize,
                     const int& controllerHorizon,
                     const int& numberOfInequalityConstraints,
                     const iDynTree::Triplets& equalConstraintsMatrixTriplets,
                     const iDynSparseMatrix& gradientSubmatrix,
                     const iDynSparseMatrix& stateWeightMatrix)
    :m_stateSize(stateSize),
     m_inputSize(inputSize),
     m_controllerHorizon(controllerHorizon),
     m_numberOfInequalityConstraints(numberOfInequalityConstraints),
     m_equalConstraintsMatrix(&equalConstraintsMatrixTriplets),
     m_gradientSubmatrix(&gradientSubmatrix),
     m_stateWeightMatrix(&stateWeightMatrix)
{
    // instantiate the solver class
    m_optimizerSolver = std::make_unique<OsqpEigen::Solver>();

    // set the number of variables
    int numberOfVariables = m_stateSize * (m_controllerHorizon + 1) +
        m_inputSize * m_controllerHorizon;
    m_optimizerSolver->data()->setNumberOfVariables(numberOfVariables);

    // set the number of constraints
    int numberOfConstraints = m_stateSize * (m_controllerHorizon + 1) +
        m_numberOfInequalityConstraints;
    m_optimizerSolver->data()->setNumberOfConstraints(numberOfConstraints);

    // resize vectors
    m_gradient = Eigen::VectorXd::Zero(numberOfVariables);
    m_lowerBound = Eigen::VectorXd::Zero(numberOfConstraints);
    m_upperBound = Eigen::VectorXd::Zero(numberOfConstraints);

    for(int i = m_stateSize * (m_controllerHorizon + 1); i < numberOfConstraints; i++)
        m_lowerBound(i) = - OsqpEigen::INFTY;

    m_optimizerSolver->settings()->setVerbosity(false);
}

bool MPCSolver::setHessianMatrix(const iDynSparseMatrix& hessian)
{
    Eigen::SparseMatrix<double> hessianEigen = iDynTree::toEigen(hessian);
    if(m_optimizerSolver->isInitialized())
    {
        std::cerr << "[setHessianMatrix] Something goes wrong. "
                  << "In this particular problem the hessian matrix is constant."
                  << std::endl;
        return false;
    }
    else
    {
        if(!m_optimizerSolver->data()->setHessianMatrix(hessianEigen))
        {
            std::cerr << "[setHessianMatrix] Unable to set first time the hessian matrix."
                      << std::endl;
            return false;
        }
    }
    return true;
}

bool MPCSolver::setConstraintsMatrix(const iDynTree::MatrixDynSize& inequalityConstraintsMatrix)
{
    // set the linear constraints matrix triplets
    iDynTree::Triplets constraintsTriplets;
    iDynTreeHelper::Triplets::pushTriplets(*m_equalConstraintsMatrix, constraintsTriplets);

    int inequalityConstraintsMatrixRowPos = m_stateSize * (m_controllerHorizon + 1);
    int inequalityConstraintsMatrixColumnPos = m_stateSize * (m_controllerHorizon + 1);
    constraintsTriplets.addSubMatrix(inequalityConstraintsMatrixRowPos,
                                     inequalityConstraintsMatrixColumnPos,
                                     inequalityConstraintsMatrix);

    // convert triplets into a sparse matrix
    int constraintsMatrixRows = m_stateSize * (m_controllerHorizon + 1) +
        m_numberOfInequalityConstraints;
    int constraintsMatrixCols = m_stateSize * (m_controllerHorizon + 1) +
        m_inputSize * m_controllerHorizon;
    iDynSparseMatrix constraintsMatrix(constraintsMatrixRows,
                                       constraintsMatrixCols);

    constraintsMatrix.setFromConstTriplets(constraintsTriplets);

    // convert iDynTree sparse matrix in eigen sparse matrix
    // it is required by the osqp library
    Eigen::SparseMatrix<double> constraintsMatrixEigen =
        iDynTree::toEigen(constraintsMatrix);

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateLinearConstraintsMatrix(constraintsMatrixEigen))
        {
            std::cerr << "[setLinearConstraintsMatrix] Unable to update the constraints matrix."
                      << std::endl;
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setLinearConstraintsMatrix(constraintsMatrixEigen))
        {
            std::cerr << "[setLinearConstraintsMatrix] Unable to set the constraints matrix."
                      << std::endl;
            return false;
        }

    }
    return true;
}

bool MPCSolver::setBounds(const iDynTree::Vector2& currentState,
                          const iDynTree::VectorDynSize& inequalityConstraintsVector)
{
    if(currentState.size() != m_stateSize)
    {
        std::cerr << "[setBounds] The size of the currentState vector has to equal: " << m_stateSize
                  << std::endl;
        return false;
    }

    if(inequalityConstraintsVector.size() != m_numberOfInequalityConstraints)
    {
        std::cerr << "[setBounds] The size of the inequalityConstraintsVector has to equal: "
                  << m_numberOfInequalityConstraints << std::endl;
        return false;
    }

    // set the lower and the upper bounds
    m_lowerBound(0) = -currentState(0);
    m_lowerBound(1) = -currentState(1);
    m_upperBound(0) = -currentState(0);
    m_upperBound(1) = -currentState(1);

    // update the inequality constraints vector
    // note: it should be removed from here. It is not necessary to update the inequality constraint
    // vector every iteration. It should be updated only when a change of phase
    // (SS->DS or vice versa) occurs
    for(int i = 0; i< m_numberOfInequalityConstraints; i++)
        m_upperBound(m_stateSize * (m_controllerHorizon + 1) + i) = inequalityConstraintsVector(i);

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateBounds(m_lowerBound, m_upperBound))
        {
            std::cerr << "[setBounds] Unable to update the bounds."
                      << std::endl;
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setLowerBound(m_lowerBound))
        {
            std::cerr << "[setBounds] Unable to set the first time the lower bound."
                      << std::endl;
            return false;
        }

        if(!m_optimizerSolver->data()->setUpperBound(m_upperBound))
        {
            std::cerr << "[setBounds] Unable to set the first time the upper bound."
                      << std::endl;
            return false;
        }
    }
    return true;
}

bool MPCSolver::setGradient(const std::deque<iDynTree::Vector2>& referenceSignal,
                            const iDynTree::Vector2& previousControllerOutput,
                            const bool& resetTrajectory)
{
    // the solver is not initialized or the trajectory was reset.
    if(!m_optimizerSolver->isInitialized() || resetTrajectory)
    {
        // check if the size of the controller horizon is lower than the size of the reference signal
        if(referenceSignal.size() >= m_controllerHorizon + 1)
        {
            for(int i = 0; i < (m_controllerHorizon + 1); i++)
            {
                m_gradient.block<2,1>(i * m_stateSize, 0) = -iDynTree::toEigen(*m_stateWeightMatrix) *
                    iDynTree::toEigen(referenceSignal[i]);
            }
        }

        // otherwise we assume the reference signal becomes constant
        else
        {
            // the first part is the same as before
            for(int i = 0; i < referenceSignal.size(); i++)
            {
                m_gradient.block<2,1>(i * m_stateSize, 0) = -iDynTree::toEigen(*m_stateWeightMatrix) *
                    iDynTree::toEigen(referenceSignal[i]);
            }
            for(int i = referenceSignal.size(); i < (m_controllerHorizon + 1); i++)
            {
                m_gradient.block<2,1>(i * m_stateSize, 0) = -iDynTree::toEigen(*m_stateWeightMatrix) *
                    iDynTree::toEigen(referenceSignal.back());
            }
        }
    }
    else
    {
        // shift the element of the gradient in order to save time
        for(int i = 0; i < (m_controllerHorizon); i++)
        {
            m_gradient.block<2,1>(i*m_stateSize, 0) = m_gradient.block<2,1>((i+1) * m_stateSize, 0);
        }

        if(referenceSignal.size() >= m_controllerHorizon + 1)
        {
            // evaluate only the new element of the gradient
            m_gradient.block<2,1>(m_controllerHorizon * m_stateSize, 0) =
                -iDynTree::toEigen(*m_stateWeightMatrix) *
                iDynTree::toEigen(referenceSignal[m_controllerHorizon]);
        }
        else
        {
            // evaluate only the new element of the gradient in this case the signal
            // is assumed to be constant
            m_gradient.block<2,1>(m_controllerHorizon * m_stateSize, 0) =
                -iDynTree::toEigen(*m_stateWeightMatrix) *
                iDynTree::toEigen(referenceSignal.back());
        }
    }

    int gradientStateSize = m_stateSize * (m_controllerHorizon + 1);
    int gradientInputSize = m_inputSize * m_controllerHorizon;

    m_gradient.block(gradientStateSize, 0, gradientInputSize, 1) =
        iDynTree::toEigen(*m_gradientSubmatrix) * iDynTree::toEigen(previousControllerOutput);

    if(m_optimizerSolver->isInitialized())
    {
        if(!m_optimizerSolver->updateGradient(m_gradient))
        {
            std::cerr << "[setGradient] Unable to update the gradient."
                      << std::endl;
            return false;
        }
    }
    else
    {
        if(!m_optimizerSolver->data()->setGradient(m_gradient))
        {
            std::cerr << "[setGradient] Unable to set first time the gradient."
                      << std::endl;
            return false;
        }
    }
    return true;
}

bool MPCSolver::getPrimalVariable(Eigen::VectorXd& primalVariable)
{
    if(!m_optimizerSolver->isInitialized())
    {
        std::cerr << "[solve] The solver is not initilialize."
                  << std::endl;
        return false;
    }
    return m_optimizerSolver->getPrimalVariable(primalVariable);
}

bool MPCSolver::setPrimalVariable(const Eigen::VectorXd& primalVariable)
{
    if(!m_optimizerSolver->isInitialized())
    {
        std::cerr << "[solve] The solver is not initilialize."
                  << std::endl;
        return false;
    }
    return m_optimizerSolver->setPrimalVariable(primalVariable);
}

bool MPCSolver::isInitialized()
{
    return m_optimizerSolver->isInitialized();
}

bool MPCSolver::initialize()
{
    return m_optimizerSolver->initSolver();
}

bool MPCSolver::solve()
{
    if(!m_optimizerSolver->isInitialized())
    {
        std::cerr << "[solve] The solver is not initilialize."
                  << std::endl;
        return false;
    }

    return m_optimizerSolver->solve();
}

iDynTree::VectorDynSize MPCSolver::getSolution()
{
    Eigen::VectorXd solutionEigen = m_optimizerSolver->getSolution();

    int solutionSize = m_stateSize * (m_controllerHorizon + 1) +
        m_inputSize * m_controllerHorizon;
    iDynTree::VectorDynSize solution(solutionSize);
    iDynTree::toEigen(solution) = solutionEigen;

    return solution;
}
