/**
 * @file MPCSolver.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_MPC_SOLVER_H
#define WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_MPC_SOLVER_H

// std
#include <deque>

// iDynTree
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/VectorDynSize.h>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

#include <WalkingControllers/iDynTreeUtilities/Helper.h>

namespace WalkingControllers
{

    /**
     * MPCSolver class
     */
    class MPCSolver
    {
        /**
         * Pointer to the optimization solver
         */
        std::unique_ptr<OsqpEigen::Solver> m_optimizerSolver;
        iDynTree::Triplets const* m_equalConstraintsMatrix; /**< Equal part of the constraints matrix. */
        iDynSparseMatrix const* m_gradientSubmatrix; /**< Matrix used to evaluate the gradient vector */
        iDynSparseMatrix const* m_stateWeightMatrix; /**< State weight stacked matrix */

        Eigen::VectorXd m_lowerBound; /**< Lower bound vector. */
        Eigen::VectorXd m_upperBound; /**< Upper bound vector. */
        Eigen::VectorXd m_gradient; /**< Gradient vector. */

        int m_stateSize; /**< Size of the state vector (2). */
        int m_inputSize; /**< Size of the controlled input vector (2). */
        int m_controllerHorizon; /**< Controller horizon (in steps)*/
        int m_numberOfInequalityConstraints; /**< Number of inequality constraints*/

    public:

        /**
         * Constructor.
         * @param stateSize size of the state vector;
         * @param inputSize size of the controlled input vector;
         * @param equalConstraintsMatrix equal submatrix  of the constraints matrix;
         * @param gradientSubmatrix matrix used to evaluate the gradient vector
         * (\f$-\Theta^T \tilde{R} e_1\f$);
         * @param stateWeightStackedMatrix \f$ \tilde{Q} = diag([Q, Q, ..., Q]) \f$.
         */
        MPCSolver(const int& stateSize, const int& inputSize,
                  const int& controllerHorizon,
                  const int& numberOfInequalityConstraints,
                  const iDynTree::Triplets& equalConstraintsMatrix,
                  const iDynSparseMatrix& gradientSubmatrix,
                  const iDynSparseMatrix& stateWeightStackedMatrix);

        /**
         * Set the hessian matrix.
         * Please do not call this function to update the hessian matrix! It can be set only once.
         * @param hessian hessian matrix.
         * @return true/false in case of success/failure.
         */
        bool setHessianMatrix(const iDynSparseMatrix& hessian);

        /**
         * Set or update the linear constraints matrix.
         * If the solver is already set the linear constraints matrix is updated otherwise it is set for
         * the first time.
         * @param inequalityConstraintsMatrix  matrix of the inequalities constraints (Ax < b)
         * @return true/false in case of success/failure.
         */
        bool setConstraintsMatrix(const iDynTree::MatrixDynSize& inequalityConstraintsMatrix);

        /**
         * Set or update the lower and the upper bounds
         * @param currentState value of the current state
         * @param inequalityConstraintsVector vector of the inequalities constraints (Ax < b)
         * @return true/false in case of success/failure.
         */
        bool setBounds(const iDynTree::Vector2& currentState,
                       const iDynTree::VectorDynSize& inequalityConstraintsVector);

        /**
         * Set or update the gradient
         * @param referenceSignal reference signal vector (it has to contain the reference trajectory
         * for the whole controller horizon);
         * @param previousControllerOutput previous controller output;
         * @param resetTrajectory set equal to true if you do not want to use the previous trajectory.
         * @return true/false in case of success/failure.
         */
        bool setGradient(const std::deque<iDynTree::Vector2>& refereceSignal,
                         const iDynTree::Vector2& previousControllerOutput,
                         const bool& resetTrajectory);

        /**
         * Get the primal variable.
         * @param primalVariable primal variable vector
         * @return true/false in case of success/failure.
         */
        bool getPrimalVariable(Eigen::VectorXd& primalVariable);

        /**
         * Set the primal variable.
         * @param primalVariable primal variable vector
         * @return true/false in case of success/failure.
         */
        bool setPrimalVariable(const Eigen::VectorXd& primalVariable);

        /**
         * Get the state of the solver.
         * @return true if the solver is initialized false otherwise.
         */
        bool isInitialized();

        /**
         * Initialize the solver.
         * @return true/false in case of success/failure.
         */
        bool initialize();

        /**
         * Solve the optimization problem.
         * @return true/false in case of success/failure.
         */
        bool solve();

        /**
         * Get the solver solution
         * @return the entire solution of the solver
         */
        iDynTree::VectorDynSize getSolution();
    };
};

#endif
