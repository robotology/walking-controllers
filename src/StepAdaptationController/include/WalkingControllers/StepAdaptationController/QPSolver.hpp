/**
 * @file StepAdaptationController.hpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef WALKING_CONTROLLERS_QP_SOLVER_H
#define WALKING_CONTROLLERS_QP_SOLVER_H

// std
#include <deque>
#include <memory>
// iDynTree
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

#include <qpOASES.hpp>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>

namespace WalkingControllers
{
/**
 * QPSolver class
 */
    class QPSolver
    {
        /**
         * Pointer to the optimization solver
         */
        std::unique_ptr<OsqpEigen::Solver> m_QPSolver; /**< OsqpEigen Optimization solver. */
        std::unique_ptr<qpOASES::SQProblem> m_QPSolver_qpOASES{nullptr}; /**< qpOASES Optimization solver. */

        iDynSparseMatrix m_hessianMatrix;/**< hessian matrix of cost function. */
        iDynSparseMatrix m_constraintsMatrix; /**< constraints matrix. */
        iDynTree::VectorDynSize m_gradient;/**< Gradient vector. */
        iDynTree::VectorDynSize m_lowerBound; /**< Lower bound vector. */
        iDynTree::VectorDynSize m_upperBound; /**< Upper bound vector. */

        iDynTree::VectorDynSize m_solution;  /**< solution vector of the optimization. */

        int m_inputSize; /**< Size of the controlled input vector . */
        int m_numberOfConstraints; /**< Size of the constraint vector . */

        bool m_isFirstTime;/**< boolean  that indicates whether the solver has been already initilized? . */

    public:

        /**
         * Constructor of QPSolver.
         * @param inputSize size of the controlled input vector;
         * @param numberOfAllConstraints number of equality and inequality constraints!
         */
        QPSolver(const int& inputSize, const int& numberOfAllConstraints);

        /**
         * Set the hessian matrix.
         * Please do not call this function to update the hessian matrix! It can be set only once.
         * @param zmpWeight weight of next step position term in cost function.
         * @param dcmOffsetWeight weight of dcm offset term in cost function.
         * @param sigmaWeight weight of next step timing term in cost function.
         * @return true/false in case of success/failure.
         */
        bool setHessianMatrix(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight,
                              const double& sigmaWeight);

        /**
         * Set or update the linear constraints matrix(A) related to equality and inequality constraints(C<Ax<B)
         * If the solver is already set the linear constraints matrix is updated otherwise it is set for
         * the first time.
         * @param currentDcmPosition This vector includes the current value of real DCM ;
         * @param currentZmpPosition This vector includes the current value of stance foot position ;
         * @param convexHullMatrix The convex hull matrix related to allowable next step position;
         * @return true/false in case of success/failure.
         */
        bool setConstraintsMatrix(const iDynTree::Vector2& currentDcmPosition, const iDynTree::Vector2& currentZmpPosition,
                                  const iDynTree::MatrixDynSize& convexHullMatrix);

        /**
         * Set or update the gradient
         * @param zmpWeight weight of next step position term in cost function.
         * @param dcmOffsetWeight weight of dcm offset term in cost function.
         * @param sigmaWeight weight of next step timing term in cost function.
         * @param zmpNominal vector of nominal values of next step position .
         * @param dcmOffsetNominal vector of nominal values of dcm offset.
         * @param sigmaNominal vector of nominal values of exp(w*steptiming) .
         * @return true/false in case of success/failure.
         */
        bool setGradientVector(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight,
                               const double& sigmaWeight,const iDynTree::Vector2& zmpNominal,
                               const iDynTree::Vector2& dcmOffsetNominal, const double& sigmaNominal);

        /**
         * Set or update the lower and the upper bounds
         * @param zmpPosition This vector includes the current value of stance foot position ;
         * @param convexHullVector The convex hull vector related to allowable next step position;
         * @param stepDuration The nominal value of step timing ;
         * @param stepDurationTollerance The tollerance of the max and min step timing value with respect to the nominal value ;
         * @param remainingSingleSupportDuration The remained amount of single support duration ;
         * @param omega The natural frequency of LIPM ;
         * @return true/false in case of success/failure.
         */
        bool setBoundsVectorOfConstraints(const iDynTree::Vector2& zmpPosition, const iDynTree::VectorDynSize& convexHullVector,
                                          const double& stepDuration, const double& stepDurationTollerance,
                                          const double& remainingSingleSupportDuration,const double& omega);

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
