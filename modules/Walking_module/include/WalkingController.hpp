/**
 * @file WalkingController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLER_HPP
#define WALKING_CONTROLLER_HPP

// eigen
#include <Eigen/Sparse>

// iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/ConvexHullHelpers.h>

// yarp
#include <yarp/os/Value.h>

#include <unordered_map>
#include <deque>

// solver
#include "MPCSolver.hpp"

/**
 * WalkingController class contains the controller instances.
 * Each controller depends on the number of the inequality constraints.
 */
class WalkingController
{
    /**
     * The hessian matrix of the QP problem. It is the same for all controllers.
     */
    iDynSparseMatrix m_hessianMatrix;

    /**
     * The equal constraints matrix. Since it depends only on the system dynamic matrix.
     * A and B. It is the same for all controllers.
     */
    iDynTree::Triplets m_equalConstraintsMatrixTriplets;

    /**
     * Submatrix of the gradient vector.
     * \f$-\Theta^T \tilde{R} e_1 \f$. It is stored in order to save time during
     * the initialization of the controllers.
     */
    iDynSparseMatrix m_gradientSubmatrix;

    /**
     * State weight stacked matrix.
     * \f$\tilde{Q} = diag([Q, Q, ..., Q])\f$. It is stored in order to save time during
     * the initialization of the controllers.
     */
    iDynSparseMatrix m_stateWeightMatrix;

    int m_stateSize; /**< Size of the state vector. It is equal to 2. */
    int m_inputSize;  /**< Size of the input vector. It is equal to 2. */
    int m_controllerHorizon; /**< Length of the controller horizon. */

    double m_convexHullTolerance; /**< This is the maximum acceptable distance between the solution and the convex hull. */

    std::pair<bool, bool> m_feetStatus; /**< Current status of the feet. Left and Right. True is used
                                           if the foot is in contact. */

    iDynTree::ConvexHullProjectionConstraint m_convexHullComputer; /**<iDynTree convex hull helper. */
    std::vector<iDynTree::Polygon> m_feetPolygons; /**<Vector containing the polygon of each foot (left and right). */

    bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

    /**
     * Pointer to the current MPCSolver.
     * A new MPC solver is initialized when a new phase occurs.
     */
    std::shared_ptr<MPCSolver> m_currentController;

    iDynTree::Vector2 m_output; /**< Vector containing the output of the controller. */

    /**
     * Initialize the quantities useful in the inequality constraints evaluation.
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure
     */
    bool initializeConstraints(const yarp::os::Searchable& config);

    /**
     * Initialize the constant matrices of useful in the optimization problem
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure.
     */
    bool initializeMatrices(const yarp::os::Searchable& config);

    /**
     * Evaluate theta matrix. For further information please refers to the
     * [literature](https://github.com/loc2/element_capture-point-walking/issues/9)
     * @return theta.
     */
    iDynSparseMatrix evaluateThetaMatrix();

    /**
     * Evaluate the input weight stacked matrix. (aka \f$ \tilde{R} \f$)
     * \f$ \tilde{R} = \text{diag}(R R ... R)\f$
     * @param inputWeightMatrix is the input weight matrix (\f$ R \f$)
     * @return the stacked matrix
     */
    iDynTree::Triplets evaluateInputWeightStackedMatrix(const iDynTree::Triplets& inputWeightMatrix);

    /**
     * Evaluate the state weight stacked matrix. (aka \f$ \tilde{Q} \f$)
     * \f$ \tilde{Q} = \text{diag}(Q Q ... Q)\f$
     * @param stateWeightMatrix is the state weight matrix (\f$ Q \f$)
     * @return the stacked matrix
     */
    iDynTree::Triplets evaluateStateWeightStackedMatrix(const iDynTree::Triplets& stateWeightMatrix);

    /**
     * Evaluate the submatrix of the hessian matrix related to the input vector.
     * @param inputWeightMatrix is the weight matrix related to the input vector (aka \f$ \tilde{R} \f$);
     * @param thetaMatrix is the matrix theta.
     * @return the hessian submatrix related to the input.
     */
    iDynSparseMatrix evaluateHessianInputSubmatrix(const iDynTree::Triplets& inputWeightStackedTriplets,
                                                   const iDynSparseMatrix& thetaMatrix);

    /**
     * Evaluate the hessian matrix.
     * @param stateSubMatrix is the submatrix related to the state;
     * @param inputSubMatrix is the submatrix related to the input.
     * @return the hessian submatrix related to the input.
     */
    iDynSparseMatrix evaluateHessianMatrix(const iDynTree::Triplets& stateSubmatrix,
                                           const iDynSparseMatrix& inputSubmatrix);

    /**
     * Evaluate the a constant submatrix that is useful to evaluate the gradient vector.
     * \f$ -\Theta^T \tilde{R} e_1 \f$ For further information please refers to the
     * [literature](https://github.com/loc2/element_capture-point-walking/issues/9).
     * @param inputWeightStackedMatrix is the input weight stacked matrix \f$ \tilde{R} \f$.
     * @param thetaMatrix is the theta matrix \f$ \Theta \f$.
     * @return the submatrix
     */
    iDynSparseMatrix evaluateGradientSubmatrix(const iDynTree::Triplets& inputWeightStackedMatrix,
                                               const iDynSparseMatrix& thetaMatrix);

    /**
     * Evaluate the equal constraint matrix.
     * @param stateDynamicsTriplets are the triplets related to the linear state dynamics matrix;
     * @param inputDynamicsTriplets are the triplets related to the linear input dynamics matrix;
     * @return The equal constraints matrix.
     */
    iDynTree::Triplets evaluateEqualConstraintsMatrix(const iDynTree::Triplets& stateDynamicsTriplets,
                                                      const iDynTree::Triplets& inputDynamicsTriplets);

    /**
     * Evaluate the equal constraint state submatrix.
     * @param stateDynamicsMatrix is the linear state dynamics matrix.
     * @return the equal constraint state submatrix.
     */
    iDynTree::Triplets evaluateEqualConstraintsStateSubmatrix(const iDynTree::Triplets& stateDynamicsMatrix);

    /**
     * Evaluate the equal constraint input submatrix.
     * @param inputDynamicsMatrix is the linear input dynamics matrix.
     * @return the equal constraint input submatrix.
     */
    iDynTree::Triplets evaluateEqualConstraintsInputSubmatrix(const iDynTree::Triplets& inputDynamicsMatrix);

    /**
     * Build the convex hull for double support phase.
     * @param leftFootTransform structure containing the homogeneous transformation of the left foot;
     * @param leftFootTransform structure containing the homogeneous transformation of the right foot;
     * @return true/false in case of success/failure.
     */
    bool buildConvexHull(const iDynTree::Transform& leftFootTransform,
                         const iDynTree::Transform& rightFootTransform);

    /**
     * Build the convex hull for single support phase.
     * @param footTransform structure containing the homogeneous transformation of the stance foot.
     * @return true/false in case of success/failure.
     */
    bool buildConvexHull(const iDynTree::Transform& footTransformfoot);

public:

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure
     */
    bool initialize(const yarp::os::Searchable& config);

    /**
     * If the phase (DS or SS) is changed the new convex hull is evaluated and a new MPCSolver
     * is initialize.
     * @param leftFoot deque containing the homogeneous transformation of the left foot during
     * the trajectory;
     * @param rightFoot deque containing the homogeneous transformation of the right foot during
     * the trajectory;
     * @param leftInContact deque containing information about the state of the left foot
     * (stance = true, swing = false);
     * @param rightInContact deque containing information about the state of the left foot
     * (stance = true, swing = false).
     * @return true/false in case of success/failure.
     */
    bool setConvexHullConstraint(const std::deque<iDynTree::Transform>& leftFoot,
                                 const std::deque<iDynTree::Transform>& rightFoot,
                                 const std::deque<bool>& leftInContact,
                                 const std::deque<bool>& rightInContact);

    /**
     * Set the feedback.
     * @param currentState current value of the state.
     * @return true/false in case of success/failure.
     */
    bool setFeedback(const iDynTree::Vector2& currentState);

    /**
     * Set the reference signal
     * @param reference signal deque containing the reference signal.
     * @param resetTrajectory set equal to true if you do clear the old trajectory.
     * @return true/false in case of success/failure.
     */
    bool setReferenceSignal(const std::deque<iDynTree::Vector2>& referenceSignal,
                            const bool& resetTrajectory);

    /**
     * Solve the Optimization problem. If the MPCSolver is not set It will be initialized.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the output of the controller.
     * @param controllerOutput is the vector containing the output the controller.
     * @return true/false in case of success/failure.
     */
    bool getControllerOutput(iDynTree::Vector2& controllerOutput);
};

#endif
