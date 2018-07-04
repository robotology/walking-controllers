/**
 * @file WalkingQPInverseKinematics.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_QP_IK_HPP
#define WALKING_QP_IK_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

#include <OsqpEigen/OsqpEigen.h>
#include "Utils.hpp"

class WalkingQPIK_osqp
{
    iDynTree::MatrixDynSize m_comJacobian; /**< CoM jacobian (mixed representation). */
    iDynTree::MatrixDynSize m_neckJacobian; /**< Neck jacobian (mixed representation). */
    iDynTree::MatrixDynSize m_leftFootJacobian; /**< Left foot Jacobian (mixed representation). */
    iDynTree::MatrixDynSize m_rightFootJacobian; /**< Right foot Jacobian (mixed representation). */

    iDynTree::Twist m_leftFootTwist; /**< Desired Twist of the left foot. */
    iDynTree::Twist m_rightFootTwist; /**< Desired Twist of the right foot. */
    iDynTree::Vector3 m_comVelocity; /**< Desired Linear velocity of the CoM. */
    iDynTree::Position m_desiredComPosition; /**< Desired Linear velocity of the CoM. */
    iDynTree::Transform m_desiredLeftFootToWorldTransform; /**< Desired left foot to world transformation.*/
    iDynTree::Transform m_desiredRightFootToWorldTransform; /**< Desired right foot to world transformation.*/
    iDynTree::Rotation m_desiredNeckOrientation; /**< Desired neck orientation.*/
    iDynTree::Rotation m_additionalRotation; /**< Additional rotation matrix (it is useful to rotate the
                                                desiredNeckOrientation rotation matrix). */
    iDynTree::VectorDynSize m_regularizationTerm; /**< Desired joint position (regularization term).*/

    iDynTree::Position m_comPosition; /**< Desired Linear velocity of the CoM. */
    iDynTree::Transform m_leftFootToWorldTransform; /**< Actual left foot to world transformation.*/
    iDynTree::Transform m_rightFootToWorldTransform; /**< Actual right foot to world transformation.*/
    iDynTree::Rotation m_neckOrientation; /**< Rotation matrix of the actual neck orientation. */
    iDynTree::VectorDynSize m_jointPosition; /**< Actual joint position .*/

    int m_numberOfVariables; /**<Number of variables in the QP problem (# of joints + 6) */
    int m_numberOfConstraints; /**<Number of constraints in the QP problem (# of joints + 12) */
    std::unique_ptr<OsqpEigen::Solver> m_optimizerSolver; /**< Optimization solver. */

    iDynSparseMatrix m_jointRegulatizationGains;  /**< Gain related to the joint regularization. */
    double m_kPosFoot; /**< Gain related to the desired foot position. */
    double m_kAttFoot; /**< Gain related to the desired foot attitude. */
    double m_kNeck; /**< Gain related to the desired neck attitude. */
    double m_kCom; /**< Gain related to the desired CoM position. */
    iDynSparseMatrix m_comWeightMatrix; /**< CoM weight matrix. */
    iDynSparseMatrix m_neckWeightMatrix; /**< Neck weight matrix. */
    iDynSparseMatrix m_jointRegulatizationHessian; /**< Contains a constant matrix that can be useful
                                                      in the hessian evaluation ($-\lambda H' H$). */
    iDynSparseMatrix m_jointRegulatizationGradient; /**< Contains a constant matrix that can be useful
                                                       in the gradient evaluation ($-\lambda H'$). */
    iDynTree::Triplets m_jointRegularizationLinearConstraintTriplets; /**< Contains a set of triplets
                                                                       useful in during the evaluation
                                                                       of linear Constraint matrix */
    Eigen::VectorXd m_lowerBound; /**< Lower bound vector. */
    Eigen::VectorXd m_upperBound; /**< Upper bound vector. */
    Eigen::VectorXd m_gradient; /**< Gradient vector. */
    Eigen::MatrixXd m_hessianEigenDense;
    Eigen::MatrixXd m_constraintsMatrixEigenDense;

    int m_actuatedDOFs; /**< Number of actuated actuated DoF. */

    bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

    bool m_useCoMAsConstraint; /**< True if the CoM is added as a constraint. */

    /**
     * Initialize all the constant matrix from the configuration file.
     * @return true/false in case of success/failure.
     */
    bool initializeMatrices(const yarp::os::Searchable& config);

    /**
     * Set the Hessian matrix.
     * If the optimization problem was already initialized the hessian matrix is updated.
     * @return true/false in case of success/failure.
     */
    bool setHessianMatrix();

    /**
     * Set the gradient vector.
     * If the optimization problem was already initialized the gradient is updated.
     * @return true/false in case of success/failure.
     */
    bool setGradientVector();

    /**
     * Set the Linear constraint matrix.
     * If the optimization problem was already initialized the constraint matrix is updated.
     * @return true/false in case of success/failure.
     */
    bool setLinearConstraintMatrix();

    /**
     * Set Lower and upper bounds
     * If the optimization problem was already initialized the bounds are updated.
     * @return true/false in case of success/failure.
     */
    bool setBounds();

    /**
     * Set joints velocity bounds
     * @param the minJointsLimit is a vector containing the min joints velocity limit;
     * @param the minJointsLimit is a vector containing the max joints velocity limit.
     * @return true/false in case of success/failure.
     */
    bool setVelocityBounds(const iDynTree::VectorDynSize& minJointsLimit,
                           const iDynTree::VectorDynSize& maxJointsLimit);

    /**
     * Check if the solution is feasible.
     * @return true if the solution is feasible false otherwise
     */
    bool isSolutionFeasible();

public:

    /**
     * Initialize the QP-IK problem.
     * @param config config of the QP-IK solver;
     * @param actuatedDOFs number of the actuated DoF
     * @param the minJointsLimit is a vector containing the min joints velocity limit;
     * @param the minJointsLimit is a vector containing the max joints velocity limit.
     * @return true/false in case of success/failure.
     */
    bool initialize(const yarp::os::Searchable& config,
                    const int& actuatedDOFs,
                    const iDynTree::VectorDynSize& minJointsLimit,
                    const iDynTree::VectorDynSize& maxJointsLimit);

    /**
     * Set the robot state.
     * @param jointPosition vector of joint positions (in rad);
     * @param leftFootToWordTransformation transformation between the inertial frame and the left foot;
     * @param rightFootToWordTransformation transformation between the inertial frame and the right foot;
     * @param neckOrientation rotation between the inertial frame and the neck;
     * @return true/false in case of success/failure.
     */
    bool setRobotState(const iDynTree::VectorDynSize& jointPosition,
                       const iDynTree::Transform& leftFootToWorldTransform,
                       const iDynTree::Transform& rightFootToWorldTransform,
                       const iDynTree::Rotation& neckOrientation,
                       const iDynTree::Position& comPosition);

    /**
     * Set the Jacobian of the CoM
     * @param comJacobian jacobian of the CoM (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setCoMJacobian(const iDynTree::MatrixDynSize& comJacobian);

    /**
     * Set the Jacobian of the left foot
     * @param leftFootJacobian jacobian of the left foot (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setLeftFootJacobian(const iDynTree::MatrixDynSize& leftFootJacobian);

    /**
     * Set the Jacobian of the right foot
     * @param leftFootJacobian jacobian of the right foot (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian);

    /**
     * Set the Jacobian of the neck
     * @param leftFootJacobian jacobian of the neck foot (mixed representation)
     * @return true/false in case of success/failure.
     */
    bool setNeckJacobian(const iDynTree::MatrixDynSize& neckJacobian);

    /**
     * Set the desired joint position.
     * Please use this term as regularization term.
     * @param regularizationTerm vector of the desired joint position.
     * @return true/false in case of success/failure.
     */
    bool setDesiredJointPosition(const iDynTree::VectorDynSize& regularizationTerm);

    /**
     * Set the desired twist of both feet
     * @param leftFootTwist contain the desired twist of the left foot (MIXED representation);
     * @param rightFootTwist contain the desired twist of the right foot (MIXED representation).
     */
    void setDesiredFeetTwist(const iDynTree::Twist& leftFootTwist,
                             const iDynTree::Twist& rightFootTwist);

    /**
     * Set the desired CoMVelocity
     * @param comVelocity contain the desired CoM velocity.
     */
    void setDesiredCoMVelocity(const iDynTree::Vector3& comVelocity);

    /**
     * Set the desired feet transformation
     * @param desiredLeftFootToWorldTransform desired transformation between the left foot and the world frame;
     * @param desiredRightFootToWorldTransform desired transformation between the right foot and the world frame.
     */
    void setDesiredFeetTransformation(const iDynTree::Transform& desiredLeftFootToWorldTransform,
                                      const iDynTree::Transform& desiredRightFootToWorldTransform);

    /**
     * Set the desired orientation of the neck
     * @param desiredNeckOrientation rotation matrix between the neck and the world frame.
     */
    void setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation);

    /**
     * Set the desired CoMVelocity
     * @param comVelocity contain the desired CoM velocity.
     */
    void setDesiredCoMPosition(const iDynTree::Position& desiredComPosition);

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    bool solve();

    /**
     * Get the solution of the optimization problem.
     * @param output joint velocity (in rad/s).
     * @return true/false in case of success/failure.
     */
    bool getSolution(iDynTree::VectorDynSize& output);

    /**
     * Get the error seen by the QP problem for the left foot.
     * @note it can be useful for debug
     * @param output error.
     * @return true/false in case of success/failure.
     */
    bool getLeftFootError(iDynTree::VectorDynSize& output);

    /**
     * Get the error seen by the QP problem for the right foot.
     * @note it can be useful for debug
     * @param output error.
     * @return true/false in case of success/failure.
     */
    bool getRightFootError(iDynTree::VectorDynSize& output);

    /**
     * Get the neck orientation error.
     * @param output error
     * @return true/false in case of success/failure.
     */
    bool getNeckOrientationError(iDynTree::Vector3& output);

    const Eigen::MatrixXd& getHessianMatrix() const;

    const Eigen::MatrixXd& getConstraintMatrix() const;

    const Eigen::VectorXd& getUpperBound() const;

    const Eigen::VectorXd& getLowerBound() const;

    const Eigen::VectorXd& getGradient() const;

};

#endif
