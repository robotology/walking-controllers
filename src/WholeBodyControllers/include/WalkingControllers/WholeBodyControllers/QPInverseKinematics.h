/**
 * @file QPInverseKinematics.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>, Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_H
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

//iDynTree
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>

#include <WalkingControllers/iDynTreeUtilities/Helper.h>

namespace WalkingControllers
{

    template<class T>
        void copyDenseIntoSparse(const T& dense, const int& startingRow, const int& startingColumn,
                                 iDynSparseMatrix& sparse)
    {
        for(int i = 0; i < dense.rows(); i++)
            for(int j = 0; j < dense.cols(); j++)
                sparse(startingRow + i, startingColumn + j) = dense(i, j);
    }

    class WalkingQPIK
    {
    private:

        /**
         * Set the joint positions and velocities bounds
         * @param jointVelocitiesBounds  joint velocities bounds in [rad/s]
         * @param jointPositionsUpperBounds joint position upper bounds in [rad]
         * @param jointPositionsLowerBounds joint position lower bounds in [rad]
         * @return true/false in case of success/failure.
         */
        bool setJointsBounds(const iDynTree::VectorDynSize& jointVelocitiesBounds,
                             const iDynTree::VectorDynSize& jointPositionsUpperBounds,
                             const iDynTree::VectorDynSize& jointPositionsLowerBounds);

    protected:
        iDynTree::MatrixDynSize m_comJacobian; /**< CoM jacobian (mixed representation). */
        iDynTree::MatrixDynSize m_neckJacobian; /**< Neck jacobian (mixed representation). */
        iDynTree::MatrixDynSize m_leftFootJacobian; /**< Left foot Jacobian (mixed representation). */
        iDynTree::MatrixDynSize m_rightFootJacobian; /**< Right foot Jacobian (mixed representation). */
        iDynTree::MatrixDynSize m_leftHandJacobian; /**< Left hand Jacobian (mixed representation). */
        iDynTree::MatrixDynSize m_rightHandJacobian; /**< Right hand Jacobian (mixed representation). */

        iDynTree::Twist m_desiredLeftFootTwist; /**< Desired Twist of the left foot. */
        iDynTree::Twist m_desiredRightFootTwist; /**< Desired Twist of the right foot. */
        iDynTree::Twist m_leftFootCorrection; /**< Correction of the desired velocity related to the left foot (evaluated using the position error). */
        iDynTree::Twist m_rightFootCorrection; /**< Correction of the desired velocity related to the left foot (evaluated using the position error). */

        iDynTree::Twist m_desiredLeftHandTwist; /**< Desired Twist of the left hand. */
        iDynTree::Twist m_desiredRightHandTwist; /**< Desired Twist of the right hand. */
        iDynTree::Vector3 m_desiredComVelocity; /**< Desired Linear velocity of the CoM. */

        iDynTree::Position m_desiredComPosition; /**< Desired Linear velocity of the CoM. */
        iDynTree::Transform m_desiredLeftFootToWorldTransform; /**< Desired left foot to world transformation.*/
        iDynTree::Transform m_desiredRightFootToWorldTransform; /**< Desired right foot to world transformation.*/
        iDynTree::Transform m_desiredLeftHandToWorldTransform; /**< Desired left hand to world transformation.*/
        iDynTree::Transform m_desiredRightHandToWorldTransform; /**< Desired right hand to world transformation.*/
        iDynTree::Twist m_leftHandCorrection; /**< Correction of the desired velocity related to the left hand (evaluated using the position error). */
        iDynTree::Twist m_rightHandCorrection; /**< Correction of the desired velocity related to the left hand (evaluated using the position error). */
        iDynTree::Rotation m_desiredNeckOrientation; /**< Desired neck orientation.*/
        iDynTree::Rotation m_additionalRotation; /**< Additional rotation matrix (it is useful to rotate the
                                                    desiredNeckOrientation rotation matrix). */

        iDynTree::VectorDynSize m_regularizationTerm; /**< Desired joint position (regularization term).*/
        iDynTree::VectorDynSize m_retargetingJointValue; /**< Desired joint retargeting position.*/

        iDynTree::Position m_comPosition; /**< Desired Linear velocity of the CoM. */
        iDynTree::Transform m_leftFootToWorldTransform; /**< Actual left foot to world transformation.*/
        iDynTree::Transform m_rightFootToWorldTransform; /**< Actual right foot to world transformation.*/
        iDynTree::Transform m_leftHandToWorldTransform; /**< Actual left foot to world transformation.*/
        iDynTree::Transform m_rightHandToWorldTransform; /**< Actual right foot to world transformation.*/
        iDynTree::Rotation m_neckOrientation; /**< Rotation matrix of the actual neck orientation. */
        iDynTree::VectorDynSize m_jointPosition; /**< Actual joint position .*/

        int m_numberOfVariables; /**<Number of variables in the QP problem (# of joints + 6) */
        int m_numberOfConstraints; /**<Number of constraints in the QP problem (# of joints + 12) */
        int m_actuatedDOFs; /**< Number of actuated actuated DoF. */

        iDynTree::VectorDynSize m_jointRegularizationGains;  /**< Gain related to the joint regularization. */
        iDynTree::VectorDynSize m_jointRetargetingGains;  /**< Gain related to the joint regularization. */
        double m_kPosFoot; /**< Gain related to the desired foot position. */
        double m_kAttFoot; /**< Gain related to the desired foot attitude. */
        double m_kPosHand; /**< Gain related to the desired Hand position. */
        double m_kAttHand; /**< Gain related to the desired hand attitude. */
        double m_kNeck; /**< Gain related to the desired neck attitude. */
        double m_kCom; /**< Gain related to the desired CoM position. */

        iDynTree::VectorDynSize m_jointRegularizationWeights; /**< Weight related to the the regularization term */
        iDynTree::Vector3 m_comWeight; /**< CoM weight. */
        double m_neckWeight; /**< Neck weight matrix. */
        iDynSparseMatrix m_jointRegularizationHessian; /**< Contains a constant matrix that can be useful
                                                          in the hessian evaluation ($-\lambda H' H$). */
        iDynSparseMatrix m_jointRegularizationGradient; /**< Contains a constant matrix that can be useful
                                                           in the gradient evaluation ($-\lambda H'$). */
        double m_kJointLimitsUpperBound; /**< Gain related to the the joint upper bound */
        double m_kJointLimitsLowerBound; /**< Gain related to the the joint lower bound */
        iDynTree::VectorDynSize m_jointVelocitiesBounds; /**< Bounds on the joint velocities*/
        iDynTree::VectorDynSize m_jointPositionsUpperBounds; /**< Upper Bounds on the joint position*/
        iDynTree::VectorDynSize m_jointPositionsLowerBounds; /**< Lower Bounds on the joint position*/

        // gain scheduling
        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_handWeightSmoother; /**< Minimum jerk trajectory
                                                                             for the hand weight matrix. */
        yarp::sig::Vector m_handWeightWalkingVector; /**< Weight matrix (only the diagonal) used for
                                                        the hand retargeting during walking. */
        yarp::sig::Vector m_handWeightStanceVector; /**< Weight matrix (only the diagonal) used for
                                                       the hand retargeting during stance. */

        // gain scheduling in case of joint retargeting
        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_jointRetargetingWeightSmoother; /**< Minimum jerk trajectory
                                                                                         for the Joint retargeting weight matrix. */
        yarp::sig::Vector m_jointRetargetingWeightWalking; /**< Weight matrix (only the diagonal) used for
                                                              the joint retargeting during walking. */
        yarp::sig::Vector m_jointRetargetingWeightStance; /**< Weight matrix (only the diagonal) used for
                                                             the joint retargeting during stance. */

        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_jointRegularizationWeightSmoother; /**< Minimum jerk trajectory
                                                                                            for the Joint regularization weight matrix. */
        yarp::sig::Vector m_jointRegularizationWeightWalking; /**< Weight matrix (only the diagonal) used for
                                                                 the joint regularization during walking. */
        yarp::sig::Vector m_jointRegularizationWeightStance; /**< Weight matrix (only the diagonal) used for
                                                                the joint regularization during stance. */

        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_torsoWeightSmoother; /**< Minimum jerk trajectory for the torso weight matrix. */
        double m_torsoWeightWalking; /**< Weight matrix (only the diagonal) used for the torso during walking. */
        double m_torsoWeightStance; /**< Weight matrix (only the diagonal) used for the torso during stance. */

        bool m_useCoMAsConstraint; /**< True if the CoM is added as a constraint. */
        bool m_useJointsLimitsConstraint; /**< True if the CoM is added as a constraint. */
        bool m_enableHandRetargeting; /**< True if the hand retargeting is used */
        bool m_enableJointRetargeting; /**< True if the joint retargeting is used */

        iDynTree::MatrixDynSize m_hessianDense; /**< Hessian matrix */
        iDynTree::VectorDynSize m_gradient; /**< Gradient vector */
        iDynSparseMatrix m_constraintsMatrixSparse; /**< Constraint matrix */
        iDynTree::VectorDynSize m_lowerBound; /**< Lower bound */
        iDynTree::VectorDynSize m_upperBound; /**< Upper bound */
        iDynTree::VectorDynSize m_solution; /**< Solution of the optimization problem */
        iDynTree::VectorDynSize m_desiredJointVelocitiesOutput; /**< Joint velocities required by the optimization problem */

        /**
         * Initialize all the constant matrix from the configuration file.
         * @param config configuration parameters
         * @return true/false in case of success/failure.
         */
        bool initializeMatrices(const yarp::os::Searchable& config);

        /**
         * Initialize hand retargeting.
         * @param config configuration parameters
         * @return true/false in case of success/failure.
         */
        bool initializeHandRetargeting(const yarp::os::Searchable& config);

        /**
         * Initialize joint retargeting.
         * @param config configuration parameters
         * @return true/false in case of success/failure.
         */
        bool initializeJointRetargeting(const yarp::os::Searchable& config);


        /**
         * Instantiate the solver
         */
        virtual void instantiateSolver() = 0;

        /**
         * Evaluate the Hessian matrix.
         */
        void evaluateHessianMatrix();

        /**
         * Evaluate the gradient vector.
         */
        void evaluateGradientVector();

        /**
         * Evaluate the Linear constraint matrix.
         */
        void evaluateLinearConstraintMatrix();

        /**
         * Evaluate Lower and upper bounds
         */
        void evaluateBounds();

        /**
         * Set the number of constraints (it may change according to the solver used)
         */
        virtual void setNumberOfConstraints(){;};

        /**
         * Initialize matrices that depends on the solver used
         */
        virtual void initializeSolverSpecificMatrices(){;};

        /**
         * Set joint velocities bounds
         */
        virtual void setJointVelocitiesBounds() = 0;

    public:
        /**
         * Initialize the QP-IK problem.
         * @param config config of the QP-IK solver
         * @param actuatedDOFs number of the actuated DoF
         * @param minJointsPosition is a vector containing the min joints position limit
         * @param minJointsPosition is a vector containing the max joints position limit
         * @param maxJointsVelocity is a vector containing the max joints velocity limit.
         * @return true/false in case of success/failure.
         */
        bool initialize(const yarp::os::Searchable &config,
                        const int &actuatedDOFs,
                        const iDynTree::VectorDynSize& maxJointsVelocity,
                        const iDynTree::VectorDynSize& maxJointsPosition,
                        const iDynTree::VectorDynSize& minJointsPosition);

        /**
         * Set the robot state.
         * @param jointPosition vector of joint positions (in rad);
         * @param leftFootToWorldTransform transformation between the inertial frame and the left foot;
         * @param rightFootToWorldTransform transformation between the inertial frame and the right foot;
         * @param leftHandToWorldTransform transformation between the inertial frame and the left foot;
         * @param rightHandToWorldTransform transformation between the inertial frame and the right foot;
         * @param neckOrientation rotation between the inertial frame and the neck;
         * @param comPosition position of the CoM
         * @return true/false in case of success/failure.
         */
        bool setRobotState(const iDynTree::VectorDynSize& jointPosition,
                           const iDynTree::Transform& leftFootToWorldTransform,
                           const iDynTree::Transform& rightFootToWorldTransform,
                           const iDynTree::Transform& leftHandToWorldTransform,
                           const iDynTree::Transform& rightHandToWorldTransform,
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
         * @param rightFootJacobian jacobian of the right foot (mixed representation)
         * @return true/false in case of success/failure.
         */
        bool setRightFootJacobian(const iDynTree::MatrixDynSize& rightFootJacobian);

        /**
         * Set the Jacobian of the left hand
         * @param leftHandJacobian jacobian of the left hand (mixed representation)
         * @return true/false in case of success/failure.
         */
        bool setLeftHandJacobian(const iDynTree::MatrixDynSize& leftHandJacobian);

        /**
         * Set the Jacobian of the right hand
         * @param rightHandJacobian jacobian of the right hand (mixed representation)
         * @return true/false in case of success/failure.
         */
        bool setRightHandJacobian(const iDynTree::MatrixDynSize& rightHandJacobian);

        /**
         * Set the Jacobian of the neck
         * @param neckJacobian jacobian of the neck (mixed representation)
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
         * Set the desired twist of both feet
         * @param leftHandTwist contain the desired twist of the left hand (MIXED representation);
         * @param rightHandTwist contain the desired twist of the right hand (MIXED representation).
         */
        void setDesiredHandsTwist(const iDynTree::Twist& leftHandTwist,
                                  const iDynTree::Twist& rightHandTwist);

        /**
         * Set the desired twist of both feet
         * @param jointPosition contain the desired joint position used in the retargeting.
         * @return true/false in case of success/failure.
         */
        bool setDesiredRetargetingJoint(const iDynTree::VectorDynSize& jointPosition);

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
         * Set the desired hand transformation
         * @param desiredLeftHandToWorldTransform desired transformation between the left hand and the world frame;
         * @param desireRightHandToWorldTransform desired transformation between the right hand and the world frame.
         */
        void setDesiredHandsTransformation(const iDynTree::Transform& desiredLeftHandToWorldTransform,
                                           const iDynTree::Transform& desiredRightHandToWorldTransform);

        /**
         * Set the desired orientation of the neck
         * @param desiredNeckOrientation rotation matrix between the neck and the world frame.
         */
        void setDesiredNeckOrientation(const iDynTree::Rotation& desiredNeckOrientation);

        /**
         * Set the desired CoMPosition
         * @param desiredComPosition contain the desired CoM position.
         */
        void setDesiredCoMPosition(const iDynTree::Position& desiredComPosition);

        /**
         * Set the robot phase. This is used by the minimum jerk trajectory.
         * @param isStancePhase true if the robot is in the stance phase
         */
        void setPhase(const bool& isStancePhase);

        /**
         * Solve the optimization problem.
         * @return true/false in case of success/failure.
         */
        virtual bool solve() = 0;

        /**
         * Get the solution of the optimization problem (base velocity + joint velocities)
         * @return the solution of the optimization problem
         */
        const iDynTree::VectorDynSize& getSolution() const;

        /**
         * Get the solution of the optimization problem (only joint velocities)
         * @return the solution of the optimization problem
         */
        const iDynTree::VectorDynSize& getDesiredJointVelocities() const;

        /**
         * Get the error seen by the QP problem for the left foot.
         * @note it can be useful for debug
         * @return the error
         */
        iDynTree::VectorDynSize getLeftFootError();

        /**
         * Get the error seen by the QP problem for the right foot.
         * @note it can be useful for debug
         * @return the error
         */
        iDynTree::VectorDynSize getRightFootError();

        /**
         * Get the hessian matrix
         * @return the hessian matrix
         */
        const iDynTree::MatrixDynSize& getHessianMatrix() const;

        /**
         * Get the gradient vector
         * @return the gradient vector
         */
        const iDynTree::VectorDynSize& getGradient() const;

        /**
         * Get the Constraint Matrix
         * @return the constraint matrix
         */
        const iDynSparseMatrix& getConstraintMatrix() const;

        /**
         * Get the upper bound
         * @return the upper bound
         */
        const iDynTree::VectorDynSize& getUpperBound() const;

        /**
         * Get the lower bound
         * @return the lower bound
         */
        const iDynTree::VectorDynSize& getLowerBound() const;
    };
};

#endif
