/**
 * @file WalkingQPInverseKinematics.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_QP_IK_QPOASES_HPP
#define WALKING_QP_IK_QPOASES_HPP

#include <WalkingQPInverseKinematics.hpp>
#include <qpOASES.hpp>

class WalkingQPIK_qpOASES : public WalkingQPIK
{
    std::shared_ptr<qpOASES::SQProblem> m_optimizer{nullptr}; /**< Optimization solver. */

    std::vector<double> m_hessian;
    std::vector<double> m_gradient;

    std::vector<double>  m_constraintMatrix;

    std::vector<double>  m_upperBound;
    std::vector<double>  m_lowerBound;

    std::vector<double>  m_minJointLimit;
    std::vector<double>  m_maxJointLimit;

    bool m_isFirstTime;

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
     * @param minJointsLimit is a vector containing the min joints velocity limit;
     * @param maxJointsLimit is a vector containing the max joints velocity limit.
     * @return true/false in case of success/failure.
     */
    bool setVelocityBounds(const iDynTree::VectorDynSize& minJointsLimit,
                           const iDynTree::VectorDynSize& maxJointsLimit);

public:

    /**
     * Initialize the QP-IK problem.
     * @param config config of the QP-IK solver;
     * @param actuatedDOFs number of the actuated DoF
     * @param minJointsLimit is a vector containing the min joints velocity limit;
     * @param maxJointsLimit is a vector containing the max joints velocity limit.
     * @return true/false in case of success/failure.
     */
    virtual bool initialize(const yarp::os::Searchable& config,
                            const int& actuatedDOFs,
                            const iDynTree::VectorDynSize& minJointsLimit,
                            const iDynTree::VectorDynSize& maxJointsLimit) final;

    /**
     * Solve the optimization problem.
     * @return true/false in case of success/failure.
     */
    virtual bool solve() final;

    /**
     * Get the solution of the optimization problem.
     * @param output joint velocity (in rad/s).
     * @return true/false in case of success/failure.
     */
    virtual bool getSolution(iDynTree::VectorDynSize& output) final;

    /**
     * Get the error seen by the QP problem for the left foot.
     * @note it can be useful for debug
     * @param output error.
     * @return true/false in case of success/failure.
     */
    virtual bool getLeftFootError(iDynTree::VectorDynSize& output) final;

    /**
     * Get the error seen by the QP problem for the right foot.
     * @note it can be useful for debug
     * @param output error.
     * @return true/false in case of success/failure.
     */
    virtual bool getRightFootError(iDynTree::VectorDynSize& output) final;

};

#endif
