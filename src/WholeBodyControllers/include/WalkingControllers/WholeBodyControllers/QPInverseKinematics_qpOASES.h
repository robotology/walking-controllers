/**
 * @file QPInverseKinematics_qpOASES.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_QPOASES_H
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_CONTROLLERS_QP_IK_QPOASES_H

#include <qpOASES.hpp>

#include <WalkingControllers/WholeBodyControllers/QPInverseKinematics.h>

namespace WalkingControllers
{
    class WalkingQPIK_qpOASES : public WalkingQPIK
    {
        std::unique_ptr<qpOASES::SQProblem> m_optimizer{nullptr}; /**< Optimization solver. */

        iDynTree::VectorDynSize  m_minJointLimit;
        iDynTree::VectorDynSize  m_maxJointLimit;

        bool m_isFirstTime;

        /**
         * Set joints velocity bounds
         * @return true/false in case of success/failure.
         */
        virtual void setJointVelocitiesBounds() final;

    protected:

        /**
         * Initialize the solver
         */
        virtual void instantiateSolver() final;

        /**
         * Set the number of constraints (it may change according to the solver used)
         */
        virtual void setNumberOfConstraints() final;

        /**
         * Initialize matrices that depends on the solver used
         */
        virtual void initializeSolverSpecificMatrices() final;

    public:

        /**
         * Solve the optimization problem.
         * @return true/false in case of success/failure.
         */
        virtual bool solve() final;
    };
};
#endif
