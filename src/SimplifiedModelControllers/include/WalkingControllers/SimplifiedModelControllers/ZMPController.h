/**
 * @file ZMPController.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_ZMP_CONTROLLER_H
#define WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_ZMP_CONTROLLER_H

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/minJerkCtrl.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Position.h>

namespace WalkingControllers
{

/**
 * WalkingZMPController class implements the ZMP controller.
 * u = \int{kZMP (r_{zmp} - r_{zmp}^{des})
 *          - kCoM (x_{com} - x_{com}^{des})
 *          + \dot{x}_{com}^{des} dt}
 */
    class WalkingZMPController
    {
        double m_kZMP; /**< CoM controller gain. */
        double m_kCoM; /**< ZMP controller gain. */

        double m_kCoMWalking; /**< Desired CoM controller gain during the walking phase. */
        double m_kZMPWalking; /**< Desired ZMP controller gain during the walking phase. */

        double m_kCoMStance; /**< Desired CoM controller gain during the stance phase. */
        double m_kZMPStance; /**< Desired ZMP controller gain during the stance phase. */

        bool m_controlEvaluated; /**< True if the control output was correctly evaluated. */

        iDynTree::Vector2 m_zmpFeedback; /**< Feedback signal containing the position of the ZMP. */
        iDynTree::Vector2 m_comFeedback; /**< Feedback signal containing the position of the CoM. */
        iDynTree::Vector2 m_zmpDesired; /**< Desired ZMP. */
        iDynTree::Vector2 m_comPositionDesired; /**< Desired CoM position. */
        iDynTree::Vector2 m_comVelocityDesired; /**< Desired CoM velocity. */

        iDynTree::Vector2 m_controllerOutput; /**< Controller output. */
        iDynTree::Vector2 m_desiredCoMVelocity; /**< Controller output. */

        /**
         * Pointer containing an integrator object.
         * It is useful to evaluate the desired CoM position from the CoM velocity.
         */
        std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};

        bool m_useGainScheduling; /**< True of the gain scheduling is used.*/
        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_kZMPSmoother; /**< Minimum jerk trajectory for the
                                                                       ZMP gain. */
        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_kCoMSmoother; /**< Minimum jerk trajectory for the
                                                                       CoM gain. */

    public:

        /**
         * Initialize the method
         * @param config yarp searchable configuration variable.
         * @return true/false in case of success/failure
         */
        bool initialize(const yarp::os::Searchable& config);

        /**
         * Set the feedback.
         * @param zmpFeedback is the ZMP position of the robot;
         * @param comFeedback is the CoM position of the robot.
         */
        void setFeedback(const iDynTree::Vector2& zmpFeedback,
                         const iDynTree::Position& comFeedback);

        /**
         * set the phase (Walking or stance)
         * @param isStancePhase true if the current phase is the stance phase.
         */
        void setPhase(const bool& isStancePhase);

        /**
         * Set the desired reference signals.
         * @param zmpDesired is the desired of the zero momentum point;
         * @param comPositionDesired is the desired position of the center of mass;
         * @param comVelocityDesired is the desired velocity of the center of mass.
         */
        void setReferenceSignal(const iDynTree::Vector2& zmpDesired,
                                const iDynTree::Vector2& comPositionDesired,
                                const iDynTree::Vector2& comVelocityDesired);

        /**
         * Evaluate the control output.
         * @return true/false in case of success/failure
         */
        bool evaluateControl();

        /**
         * Get the controller output.
         * @param controllerOutputPosition is the output position of the controller;
         * @param controllerOutputVelocity is the output velocity of the controller.
         * @return true/false in case of success/failure
         */
        bool getControllerOutput(iDynTree::Vector2& controllerOutputPosition,
                                 iDynTree::Vector2& controllerOutputVelocity);

        /**
         * Reset the controller
         * @param initialValue desired initial value of the controller
         * @return true/false in case of success/failure
         */
        bool reset(const iDynTree::Vector2& initialValue);
    };
};
#endif
