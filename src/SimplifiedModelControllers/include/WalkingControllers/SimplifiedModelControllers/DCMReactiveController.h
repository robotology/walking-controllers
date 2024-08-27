/**
 * @file DCMReactiveController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_DCM_REACTIVE_CONTROLLER_H
#define WALKING_CONTROLLERS_SIMPLIFIED_MODEL_CONTROLLERS_DCM_REACTIVE_CONTROLLER_H

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/VectorFixSize.h>

namespace WalkingControllers
{

/**
 * WalkingDCMReactiveController class implements the reactive
 * control for the DCM traking .
 * u = dcm - 1/omega * (Ddcm_des + kDCM * (dcm_des - dcm))
 */
    class WalkingDCMReactiveController
    {
        double m_kDCM; /**< Controller gain. */
        double m_omega; /**< LIPM time constant. */

        bool m_isInitialized{false}; /**< True if the control was initialized. */
        bool m_controlEvaluated{false}; /**< True if the control output was correctly evaluated. */

        iDynTree::Vector2 m_dcmFeedback; /**< Feedback signal containing the position of the CoM. */
        iDynTree::Vector2 m_dcmPositionDesired; /**< Desired CoM position. */
        iDynTree::Vector2 m_dcmVelocityDesired; /**< Desired CoM velocity. */

        iDynTree::Vector2 m_controllerOutput; /**< Controller output. */

    public:

        /**
         * Initialize the method
         * @param config yarp searchable configuration variable.
         * @return true/false in case of success/failure
         */
        bool initialize(const yarp::os::Searchable& config);

        /**
         * Set the feedback.
         * @param dcmFeedback is position of the robot DCM;
         */
        void setFeedback(const iDynTree::Vector2& dcmFeedback);

        /**
         * Set the desired reference signals.
         * @param dcmPositionDesired is the desired position of the DCM;
         * @param dcmVelocityDesired is the desired velocity of the DCM.
         */
        void setReferenceSignal(const iDynTree::Vector2& dcmPositionDesired,
                                const iDynTree::Vector2& dcmVelocityDesired);

        /**
         * Evaluate the control output.
         * @return true/false in case of success/failure
         */
        bool evaluateControl();

        /**
         * Get the controller output.
         * @return the output of the controller.
         */
        const iDynTree::Vector2& getControllerOutput() const;
    };
};

#endif
