/**
 * @file WalkingZMPController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_ZMP_CONTROLLER_HPP
#define WALKING_ZMP_CONTROLLER_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>

/**
 * WalkingZMPController class implements the ZMP controller.
 * u = \int{kZMP (r_{zmp} - r_{zmp}^{des})
 *          - kCoM (x_{com} - x_{com}^{des})
 *          + \dot{x}_{com}^{des} dt}
 */
class WalkingZMPController
{
    double m_kZMP; /**< Controller gain. */
    double m_kCoM; /**< Controller gain. */

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

#endif
