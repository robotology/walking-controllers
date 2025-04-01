// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_ADMITTANCE_CONTROLLER
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_ADMITTANCE_CONTROLLER

#include <Eigen/Dense>
#include <memory>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace WalkingControllers {

class AdmittanceController {

    struct Input {
        Eigen::VectorXd jointTorqueFeedforward; /**< Joint position desired */
        Eigen::VectorXd jointPositionDesired;   /**< Joint position desired */
        Eigen::VectorXd jointVelocityDesired;   /**< Joint velocity desired*/
        Eigen::VectorXd jointPosition;          /**< Joint position */
        Eigen::VectorXd jointVelocity;          /**< Joint velocity */
    };
    Input m_input; /**< Input of the admittance controller */

    struct Output {
        Eigen::VectorXd jointTorque; /**< Joint Torque */
    };
    Output m_output; /**< Output of the admittance controller */

    bool m_isInitialized{false}; /**< true if the admittance controller is
                                    successfully initialized */
    struct Gains {
        Eigen::VectorXd kp; /**< Proportional gains */
        Eigen::VectorXd kd; /**< Derivative gains */
    };
    Gains m_gains; /**< gains of the admittance controller */

    int m_numberOfJoints; /**< Number of controlled joints */

  public:
    AdmittanceController() = default;

    ~AdmittanceController() = default;

    /**
     * @brief Initialize the admittance controller.
     * @param parametersHandler parameters handler.
     * @return true if successful.
     */
    bool
    initialize(std::shared_ptr<
               const BipedalLocomotion::ParametersHandler::IParametersHandler>
                   parametersHandler);
    /**
     * @brief Advance the admittance controller.
     * @return true if successful.
     */
    bool advance();

    /**
     * @brief Set the input to the admittance controller.
     * @return output of the admittance controller.
     */
    bool
    setInput(const Eigen::Ref<const Eigen::VectorXd> &jointTorqueFeedforward,
             const Eigen::Ref<const Eigen::VectorXd> &jointPositionDesired,
             const Eigen::Ref<const Eigen::VectorXd> &jointVelocityDesired,
             const Eigen::Ref<const Eigen::VectorXd> &jointPosition,
             const Eigen::Ref<const Eigen::VectorXd> &jointVelocity);

    /**
     * @brief Get the output of the admittance controller (i.e., joint torques).
     * @return output of the admittance controller.
     */
     const Eigen::VectorXd &getOutput() const;
};

} // namespace WalkingControllers

#endif // WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_ADMITTANCE_CONTROLLER