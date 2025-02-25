// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/RK4.h>

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_JOINT_INTEGRATORS
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_JOINT_INTEGRATORS

namespace WalkingControllers {

class JointAccelerationIntegrator {

  private:
    std::shared_ptr<
        BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>
        dynamics;
    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::RK4<
        BipedalLocomotion::ContinuousDynamicalSystem::
            LinearTimeInvariantSystem>>
        integrator;

    bool m_isInitialized{
        false}; /**< true if the integrator is successfully initialized */

    int m_numberOfJoints{0}; /**< number of joints */

    std::chrono::nanoseconds m_dT{0}; /**< time step */

    Eigen::VectorXd m_jointPosition; /**< joint position */
    Eigen::VectorXd m_jointVelocity; /**< joint velocity */

  public:
    /**
     * @brief Initialize the integrator.
     * @param numberOfJoints number of joints.
     * @param dt time step in seconds.
     * @return true if successful.
     */
    bool initialize(int numberOfJoints, double dt);

    /**
     * @brief Set the input of the integrator.
     * @param jointAcceleration joint acceleration.
     * @return true if successful.
     */
    bool setInput(const Eigen::Ref<const Eigen::VectorXd> &jointAcceleration);

    /**
     * @brief Set the state of the integrator.
     * @param jointposition joint position.
     * @param jointVelocity joint velocity.
     * @return true if successful.
     */
    bool setState(const Eigen::Ref<const Eigen::VectorXd> &jointposition,
                  const Eigen::Ref<const Eigen::VectorXd> &jointVelocity);

    /**
     * @brief Perform one step integration.
     * @return true if successful.
     */
    bool oneStepIntegration();

    /**
     * @brief Get the joint position.
     * @return joint position.
     */
    const Eigen::VectorXd &getJointPosition() const;

    /**
     * @brief Get the joint velocity.
     * @return joint velocity.
     */
    const Eigen::VectorXd &getJointVelocity() const;
};

} // namespace WalkingControllers

#endif // WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_JOINT_INTEGRATORS