// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <WalkingControllers/Integrators/jointIntegrators.h>

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <chrono>

using namespace WalkingControllers;

bool JointAccelerationIntegrator::initialize(int numberOfJoints, double dt) {

    constexpr auto logPrefix = "[JointAccelerationIntegrator::initialize]";

    dynamics = std::make_shared<BipedalLocomotion::ContinuousDynamicalSystem::
                                    LinearTimeInvariantSystem>();

    m_numberOfJoints = numberOfJoints;

    // Set joints state space system matrices.
    //
    // [s_dot; s_ddot] = [0, I; 0, 0] * [s; s_dot] + [0; I] * sddot
    //
    // therefore
    //
    // A = [0, I; 0, 0]
    // b = [0; 1]
    Eigen::MatrixXd A =
        Eigen::MatrixXd::Zero(2 * m_numberOfJoints, 2 * m_numberOfJoints);
    Eigen::MatrixXd B =
        Eigen::MatrixXd::Zero(2 * m_numberOfJoints, m_numberOfJoints);
    A.block(0, m_numberOfJoints, m_numberOfJoints, m_numberOfJoints)
        .setIdentity();
    B.block(m_numberOfJoints, 0, m_numberOfJoints, m_numberOfJoints)
        .setIdentity();

    if (!dynamics->setSystemMatrices(A, B)) {
        BipedalLocomotion::log()->error(
            "{} Failed to set system matrices of the dynamical system",
            logPrefix);
        return false;
    };

    // set dynamical system of the integrator
    integrator =
        std::make_shared<BipedalLocomotion::ContinuousDynamicalSystem::RK4<
            BipedalLocomotion::ContinuousDynamicalSystem::
                LinearTimeInvariantSystem>>();
    if (!integrator->setDynamicalSystem(dynamics)) {
        BipedalLocomotion::log()->error(
            "{} Failed to set dynamical system of the integrator", logPrefix);
        return false;
    };

    // set integration step
    m_dT = std::chrono::milliseconds(static_cast<int>(dt * 1000));
    if (!integrator->setIntegrationStep(m_dT)) {
        BipedalLocomotion::log()->error(
            "{} Failed to set integration step of the integrator", logPrefix);
        return false;
    };

    m_jointPosition.resize(m_numberOfJoints);
    m_jointPosition.setZero();
    m_jointVelocity.resize(m_numberOfJoints);
    m_jointVelocity.setZero();

    m_isInitialized = true;
    return true;
}

bool JointAccelerationIntegrator::setInput(
    const Eigen::Ref<const Eigen::VectorXd> &jointAcceleration) {

    constexpr auto logPrefix = "[JointAccelerationIntegrator::setInput]";

    if (!m_isInitialized) {
        BipedalLocomotion::log()->error("{} Integrator is not initialized",
                                        logPrefix);
        return false;
    }

    if (jointAcceleration.size() != m_numberOfJoints) {
        BipedalLocomotion::log()->error("{} Size of joint acceleration is not "
                                        "equal to the number of joints",
                                        logPrefix);
        return false;
    }

    if (!dynamics->setControlInput({jointAcceleration})) {
        BipedalLocomotion::log()->error(
            "{} Failed to set input of the dynamical system", logPrefix);
        return false;
    }

    return true;
}

bool JointAccelerationIntegrator::setState(
    const Eigen::Ref<const Eigen::VectorXd> &jointposition,
    const Eigen::Ref<const Eigen::VectorXd> &jointVelocity) {

    constexpr auto logPrefix = "[JointAccelerationIntegrator::setState]";

    if (!m_isInitialized) {
        BipedalLocomotion::log()->error("{} Integrator is not initialized",
                                        logPrefix);
        return false;
    }

    if (jointposition.size() != m_numberOfJoints) {
        BipedalLocomotion::log()->error(
            "{} Size of joint position is not equal to the number of joints",
            logPrefix);
        return false;
    }

    if (jointVelocity.size() != m_numberOfJoints) {
        BipedalLocomotion::log()->error(
            "{} Size of joint velocity is not equal to the number of joints",
            logPrefix);
        return false;
    }

    Eigen::VectorXd state(2 * m_numberOfJoints);
    state << jointposition, jointVelocity;

    if (!dynamics->setState({state})) {
        BipedalLocomotion::log()->error(
            "[JointAccelerationIntegrator::setState] Failed to set state of "
            "the dynamical system");
        return false;
    }

    return true;
}

bool JointAccelerationIntegrator::oneStepIntegration() {

    constexpr auto logPrefix =
        "[JointAccelerationIntegrator::oneStepIntegration]";

    if (!m_isInitialized) {
        BipedalLocomotion::log()->error("{} Integrator is not initialized",
                                        logPrefix);
        return false;
    }

    if (!integrator->oneStepIntegration(std::chrono::nanoseconds(0), m_dT)) {
        BipedalLocomotion::log()->error("{} Failed to advance the integrator",
                                        logPrefix);
        return false;
    }

    const auto &[stateX] = integrator->getSolution();
    m_jointPosition = stateX.head(m_numberOfJoints);
    m_jointVelocity = stateX.tail(m_numberOfJoints);

    return true;
}

const Eigen::VectorXd &JointAccelerationIntegrator::getJointPosition() const {
    const auto &[stateX] = integrator->getSolution();
    return m_jointPosition;
}

const Eigen::VectorXd &JointAccelerationIntegrator::getJointVelocity() const {
    return m_jointVelocity;
}