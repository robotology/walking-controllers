// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <WalkingControllers/JointControllers/AdmittanceController.h>

#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace WalkingControllers;

bool AdmittanceController::initialize(
    std::shared_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        parametersHandler) {

    constexpr auto logPrefix = "[AdmittanceController::initialize]";

    if (parametersHandler == nullptr) {
        BipedalLocomotion::log()->error(
            "{} The parameter handler is not valid.", logPrefix);
        return false;
    }

    if (!parametersHandler->getParameter("number_of_joints",
                                         m_numberOfJoints)) {
        BipedalLocomotion::log()->error(
            "{} Unable to find the number_of_joints parameter.", logPrefix);
        return false;
    }

    auto gainsHandler = parametersHandler->getGroup("GAINS").lock();
    if (gainsHandler == nullptr) {
        BipedalLocomotion::log()->error("{} Unable to find the gains group.",
                                        logPrefix);
        return false;
    }

    if (!gainsHandler->getParameter("kp", m_gains.kp)) {
        BipedalLocomotion::log()->error("{} Unable to find the kp parameter.",
                                        logPrefix);
        return false;
    }

    if (!gainsHandler->getParameter("kd", m_gains.kd)) {
        BipedalLocomotion::log()->error("{} Unable to find the kd parameter.",
                                        logPrefix);
        return false;
    }

    if (m_gains.kp.size() != m_numberOfJoints) {
        BipedalLocomotion::log()->error(
            "{} The size of the proportional gain vector is "
            "not equal to the number of joints.",
            logPrefix);
        return false;
    }

    if (m_gains.kd.size() != m_numberOfJoints) {
        BipedalLocomotion::log()->error(
            "{} The size of the derivative gain vector "
            "is not equal to the number of joints.",
            logPrefix);
        return false;
    }

    m_gains.kp.resize(m_numberOfJoints);
    m_gains.kd.resize(m_numberOfJoints);

    m_input.jointTorqueFeedforward.resize(m_numberOfJoints);
    m_input.jointPositionDesired.resize(m_numberOfJoints);
    m_input.jointVelocityDesired.resize(m_numberOfJoints);
    m_input.jointPosition.resize(m_numberOfJoints);
    m_input.jointVelocity.resize(m_numberOfJoints);
    m_output.jointTorque.resize(m_numberOfJoints);

    m_isInitialized = true;

    return true;
};

bool AdmittanceController::setInput(
    const Eigen::Ref<const Eigen::VectorXd> &jointTorqueFeedforward,
    const Eigen::Ref<const Eigen::VectorXd> &jointPositionDesired,
    const Eigen::Ref<const Eigen::VectorXd> &jointVelocityDesired,
    const Eigen::Ref<const Eigen::VectorXd> &jointPosition,
    const Eigen::Ref<const Eigen::VectorXd> &jointVelocity) {

    if (!m_isInitialized) {
        return false;
    }
    if (jointTorqueFeedforward.size() != m_numberOfJoints) {
        return false;
    }
    if (jointPositionDesired.size() != m_numberOfJoints) {
        return false;
    }
    if (jointVelocityDesired.size() != m_numberOfJoints) {
        return false;
    }
    if (jointPosition.size() != m_numberOfJoints) {
        return false;
    }
    if (jointVelocity.size() != m_numberOfJoints) {
        return false;
    }

    m_input.jointPositionDesired = jointPositionDesired;
    m_input.jointVelocityDesired = jointVelocityDesired;
    m_input.jointPosition = jointPosition;
    m_input.jointVelocity = jointVelocity;

    return true;
};

const Eigen::VectorXd &
AdmittanceController::getOutput() const {
    return m_output.jointTorque;
};

bool AdmittanceController::advance() {

    constexpr auto logPrefix = "[AdmittanceController::advance]";

    m_output.jointTorque.setZero();

    if (!m_isInitialized) {
        BipedalLocomotion::log()->error(
            "{} The admittance controller is not initialized.", logPrefix);
        return false;
    }

    m_output.jointTorque =
        m_gains.kp.cwiseProduct(m_input.jointPositionDesired -
                                m_input.jointPosition) +
        m_gains.kd.cwiseProduct(m_input.jointVelocityDesired -
                                m_input.jointVelocity) +
        m_input.jointTorqueFeedforward;

    return true;
};
