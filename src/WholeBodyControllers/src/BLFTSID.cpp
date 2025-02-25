// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <WalkingControllers/WholeBodyControllers/BLFTSID.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Position.h>
#include <iDynTree/Twist.h>
#include <iDynTree/VectorFixSize.h>

#include <manif/manif.h>

#include <memory>

using namespace WalkingControllers;

bool BLFTSID::initialize(
    std::weak_ptr<
        const BipedalLocomotion::ParametersHandler::IParametersHandler>
        handler,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn) {
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateTSID]";

    m_jointAccelerations.resize(kinDyn->getNrOfDegreesOfFreedom());
    m_jointTorques.resize(kinDyn->getNrOfDegreesOfFreedom());

    auto getTask = [this, logPrefix](const std::string &taskName,
                                     auto &task) -> bool {
        auto ptr = m_qpTSID.problem.tsid->getTask(taskName).lock();
        if (ptr == nullptr) {
            BipedalLocomotion::log()->error(
                "{} Unable to get the task named {}.", logPrefix, taskName);
            return false;
        }

        // cast the task
        task = std::dynamic_pointer_cast<
            typename std::remove_reference<decltype(task)>::type::element_type>(
            ptr);
        if (task == nullptr) {
            BipedalLocomotion::log()->error(
                "{} Unable to cast the task named {} to the expected "
                "type.",
                logPrefix, taskName);
            return false;
        }
        return true;
    };
    m_qpTSID.problem = BipedalLocomotion::TSID::QPTSID::build(handler, kinDyn);
    if (!m_qpTSID.problem.isValid()) {
        BipedalLocomotion::log()->error("{} Unable to initialize the TSID.",
                                        logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{}", m_qpTSID.problem.tsid->toString());

    return getTask("LEFT_FOOT_TRACKING", m_qpTSID.tasks.leftFootTracking) &&
           getTask("RIGHT_FOOT_TRACKING", m_qpTSID.tasks.rightFootTracking) &&
           getTask("COM_TRACKING", m_qpTSID.tasks.comTracking) //
           && getTask("TORSO_TRACKING", m_qpTSID.tasks.torsoTracking) &&
           getTask("JOINT_TRACKING", m_qpTSID.tasks.jointTracking) &&
           getTask("ROOT_TRACKING", m_qpTSID.tasks.rootTracking) &&
           getTask("BASE_DYNAMICS", m_qpTSID.tasks.baseDynamics) &&
           getTask("JOINT_DYNAMICS", m_qpTSID.tasks.jointDynamics) &&
           getTask("LEFT_FOOT_FEASIBLE_WRENCH",
                   m_qpTSID.tasks.leftFeasibleContactWrench) &&
           getTask("RIGHT_FOOT_FEASIBLE_WRENCH",
                   m_qpTSID.tasks.rightFeasibleContactWrench) &&
           getTask("LEFT_FOOT_WRENCH_REGULARIZATION",
                   m_qpTSID.tasks.leftContactWrenchRegularization) &&
           getTask("RIGHT_FOOT_WRENCH_REGULARIZATION",
                   m_qpTSID.tasks.rightContactWrenchRegularization) &&
           getTask("TORQUE_REGULARIZATION",
                   m_qpTSID.tasks.torqueRegularization);
    ;
}

bool BLFTSID::solve() {
    bool ok = m_qpTSID.problem.tsid->advance();
    ok = ok && m_qpTSID.problem.tsid->isOutputValid();

    if (ok) {
        iDynTree::toEigen(m_jointAccelerations) =
            m_qpTSID.problem.tsid->getOutput().jointAccelerations;
        iDynTree::toEigen(m_jointTorques) =
            m_qpTSID.problem.tsid->getOutput().jointTorques;
    }

    return ok;
}

bool BLFTSID::setLeftFootTrackingSetPoint(
    const iDynTree::Transform &desiredTransform,
    const iDynTree::Twist &desiredVelocity,
    const iDynTree::Twist &desiredAcceleration) {
    return m_qpTSID.tasks.leftFootTracking->setSetPoint(
        BipedalLocomotion::Conversions::toManifPose(desiredTransform),
        BipedalLocomotion::Conversions::toManifTwist(desiredVelocity),
        BipedalLocomotion::Conversions::toManifTwist(desiredAcceleration));
}

bool BLFTSID::setRightFootTrackingSetPoint(
    const iDynTree::Transform &desiredTransform,
    const iDynTree::Twist &desiredVelocity,
    const iDynTree::Twist &desiredAcceleration) {
    return m_qpTSID.tasks.rightFootTracking->setSetPoint(
        BipedalLocomotion::Conversions::toManifPose(desiredTransform),
        BipedalLocomotion::Conversions::toManifTwist(desiredVelocity),
        BipedalLocomotion::Conversions::toManifTwist(desiredAcceleration));
}

bool BLFTSID::setJointTrackingSetPoint(
    const iDynTree::VectorDynSize &jointPosition,
    const iDynTree::VectorDynSize &jointVelocity,
    const iDynTree::VectorDynSize &jointAcceleration) {
    return m_qpTSID.tasks.jointTracking->setSetPoint(
        iDynTree::toEigen(jointPosition), iDynTree::toEigen(jointVelocity),
        iDynTree::toEigen(jointAcceleration));
}

bool BLFTSID::setCoMTrackingSetPoint(const iDynTree::Position &position,
                                     const iDynTree::Vector3 &velocity,
                                     const iDynTree::Vector3 &acceleration) {
    return m_qpTSID.tasks.comTracking->setSetPoint(
        iDynTree::toEigen(position), iDynTree::toEigen(velocity),
        iDynTree::toEigen(acceleration));
}

bool BLFTSID::setRootTrackingSetPoint(
    const iDynTree::Position &position, const iDynTree::Vector3 &linearVelocity,
    const iDynTree::Vector3 &linearAcceleration) {
    if (m_useRootLinkForHeight) {
        return m_qpTSID.tasks.rootTracking->setSetPoint(
            iDynTree::toEigen(position), iDynTree::toEigen(linearVelocity),
            iDynTree::toEigen(linearAcceleration));
    }
    return true;
}

bool BLFTSID::setTorsoTrackingSetPoint(
    const iDynTree::Rotation &rotation,
    const iDynTree::Vector3 &angularVelocity,
    const iDynTree::Vector3 &angularAcceleration) {
    return m_qpTSID.tasks.torsoTracking->setSetPoint(
        BipedalLocomotion::Conversions::toManifRot(rotation),
        iDynTree::toEigen(angularVelocity),
        iDynTree::toEigen(angularAcceleration));
}

bool BLFTSID::setAngularMomentumTrackingSetPoint(
    const iDynTree::Vector3 &angularMomentum,
    const iDynTree::Vector3 &angularMomentumRate) {
    return m_qpTSID.tasks.angularMomentumTracking->setSetPoint(
        iDynTree::toEigen(angularMomentum),
        iDynTree::toEigen(angularMomentumRate));
}

bool BLFTSID::setLeftContactWrenchSetPoint(
    const iDynTree::Wrench &contactWrench) {
    return m_qpTSID.tasks.leftContactWrenchRegularization->setSetPoint(
        iDynTree::toEigen(contactWrench));
}

bool BLFTSID::setRightContactWrenchSetPoint(
    const iDynTree::Wrench &contactWrench) {
    return m_qpTSID.tasks.rightContactWrenchRegularization->setSetPoint(
        iDynTree::toEigen(contactWrench));
}

bool BLFTSID::setTorqueRegularizationSetPoint(
    const iDynTree::VectorDynSize &torque) {
    return m_qpTSID.tasks.torqueRegularization->setSetPoint(
        iDynTree::toEigen(torque));
}

void BLFTSID::setLeftContactActive(bool isActive) {
    m_qpTSID.tasks.leftFeasibleContactWrench->setContactActive(isActive);
}

void BLFTSID::setRightContactActive(bool isActive) {
    m_qpTSID.tasks.rightFeasibleContactWrench->setContactActive(isActive);
}

const iDynTree::VectorDynSize &BLFTSID::getDesiredJointAcceleration() const {
    return m_jointAccelerations;
}

const iDynTree::VectorDynSize &BLFTSID::getDesiredJointTorque() const {
    return m_jointTorques;
}