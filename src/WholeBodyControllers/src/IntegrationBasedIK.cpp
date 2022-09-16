/**
 * @file IntegrationBasedIK.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <WalkingControllers/WholeBodyControllers/IntegrationBasedIK.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>

#include <manif/manif.h>

#include <memory>

using namespace WalkingControllers;

bool IntegrationBasedIK::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto prefix = "[IntegrationBasedIK::initialize]";
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid parameter handler", prefix);
        return false;
    }

    m_usejointRetargeting = false;
    int usejointRetargeting = 0;
    ptr->getParameter("use_joint_retargeting", usejointRetargeting);
    m_usejointRetargeting = usejointRetargeting != 0;

    m_useRootLinkForHeight = false;
    ptr->getParameter("use_root_link_for_height", m_useRootLinkForHeight);

    // weight providers
    bool ok = m_qpIK.initialize(ptr->getGroup("IK"));
    auto group = ptr->getGroup("IK").lock();
    std::string variable;
    group->getParameter("robot_velocity_variable_name", variable);
    m_variableHandler.addVariable(variable, kinDyn->getNrOfDegreesOfFreedom() + 6);

    m_torsoWeight
        = std::make_shared<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>();
    ok = ok && m_torsoWeight->initialize(ptr->getGroup("TORSO_TASK"));

    if (m_usejointRetargeting)
    {
        m_jointRetargetingWeight = std::make_shared<
            BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>();
        ok = ok && m_jointRetargetingWeight->initialize(ptr->getGroup("JOINT_RETARGETING_TASK"));
    }

    m_jointRegularizationWeight
        = std::make_shared<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>();
    ok = ok && m_jointRegularizationWeight->initialize(ptr->getGroup("JOINT_REGULARIZATION_TASK"));

    // CoM Task
    // TODO remvove the z
    m_comTask = std::make_shared<BipedalLocomotion::IK::CoMTask>();
    ok = ok && m_comTask->setKinDyn(kinDyn);
    ok = ok && m_comTask->initialize(ptr->getGroup("COM_TASK"));
    ok = ok && m_qpIK.addTask(m_comTask, "com_task", highPriority);

    m_rightFootTask = std::make_shared<BipedalLocomotion::IK::SE3Task>();
    ok = ok && m_rightFootTask->setKinDyn(kinDyn);
    ok = ok && m_rightFootTask->initialize(ptr->getGroup("RIGHT_FOOT_TASK"));
    ok = ok && m_qpIK.addTask(m_rightFootTask, "right_foot_task", highPriority);

    m_leftFootTask = std::make_shared<BipedalLocomotion::IK::SE3Task>();
    ok = ok && m_leftFootTask->setKinDyn(kinDyn);
    ok = ok && m_leftFootTask->initialize(ptr->getGroup("LEFT_FOOT_TASK"));
    ok = ok && m_qpIK.addTask(m_leftFootTask, "left_foot_task", highPriority);


    m_torsoTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_torsoTask->setKinDyn(kinDyn);
    ok = ok && m_torsoTask->initialize(ptr->getGroup("TORSO_TASK"));
    ok = ok &&  m_qpIK.addTask(m_torsoTask, "torso_task", lowPriority, m_torsoWeight);

    m_jointRegularizationTask = std::make_shared<BipedalLocomotion::IK::JointTrackingTask>();
    ok = ok && m_jointRegularizationTask->setKinDyn(kinDyn);
    ok = ok && m_jointRegularizationTask->initialize(ptr->getGroup("JOINT_REGULARIZATION_TASK"));
    ok = ok &&  m_qpIK.addTask(m_jointRegularizationTask,
                               "joint_regularization_task",
                               lowPriority,
                               m_jointRegularizationWeight);

    if (m_usejointRetargeting)
    {
        m_jointRetargetingTask = std::make_shared<BipedalLocomotion::IK::JointTrackingTask>();
        ok = ok && m_jointRetargetingTask->setKinDyn(kinDyn);
        ok = ok && m_jointRetargetingTask->initialize(ptr->getGroup("JOINT_RETARGETING_TASK"));
        ok = ok &&  m_qpIK.addTask(m_jointRetargetingTask,
                                   "joint_retargeting_task",
                                   lowPriority,
                                   m_jointRetargetingWeight);
    }


    if (m_useRootLinkForHeight)
    {
        m_rootTask = std::make_shared<BipedalLocomotion::IK::R3Task>();
        ok = ok && m_rootTask->setKinDyn(kinDyn);
        ok = ok && m_rootTask->initialize(ptr->getGroup("ROOT_TASK"));
        ok = ok && m_qpIK.addTask(m_rootTask, "root_task", highPriority);
    }

    ok = ok && m_qpIK.finalize(m_variableHandler);

    std::cout << m_qpIK.toString() << std::endl;

    m_jointVelocity.resize(kinDyn->getNrOfDegreesOfFreedom());

    return ok;
}

bool IntegrationBasedIK::solve()
{
    bool ok = m_torsoWeight->advance();
    ok = ok && m_jointRegularizationWeight->advance();
    if (m_usejointRetargeting)
    {
        ok = ok && m_jointRetargetingWeight->advance();
    }

    ok = ok && m_qpIK.advance();
    ok = ok && m_qpIK.isOutputValid();

    if (ok)
    {
        iDynTree::toEigen(m_jointVelocity) = m_qpIK.getOutput().jointVelocity;
    }

    return ok;
}

bool IntegrationBasedIK::setPhase(const std::string& phase)
{
    bool ok = m_torsoWeight->setState(phase);
    ok = ok && m_jointRegularizationWeight->setState(phase);

    if (m_usejointRetargeting)
        ok = ok && m_jointRetargetingWeight->setState(phase);

    return ok;
}

bool IntegrationBasedIK::setLeftFootSetPoint(const iDynTree::Transform& desiredTransform,
                                             const iDynTree::Twist& desiredVelocity)
{
    return m_leftFootTask
        ->setSetPoint(BipedalLocomotion::Conversions::toManifPose(desiredTransform),
                      BipedalLocomotion::Conversions::toManifTwist(desiredVelocity));
}

bool IntegrationBasedIK::setRightFootSetPoint(const iDynTree::Transform& desiredTransform,
                                              const iDynTree::Twist& desiredVelocity)
{
    return m_rightFootTask
        ->setSetPoint(BipedalLocomotion::Conversions::toManifPose(desiredTransform),
                      BipedalLocomotion::Conversions::toManifTwist(desiredVelocity));
}

bool IntegrationBasedIK::setRetargetingJointSetPoint(const iDynTree::VectorDynSize& jointPosition)
{
    if (m_usejointRetargeting)
        return m_jointRetargetingTask->setSetPoint(iDynTree::toEigen(jointPosition));

    return true;
}

bool IntegrationBasedIK::setRegularizationJointSetPoint(const iDynTree::VectorDynSize& jointPosition)
{
    return m_jointRegularizationTask->setSetPoint(iDynTree::toEigen(jointPosition));
}

bool IntegrationBasedIK::setCoMSetPoint(const iDynTree::Position& position,
                                        const iDynTree::Vector3& velocity)
{
    return m_comTask->setSetPoint(iDynTree::toEigen(position), iDynTree::toEigen(velocity));
}

bool IntegrationBasedIK::setRootSetPoint(const iDynTree::Position& position,
                                         const iDynTree::Vector3& velocity)
{
    if (m_useRootLinkForHeight)
    {
        return m_rootTask->setSetPoint(iDynTree::toEigen(position), iDynTree::toEigen(velocity));
    }
    return true;
}

bool IntegrationBasedIK::setTorsoSetPoint(const iDynTree::Rotation& rotation)
{
    return m_torsoTask->setSetPoint(BipedalLocomotion::Conversions::toManifRot(rotation));
}

const iDynTree::VectorDynSize& IntegrationBasedIK::getDesiredJointVelocity() const
{
    return m_jointVelocity;
}
