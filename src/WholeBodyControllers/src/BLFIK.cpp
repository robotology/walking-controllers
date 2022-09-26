/**
 * @file BLFIK.h
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

#include <WalkingControllers/WholeBodyControllers/BLFIK.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>

#include <manif/manif.h>

#include <memory>

using namespace WalkingControllers;

bool BLFIK::initialize(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    constexpr auto prefix = "[BLFIK::initialize]";
    constexpr std::size_t highPriority = 0;
    constexpr std::size_t lowPriority = 1;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Invalid parameter handler", prefix);
        return false;
    }

    m_usejointRetargeting = false;
    ptr->getParameter("use_joint_retargeting", m_usejointRetargeting);

    ptr->getParameter("use_feedforward_term_for_joint_retargeting",
                      m_useFeedforwardTermForJointRetargeting);

    m_useRootLinkForHeight = false;
    ptr->getParameter("use_root_link_for_height", m_useRootLinkForHeight);

    m_useExternalFeedbackForLeftFootOrientation = false;
    m_useExternalFeedbackForRightFootOrientation = false;
    ptr->getParameter("use_left_foot_imu", m_useExternalFeedbackForLeftFootOrientation);
    ptr->getParameter("use_right_foot_imu", m_useExternalFeedbackForRightFootOrientation);

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
    m_comTask = std::make_shared<BipedalLocomotion::IK::CoMTask>();
    ok = ok && m_comTask->setKinDyn(kinDyn);
    ok = ok && m_comTask->initialize(ptr->getGroup("COM_TASK"));
    ok = ok && m_qpIK.addTask(m_comTask, "com_task", highPriority);

    // right foot task
    m_rightFootTask = std::make_shared<BipedalLocomotion::IK::SE3Task>();
    ok = ok && m_rightFootTask->setKinDyn(kinDyn);
    auto ptrRight = ptr->getGroup("RIGHT_FOOT_TASK").lock();
    if (ptrRight == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the 'RIGHT_FOOT_TASK' group.", prefix);
        return false;
    }
    ptrRight->setParameter("use_orientation_exogenous_feedback", m_useExternalFeedbackForRightFootOrientation);
    ok = ok && m_rightFootTask->initialize(ptrRight);
    ok = ok && m_qpIK.addTask(m_rightFootTask, "right_foot_task", highPriority);


    // left foot task
    m_leftFootTask = std::make_shared<BipedalLocomotion::IK::SE3Task>();
    ok = ok && m_leftFootTask->setKinDyn(kinDyn);

    auto ptrLeft = ptr->getGroup("LEFT_FOOT_TASK").lock();
    if (ptrLeft == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the 'LEFT_FOOT_TASK' group.", prefix);
        return false;
    }
    ptrLeft->setParameter("use_orientation_exogenous_feedback",
                          m_useExternalFeedbackForLeftFootOrientation);
    ok = ok && m_leftFootTask->initialize(ptrLeft);
    ok = ok && m_qpIK.addTask(m_leftFootTask, "left_foot_task", highPriority);

    // torso
    m_torsoTask = std::make_shared<BipedalLocomotion::IK::SO3Task>();
    ok = ok && m_torsoTask->setKinDyn(kinDyn);
    ok = ok && m_torsoTask->initialize(ptr->getGroup("TORSO_TASK"));
    ok = ok && m_qpIK.addTask(m_torsoTask, "torso_task", lowPriority, m_torsoWeight);

    m_jointRegularizationTask = std::make_shared<BipedalLocomotion::IK::JointTrackingTask>();
    ok = ok && m_jointRegularizationTask->setKinDyn(kinDyn);
    ok = ok && m_jointRegularizationTask->initialize(ptr->getGroup("JOINT_REGULARIZATION_TASK"));
    ok = ok
         && m_qpIK.addTask(m_jointRegularizationTask,
                           "joint_regularization_task",
                           lowPriority,
                           m_jointRegularizationWeight);

    if (m_usejointRetargeting)
    {
        m_jointRetargetingTask = std::make_shared<BipedalLocomotion::IK::JointTrackingTask>();
        ok = ok && m_jointRetargetingTask->setKinDyn(kinDyn);
        ok = ok && m_jointRetargetingTask->initialize(ptr->getGroup("JOINT_RETARGETING_TASK"));
        ok = ok
             && m_qpIK.addTask(m_jointRetargetingTask,
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

    BipedalLocomotion::log()->info("[BLFIK::initialize] {}", m_qpIK.toString());

    m_jointVelocity.resize(kinDyn->getNrOfDegreesOfFreedom());

    return ok;
}

bool BLFIK::solve()
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

bool BLFIK::setPhase(const std::string& phase)
{
    bool ok = m_torsoWeight->setState(phase);
    ok = ok && m_jointRegularizationWeight->setState(phase);

    if (m_usejointRetargeting)
        ok = ok && m_jointRetargetingWeight->setState(phase);

    return ok;
}

bool BLFIK::setLeftFootSetPoint(const iDynTree::Transform& desiredTransform,
                                const iDynTree::Twist& desiredVelocity)
{
    return m_leftFootTask
        ->setSetPoint(BipedalLocomotion::Conversions::toManifPose(desiredTransform),
                      BipedalLocomotion::Conversions::toManifTwist(desiredVelocity));
}

bool BLFIK::setRightFootSetPoint(const iDynTree::Transform& desiredTransform,
                                 const iDynTree::Twist& desiredVelocity)
{
    return m_rightFootTask
        ->setSetPoint(BipedalLocomotion::Conversions::toManifPose(desiredTransform),
                      BipedalLocomotion::Conversions::toManifTwist(desiredVelocity));
}

bool BLFIK::setRetargetingJointSetPoint(const iDynTree::VectorDynSize& jointPositions,
                                        const iDynTree::VectorDynSize& jointVelocities)
{
    if (m_usejointRetargeting)
    {
        if (m_useFeedforwardTermForJointRetargeting)
        {
            return m_jointRetargetingTask->setSetPoint(iDynTree::toEigen(jointPositions),
                                                       iDynTree::toEigen(jointVelocities));
        } else
        {
            return m_jointRetargetingTask->setSetPoint(iDynTree::toEigen(jointPositions));
        }
    }
    return true;
}

bool BLFIK::setRegularizationJointSetPoint(const iDynTree::VectorDynSize& jointPosition)
{
    return m_jointRegularizationTask->setSetPoint(iDynTree::toEigen(jointPosition));
}

bool BLFIK::setCoMSetPoint(const iDynTree::Position& position, const iDynTree::Vector3& velocity)
{
    return m_comTask->setSetPoint(iDynTree::toEigen(position), iDynTree::toEigen(velocity));
}

bool BLFIK::setRootSetPoint(const iDynTree::Position& position, const iDynTree::Vector3& velocity)
{
    if (m_useRootLinkForHeight)
    {
        return m_rootTask->setSetPoint(iDynTree::toEigen(position), iDynTree::toEigen(velocity));
    }
    return true;
}

bool BLFIK::setTorsoSetPoint(const iDynTree::Rotation& rotation)
{
    return m_torsoTask->setSetPoint(BipedalLocomotion::Conversions::toManifRot(rotation));
}

bool BLFIK::setLeftFootMeasuredOrientation(const iDynTree::Rotation& I_R_LF)
{
    if (m_useExternalFeedbackForLeftFootOrientation)
    {
        return m_leftFootTask->setFeedback(BipedalLocomotion::Conversions::toManifRot(I_R_LF));
    }
    return true;
}

bool BLFIK::setRightFootMeasuredOrientation(const iDynTree::Rotation& I_R_RF)
{
    if (m_useExternalFeedbackForRightFootOrientation)
    {
        return m_rightFootTask->setFeedback(BipedalLocomotion::Conversions::toManifRot(I_R_RF));
    }
    return true;
}

const iDynTree::VectorDynSize& BLFIK::getDesiredJointVelocity() const
{
    return m_jointVelocity;
}
