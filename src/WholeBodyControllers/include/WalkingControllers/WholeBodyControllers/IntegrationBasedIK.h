/**
 * @file IntegrationBasedIK.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_INTEGRATION_BASED_IK
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_INTEGRATION_BASED_IK

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotion/ContinuousDynamicalSystem/MultiStateWeightProvider.h>

#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/R3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

#include <BipedalLocomotion/System/VariablesHandler.h>
#include <memory>

namespace WalkingControllers
{

class IntegrationBasedIK
{
public:
    bool
    initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
               std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    bool setPhase(const std::string& phase);

    bool solve();

    bool setLeftFootSetPoint(const iDynTree::Transform& desiredTransform,
                             const iDynTree::Twist& desiredVelocity);

    bool setRightFootSetPoint(const iDynTree::Transform& desiredTransform,
                             const iDynTree::Twist& desiredVelocity);

    bool setRetargetingJointSetPoint(const iDynTree::VectorDynSize& jointPosition);
    bool setRegularizationJointSetPoint(const iDynTree::VectorDynSize& jointPosition);
    bool setCoMSetPoint(const iDynTree::Position& position, const iDynTree::Vector3& velocity);
    bool setRootSetPoint(const iDynTree::Position& position, const iDynTree::Vector3& velocity);
    bool setTorsoSetPoint(const iDynTree::Rotation& rotation);
    const iDynTree::VectorDynSize& getDesiredJointVelocity() const;

private:
    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>
        m_torsoWeight;
    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>
        m_jointRegularizationWeight;
    std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::MultiStateWeightProvider>
        m_jointRetargetingWeight;

    BipedalLocomotion::IK::QPInverseKinematics m_qpIK;
    BipedalLocomotion::System::VariablesHandler m_variableHandler;

    std::shared_ptr<BipedalLocomotion::IK::CoMTask> m_comTask;
    std::shared_ptr<BipedalLocomotion::IK::SO3Task> m_torsoTask;
    std::shared_ptr<BipedalLocomotion::IK::SE3Task> m_leftFootTask;
    std::shared_ptr<BipedalLocomotion::IK::SE3Task> m_rightFootTask;
    std::shared_ptr<BipedalLocomotion::IK::R3Task> m_rootTask;
    std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> m_jointRetargetingTask;
    std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> m_jointRegularizationTask;

    iDynTree::VectorDynSize m_jointVelocity;
    bool m_usejointRetargeting{false};
    bool m_useRootLinkForHeight{false};
};

} // namespace WalkingControllers

#endif // WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_INTEGRATION_BASED_IK
