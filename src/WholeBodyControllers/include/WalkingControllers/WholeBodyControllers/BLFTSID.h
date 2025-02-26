// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_BLF_TSID
#define WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_BLF_TSID

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/VectorDynSize.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/TSID/AngularMomentumTask.h>
#include <BipedalLocomotion/TSID/BaseDynamicsTask.h>
#include <BipedalLocomotion/TSID/CoMTask.h>
#include <BipedalLocomotion/TSID/FeasibleContactWrenchTask.h>
#include <BipedalLocomotion/TSID/JointDynamicsTask.h>
#include <BipedalLocomotion/TSID/JointTrackingTask.h>
#include <BipedalLocomotion/TSID/QPTSID.h>
#include <BipedalLocomotion/TSID/R3Task.h>
#include <BipedalLocomotion/TSID/SE3Task.h>
#include <BipedalLocomotion/TSID/SO3Task.h>
#include <BipedalLocomotion/TSID/VariableRegularizationTask.h>

#include <memory>

namespace WalkingControllers {

class BLFTSID {
  public:
    bool initialize(
        std::weak_ptr<
            const BipedalLocomotion::ParametersHandler::IParametersHandler>
            handler,
        std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    bool solve();

    bool
    setLeftFootTrackingSetPoint(const iDynTree::Transform &desiredTransform,
                                const iDynTree::Twist &desiredVelocity,
                                const iDynTree::Twist &desiredAcceleration);

    bool
    setRightFootTrackingSetPoint(const iDynTree::Transform &desiredTransform,
                                 const iDynTree::Twist &desiredVelocity,
                                 const iDynTree::Twist &desiredAcceleration);
    bool
    setJointTrackingSetPoint(const iDynTree::VectorDynSize &jointPosition,
                             const iDynTree::VectorDynSize &jointVelocity,
                             const iDynTree::VectorDynSize &jointAcceleration);

    bool setAngularMomentumTrackingSetPoint(
        const iDynTree::Vector3 &angularMomentum,
        const iDynTree::Vector3 &angularMomentumRate);

    bool setCoMTrackingSetPoint(const iDynTree::Position &position,
                                const iDynTree::Vector3 &velocity,
                                const iDynTree::Vector3 &acceleration);

    bool setRootTrackingSetPoint(const iDynTree::Position &position,
                                 const iDynTree::Vector3 &linearVelocity,
                                 const iDynTree::Vector3 &linearAcceleration);

    bool setTorsoTrackingSetPoint(const iDynTree::Rotation &rotation,
                                  const iDynTree::Vector3 &angularVelocity,
                                  const iDynTree::Vector3 &angularAcceleration);

    bool setLeftContactWrenchSetPoint(const iDynTree::Wrench &contactWrench);

    bool setRightContactWrenchSetPoint(const iDynTree::Wrench &contactWrench);

    bool setTorqueRegularizationSetPoint(const iDynTree::VectorDynSize &torque);

    void setLeftContactActive(bool isActive);

    void setRightContactActive(bool isActive);

    const iDynTree::VectorDynSize &getDesiredJointAcceleration() const;
    const iDynTree::VectorDynSize &getDesiredJointTorque() const;

  private:
    struct TSIDProblemAndTasks {
        BipedalLocomotion::TSID::TaskSpaceInverseDynamicsProblem problem;

        struct TSIDtasks {
            std::shared_ptr<BipedalLocomotion::TSID::CoMTask> comTracking;
            std::shared_ptr<BipedalLocomotion::TSID::SO3Task> torsoTracking;
            std::shared_ptr<BipedalLocomotion::TSID::SE3Task> leftFootTracking;
            std::shared_ptr<BipedalLocomotion::TSID::SE3Task> rightFootTracking;
            std::shared_ptr<BipedalLocomotion::TSID::R3Task> rootTracking;
            std::shared_ptr<BipedalLocomotion::TSID::AngularMomentumTask>
                angularMomentumTracking;
            std::shared_ptr<BipedalLocomotion::TSID::JointTrackingTask>
                jointTracking;
            std::shared_ptr<BipedalLocomotion::TSID::FeasibleContactWrenchTask>
                leftFeasibleContactWrench;
            std::shared_ptr<BipedalLocomotion::TSID::FeasibleContactWrenchTask>
                rightFeasibleContactWrench;
            std::shared_ptr<BipedalLocomotion::TSID::VariableRegularizationTask>
                leftContactWrenchRegularization;
            std::shared_ptr<BipedalLocomotion::TSID::VariableRegularizationTask>
                rightContactWrenchRegularization;
            std::shared_ptr<BipedalLocomotion::TSID::VariableRegularizationTask>
                torqueRegularization;
            std::shared_ptr<BipedalLocomotion::TSID::JointDynamicsTask>
                jointDynamics;
            std::shared_ptr<BipedalLocomotion::TSID::BaseDynamicsTask>
                baseDynamics;
        };

        TSIDtasks tasks;
    };

    TSIDProblemAndTasks m_qpTSID;

    iDynTree::VectorDynSize m_jointAccelerations;
    iDynTree::VectorDynSize m_jointTorques;
};

} // namespace WalkingControllers

#endif // WALKING_CONTROLLERS_WHOLE_BODY_CONTROLLERS_BLF_TSID
