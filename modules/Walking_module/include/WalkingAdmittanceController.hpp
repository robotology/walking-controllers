/**
 * @file WalkingAdmittanceController.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_ADMITTANCE_CONTROLLER_HPP
#define WALKING_ADMITTANCE_CONTROLLER_HPP

#include <memory>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

#include <yarp/os/Searchable.h>

class WalkingAdmittanceController
{
    class Implementation;
    std::unique_ptr<Implementation> m_pimpl;

public:

    WalkingAdmittanceController();

    ~WalkingAdmittanceController();

    bool initialize(yarp::os::Searchable &config, const int& actuatedDoFs);

    void setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                   const iDynTree::VectorDynSize& desiredJointVelocity,
                                   const iDynTree::VectorDynSize& desiredJointAcceleration);

    void setJointState(const iDynTree::VectorDynSize& jointPosition,
                       const iDynTree::VectorDynSize& jointVelocity);

    void setMassMatrix(const iDynTree::MatrixDynSize& massMatrix);

    void setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces);

    void setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                         const iDynTree::MatrixDynSize& rightFootJacobian);

    void setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                 const iDynTree::Vector6 &rightFootBiasAcceleration);

    bool setFeetState(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::Wrench& leftFootWrench, bool leftInContact,
                      const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::Wrench& rightFootWrench, bool rightInContact);

    bool setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::SpatialAcc& leftFootAcceleration,
                                  const iDynTree::Wrench& leftFootWrench,
                                  const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::SpatialAcc& rightFootAcceleration,
                                  const iDynTree::Wrench& rightFootWrench);

    bool setNeckState(const iDynTree::Rotation& neckOrientation, const iDynTree::Twist& neckVelocity);

    bool setDesiredNeckTrajectory(const iDynTree::Rotation& neckOrientation);

    void setNeckJacobian(const iDynTree::MatrixDynSize& jacobian);

    void setNeckBiasAcceleration(const iDynTree::Vector6 &biasAcceleration);

    bool solve();

    const iDynTree::VectorDynSize& desiredJointTorque() const;

};
#endif
