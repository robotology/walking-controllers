/**
 * @file ContactWrenchMapping.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_CONTACT_WRENCH_MAPPING_HPP
#define WALKING_CONTACT_WRENCH_MAPPING_HPP

#include <memory>

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Transform.h>

#include <CartesianPID.hpp>
#include <WalkingConstraint.hpp>

class ContactWrenchMapping
{
    class Implementation;
    std::unique_ptr<Implementation> m_pimpl;

public:

    ContactWrenchMapping();

    ~ContactWrenchMapping();

    bool initialize(yarp::os::Searchable& config);

    bool setRobotMass(const double& mass);

    void setFeetState(const bool &leftInContact, const bool &rightInContact);

    bool setCentroidalMomentum(const iDynTree::SpatialMomentum& centroidalMomentum);

    void setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                      const iDynTree::Transform& rightFootToWorldTransform);

    bool setCoMState(const iDynTree::Position& comPosition,
                     const iDynTree::Vector3& comVelocity);

    bool setDesiredVRP(const iDynTree::Vector3 &vrp);

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                 const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration);

    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight);

    bool setDesiredAngularMomentumRateOfChange(const iDynTree::Vector3& angularMomentumrateOfChange);

    bool solve();

    const iDynTree::Wrench& getDesiredLeftWrench() const;

    const iDynTree::Wrench& getDesiredRightWrench() const;
};

#endif
