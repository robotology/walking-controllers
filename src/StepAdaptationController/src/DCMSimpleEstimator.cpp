/**
 * @file DCMSimpleEstimator.cpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

//iDynTree
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <WalkingControllers/StepAdaptationController/DCMSimpleEstimator.hpp>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

bool DCMSimpleEstimator::configure(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[DCMSimpleEstimator::configure] Empty configuration.";
        return false;
    }

    double comHeight;
    if(!YarpUtilities::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[DCMSimpleEstimator::configure] Unable to get CoM height from configuration file. ";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

    m_omega = sqrt(gravityAcceleration / comHeight);
    return true;
}

bool DCMSimpleEstimator::update(const iDynTree::Rotation& footOrientation,const iDynTree::Vector3& zmp,const iDynTree::Vector3& com,const iDynTree::LinVelocity& CoMVelocity3d)
{
    iDynTree::Vector3 CoMPositionEstimated;
    iDynTree::toEigen(CoMPositionEstimated)=iDynTree::toEigen(zmp)+iDynTree::toEigen(footOrientation)*(iDynTree::toEigen(com)-iDynTree::toEigen(zmp));

    m_dcmEstimatedPosition(0)=CoMPositionEstimated(0)+CoMVelocity3d(0)/m_omega;
    m_dcmEstimatedPosition(1)=CoMPositionEstimated(1)+CoMVelocity3d(1)/m_omega;

    return true;
}

const iDynTree::Vector2& DCMSimpleEstimator::getDCMPosition() const
{
    return m_dcmEstimatedPosition;
}
