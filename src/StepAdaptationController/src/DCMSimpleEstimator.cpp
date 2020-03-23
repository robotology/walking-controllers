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
        yError() << "[initialize] Empty configuration for Step Adaptation Utils.";
        return false;
    }

    double comHeight;
    if(!YarpUtilities::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

    m_omega = sqrt(gravityAcceleration / comHeight);
    return true;
}

bool DCMSimpleEstimator::pendulumEstimator(iDynTree::Rotation footOrientation,iDynTree::Vector3 zmp,iDynTree::Vector3 com,iDynTree::LinVelocity CoMVelocity3d)
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
