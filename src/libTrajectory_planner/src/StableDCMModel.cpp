/**
 * @file StableDCMModel.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <math.h>

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

//iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <WalkingControllers/TrajectoryPlanner/StableDCMModel.h>
#include <WalkingControllers/YarpHelper/Helper.h>

using namespace WalkingControllers;

bool StableDCMModel::initialize(const yarp::os::Searchable& config)
{
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for ZMP controller.";
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

    m_omega = sqrt(gravityAcceleration / comHeight);

    // set the sampling time
    double samplingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }

    yarp::sig::Vector buffer;
    buffer.resize(2, 0.0);

    // instantiate Integrator object
    m_comIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    return true;
}

void StableDCMModel::setInput(const iDynTree::Vector2& input)
{
    m_dcmPosition = input;
}

bool StableDCMModel::integrateModel()
{
    if(m_comIntegrator == nullptr)
    {
        yError() << "[integrateModel] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }

    // evaluate the velocity of the CoM
    yarp::sig::Vector comVelocityYarp(2);
    iDynTree::toEigen(comVelocityYarp) = -m_omega * (iDynTree::toEigen(m_comPosition) -
                                                     iDynTree::toEigen(m_dcmPosition));

    // integrate velocities
    yarp::sig::Vector comPositionYarp(2);
    comPositionYarp = m_comIntegrator->integrate(comVelocityYarp);

    // convert YARP vector into iDynTree vector
    iDynTree::toiDynTree(comVelocityYarp, m_comVelocity);
    iDynTree::toiDynTree(comPositionYarp, m_comPosition);

    return true;
}

const iDynTree::Vector2& StableDCMModel::getCoMPosition() const
{
    return m_comPosition;
}

const iDynTree::Vector2& StableDCMModel::getCoMVelocity() const
{
    return m_comVelocity;
}

bool StableDCMModel::reset(const iDynTree::Vector2& initialValue)
{
    if(m_comIntegrator == nullptr)
    {
        yError() << "[reset] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }

    yarp::sig::Vector buffer;
    iDynTree::toYarp(initialValue, buffer);

    m_comIntegrator->reset(buffer);
    m_comPosition = initialValue;
    return true;
}
