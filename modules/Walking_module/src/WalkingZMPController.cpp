/**
 * @file WalkingZMPController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include "WalkingZMPController.hpp"
#include "Utils.hpp"

bool WalkingZMPController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for ZMP controller.";
        return false;
    }

    m_useGainScheduling = config.check("useGainScheduling", yarp::os::Value(false)).asBool();

    // set the gain of the CoM controller
    if(!YarpHelper::getDoubleFromSearchable(config, "kCoM_x_walking", m_kCoMXWalking))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    // set the ZMP controller gain
    if(!YarpHelper::getDoubleFromSearchable(config, "kZMP_x_walking", m_kZMPXWalking))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(config, "kCoM_y_walking", m_kCoMYWalking))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    // set the ZMP controller gain
    if(!YarpHelper::getDoubleFromSearchable(config, "kZMP_y_walking", m_kZMPYWalking))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    
    // set the sampling time
    double samplingTime;
    if(!YarpHelper::getDoubleFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    if(samplingTime < 0)
    {
        yError() << "[initialize] The sampling time has to be a positive number.";
        return false;
    }

    yarp::sig::Vector buffer;
    buffer.resize(2, 0.0);

    // instantiate Integrator object
    m_velocityIntegral = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    // if gain scheduling is used the stance gains has to be loaded
    if(m_useGainScheduling)
    {
        double smoothingTime;
        if(!YarpHelper::getDoubleFromSearchable(config, "smoothingTime", smoothingTime))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

        if(!YarpHelper::getDoubleFromSearchable(config, "kCoM_x_stance", m_kCoMXStance))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

        if(!YarpHelper::getDoubleFromSearchable(config, "kZMP_x_stance", m_kZMPXStance))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

	if(!YarpHelper::getDoubleFromSearchable(config, "kCoM_y_stance", m_kCoMYStance))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

        if(!YarpHelper::getDoubleFromSearchable(config, "kZMP_y_stance", m_kZMPYStance))
        {
            yError() << "[initialize] Unable to get the double from searchable.";
            return false;
        }

	
        m_kZMPXSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, samplingTime,
                                                                      smoothingTime);
        m_kCoMXSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, samplingTime,
                                                                      smoothingTime);

	m_kZMPYSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, samplingTime,
                                                                      smoothingTime);
        m_kCoMYSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, samplingTime,
                                                                      smoothingTime);

        // initialize the minimum jerk trajectories
        m_kZMPXSmoother->init(yarp::sig::Vector(1, m_kZMPXStance));
        m_kCoMXSmoother->init(yarp::sig::Vector(1, m_kCoMXStance));

	m_kZMPYSmoother->init(yarp::sig::Vector(1, m_kZMPYStance));
        m_kCoMYSmoother->init(yarp::sig::Vector(1, m_kCoMYStance));

	
        m_kCoMX = m_kCoMXStance;
        m_kZMPX = m_kZMPXStance;

        m_kCoMY = m_kCoMYStance;
        m_kZMPY = m_kZMPYStance;

    }
    else
    {
        m_kCoMX = m_kCoMXWalking;
        m_kZMPX = m_kZMPXWalking;

        m_kCoMY = m_kCoMYWalking;
        m_kZMPY = m_kZMPYWalking;
    }

    return true;
}

void WalkingZMPController::setPhase(const bool& isStancePhase)
{
    if(m_useGainScheduling)
    {
        if(isStancePhase)
        {
            m_kCoMXSmoother->computeNextValues(yarp::sig::Vector(1, m_kCoMXStance));
            m_kZMPXSmoother->computeNextValues(yarp::sig::Vector(1, m_kZMPXStance));

            m_kCoMYSmoother->computeNextValues(yarp::sig::Vector(1, m_kCoMYStance));
            m_kZMPYSmoother->computeNextValues(yarp::sig::Vector(1, m_kZMPYStance));

        }
        else
        {
	    m_kCoMXSmoother->computeNextValues(yarp::sig::Vector(1, m_kCoMXWalking));
            m_kZMPXSmoother->computeNextValues(yarp::sig::Vector(1, m_kZMPXWalking));

            m_kCoMYSmoother->computeNextValues(yarp::sig::Vector(1, m_kCoMYWalking));
            m_kZMPYSmoother->computeNextValues(yarp::sig::Vector(1, m_kZMPYWalking));

        }
        m_kCoMX = m_kCoMXSmoother->getPos()[0];
        m_kZMPX = m_kZMPXSmoother->getPos()[0];

	m_kCoMY = m_kCoMYSmoother->getPos()[0];
        m_kZMPY = m_kZMPYSmoother->getPos()[0];
    }
}

void WalkingZMPController::setFeedback(const iDynTree::Vector2& zmpFeedback,
                                       const iDynTree::Position& comFeedback)
{
    m_zmpFeedback = zmpFeedback;

    // take only the XY projection of the CoM
    m_comFeedback(0) = comFeedback(0);
    m_comFeedback(1) = comFeedback(1);
}

void WalkingZMPController::setReferenceSignal(const iDynTree::Vector2& zmpDesired,
                                              const iDynTree::Vector2& comPositionDesired,
                                              const iDynTree::Vector2& comVelocityDesired)
{
    m_zmpDesired = zmpDesired;
    m_comPositionDesired = comPositionDesired;
    m_comVelocityDesired = comVelocityDesired;
}

bool WalkingZMPController::evaluateControl()
{
    m_controlEvaluated = false;
    if(m_velocityIntegral == nullptr)
    {
        yError() << "[evaluateControl] The integrator is not initialized.";
        return false;
    }

    // evaluate the control law
    m_desiredCoMVelocity(0) = m_kCoMX * (m_comPositionDesired(0) - m_comFeedback(0))
                             -m_kZMPX * (m_zmpDesired(0) - m_zmpFeedback(0)) +
                              m_comVelocityDesired(0);

    m_desiredCoMVelocity(1) = m_kCoMY * (m_comPositionDesired(1) - m_comFeedback(1))
                             -m_kZMPY * (m_zmpDesired(1) - m_zmpFeedback(1)) +
                              m_comVelocityDesired(1);

    // integrate the velocity
    yarp::sig::Vector desiredCoMVelocityYarp(2);
    yarp::sig::Vector desiredCoMPositionYarp(2);
    iDynTree::toYarp(m_desiredCoMVelocity, desiredCoMVelocityYarp);

    desiredCoMPositionYarp = m_velocityIntegral->integrate(desiredCoMVelocityYarp);

    iDynTree::toiDynTree(desiredCoMPositionYarp, m_controllerOutput);

    m_controlEvaluated = true;
    return true;
}

bool WalkingZMPController::getControllerOutput(iDynTree::Vector2& controllerOutputPosition,
                                               iDynTree::Vector2& controllerOutputVelocity)
{
    if(!m_controlEvaluated)
    {
        yError() << "[getControllerOutput] Please before call evaluateControl() method.";
        return false;
    }

    controllerOutputPosition = m_controllerOutput;
    controllerOutputVelocity = m_desiredCoMVelocity;
    return true;
}

bool WalkingZMPController::reset(const iDynTree::Vector2& initialValue)
{
    if(m_velocityIntegral == nullptr)
    {
        yError() << "[reset] The integrator is not initialized.";
        return false;
    }

    yarp::sig::Vector buffer(2);
    iDynTree::toYarp(initialValue, buffer);

    m_velocityIntegral->reset(buffer);
    return true;
}
