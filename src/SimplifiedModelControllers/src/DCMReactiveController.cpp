// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/YARPConversions.h>
#include <iDynTree/YARPEigenConversions.h>

#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/SimplifiedModelControllers/DCMReactiveController.h>

using namespace WalkingControllers;

bool WalkingDCMReactiveController::initialize(const yarp::os::Searchable& config)
{
    // check if the configuration file is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for DCM controller.";
        return false;
    }

    // set the gain of the DCM controller
    if(!YarpUtilities::getNumberFromSearchable(config, "kDCM", m_kDCM))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    double comHeight;
    if(!YarpUtilities::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asFloat64();
    m_omega = sqrt(gravityAcceleration / comHeight);

    m_isInitialized = true;

    return true;
}

void WalkingDCMReactiveController::setFeedback(const iDynTree::Vector2& dcmFeedback)
{
    m_dcmFeedback = dcmFeedback;
}

void WalkingDCMReactiveController::setReferenceSignal(const iDynTree::Vector2& dcmPositionDesired,
                                                      const iDynTree::Vector2& dcmVelocityDesired)
{
    m_dcmPositionDesired = dcmPositionDesired;
    m_dcmVelocityDesired = dcmVelocityDesired;
}

bool WalkingDCMReactiveController::evaluateControl()
{
    if(!m_isInitialized)
    {
        yError() << "[evaluateControl] The controller is not initialized. "
                 << "Please call 'initialize()'";
        return false;
    }

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_dcmPositionDesired) -
        1 / m_omega * (iDynTree::toEigen(m_dcmVelocityDesired)) -
        m_kDCM * (iDynTree::toEigen(m_dcmPositionDesired) -
                  iDynTree::toEigen(m_dcmFeedback));

    return true;
}

const iDynTree::Vector2& WalkingDCMReactiveController::getControllerOutput() const
{
    return m_controllerOutput;
}
