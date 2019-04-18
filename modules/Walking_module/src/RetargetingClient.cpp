/**
 * @file WalkingRetargetingClient.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <yarp/os/LogStream.h>

#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <Utils.hpp>
#include <RetargetingClient.hpp>

void RetargetingClient::convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                                           iDynTree::Transform& transform)
{
    transform.setPosition(iDynTree::Position(vector(0), vector(1), vector(2)));
    transform.setRotation(iDynTree::Rotation::RPY(vector(3), vector(4), vector(5)));
}

bool RetargetingClient::initialize(const yarp::os::Searchable &config,
                                   const std::string &name,
                                   const double &period)
{
    if(config.isNull())
    {
        yInfo() << "[RetargetingClient::initialize] the hand retargeting is disable";
        m_useHandRetargeting = false;
        return true;
    }

    m_useHandRetargeting = true;

    std::string portName;

    // open left hand port
    if(!YarpHelper::getStringFromSearchable(config, "left_hand_transform_port_name",
                                            portName))
    {
        yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_leftHandTransformPort.open("/" + name + portName);

    // open right hand port
    if(!YarpHelper::getStringFromSearchable(config, "right_hand_transform_port_name",
                                            portName))
    {
        yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
        return false;
    }
    m_rightHandTransformPort.open("/" + name + portName);

    m_leftHandTransformYarp.resize(6);
    m_rightHandTransformYarp.resize(6);

    double smoothingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "smoothing_time", smoothingTime))
    {
        yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
        return false;
    }

    m_leftHandSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);
    m_rightHandSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);

    return true;
}

void RetargetingClient::reset(const iDynTree::Transform& leftHandTransform,
                              const iDynTree::Transform& rightHandTransform)
{
    if(!m_useHandRetargeting)
        return;

    m_leftHandTransform = leftHandTransform;
    m_rightHandTransform = rightHandTransform;

    iDynTree::toEigen(m_leftHandTransformYarp).block(0, 0, 3, 1) =
        iDynTree::toEigen(m_leftHandTransform.getPosition());

    iDynTree::toEigen(m_leftHandTransformYarp).block(3, 0, 3, 1) =
        iDynTree::toEigen(m_leftHandTransform.getRotation().asRPY());

    iDynTree::toEigen(m_rightHandTransformYarp).block(0, 0, 3, 1) =
        iDynTree::toEigen(m_rightHandTransform.getPosition());

    iDynTree::toEigen(m_rightHandTransformYarp).block(3, 0, 3, 1) =
        iDynTree::toEigen(m_rightHandTransform.getRotation().asRPY());

    m_leftHandSmoother->init(m_leftHandTransformYarp);
    m_rightHandSmoother->init(m_rightHandTransformYarp);
}

void RetargetingClient::getFeedback()
{
    if(!m_useHandRetargeting)
        return;

    auto desiredLeftHandPose = m_leftHandTransformPort.read(false);
    if(desiredLeftHandPose != nullptr)
        m_leftHandTransformYarp = *desiredLeftHandPose;

    m_leftHandSmoother->computeNextValues(m_leftHandTransformYarp);
    convertYarpVectorPoseIntoTransform(m_leftHandSmoother->getPos(), m_leftHandTransform);

    auto desiredRightHandPose = m_rightHandTransformPort.read(false);
    if(desiredRightHandPose != nullptr)
        m_rightHandTransformYarp = *desiredRightHandPose;

    m_rightHandSmoother->computeNextValues(m_rightHandTransformYarp);
    convertYarpVectorPoseIntoTransform(m_rightHandSmoother->getPos(), m_rightHandTransform);
}

const iDynTree::Transform& RetargetingClient::leftHandTransform() const
{
    return m_leftHandTransform;
}

const iDynTree::Transform& RetargetingClient::rightHandTransform() const
{
    return m_rightHandTransform;
}
