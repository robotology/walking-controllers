/**
 * @file WalkingRetargetingClient.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <yarp/os/LogStream.h>

#include <RetargetingClient.hpp>
#include <Utils.hpp>

void RetargetingClient::convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                                           iDynTree::Transform& transform)
{
    transform.setPosition(iDynTree::Position(vector(0), vector(1), vector(2)));
    transform.setRotation(iDynTree::Rotation::RPY(vector(3), vector(4), vector(5)));
}

bool RetargetingClient::initialize(const yarp::os::Searchable &config,
                                   const std::string &name)
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

    return true;
}


void RetargetingClient::reset(const iDynTree::Transform& leftHandTransform,
                              const iDynTree::Transform& rightHandTransform)
{
    m_leftHandTransform = leftHandTransform;
    m_rightHandTransform = rightHandTransform;
}

void RetargetingClient::getFeedback()
{
    if(!m_useHandRetargeting)
        return;

    auto desiredLeftHandPose = m_leftHandTransformPort.read(false);
    if(desiredLeftHandPose != nullptr)
        convertYarpVectorPoseIntoTransform(*desiredLeftHandPose, m_leftHandTransform);

    auto desiredRightHandPose = m_rightHandTransformPort.read(false);
    if(desiredRightHandPose != nullptr)
        convertYarpVectorPoseIntoTransform(*desiredRightHandPose, m_rightHandTransform);
}

const iDynTree::Transform& RetargetingClient::leftHandTransform() const
{
    return m_leftHandTransform;
}

const iDynTree::Transform& RetargetingClient::rightHandTransform() const
{
    return m_rightHandTransform;
}
