/**
 * @file Helper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/RetargetingHelper/Helper.h>

using namespace WalkingControllers;

void RetargetingClient::convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                                           iDynTree::Transform& transform)
{
    transform.setPosition(iDynTree::Position(vector(0), vector(1), vector(2)));
    transform.setRotation(iDynTree::Rotation::RPY(vector(3), vector(4), vector(5)));
}

bool RetargetingClient::initialize(const yarp::os::Searchable &config,
                                   const std::string &name,
                                   const double &period,
                                   const std::vector<std::string>& controlledJointsName)
{
    if(config.isNull())
    {
        yInfo() << "[RetargetingClient::initialize] the hand retargeting is disable";
        m_useHandRetargeting = false;
        m_useVirtualizer = false;
        m_useJointRetargeting = false;
        return true;
    }

    m_useHandRetargeting = config.check("use_hand_retargeting", yarp::os::Value(false)).asBool();
    m_useJointRetargeting = config.check("use_joint_retargeting", yarp::os::Value(false)).asBool();
    m_useVirtualizer = config.check("use_virtualizer", yarp::os::Value(false)).asBool();

    if(m_useJointRetargeting && m_useHandRetargeting)
    {
        yError() << "[RetargetingClient::initialize] You cannot enable the joint retargeting along with the hand retargeting.";
        return false;
    }

    m_retargetJoints.resize(controlledJointsName.size());

    std::string portName;

    if(m_useHandRetargeting)
    {
        // open left hand port
        if(!YarpUtilities::getStringFromSearchable(config, "left_hand_transform_port_name",
                                                portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_leftHandTransformPort.open("/" + name + portName);

        // open right hand port
        if(!YarpUtilities::getStringFromSearchable(config, "right_hand_transform_port_name",
                                                portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_rightHandTransformPort.open("/" + name + portName);

        m_leftHandTransformYarp.resize(6);
        m_rightHandTransformYarp.resize(6);

        double smoothingTime;
        if(!YarpUtilities::getNumberFromSearchable(config, "smoothing_time", smoothingTime))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_leftHandSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);
        m_rightHandSmoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);
    }

    if(m_useJointRetargeting)
    {
        std::vector<std::string> retargetJointNames;

        yarp::os::Value *retargetJointNamesYarp;
        if(!config.check("retargeting_joint_list", retargetJointNamesYarp))
        {
            yError() << "[RetargetingClient::initialize] Unable to find joints_list into config file.";
            return false;
        }
        if(!YarpUtilities::yarpListToStringVector(retargetJointNamesYarp, retargetJointNames))
        {
            yError() << "[RetargetingClient::initialize] Unable to convert yarp list into a vector of strings.";
            return false;
        }

        // find the indices
        for(const std::string& joint : retargetJointNames)
        {
            int index = std::distance(controlledJointsName.begin(), std::find(controlledJointsName.begin(),
                                                                              controlledJointsName.end(),
                                                                              joint));

            m_retargetJointsIndex.push_back(index);
        }

        if(!YarpUtilities::getStringFromSearchable(config, "joint_retargeting_port_name",
                                                portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_jointRetargetingPort.open("/" + name + portName);
    }

    if(m_useVirtualizer)
    {
        if(!YarpUtilities::getStringFromSearchable(config, "robot_orientation_port_name",
                                                portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_robotOrientationPort.open("/" + name + portName);
    }

    return true;
}

void RetargetingClient::reset(const iDynTree::Transform& leftHandTransform,
                              const iDynTree::Transform& rightHandTransform,
                              const iDynTree::VectorDynSize& jointValues)
{
    if(m_useHandRetargeting)
    {
        m_leftHandTransform = leftHandTransform;
        m_rightHandTransform = rightHandTransform;

        iDynTree::toEigen(m_leftHandTransformYarp).segment<3>(0) =
            iDynTree::toEigen(m_leftHandTransform.getPosition());

        iDynTree::toEigen(m_leftHandTransformYarp).segment<3>(3) =
            iDynTree::toEigen(m_leftHandTransform.getRotation().asRPY());

        iDynTree::toEigen(m_rightHandTransformYarp).segment<3>(0) =
            iDynTree::toEigen(m_rightHandTransform.getPosition());

        iDynTree::toEigen(m_rightHandTransformYarp).segment<3>(3) =
            iDynTree::toEigen(m_rightHandTransform.getRotation().asRPY());

        m_leftHandSmoother->init(m_leftHandTransformYarp);
        m_rightHandSmoother->init(m_rightHandTransformYarp);
    }

    m_retargetJoints = jointValues;
}

void RetargetingClient::getFeedback()
{
    if(m_useHandRetargeting)
    {
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

    if(m_useJointRetargeting)
    {
        auto desiredJoint = m_jointRetargetingPort.read(false);
        if(desiredJoint != nullptr)
        {
            for(int i =0; i < desiredJoint->size(); i++)
                m_retargetJoints(m_retargetJointsIndex[i]) = (*desiredJoint)(i);
        }
    }

    return;
}

const iDynTree::Transform& RetargetingClient::leftHandTransform() const
{
    return m_leftHandTransform;
}

const iDynTree::Transform& RetargetingClient::rightHandTransform() const
{
    return m_rightHandTransform;
}

const iDynTree::VectorDynSize& RetargetingClient::jointValues() const
{
    return m_retargetJoints;
}

void RetargetingClient::close()
{
    if(m_useHandRetargeting)
    {
        m_leftHandTransformPort.close();
        m_rightHandTransformPort.close();
    }

    if(m_useJointRetargeting)
        m_jointRetargetingPort.close();
}

void RetargetingClient::setRobotBaseOrientation(const iDynTree::Rotation& rotation)
{
    if(!m_useVirtualizer)
        return;

    yarp::sig::Vector& output = m_robotOrientationPort.prepare();
    output.clear();
    output.push_back(rotation.asRPY()(2));
    m_robotOrientationPort.write(false);
}
