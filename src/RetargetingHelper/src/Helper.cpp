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
        m_useCoMHeightRetargeting = false;
        return true;
    }

    m_useHandRetargeting = config.check("use_hand_retargeting", yarp::os::Value(false)).asBool();
    m_useJointRetargeting = config.check("use_joint_retargeting", yarp::os::Value(false)).asBool();
    m_useVirtualizer = config.check("use_virtualizer", yarp::os::Value(false)).asBool();
    m_useCoMHeightRetargeting = config.check("use_com_retargeting", yarp::os::Value(false)).asBool();

    if(m_useJointRetargeting && m_useHandRetargeting)
    {
        yError() << "[RetargetingClient::initialize] You cannot enable the joint retargeting along with the hand retargeting.";
        return false;
    }

    m_retargetJoints.resize(controlledJointsName.size());
    m_jointRetargeting.yarpVector.resize(controlledJointsName.size());

    std::string portName;

    if(m_useHandRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("HAND_RETARGETING");

        // open left hand port
        if(!YarpUtilities::getStringFromSearchable(option, "left_hand_transform_port_name",
                                                   portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_leftHand.port.open("/" + name + portName);

        // open right hand port
        if(!YarpUtilities::getStringFromSearchable(option, "right_hand_transform_port_name",
                                                   portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_rightHand.port.open("/" + name + portName);

        m_leftHand.yarpVector.resize(6);
        m_rightHand.yarpVector.resize(6);

        double smoothingTime;
        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time", smoothingTime))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_leftHand.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);
        m_rightHand.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, smoothingTime);
    }

    if(m_useJointRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("JOINT_RETARGETING");

        std::vector<std::string> retargetJointNames;
        yarp::os::Value *retargetJointNamesYarp;
        if(!option.check("retargeting_joint_list", retargetJointNamesYarp))
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

        if(!YarpUtilities::getStringFromSearchable(option, "joint_retargeting_port_name",
                                                portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_jointRetargeting.port.open("/" + name + portName);

        double smoothingTime;
        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time", smoothingTime))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_jointRetargeting.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(controlledJointsName.size(), period, smoothingTime);

    }

    if(m_useVirtualizer)
    {
        const yarp::os::Bottle& option = config.findGroup("VIRTUALIZER");

        if(!YarpUtilities::getStringFromSearchable(option, "robot_orientation_port_name",
                                                   portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_robotOrientationPort.open("/" + name + portName);
    }

    if(m_useCoMHeightRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("COM_RETARGETING");

        m_comHeight.yarpVector.resize(1);

        if(!YarpUtilities::getStringFromSearchable(option, "com_height_retargeting_port_name", portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_comHeight.port.open("/" + name + portName);

        double smoothingTime;
        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time", smoothingTime))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_comHeight.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, period, smoothingTime);
    }

    return true;
}

bool RetargetingClient::reset(const iDynTree::Transform& leftHandTransform,
                              const iDynTree::Transform& rightHandTransform,
                              const iDynTree::VectorDynSize& jointValues,
                              const double& comHeight)
{
    m_leftHandTransform = leftHandTransform;
    m_rightHandTransform = rightHandTransform;

    if(m_useHandRetargeting)
    {
        iDynTree::toEigen(m_leftHand.yarpVector).segment<3>(0) =
            iDynTree::toEigen(m_leftHandTransform.getPosition());

        iDynTree::toEigen(m_leftHand.yarpVector).segment<3>(3) =
            iDynTree::toEigen(m_leftHandTransform.getRotation().asRPY());

        iDynTree::toEigen(m_rightHand.yarpVector).segment<3>(0) =
            iDynTree::toEigen(m_rightHandTransform.getPosition());

        iDynTree::toEigen(m_rightHand.yarpVector).segment<3>(3) =
            iDynTree::toEigen(m_rightHandTransform.getRotation().asRPY());

        m_leftHand.smoother->init(m_leftHand.yarpVector);
        m_rightHand.smoother->init(m_rightHand.yarpVector);
    }

    // joint retargeting
    m_retargetJoints = jointValues;
    iDynTree::toEigen(m_jointRetargeting.yarpVector) = iDynTree::toEigen(jointValues);
    if (m_useJointRetargeting)
        m_jointRetargeting.smoother->init(m_jointRetargeting.yarpVector);

    m_comHeightValue = comHeight;
    m_comHeightVelocity = 0;
    m_comConstantHeight = comHeight;

    if(m_useCoMHeightRetargeting)
    {
        m_comHeight.yarpVector(0) = comHeight;
        m_comHeight.smoother->init(m_comHeight.yarpVector);

        // let's read the port to reset the comHeightInput
        bool okCoMHeight = false;
        unsigned int attempt = 0;
        do
        {
            if(!okCoMHeight)
            {
                auto desiredCoMHeight = m_comHeight.port.read(false);
                if(desiredCoMHeight != nullptr)
                {
                    m_comHeightInputZero = (*desiredCoMHeight)(2);
                    okCoMHeight = true;
                }
            }

            if(okCoMHeight)
                return true;

            yarp::os::Time::delay(0.001);
            attempt++;
        } while (attempt < 100);

        yError() << "[RetargetingClient::reset] The retargeting port is not streaming data";
        return false;
    }

    return true;
}

void RetargetingClient::getFeedback()
{
    if(m_useHandRetargeting)
    {
        auto desiredLeftHandPose = m_leftHand.port.read(false);
        if(desiredLeftHandPose != nullptr)
            m_leftHand.yarpVector = *desiredLeftHandPose;

        m_leftHand.smoother->computeNextValues(m_leftHand.yarpVector);
        convertYarpVectorPoseIntoTransform(m_leftHand.smoother->getPos(), m_leftHandTransform);

        auto desiredRightHandPose = m_rightHand.port.read(false);
        if(desiredRightHandPose != nullptr)
            m_rightHand.yarpVector = *desiredRightHandPose;

        m_rightHand.smoother->computeNextValues(m_rightHand.yarpVector);
        convertYarpVectorPoseIntoTransform(m_rightHand.smoother->getPos(), m_rightHandTransform);
    }

    if(m_useJointRetargeting)
    {
        auto desiredJoint = m_jointRetargeting.port.read(false);
        if(desiredJoint != nullptr)
        {
            for(int i =0; i < desiredJoint->size(); i++)
                m_jointRetargeting.yarpVector(m_retargetJointsIndex[i]) = (*desiredJoint)(i);
        }

        m_jointRetargeting.smoother->computeNextValues(m_jointRetargeting.yarpVector);
        iDynTree::toEigen(m_retargetJoints) = iDynTree::toEigen(m_jointRetargeting.smoother->getPos());
    }


    if(m_useCoMHeightRetargeting)
    {
        if(!m_isStancePhase)
            m_comHeight.yarpVector(0) = m_comConstantHeight;
        else
        {
            auto desiredCoMHeight = m_comHeight.port.read(false);
            if(desiredCoMHeight != nullptr)
                m_comHeight.yarpVector(0) = (*desiredCoMHeight)(2) - m_comHeightInputZero
                    + m_comConstantHeight;
        }

        m_comHeight.smoother->computeNextValues(m_comHeight.yarpVector);
        m_comHeightValue = m_comHeight.smoother->getPos()(0);
        m_comHeightVelocity = m_comHeight.smoother->getVel()(0);
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

double RetargetingClient::comHeight() const
{
    return m_comHeightValue;
}

double RetargetingClient::comHeightVelocity() const
{
    return m_comHeightVelocity;
}

void RetargetingClient::close()
{
    if(m_useHandRetargeting)
    {
        m_leftHand.port.close();
        m_rightHand.port.close();
    }

    if(m_useJointRetargeting)
        m_jointRetargeting.port.close();

    if(m_useCoMHeightRetargeting)
        m_comHeight.port.close();

    if(m_useVirtualizer)
        m_robotOrientationPort.close();
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

void RetargetingClient::setPhase(bool isStancePhase)
{
    m_isStancePhase = isStancePhase;
}
