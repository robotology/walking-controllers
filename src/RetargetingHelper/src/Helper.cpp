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

    // The approaching phase is set only if the startApproacingPhase method is called
    m_isApproachingPhase = false;

    if(m_useJointRetargeting && m_useHandRetargeting)
    {
        yError() << "[RetargetingClient::initialize] You cannot enable the joint retargeting along with the hand retargeting.";
        return false;
    }

    m_jointRetargeting.data.resize(controlledJointsName.size());
    m_jointRetargeting.yarpVector.resize(controlledJointsName.size());

    if(!m_useHandRetargeting && !m_useVirtualizer &&
       !m_useJointRetargeting && !m_useCoMHeightRetargeting)
    {
        return true;
    }

    // get the approaching phase duration
    if(!YarpUtilities::getNumberFromSearchable(config, "approaching_phase_duration", m_approachPhaseDuration))
    {
        yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
        return false;
    }

    std::string portName;
    double smoothingTimeApproching;
    double smoothingTimeWalking;
    if(m_useHandRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("HAND_RETARGETING");

        auto initializeHand = [&](auto& hand, const std::string& portNameLabel) -> bool
        {
            // open left hand port
            if(!YarpUtilities::getStringFromSearchable(config, portNameLabel,
                                                       portName))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
                return false;
            }
            hand.port.open("/" + name + portName);


            if(!YarpUtilities::getNumberFromSearchable(config, "smoothing_time_approaching", smoothingTimeApproching))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
                return false;
            }
            hand.smoothingTimeInApproaching = smoothingTimeApproching;


            if(!YarpUtilities::getNumberFromSearchable(config, "smoothing_time_walking", smoothingTimeWalking))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
                return false;
            }
            hand.smoothingTimeInWalking = smoothingTimeWalking;

            hand.yarpVector.resize(6);
            hand.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, hand.smoothingTimeInApproaching);

            return true;
        };

        if(!initializeHand(m_leftHand, "left_hand_transform_port_name")
           || !initializeHand(m_rightHand, "right_hand_transform_port_name"))
            return false;
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

        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time_approaching",
                                                   m_jointRetargeting.smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time_walking", m_jointRetargeting.smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_jointRetargeting.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(controlledJointsName.size(), period,
                                                                                   m_jointRetargeting.smoothingTimeInApproaching);

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

        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time_approaching", m_comHeight.smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time_walking", m_comHeight.smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_comHeight.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(1, period, m_comHeight.smoothingTimeInApproaching);
    }

    return true;
}

bool RetargetingClient::reset(const iDynTree::Transform& leftHandTransform,
                              const iDynTree::Transform& rightHandTransform,
                              const iDynTree::VectorDynSize& jointValues,
                              const double& comHeight)
{
    m_leftHand.data = leftHandTransform;
    m_rightHand.data = rightHandTransform;

    if(m_useHandRetargeting)
    {
        auto resetHandSmoother = [](auto& hand)
        {
            iDynTree::toEigen(hand.yarpVector).template segment<3>(0) =
            iDynTree::toEigen(hand.data.getPosition());

            iDynTree::toEigen(hand.yarpVector).template segment<3>(3) =
            iDynTree::toEigen(hand.data.getRotation().asRPY());

            hand.smoother->init(hand.yarpVector);
        };

        resetHandSmoother(m_leftHand);
        resetHandSmoother(m_rightHand);
    }

    // joint retargeting
    m_jointRetargeting.data = jointValues;
    iDynTree::toEigen(m_jointRetargeting.yarpVector) = iDynTree::toEigen(jointValues);
    if (m_useJointRetargeting)
        m_jointRetargeting.smoother->init(m_jointRetargeting.yarpVector);

    m_comHeight.data.position = comHeight;
    m_comHeight.data.velocity = 0;
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

        if(!okCoMHeight)
            yError() << "[RetargetingClient::reset] The CoM height is not coming from the yarp port.";
        return false;
    }

    return true;
}

void RetargetingClient::getFeedback()
{
    if(m_useHandRetargeting)
    {
        auto getHandFeedback = [this](auto& hand)
        {
            auto desiredHandPose = hand.port.read(false);
            if(desiredHandPose != nullptr)
                hand.yarpVector = *desiredHandPose;

            hand.smoother->computeNextValues(hand.yarpVector);
            convertYarpVectorPoseIntoTransform(hand.smoother->getPos(), hand.data);
        };

        getHandFeedback(m_leftHand);
        getHandFeedback(m_rightHand);
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
        iDynTree::toEigen(m_jointRetargeting.data) = iDynTree::toEigen(m_jointRetargeting.smoother->getPos());
    }


    if(m_useCoMHeightRetargeting)
    {
        if(!m_isStancePhase)
            m_comHeight.yarpVector(0) = m_comConstantHeight;
        else
        {
            auto desiredCoMHeight = m_comHeight.port.read(false);
            if(desiredCoMHeight != nullptr)

                m_comHeight.yarpVector(0) = ((*desiredCoMHeight)(2) - m_comHeightInputZero) * 0.5
                    + m_comConstantHeight;
        }

        m_comHeight.smoother->computeNextValues(m_comHeight.yarpVector);
        m_comHeight.data.position = m_comHeight.smoother->getPos()(0);
        m_comHeight.data.velocity = m_comHeight.smoother->getVel()(0);
    }

    // check if the approaching phase is finished
    if(m_isApproachingPhase)
    {
        double now = yarp::os::Time::now();
        if(now - m_startingApproachingPhaseTime > m_approachPhaseDuration)
            stopApproachingPhase();
    }

    return;
}

const iDynTree::Transform& RetargetingClient::leftHandTransform() const
{
    return m_leftHand.data;
}

const iDynTree::Transform& RetargetingClient::rightHandTransform() const
{
    return m_rightHand.data;
}

const iDynTree::VectorDynSize& RetargetingClient::jointValues() const
{
    return m_jointRetargeting.data;
}

double RetargetingClient::comHeight() const
{
    return m_comHeight.data.position;
}

double RetargetingClient::comHeightVelocity() const
{
    return m_comHeight.data.velocity;
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

void RetargetingClient::stopApproachingPhase()
{
    if(m_useHandRetargeting)
    {
        m_leftHand.smoother->setT(m_leftHand.smoothingTimeInWalking);
        m_rightHand.smoother->setT(m_rightHand.smoothingTimeInWalking);
    }

    if(m_useJointRetargeting)
    {
        m_jointRetargeting.smoother->setT(m_jointRetargeting.smoothingTimeInWalking);
    }

    if(m_useCoMHeightRetargeting)
    {
        m_comHeight.smoother->setT(m_comHeight.smoothingTimeInWalking);
    }

    m_isApproachingPhase = false;
}

void RetargetingClient::startApproachingPhase()
{
    // if the retargeting is not used the approaching phase is not required
    if(!m_useHandRetargeting && !m_useJointRetargeting && !m_useCoMHeightRetargeting)
        return;

    m_startingApproachingPhaseTime = yarp::os::Time::now();

    if(m_useHandRetargeting)
    {
        m_leftHand.smoother->setT(m_leftHand.smoothingTimeInApproaching);
        m_rightHand.smoother->setT(m_rightHand.smoothingTimeInApproaching);
    }

    if(m_useJointRetargeting)
    {
        m_jointRetargeting.smoother->setT(m_jointRetargeting.smoothingTimeInApproaching);
    }

    if(m_useCoMHeightRetargeting)
    {
        m_comHeight.smoother->setT(m_comHeight.smoothingTimeInApproaching);
    }

    m_isApproachingPhase = true;
}

bool RetargetingClient::isApproachingPhase() const
{
    return m_isApproachingPhase;
}
