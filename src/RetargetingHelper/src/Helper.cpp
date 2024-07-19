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
                                   const std::vector<std::string>& controlledJointNames)
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

    m_hdeRetargeting.joints.initialState.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.rawPosition.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.position.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.velocity.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.smoother.yarpBuffer.resize(controlledJointNames.size());
    m_hdeRetargeting.com.smoother.yarpBuffer(1);

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
    if(m_useHandRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("HAND_RETARGETING");

        auto initializeHand = [&](auto& hand, const std::string& portNameLabel) -> bool
        {
            // open left hand port
            if(!YarpUtilities::getStringFromSearchable(option, portNameLabel,
                                                       portName))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
                return false;
            }
            hand.port.open("/" + name + portName);

            if (!YarpUtilities::getNumberFromSearchable(option,
                                                        "smoothing_time_approaching",
                                                        hand.smoother.smoothingTimeInApproaching))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the number from "
                            "searchable.";
                return false;
            }

            if (!YarpUtilities::getNumberFromSearchable(option,
                                                        "smoothing_time_walking",
                                                        hand.smoother.smoothingTimeInWalking))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
                return false;
            }

            hand.smoother.yarpBuffer.resize(6);
            hand.smoother.smoother = std::make_unique<iCub::ctrl::minJerkTrajGen>(6, period, hand.smoother.smoothingTimeInApproaching);

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
            const auto element = std::find(controlledJointNames.begin(), controlledJointNames.end(), joint);
            if (element == controlledJointNames.end())
            {
                yError() << "[RetargetingClient::initialize] Unable to find the joint named: "
                         << joint << " in the list of the controlled joints.";
                return false;
            }
            m_retargetedJointsToControlJoints[joint] = std::distance(controlledJointNames.begin(), element);
        }

        if(!YarpUtilities::getNumberFromSearchable(option, "smoothing_time_approaching",
                                                   m_hdeRetargeting.joints.smoother.smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_walking",
                                                    m_hdeRetargeting.joints.smoother.smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_hdeRetargeting.joints.smoother.smoother
            = std::make_unique<iCub::ctrl::minJerkTrajGen>(controlledJointNames.size(),
                                                           period,
                                                           m_hdeRetargeting.joints.smoother.smoothingTimeInApproaching);
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

    if (m_useCoMHeightRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("COM_RETARGETING");

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_approaching",
                                                    m_hdeRetargeting.com.smoother.smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_walking",
                                                    m_hdeRetargeting.com.smoother.smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_hdeRetargeting.com.smoother.smoother
            = std::make_unique<iCub::ctrl::minJerkTrajGen>(1,
                                                           period,
                                                           m_hdeRetargeting.com.smoother.smoothingTimeInApproaching);

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "com_height_scaling_factor",
                                                    m_comHeightScalingFactor))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting)
    {
        if (!YarpUtilities::getStringFromSearchable(config, "hde_port_name", portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_hdeRetargeting.port.open("/" + name + portName);
    }

    return true;
}

bool RetargetingClient::reset(WalkingFK& kinDynWrapper)
{
    m_leftHand.transform = kinDynWrapper.getHeadToWorldTransform().inverse() * kinDynWrapper.getLeftHandToWorldTransform();
    m_rightHand.transform = kinDynWrapper.getHeadToWorldTransform().inverse() * kinDynWrapper.getRightHandToWorldTransform();

    if(m_useHandRetargeting)
    {
        auto resetHandSmoother = [](auto& hand)
        {
            iDynTree::toEigen(hand.yarpReadBuffer).template segment<3>(0) =
            iDynTree::toEigen(hand.transform.getPosition());

            iDynTree::toEigen(hand.yarpReadBuffer).template segment<3>(3) =
            iDynTree::toEigen(hand.transform.getRotation().asRPY());

            hand.smoother.smoother->init(hand.yarpReadBuffer);
        };

        resetHandSmoother(m_leftHand);
        resetHandSmoother(m_rightHand);
    }

    // joint retargeting
    m_hdeRetargeting.joints.position = kinDynWrapper.getJointPos();
    m_hdeRetargeting.joints.initialState = m_hdeRetargeting.joints.position;
    m_hdeRetargeting.joints.rawPosition = m_hdeRetargeting.joints.position;
    m_hdeRetargeting.joints.velocity.zero();
    iDynTree::toEigen(m_hdeRetargeting.joints.smoother.yarpBuffer) = iDynTree::toEigen(m_hdeRetargeting.joints.position);
    if (m_useJointRetargeting)
    {
        m_hdeRetargeting.joints.smoother.smoother->init(m_hdeRetargeting.joints.smoother.yarpBuffer);
    }

    m_hdeRetargeting.com.position = kinDynWrapper.getCoMPosition()(2);
    m_hdeRetargeting.com.velocity = 0;
    m_comConstantHeight = m_hdeRetargeting.com.position;

    if(m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.com.smoother.yarpBuffer(0) = m_hdeRetargeting.com.position;
        m_hdeRetargeting.joints.smoother.smoother->init(m_hdeRetargeting.com.smoother.yarpBuffer);

        // let's read the port to reset the comHeightInput
        bool okCoMHeight = false;
        unsigned int attempt = 0;
        do
        {
            if(!okCoMHeight)
            {
                auto data = m_hdeRetargeting.port.read(false);
                if(data != nullptr)
                {
                    m_comHeightInputOffset = data->CoMPositionWRTGlobal.z;
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

void RetargetingClient::enableApproachingIfNecessary()
{

    if (!m_isFirstDataArrived || yarp::os::Time::now() - m_timestampLastDataArrived > m_isFirstDataArrived)
    {
        this->setPhase(Phase::Approaching);
    }
    m_isFirstDataArrived = true;
    m_timestampLastDataArrived = yarp::os::Time::now();
}

bool RetargetingClient::getFeedback()
{
    if(m_useHandRetargeting)
    {
        auto getHandFeedback = [this](HandRetargeting& hand)
        {
            auto desiredHandPose = hand.port.read(false);
            if(desiredHandPose != nullptr)
            {
                this->enableApproachingIfNecessary();
                hand.smoother.smoother->computeNextValues(*desiredHandPose);
            }
            convertYarpVectorPoseIntoTransform(hand.smoother.smoother->getPos(), hand.transform);
        };

        getHandFeedback(m_leftHand);
        getHandFeedback(m_rightHand);
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting)
    {
        const auto HDEData = m_hdeRetargeting.port.read(false);
        if (HDEData != nullptr)
        {
            this->enableApproachingIfNecessary();
            if (m_useCoMHeightRetargeting)
            {
                if (m_phase == Phase::Walking)
                {
                    m_hdeRetargeting.com.smoother.yarpBuffer(0) = m_comConstantHeight;
                } else
                {
                    const auto& desiredCoMHeight = HDEData->CoMPositionWRTGlobal.z;
                    m_hdeRetargeting.com.smoother.yarpBuffer(0)
                        = (desiredCoMHeight - m_comHeightInputOffset) * m_comHeightScalingFactor
                          + m_comConstantHeight;
                }
            }

            if (m_useJointRetargeting)
            {
                // the first time we should define the map between the retargeted joints and the
                // data coming from HDE
                if (m_retargetedJointsToHDEJoints.empty())
                {
                    const auto& HDEJointNames = HDEData->jointNames;
                    for (const auto& [joint, index] : m_retargetedJointsToControlJoints)
                    {
                        const auto element = std::find(HDEJointNames.begin(), HDEJointNames.end(), joint);
                        if (element == HDEJointNames.end())
                        {
                            yError() << "[RetargetingClient::getFeedback] Unable to find the joint "
                                        "named: "
                                     << joint << " in the list of the HDE joints.";
                            return false;
                        }
                        m_retargetedJointsToHDEJoints[joint] = std::distance(HDEJointNames.begin(), element);
                    }
                }

                const auto& HDEJoints = HDEData->positions;
                for (const auto& [joint, index] : m_retargetedJointsToControlJoints)
                {
                    m_hdeRetargeting.joints.rawPosition(index) = HDEJoints[m_retargetedJointsToHDEJoints[joint]];
                    m_hdeRetargeting.joints.smoother.yarpBuffer(index) = m_hdeRetargeting.joints.rawPosition(index);
                }
            }
        } else
        {
            if (m_phase != Phase::Approaching)
            {
                if (!m_isFirstDataArrived || yarp::os::Time::now() - m_timestampLastDataArrived > m_isFirstDataArrived)
                {
                    this->setPhase(Phase::Approaching);
                    m_retargetedJointsToHDEJoints.clear();
                    iDynTree::toEigen(m_hdeRetargeting.joints.smoother.yarpBuffer) = iDynTree::toEigen(m_hdeRetargeting.joints.initialState);
                }
            }
        }
    }

    // even if the data is not arrived the minimum jerk trajectory has to be updated. This will
    // generate a smoother trajectory
    if (m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.com.smoother.smoother->computeNextValues(
            m_hdeRetargeting.com.smoother.yarpBuffer);
        m_hdeRetargeting.com.position = m_hdeRetargeting.com.smoother.smoother->getPos()(0);
        m_hdeRetargeting.com.velocity = m_hdeRetargeting.com.smoother.smoother->getVel()(0);
    }

    if (m_useJointRetargeting)
    {
        m_hdeRetargeting.joints.smoother.smoother->computeNextValues(
            m_hdeRetargeting.joints.smoother.yarpBuffer);
        iDynTree::toEigen(m_hdeRetargeting.joints.position)
            = iDynTree::toEigen(m_hdeRetargeting.joints.smoother.smoother->getPos());
        iDynTree::toEigen(m_hdeRetargeting.joints.velocity)
            = iDynTree::toEigen(m_hdeRetargeting.joints.smoother.smoother->getVel());
    }

    // check if the approaching phase is finished
    if(m_phase == Phase::Approaching && m_isFirstDataArrived)
    {
        double now = yarp::os::Time::now();
        if(now - m_startingApproachingPhaseTime > m_approachPhaseDuration)
            stopApproachingPhase();
    }

    return true;
}

const iDynTree::Transform& RetargetingClient::leftHandTransform() const
{
    return m_leftHand.transform;
}

const iDynTree::Transform& RetargetingClient::rightHandTransform() const
{
    return m_rightHand.transform;
}

const iDynTree::VectorDynSize& RetargetingClient::jointPositions() const
{
    return m_hdeRetargeting.joints.position;
}

const iDynTree::VectorDynSize& RetargetingClient::jointVelocities() const
{
    return m_hdeRetargeting.joints.velocity;
}

const iDynTree::VectorDynSize& WalkingControllers::RetargetingClient::rawJointPositions() const
{
    return m_hdeRetargeting.joints.rawPosition;
}

double RetargetingClient::comHeight() const
{
    return m_hdeRetargeting.com.position;
}

double RetargetingClient::comHeightVelocity() const
{
    return m_hdeRetargeting.com.velocity;
}

void RetargetingClient::close()
{
    if (m_useHandRetargeting)
    {
        m_leftHand.port.close();
        m_rightHand.port.close();
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.port.close();
    }
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

void RetargetingClient::setPhase(Phase phase)
{
    if (phase != Phase::Approaching && m_phase == Phase::Approaching)
    {
        return;
    }

    if (phase == Phase::Approaching)
    {
        startApproachingPhase();
    }

    if (m_phase == Phase::Approaching && phase == Phase::Walking)
        stopApproachingPhase();

    if (m_phase == Phase::Approaching && phase == Phase::Stance)
        stopApproachingPhase();

    m_phase = phase;
}

void RetargetingClient::stopApproachingPhase()
{
    if (m_useHandRetargeting)
    {
        m_leftHand.smoother.smoother->setT(m_leftHand.smoother.smoothingTimeInWalking);
        m_rightHand.smoother.smoother->setT(m_rightHand.smoother.smoothingTimeInWalking);
    }

    if (m_useJointRetargeting)
    {
        m_hdeRetargeting.joints.smoother.smoother->setT(
            m_hdeRetargeting.joints.smoother.smoothingTimeInWalking);
    }

    if (m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.com.smoother.smoother->setT(
            m_hdeRetargeting.com.smoother.smoothingTimeInWalking);
    }

    m_phase = Phase::Stance;
}

void RetargetingClient::startApproachingPhase()
{
    // if the retargeting is not used the approaching phase is not required
    if(!m_useHandRetargeting && !m_useJointRetargeting && !m_useCoMHeightRetargeting)
        return;

    m_startingApproachingPhaseTime = yarp::os::Time::now();

    if(m_useHandRetargeting)
    {
        m_leftHand.smoother.smoother->setT(m_leftHand.smoother.smoothingTimeInApproaching);
        m_rightHand.smoother.smoother->setT(m_rightHand.smoother.smoothingTimeInApproaching);
    }

    if (m_useJointRetargeting)
    {
        m_hdeRetargeting.joints.smoother.smoother->setT(
            m_hdeRetargeting.joints.smoother.smoothingTimeInApproaching);
    }

    if (m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.com.smoother.smoother->setT(
            m_hdeRetargeting.com.smoother.smoothingTimeInApproaching);
    }

}

bool RetargetingClient::isApproachingPhase() const
{
    return m_phase == Phase::Approaching;
}
