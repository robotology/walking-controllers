/**
 * @file Helper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <WalkingControllers/RetargetingHelper/Helper.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

void RetargetingClient::convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                                           iDynTree::Transform& transform)
{
    transform.setPosition(iDynTree::Position(vector(0), vector(1), vector(2)));
    transform.setRotation(iDynTree::Rotation::RPY(vector(3), vector(4), vector(5)));
}

bool RetargetingClient::initialize(const yarp::os::Searchable& config,
                                   const std::string& name,
                                   const double& period,
                                   const std::vector<std::string>& controlledJointNames)
{
    if (config.isNull())
    {
        yInfo() << "[RetargetingClient::initialize] the hand retargeting is disable";
        m_useHandRetargeting = false;
        m_useVirtualizer = false;
        m_useJointRetargeting = false;
        m_useCoMHeightRetargeting = false;
        m_useCoMHorizontalRetargeting = false;
        return true;
    }

    m_useHandRetargeting = config.check("use_hand_retargeting", yarp::os::Value(false)).asBool();
    m_useJointRetargeting = config.check("use_joint_retargeting", yarp::os::Value(false)).asBool();
    m_useVirtualizer = config.check("use_virtualizer", yarp::os::Value(false)).asBool();
    m_useCoMHeightRetargeting
        = config.check("use_com_height_retargeting", yarp::os::Value(false)).asBool();
    m_useCoMHorizontalRetargeting
        = config.check("use_com_horizontal_retargeting", yarp::os::Value(false)).asBool();

    if (m_useJointRetargeting && m_useHandRetargeting)
    {
        yError() << "[RetargetingClient::initialize] You cannot enable the joint retargeting along "
                    "with the hand retargeting.";
        return false;
    }

    m_hdeRetargeting.joints.position.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.velocity.resize(controlledJointNames.size());
    m_hdeRetargeting.joints.smoother.yarpBuffer.resize(controlledJointNames.size());
    m_hdeRetargeting.comHeight.smoother.yarpBuffer(1);
    m_hdeRetargeting.comHorizontal.smoother.yarpBuffer(2);

    if (!m_useHandRetargeting && !m_useVirtualizer && !m_useJointRetargeting
        && !m_useCoMHeightRetargeting)
    {
        return true;
    }

    // get the approaching phase duration
    if (!YarpUtilities::getNumberFromSearchable(config,
                                                "approaching_phase_duration",
                                                m_approachPhaseDuration))
    {
        yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
        return false;
    }

    std::string portName;
    if (m_useHandRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("HAND_RETARGETING");

        auto initializeHand = [&](auto& hand, const std::string& portNameLabel) -> bool {
            // open left hand port
            if (!YarpUtilities::getStringFromSearchable(option, portNameLabel, portName))
            {
                yError() << "[RetargetingClient::initialize] Unable to get the string from "
                            "searchable.";
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
                yError() << "[RetargetingClient::initialize] Unable to get the number from "
                            "searchable.";
                return false;
            }

            hand.smoother.yarpBuffer.resize(6);
            hand.smoother.smoother
                = std::make_unique<iCub::ctrl::minJerkTrajGen>(6,
                                                               period,
                                                               hand.smoother
                                                                   .smoothingTimeInApproaching);

            return true;
        };

        if (!initializeHand(m_leftHand, "left_hand_transform_port_name")
            || !initializeHand(m_rightHand, "right_hand_transform_port_name"))
            return false;
    }

    if (m_useJointRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("JOINT_RETARGETING");

        std::vector<std::string> retargetJointNames;
        yarp::os::Value* retargetJointNamesYarp;
        if (!option.check("retargeting_joint_list", retargetJointNamesYarp))
        {
            yError() << "[RetargetingClient::initialize] Unable to find joints_list into config "
                        "file.";
            return false;
        }
        if (!YarpUtilities::yarpListToStringVector(retargetJointNamesYarp, retargetJointNames))
        {
            yError() << "[RetargetingClient::initialize] Unable to convert yarp list into a vector "
                        "of strings.";
            return false;
        }

        // find the indices
        for (const std::string& joint : retargetJointNames)
        {
            const auto element
                = std::find(controlledJointNames.begin(), controlledJointNames.end(), joint);
            if (element == controlledJointNames.end())
            {
                yError() << "[RetargetingClient::initialize] Unable to find the joint named: "
                         << joint << " in the list of the controlled joints.";
                return false;
            }
            m_retargetedJointsToControlJoints[joint]
                = std::distance(controlledJointNames.begin(), element);
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_approaching",
                                                    m_hdeRetargeting.joints.smoother
                                                        .smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_walking",
                                                    m_hdeRetargeting.joints.smoother
                                                        .smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_hdeRetargeting.joints.smoother.smoother
            = std::make_unique<iCub::ctrl::minJerkTrajGen>(controlledJointNames.size(),
                                                           period,
                                                           m_hdeRetargeting.joints.smoother
                                                               .smoothingTimeInApproaching);
    }

    if (m_useVirtualizer)
    {
        const yarp::os::Bottle& option = config.findGroup("VIRTUALIZER");

        if (!YarpUtilities::getStringFromSearchable(option, "robot_orientation_port_name", portName))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the string from searchable.";
            return false;
        }
        m_robotOrientationPort.open("/" + name + portName);
    }

    if (m_useCoMHeightRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("COM_HEIGHT_RETARGETING");

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_approaching",
                                                    m_hdeRetargeting.comHeight.smoother
                                                        .smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_walking",
                                                    m_hdeRetargeting.comHeight.smoother
                                                        .smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_hdeRetargeting.comHeight.smoother.smoother
            = std::make_unique<iCub::ctrl::minJerkTrajGen>(1,
                                                           period,
                                                           m_hdeRetargeting.comHeight.smoother
                                                               .smoothingTimeInApproaching);

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "com_height_scaling_factor",
                                                    m_comHeightScalingFactor))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }
    }

    if (m_useCoMHorizontalRetargeting)
    {
        const yarp::os::Bottle& option = config.findGroup("COM_HORIZONTAL_RETARGETING");

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_approaching",
                                                    m_hdeRetargeting.comHorizontal.smoother
                                                        .smoothingTimeInApproaching))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getNumberFromSearchable(option,
                                                    "smoothing_time_walking",
                                                    m_hdeRetargeting.comHorizontal.smoother
                                                        .smoothingTimeInWalking))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        m_hdeRetargeting.comHeight.smoother.smoother
            = std::make_unique<iCub::ctrl::minJerkTrajGen>(2,
                                                           period,
                                                           m_hdeRetargeting.comHorizontal.smoother
                                                               .smoothingTimeInApproaching);

        if (!YarpUtilities::getVectorFromSearchable(option, "com_x_limits", m_comXLimits))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }

        if (!YarpUtilities::getVectorFromSearchable(option, "com_y_limits", m_comYLimits))
        {
            yError() << "[RetargetingClient::initialize] Unable to get the number from searchable.";
            return false;
        }
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting || m_useCoMHorizontalRetargeting)
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
    m_leftHand.transform = kinDynWrapper.getHeadToWorldTransform().inverse()
                           * kinDynWrapper.getLeftHandToWorldTransform();
    m_rightHand.transform = kinDynWrapper.getHeadToWorldTransform().inverse()
                            * kinDynWrapper.getRightHandToWorldTransform();

    if (m_useHandRetargeting)
    {
        auto resetHandSmoother = [](auto& hand) {
            iDynTree::toEigen(hand.yarpReadBuffer).template segment<3>(0)
                = iDynTree::toEigen(hand.transform.getPosition());

            iDynTree::toEigen(hand.yarpReadBuffer).template segment<3>(3)
                = iDynTree::toEigen(hand.transform.getRotation().asRPY());

            hand.smoother.smoother->init(hand.yarpReadBuffer);
        };

        resetHandSmoother(m_leftHand);
        resetHandSmoother(m_rightHand);
    }

    // joint retargeting
    m_hdeRetargeting.joints.position = kinDynWrapper.getJointPos();
    m_hdeRetargeting.joints.velocity.zero();
    iDynTree::toEigen(m_hdeRetargeting.joints.smoother.yarpBuffer)
        = iDynTree::toEigen(m_hdeRetargeting.joints.position);
    if (m_useJointRetargeting)
    {
        m_hdeRetargeting.joints.smoother.smoother->init(
            m_hdeRetargeting.joints.smoother.yarpBuffer);
    }
    m_comHorizontalRetargetingReady = false;

    m_hdeRetargeting.comHorizontal.position.zero();
    m_hdeRetargeting.comHorizontal.velocity.zero();
    iDynTree::toEigen(m_hdeRetargeting.comHorizontal.smoother.yarpBuffer)
        = iDynTree::toEigen(m_hdeRetargeting.comHorizontal.position);

    if (m_useCoMHorizontalRetargeting)
    {
        m_hdeRetargeting.comHorizontal.smoother.smoother->init(
            m_hdeRetargeting.comHorizontal.smoother.yarpBuffer);
    }

    m_hdeRetargeting.comHeight.position = kinDynWrapper.getCoMPosition()(2);
    m_hdeRetargeting.comHeight.velocity = 0;
    m_comConstantHeight = m_hdeRetargeting.comHeight.position;

    if (m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.comHeight.smoother.yarpBuffer(0) = m_hdeRetargeting.comHeight.position;
        m_hdeRetargeting.joints.smoother.smoother->init(
            m_hdeRetargeting.comHeight.smoother.yarpBuffer);

        // let's read the port to reset the comHeightInput
        bool okCoMHeight = false;
        unsigned int attempt = 0;
        do
        {
            if (!okCoMHeight)
            {
                auto data = m_hdeRetargeting.port.read(false);
                if (data != nullptr)
                {
                    m_comHeightInputOffset = data->CoMPositionWRTGlobal.z;
                    okCoMHeight = true;
                }
            }

            if (okCoMHeight)
                return true;

            yarp::os::Time::delay(0.001);
            attempt++;
        } while (attempt < 100);

        if (!okCoMHeight)
            yError() << "[RetargetingClient::reset] The CoM height is not coming from the yarp "
                        "port.";
        return false;
    }

    return true;
}

bool RetargetingClient::getFeedback()
{
    if (m_useHandRetargeting)
    {
        auto getHandFeedback = [this](HandRetargeting& hand) {
            auto desiredHandPose = hand.port.read(false);
            if (desiredHandPose != nullptr)
            {
                hand.smoother.smoother->computeNextValues(*desiredHandPose);
            }
            convertYarpVectorPoseIntoTransform(hand.smoother.smoother->getPos(), hand.transform);
        };

        getHandFeedback(m_leftHand);
        getHandFeedback(m_rightHand);
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting || m_useCoMHorizontalRetargeting)
    {
        const auto HDEData = m_hdeRetargeting.port.read(false);
        if (HDEData != nullptr)
        {
            if (m_useCoMHeightRetargeting)
            {
                if (m_phase == Phase::Walking)
                {
                    m_hdeRetargeting.comHeight.smoother.yarpBuffer(0) = m_comConstantHeight;
                } else
                {
                    const auto& desiredCoMHeight = HDEData->CoMPositionWRTGlobal.z;
                    m_hdeRetargeting.comHeight.smoother.yarpBuffer(0)
                        = (desiredCoMHeight - m_comHeightInputOffset) * m_comHeightScalingFactor
                          + m_comConstantHeight;
                }
            }

            if (m_useCoMHorizontalRetargeting)
            {
                if (!m_comHorizontalRetargetingReady)
                {
                    // the following line is valid only because in the human state provider the
                    // root link orientation is set constant
                    m_comHorizontalInputOffset[0] = HDEData->CoMPositionWRTGlobal.x;
                    m_comHorizontalInputOffset[1] = HDEData->CoMPositionWRTGlobal.y;
                    m_comHorizontalRetargetingReady = true;
                }

                const auto& HDECoM = HDEData->CoMPositionWRTGlobal;

                if (m_phase == Phase::Walking)
                {
                    m_hdeRetargeting.comHorizontal.smoother.yarpBuffer.zero();
                } else{
                    // Phase::Approaching or Phase::Stance
                    m_hdeRetargeting.comHorizontal.smoother.yarpBuffer[0]
                        = std::max(m_comXLimits[0],
                                   std::min(HDECoM.x - m_comHorizontalInputOffset[0],
                                            m_comXLimits[1]));

                    m_hdeRetargeting.comHorizontal.smoother.yarpBuffer[1]
                        = std::max(m_comYLimits[0],
                                   std::min(HDECoM.y - m_comHorizontalInputOffset[1],
                                            m_comYLimits[1]));
                }

                for (const auto& [joint, index] : m_retargetedJointsToControlJoints)
                {
                    m_hdeRetargeting.joints.smoother.yarpBuffer(index)
                        = HDEData->positions[m_retargetedJointsToHDEJoints[joint]];
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
                        const auto element
                            = std::find(HDEJointNames.begin(), HDEJointNames.end(), joint);
                        if (element == HDEJointNames.end())
                        {
                            yError() << "[RetargetingClient::getFeedback] Unable to find the joint "
                                        "named: "
                                     << joint << " in the list of the HDE joints.";
                            return false;
                        }
                        m_retargetedJointsToHDEJoints[joint]
                            = std::distance(HDEJointNames.begin(), element);
                    }
                }

                const auto& HDEJoints = HDEData->positions;
                for (const auto& [joint, index] : m_retargetedJointsToControlJoints)
                {
                    m_hdeRetargeting.joints.smoother.yarpBuffer(index)
                        = HDEData->positions[m_retargetedJointsToHDEJoints[joint]];
                }
            }
        }
    }

    // even if the data is not arrived the minimum jerk trajectory has to be updated. This will
    // generate a smoother trajectory
    if (m_useCoMHeightRetargeting)
    {
        m_hdeRetargeting.comHeight.smoother.smoother->computeNextValues(
            m_hdeRetargeting.comHeight.smoother.yarpBuffer);
        m_hdeRetargeting.comHeight.position
            = m_hdeRetargeting.comHeight.smoother.smoother->getPos()(0);
        m_hdeRetargeting.comHeight.velocity
            = m_hdeRetargeting.comHeight.smoother.smoother->getVel()(0);
    }

    if (m_useCoMHorizontalRetargeting)
    {
        m_hdeRetargeting.comHorizontal.smoother.smoother->computeNextValues(
            m_hdeRetargeting.comHorizontal.smoother.yarpBuffer);
        iDynTree::toEigen(m_hdeRetargeting.comHorizontal.position)
            = iDynTree::toEigen(m_hdeRetargeting.comHorizontal.smoother.smoother->getPos());
        iDynTree::toEigen(m_hdeRetargeting.comHorizontal.velocity)
            = iDynTree::toEigen(m_hdeRetargeting.comHorizontal.smoother.smoother->getVel());
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
    if (m_phase == Phase::Approaching)
    {
        double now = yarp::os::Time::now();
        if (now - m_startingApproachingPhaseTime > m_approachPhaseDuration)
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

double RetargetingClient::comHeight() const
{
    return m_hdeRetargeting.comHeight.position;
}

double RetargetingClient::comHeightVelocity() const
{
    return m_hdeRetargeting.comHeight.velocity;
}

const iDynTree::Vector2& RetargetingClient::comHorizonal() const
{
    return m_hdeRetargeting.comHorizontal.position;
}

void RetargetingClient::close()
{
    if (m_useHandRetargeting)
    {
        m_leftHand.port.close();
        m_rightHand.port.close();
    }

    if (m_useJointRetargeting || m_useCoMHeightRetargeting || m_useCoMHorizontalRetargeting)
    {
        m_hdeRetargeting.port.close();
    }
}

void RetargetingClient::setRobotBaseOrientation(const iDynTree::Rotation& rotation)
{
    if (!m_useVirtualizer)
        return;

    yarp::sig::Vector& output = m_robotOrientationPort.prepare();
    output.clear();
    output.push_back(rotation.asRPY()(2));
    m_robotOrientationPort.write(false);
}

void RetargetingClient::setPhase(Phase phase)
{
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
        m_hdeRetargeting.comHeight.smoother.smoother->setT(
            m_hdeRetargeting.comHeight.smoother.smoothingTimeInWalking);
    }

    if (m_useCoMHorizontalRetargeting)
    {
        m_hdeRetargeting.comHorizontal.smoother.smoother->setT(
            m_hdeRetargeting.comHorizontal.smoother.smoothingTimeInWalking);
    }

    m_phase = Phase::Stance;
}

void RetargetingClient::startApproachingPhase()
{
    // if the retargeting is not used the approaching phase is not required
    if (!m_useHandRetargeting && !m_useJointRetargeting && !m_useCoMHeightRetargeting)
        return;

    m_startingApproachingPhaseTime = yarp::os::Time::now();

    if (m_useHandRetargeting)
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
        m_hdeRetargeting.comHeight.smoother.smoother->setT(
            m_hdeRetargeting.comHeight.smoother.smoothingTimeInApproaching);
    }

    if (m_useCoMHorizontalRetargeting)
    {
        m_hdeRetargeting.comHorizontal.smoother.smoother->setT(
            m_hdeRetargeting.comHorizontal.smoother.smoothingTimeInApproaching);
    }

}

bool RetargetingClient::isApproachingPhase() const
{
    return m_phase == Phase::Approaching;
}
