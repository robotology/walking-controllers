/**
 * @file WalkingForwardKinematics.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>

#include "WalkingForwardKinematics.hpp"
#include "Utils.hpp"

bool WalkingFK::setRobotModel(const iDynTree::Model& model)
{
    if(!m_kinDyn.loadRobotModel(model))
    {
        yError() << "[setRobotModel] Error while loading into KinDynComputations object.";
        return false;
    }

    m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    // initialize some quantities needed for the first step
    m_prevContactLeft = false;

    return true;
}

bool WalkingFK::setBaseFrames(const std::string& lFootFrame, const std::string& rFootFrame)
{
    if(!m_kinDyn.isValid())
    {
        yError() << "[setBaseFrames] Please set the Robot model before calling this method.";
        return false;
    }

    // set frames base frames
    // note: in the following the base frames will be:
    // - left_foot when the left foot is the stance foot;
    // - right_foot when the right foot is the stance foot.
    m_frameLeftIndex = m_kinDyn.model().getFrameIndex(lFootFrame);
    if(m_frameLeftIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[setBaseFrames] Unable to find the frame named: " << lFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkLeftIndex = m_kinDyn.model().getFrameLink(m_frameLeftIndex);
    m_baseFrameLeft = m_kinDyn.model().getLinkName(linkLeftIndex);
    m_frameHlinkLeft = m_kinDyn.getRelativeTransform(m_frameLeftIndex, linkLeftIndex);

    m_frameRightIndex = m_kinDyn.model().getFrameIndex(rFootFrame);
    if(m_frameRightIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[setBaseFrames] Unable to find the frame named: " << rFootFrame;
        return false;
    }
    iDynTree::LinkIndex linkRightIndex = m_kinDyn.model().getFrameLink(m_frameRightIndex);
    m_baseFrameRight = m_kinDyn.model().getLinkName(linkRightIndex);
    m_frameHlinkRight = m_kinDyn.getRelativeTransform(m_frameRightIndex, linkRightIndex);

    return true;
}

bool WalkingFK::initialize(const yarp::os::Searchable& config,
                           const iDynTree::Model& model)
{
    // check if the config is empty
    if(!setRobotModel(model))
    {
        yError() << "[initialize] Unable to set the robot model.";
        return false;
    }

    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for fk solver.";
        return false;
    }

    // set the left foot frame
    std::string lFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "left_foot_frame", lFootFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }

    // set the right foot frame
    std::string rFootFrame;
    if(!YarpHelper::getStringFromSearchable(config, "right_foot_frame", rFootFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }

    // set base frames
    if(!setBaseFrames(lFootFrame, rFootFrame))
    {
        yError() << "[initialize] Unable to set the base frames.";
        return false;
    }

    // set the left hand frame
    std::string lHandFrame;
    if(!YarpHelper::getStringFromSearchable(config, "left_hand_frame", lHandFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameLeftHandIndex = m_kinDyn.model().getFrameIndex(lHandFrame);
    if(m_frameLeftHandIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: " << lHandFrame;
        return false;
    }

    // set the right hand frame
    std::string rHandFrame;
    if(!YarpHelper::getStringFromSearchable(config, "right_hand_frame", rHandFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameRightHandIndex = m_kinDyn.model().getFrameIndex(rHandFrame);
    if(m_frameRightHandIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: " << rHandFrame;
        return false;
    }

    std::string headFrame;
    if(!YarpHelper::getStringFromSearchable(config, "head_frame", headFrame))
    {
        yError() << "[initialize] Unable to get the string from searchable.";
        return false;
    }
    m_frameHeadIndex = m_kinDyn.model().getFrameIndex(headFrame);
    if(m_frameHeadIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: " << headFrame;
        return false;
    }

    m_frameRootIndex = m_kinDyn.model().getFrameIndex("root_link");
    if(m_frameRootIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    m_frameNeckIndex = m_kinDyn.model().getFrameIndex("neck_2");
    if(m_frameNeckIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[initialize] Unable to find the frame named: root_link";
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();
    m_omega = sqrt(gravityAcceleration / comHeight);

    // init filters
    double samplingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    double cutFrequency;
    if(!YarpHelper::getNumberFromSearchable(config, "cut_frequency", cutFrequency))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }

    m_comPositionFiltered.zero();
    m_comVelocityFiltered.zero();
    m_comPositionFiltered(2) = comHeight;


    m_comPositionFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);
    m_comVelocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, samplingTime);

    // TODO this is wrong, we shold initialize the filter with a meaningful value;
    yarp::sig::Vector buff(3);
    iDynTree::toYarp(m_comPositionFiltered, buff);
    m_comPositionFilter->init(buff);

    iDynTree::toYarp(m_comVelocityFiltered, buff);
    m_comVelocityFilter->init(buff);

    m_useFilters = config.check("use_filters", yarp::os::Value(false)).asBool();
    m_firstStep = true;
    return true;
}

bool WalkingFK::evaluateFirstWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform)
{
    m_worldToBaseTransform = leftFootTransform * m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                 << "base on link " << m_baseFrameLeft;
        return false;
    }
    m_prevContactLeft = true;
    return true;
}

bool WalkingFK::evaluateWorldToBaseTransformation(const bool& isLeftFixedFrame)
{
    if(isLeftFixedFrame)
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the right foot
        if(!m_prevContactLeft)
        {
            m_worldToBaseTransform =  this->getLeftFootToWorldTransform() * m_frameHlinkLeft;
            if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameLeft;
                return false;
            }
            m_prevContactLeft = true;
        }
    }
    else
    {
        // evaluate the new world to base transformation only if the previous if the previous fixed frame was
        // the left foot
        if(m_prevContactLeft)
        {
            m_worldToBaseTransform = this->getRightFootToWorldTransform() * m_frameHlinkRight;
            if(!m_kinDyn.setFloatingBase(m_baseFrameRight))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameRight;
                return false;
            }
            m_prevContactLeft = false;
        }
    }
    return true;
}

bool WalkingFK::evaluateWorldToBaseTransformation(const iDynTree::Transform& leftFootTransform,
                                                  const iDynTree::Transform& rightFootTransform,
                                                  const bool& isLeftFixedFrame)
{
    if(isLeftFixedFrame)
    {
        // evaluate the new world to base transformation only if the previous fixed frame was
        // the right foot
        if(!m_prevContactLeft || m_firstStep)
        {
            m_worldToBaseTransform = leftFootTransform * m_frameHlinkLeft;
            if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameLeft;
                return false;
            }
            m_prevContactLeft = true;
        }
    }
    else
    {
        // evaluate the new world to base transformation only if the previous if the previous fixed frame was
        // the left foot
        if(m_prevContactLeft || m_firstStep)
        {
            m_worldToBaseTransform = rightFootTransform * m_frameHlinkRight;
            if(!m_kinDyn.setFloatingBase(m_baseFrameRight))
            {
                yError() << "[evaluateWorldToBaseTransformation] Error while setting the floating "
                         << "base on link " << m_baseFrameRight;
                return false;
            }
            m_prevContactLeft = false;
        }
    }

    m_firstStep = false;
    return true;
}

bool WalkingFK::setInternalRobotState(const iDynTree::VectorDynSize& positionFeedbackInRadians,
                                      const iDynTree::VectorDynSize& velocityFeedbackInRadians)
{
    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;

    if(!m_kinDyn.setRobotState(m_worldToBaseTransform, positionFeedbackInRadians,
                               iDynTree::Twist::Zero(), velocityFeedbackInRadians,
                               gravity))
    {
        yError() << "[setInternalRobotState] Error while updating the state.";
        return false;
    }

    m_comEvaluated = false;
    m_dcmEvaluated = false;

    return true;
}

void WalkingFK::evaluateCoM()
{
    if(m_comEvaluated)
        return;

    m_comPosition = m_kinDyn.getCenterOfMassPosition();
    m_comVelocity = m_kinDyn.getCenterOfMassVelocity();

    yarp::sig::Vector temp;
    temp.resize(3);

    iDynTree::toYarp(m_comPosition, temp);
    iDynTree::toEigen(m_comPositionFiltered) = iDynTree::toEigen(m_comPositionFilter->filt(temp));

    iDynTree::toYarp(m_comVelocity, temp);
    iDynTree::toEigen(m_comVelocityFiltered) = iDynTree::toEigen(m_comVelocityFilter->filt(temp));

    m_comEvaluated = true;

    return;
}

void WalkingFK::evaluateDCM()
{
    if(m_dcmEvaluated)
        return;

    evaluateCoM();

    iDynTree::Vector3 dcm3D;

    // evaluate the 3D-DCM
    if(m_useFilters)
        iDynTree::toEigen(dcm3D) = iDynTree::toEigen(m_comPositionFiltered) +
            iDynTree::toEigen(m_comVelocityFiltered) / m_omega;
    else
        iDynTree::toEigen(dcm3D) = iDynTree::toEigen(m_comPosition) +
            iDynTree::toEigen(m_comVelocity) / m_omega;

    // take only the 2D projection
    m_dcm(0) = dcm3D(0);
    m_dcm(1) = dcm3D(1);

    m_dcmEvaluated = true;

    return;
}

const iDynTree::Vector2& WalkingFK::getDCM()
{
    evaluateDCM();
    return m_dcm;
}

const iDynTree::Position& WalkingFK::getCoMPosition()
{
    evaluateCoM();

    if(m_useFilters)
        return m_comPositionFiltered;
    else
        return m_comPosition;
}

const iDynTree::Vector3& WalkingFK::getCoMVelocity()
{
    evaluateCoM();

    if(m_useFilters)
        return m_comVelocityFiltered;
    else
        return m_comVelocity;
}

bool WalkingFK::setBaseOnTheFly()
{
    m_worldToBaseTransform = m_frameHlinkLeft;
    if(!m_kinDyn.setFloatingBase(m_baseFrameLeft))
    {
        yError() << "[setBaseOnTheFly] Error while setting the floating base on link "
                 << m_baseFrameLeft;
        return false;
    }

    return true;
}

iDynTree::Transform WalkingFK::getLeftFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameLeftIndex);
}

iDynTree::Transform WalkingFK::getRightFootToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRightIndex);
}

iDynTree::Transform WalkingFK::getLeftHandToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameLeftHandIndex);
}

iDynTree::Transform WalkingFK::getRightHandToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRightHandIndex);
}

iDynTree::Transform WalkingFK::getHeadToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameHeadIndex);
}

iDynTree::Transform WalkingFK::getRootLinkToWorldTransform()
{
    return m_kinDyn.getWorldTransform(m_frameRootIndex);
}

iDynTree::Twist WalkingFK::getRootLinkVelocity()
{
    return m_kinDyn.getFrameVel(m_frameRootIndex);
}

iDynTree::Rotation WalkingFK::getNeckOrientation()
{
    return m_kinDyn.getWorldTransform(m_frameNeckIndex).getRotation();
}

bool WalkingFK::getLeftFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameLeftIndex, jacobian);
}

bool WalkingFK::getRightFootJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameRightIndex, jacobian);
}

bool WalkingFK::getRightHandJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameRightHandIndex, jacobian);
}

bool WalkingFK::getLeftHandJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameLeftHandIndex, jacobian);
}

bool WalkingFK::getNeckJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getFrameFreeFloatingJacobian(m_frameNeckIndex, jacobian);
}

bool WalkingFK::getCoMJacobian(iDynTree::MatrixDynSize &jacobian)
{
    return m_kinDyn.getCenterOfMassJacobian(jacobian);
}
