/**
 * @file WalkingRetargetingClient.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef RETARGETING_CLIENT_HPP
#define RETARGETING_CLIENT_HPP

// std
#include <memory>

// iDyntree
#include <iDynTree/Core/Transform.h>

// yarp
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>

/**
 * Client for the retargeting application
 */
class RetargetingClient
{
private:

    bool m_useHandRetargeting; /**< True if the hand retargeting is used */
    bool m_useVirtualizer; /**< True if the virtualizer is used */

    iDynTree::Transform m_leftHandTransform; /**< Left hand transform head_T_leftHand */
    iDynTree::Transform m_rightHandTransform; /**< Right hand transform head_T_rightHand*/

    yarp::sig::Vector m_leftHandTransformYarp; /**< Left hand position + rpy head_T_leftHand */
    yarp::sig::Vector m_rightHandTransformYarp; /**< Right hand position + rpy head_T_rightHand*/

    yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandTransformPort; /**< Right hand position + rpy head_T_rightHand*/
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandTransformPort; /**< Right hand position + rpy head_T_rightHand*/

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_leftHandSmoother; /**< Minimum jerk trajectory
                                                                       for the left hand. */

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_rightHandSmoother; /**< Minimum jerk trajectory
                                                                        for the right hand. */

    yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort; /**< Average orientation of the robot.*/

    /**
     * Convert a yarp vector containing position + rpy into an iDynTree homogeneous transform
     * @param vector a 6d yarp vector
     * @param transform an iDyntree homogeneous transformation
     */
    void convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                            iDynTree::Transform& transform);

public:

    /**
     * Initialize the client
     * @param config configuration parameters
     * @param name name of the module
     * @param period period of the module
     * @return true/false in case of success/failure
     */
    bool initialize(const yarp::os::Searchable &config,
                    const std::string &name,
                    const double &period);

    /**
     * Reset the client
     * @param leftHandTransform head_T_leftHand transform
     * @param rightHandTransform head_T_rightHand transform
     */
    void reset(const iDynTree::Transform& leftHandTransform,
               const iDynTree::Transform& rightHandTransform);

    /**
     * Close the client
     */
    void close();

    /**
     * Get the feedback of the server
     */
    void getFeedback();

    /**
     * Get the homogeneous transform of the left hand w.r.t. the head frame head_T_leftHand
     */
    const iDynTree::Transform& leftHandTransform() const;

    /**
     * Get the homogeneous transform of the right hand w.r.t. the head frame head_T_rightHand
     */
    const iDynTree::Transform& rightHandTransform() const;

    /**
     * Get the homogeneous transform of the right hand w.r.t. the head frame head_T_rightHand
     */
    void setRobotBaseOrientation(const iDynTree::Rotation& rotation);
};

#endif
