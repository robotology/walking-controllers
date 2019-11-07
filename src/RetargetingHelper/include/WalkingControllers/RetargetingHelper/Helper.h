/**
 * @file Helper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_CONTROLLERS_RETARGETING_HELPER_HELPER_H
#define WALKING_CONTROLLERS_RETARGETING_HELPER_HELPER_H

// std
#include <memory>
#include <vector>

// iDyntree
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/VectorDynSize.h>

// yarp
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/minJerkCtrl.h>

namespace WalkingControllers
{

/**
 * Client for the retargeting application
 */
    class RetargetingClient
    {
    private:

        bool m_useHandRetargeting; /**< True if the hand retargeting is used */
        bool m_useVirtualizer; /**< True if the virtualizer is used */
        bool m_useJointRetargeting; /**< True if the joint retargeting is used */
        bool m_useCoMHeightRetargeting; /**< True if the com retargeting is used */

        iDynTree::Transform m_leftHandTransform; /**< Left hand transform head_T_leftHand */
        iDynTree::Transform m_rightHandTransform; /**< Right hand transform head_T_rightHand*/

        yarp::sig::Vector m_leftHandTransformYarp; /**< Left hand position + rpy head_T_leftHand */
        yarp::sig::Vector m_rightHandTransformYarp; /**< Right hand position + rpy head_T_rightHand*/
        yarp::sig::Vector m_comHeightYarp; /**< Left hand position + rpy head_T_leftHand */

        yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandTransformPort; /**< Right hand position + rpy head_T_rightHand*/
        yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandTransformPort; /**< Right hand position + rpy head_T_rightHand*/
        yarp::os::BufferedPort<yarp::sig::Vector> m_comHeightPort; /**< CoMHeight */

        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_leftHandSmoother; /**< Minimum jerk trajectory
                                                                           for the left hand. */

        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_rightHandSmoother; /**< Minimum jerk trajectory
                                                                            for the right hand. */

        std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_comHeightSmoother; /**< CoM Height smooter. */


        yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort; /**< Average orientation of the robot.*/

        std::vector<int> m_retargetJointsIndex; /**< Vector containing the indices of the retarget joints. */
        yarp::os::BufferedPort<yarp::sig::Vector> m_jointRetargetingPort; /**< joint retargeting port. */
        iDynTree::VectorDynSize m_retargetJoints; /**< Values of the retarget Joints. */

        double m_comHeightInput;
        double m_comHeight;
        double m_comHeightVelocity;
        double m_comConstantHeight;

        bool m_isStancePhase{true};

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
         * @param controlledJointsName name of the controlled joints
         * @return true/false in case of success/failure
         */
        bool initialize(const yarp::os::Searchable &config,
                        const std::string &name,
                        const double &period,
                        const std::vector<std::string>& controlledJointsName);

        /**
         * Reset the client
         * @param leftHandTransform head_T_leftHand transform
         * @param rightHandTransform head_T_rightHand transform
         * @param jointValues joint values [rad]
         * @param comHeight height of the CoM
         * @return true/false in case of success/failure
         */
        bool reset(const iDynTree::Transform& leftHandTransform,
                   const iDynTree::Transform& rightHandTransform,
                   const iDynTree::VectorDynSize& jointValues,
                   const double& comHeight);

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
         * Get the value of the retarget joints
         */
        const iDynTree::VectorDynSize& jointValues() const;

        double comHeight() const;

        double comHeightVelocity() const;

        /**
         * Get the homogeneous transform of the right hand w.r.t. the head frame head_T_rightHand
         */
        void setRobotBaseOrientation(const iDynTree::Rotation& rotation);

        bool setPhase(bool isStancePhase);
    };
};
#endif
