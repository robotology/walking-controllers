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

        typedef struct
        {
            yarp::sig::Vector yarpVector;
            std::unique_ptr<iCub::ctrl::minJerkTrajGen> smoother;
            yarp::os::BufferedPort<yarp::sig::Vector> port;
            double smoothingTimeInApproaching;
            double smoothingTimeInWalking;
        } retargetingElement;

        bool m_useHandRetargeting; /**< True if the hand retargeting is used */
        bool m_useVirtualizer; /**< True if the virtualizer is used */
        bool m_useJointRetargeting; /**< True if the joint retargeting is used */
        bool m_useCoMHeightRetargeting; /**< True if the com retargeting is used */

        iDynTree::Transform m_leftHandTransform; /**< Desired left hand transform */
        retargetingElement m_leftHand; /**< Left hand retargeting element */

        iDynTree::Transform m_rightHandTransform; /**< Desired right hand transform */
        retargetingElement m_rightHand; /**< Right hand retargeting element */

        double m_comHeightValue;
        double m_comHeightInputZero;
        double m_comHeightVelocity;
        double m_comConstantHeight;
        retargetingElement m_comHeight;

        std::vector<int> m_retargetJointsIndex; /**< Vector containing the indices of the retarget joints. */
        iDynTree::VectorDynSize m_jointRetargetingValue; /**< Values of the retarget Joints. */
        retargetingElement m_jointRetargeting; /**< Joint retargeting element */

        yarp::os::BufferedPort<yarp::sig::Vector> m_robotOrientationPort; /**< Average orientation of the robot.*/

        bool m_isStancePhase{true}; /**< True if the robot is not walking */

        bool m_isApproachingPhase; /**< True if the robot is in the approaching phase */
        double m_startingApproachingPhaseTime; /**< Initial time of the approaching phase (seconds) */
        double m_approachPhaseDuration; /**< Duration of the approaching phase (seconds) */

        /**
         * Convert a yarp vector containing position + rpy into an iDynTree homogeneous transform
         * @param vector a 6d yarp vector
         * @param transform an iDyntree homogeneous transformation
         */
        void convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                                iDynTree::Transform& transform);

        /**
         * Terminate the approaching phase
         */
        void stopApproachingPhase();

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

        /**
         * Get the CoM height
         * @return height of the CoM
         */
        double comHeight() const;

        /**
         * Get the CoM height velocity
         * @return height velocity of the CoM
         */
        double comHeightVelocity() const;

        /**
         * Get the homogeneous transform of the right hand w.r.t. the head frame head_T_rightHand
         */
        void setRobotBaseOrientation(const iDynTree::Rotation& rotation);


        void setPhase(bool isStancePhase);

        /**
         * Start the approaching phase
         */
        void startApproachingPhase();

        /**
         * Check if the approaching phase is running
         * @return true if the approaching phase is running
         */
        bool isApproachingPhase() const;
    };
};
#endif
