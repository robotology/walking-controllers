/**
 * @file Helper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef WALKING_CONTROLLERS_ROBOT_HELPER_HELPER_H
#define WALKING_CONTROLLERS_ROBOT_HELPER_HELPER_H

// std
#include <memory>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IInteractionMode.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Timer.h>

#include <iCub/ctrl/filters.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

#include <WalkingControllers/RobotInterface/PIDHandler.h>
namespace WalkingControllers
{
    class RobotInterface
    {
        yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
        std::vector<std::string> m_axesList; /**< Vector containing the name of the controlled joints. */

        std::vector<yarp::dev::InteractionModeEnum> m_jointInteractionMode;/**< Joint is in the stiff or compliance mode */
        std::vector<yarp::dev::InteractionModeEnum> m_currentJointInteractionMode;/**< Joint is in the stiff or compliance mode based on the walking architecture phases */

        std::vector<bool> m_isGoodTrackingRequired; /**< Vector containing the the information related to the importance of the joint. */
        unsigned int m_actuatedDOFs; /**< Number of the actuated DoFs. */

        // YARP Interfaces exposed by the remotecontrolboardremapper
        yarp::dev::IEncodersTimed *m_encodersInterface{nullptr}; /**< Encorders interface. */
        yarp::dev::IPositionDirect *m_positionDirectInterface{nullptr}; /**< Direct position control interface. */
        yarp::dev::IPositionControl *m_positionInterface{nullptr}; /**< Position control interface. */
        yarp::dev::IVelocityControl *m_velocityInterface{nullptr}; /**< Position control interface. */
        yarp::dev::IControlMode *m_controlModeInterface{nullptr}; /**< Control mode interface. */
        yarp::dev::IControlLimits *m_limitsInterface{nullptr}; /**< Encorders interface. */
        yarp::dev::IInteractionMode *m_interactionInterface{nullptr}; /**< Stiff/compliant mode interface. */

        std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */

        yarp::os::Bottle m_remoteControlBoards; /**< Contain all the name of the controlled joints. */

        double m_positioningTime;

        yarp::sig::Vector m_positionFeedbackDeg; /**< Current joint position [deg]. */
        yarp::sig::Vector m_velocityFeedbackDeg; /**< Current joint velocity [deg/s]. */
        iDynTree::VectorDynSize m_positionFeedbackRad; /**< Current joint position [rad]. */
        iDynTree::VectorDynSize m_velocityFeedbackRad; /**< Current joint velocity [rad/s]. */

        iDynTree::VectorDynSize m_desiredJointPositionRad; /**< Desired Joint Position [rad]. */
        iDynTree::VectorDynSize m_desiredJointValueDeg; /**< Desired joint position or velocity [deg or deg/s]. */

        iDynTree::VectorDynSize m_jointVelocitiesBounds; /**< Joint Velocity bounds [rad/s]. */
        iDynTree::VectorDynSize m_jointPositionsUpperBounds; /**< Joint Position upper bound [rad]. */
        iDynTree::VectorDynSize m_jointPositionsLowerBounds; /**< Joint Position lower bound [rad]. */
        // yarp::sig::Vector m_positionFeedbackDegFiltered;
        yarp::sig::Vector m_velocityFeedbackDegFiltered; /**< Vector containing the filtered joint velocity [deg/s]. */
        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_positionFilter; /**< Joint position low pass filter .*/
        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_velocityFilter; /**< Joint velocity low pass filter .*/
        bool m_useVelocityFilter; /**< True if the joint velocity filter is used. */

        yarp::os::BufferedPort<yarp::sig::Vector> m_leftWrenchPort; /**< Left foot wrench port. */
        yarp::os::BufferedPort<yarp::sig::Vector> m_rightWrenchPort; /**< Right foot wrench port. */
        yarp::sig::Vector m_leftWrenchInput; /**< YARP vector that contains left foot wrench. */
        yarp::sig::Vector m_rightWrenchInput; /**< YARP vector that contains right foot wrench. */
        yarp::sig::Vector m_leftWrenchInputFiltered; /**< YARP vector that contains left foot filtered wrench. */
        yarp::sig::Vector m_rightWrenchInputFiltered; /**< YARP vector that contains right foot filtered wrench. */
        iDynTree::Wrench m_leftWrench; /**< iDynTree vector that contains left foot wrench. */
        iDynTree::Wrench m_rightWrench; /**< iDynTree vector that contains right foot wrench. */
        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_leftWrenchFilter; /**< Left wrench low pass filter.*/
        std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_rightWrenchFilter; /**< Right wrench low pass filter.*/
        bool m_useWrenchFilter; /**< True if the wrench filter is used. */

        double m_startingPositionControlTime;
        bool m_positionMoveSkipped;

        bool m_useExternalRobotBase; /**< True if an the base is provided by the external software(Gazebo). */
        iDynTree::Transform m_robotBaseTransform; /**< Robot base to world transform */
        iDynTree::Twist m_robotBaseTwist; /**< Robot twist base expressed in mixed representation. */
        yarp::os::BufferedPort<yarp::sig::Vector> m_robotBasePort; /**< Robot base data port. */
        double m_heightOffset;/**< Offset between r_sole frame and ground in Z direction */

        int m_controlMode{-1}; /**< Current position control mode */

        /**
         * Get the higher position error among all joints.
         * @param desiredJointPositionsRad desired joint position in radiants;
         * @param worstError is a pair containing the indices of the joint with the
         * worst error and its value.
         * @return true in case of success and false otherwise.
         */
        bool getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                           std::pair<int, double>& worstError);

        /**
         * Switch the control mode.
         * @param controlMode is the control mode.
         * @return true in case of success and false otherwise.
         */
        bool switchToControlMode(const int& controlMode);

        bool setInteractionMode(yarp::dev::InteractionModeEnum interactionMode);

        bool setInteractionMode(std::vector<yarp::dev::InteractionModeEnum>& interactionModes);
    public:

        /**
         * Configure the Robot.
         * @param config is the reference to a resource finder object.
         * @param name robot name
         * @return true in case of success and false otherwise.
         */
        bool configureRobot(const yarp::os::Searchable& rf);

        /**
         * Configure the Force torque sensors. The FT ports are only opened please use yarpamanger
         * to connect them.
         * @param config is the reference to a resource finder object.
         * @return true in case of success and false otherwise.
         */
        bool configureForceTorqueSensors(const yarp::os::Searchable& config);

        bool configurePIDHandler(const yarp::os::Bottle& config);

        /**
         * Get all the feedback signal from the interfaces
         * @return true in case of success and false otherwise.
         */
        bool getFeedbacks(unsigned int maxAttempts = 1);

        bool getFeedbacksRaw(unsigned int maxAttempts = 1);

        /**
         * Set the desired position reference. (The position will be sent using PositionControl mode)
         * @param jointPositionsRadians desired final joint position;
         * @param positioningTimeSec minimum jerk trajectory duration.
         * @return true in case of success and false otherwise.
         */
        bool setPositionReferences(const iDynTree::VectorDynSize& jointPositionsRadians,
                                   const double& positioningTimeSec);

        bool checkMotionDone(bool& motionDone);
        /**
         * Set the desired position reference.
         * (The position will be sent using DirectPositionControl mode)
         * @param desiredPositionsRad desired final joint position;
         * @return true in case of success and false otherwise.
         */
        bool setDirectPositionReferences(const iDynTree::VectorDynSize&  desiredPositionsRad);

        /**
         * Set the desired velocity reference.
         * (The position will be sent using DirectPositionControl mode)
         * @param desiredVelocityRad desired joints velocity;
         * @return true in case of success and false otherwise.
         */
        bool setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad);

        /**
         * Reset filters.
         * @return true in case of success and false otherwise.
         */
        bool resetFilters();

        /**
         * Close the polydrives.
         * @return true in case of success and false otherwise.
         */
        bool close();

        /**
         * Get the joint positions
         * @return the joint positions in radiants
         */
        const iDynTree::VectorDynSize& getJointPosition() const;

        /**
         * Get the joint velocities
         * @return the joint velocities in radiants per second
         */
        const iDynTree::VectorDynSize& getJointVelocity() const;

        /**
         * Get the joint upper limit
         * @return the joint upper bound in radiants
         */
        const iDynTree::VectorDynSize& getPositionUpperLimits() const;

        /**
         * Get the joint lower limit
         * @return the joint lower bound in radiants
         */
        const iDynTree::VectorDynSize& getPositionLowerLimits() const;

        /**
         * Get the joint velocity bounds
         * @return the joint velocity bound in radiants per second
         */
        const iDynTree::VectorDynSize& getVelocityLimits() const;

        const iDynTree::Wrench& getLeftWrench() const;
        const iDynTree::Wrench& getRightWrench() const;

        const std::vector<std::string>& getAxesList() const;

        int getActuatedDoFs();

        WalkingPIDHandler& getPIDHandler();

        /**
         * Set the intraction mode stored in the configuration
         * @return true in case of success and false otherwise.
         */
        bool loadCustomInteractionMode();

        /**
         * Get the base Transform from external software.
         * @return the base transform
         */
        const iDynTree::Transform& getBaseTransform() const;

        /**
         * Get the base Twist from external software.
         * @return the base transform
         */
        const iDynTree::Twist& getBaseTwist() const;

        /**
         * Set the height of the offset coming from the base estimation
         * @param offset of the height of the base in meters
         */
        void setHeightOffset(const double& offset);

        /**
         * Return true if the base of the robot is provided by an external source
         */
        bool isExternalRobotBaseUsed();

    };
};
#endif
