/**
 * @file RobotHelper.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef ROBOT_HELPER_HPP
#define ROBOT_HELPER_HPP

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
#include <yarp/sig/Vector.h>
#include <yarp/os/Timer.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <iCub/ctrl/filters.h>

#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>

#include <WalkingPIDHandler.hpp>
#include <iDynTree/Model/Model.h>
//#include <yarp/dev/MultipleAnalogSensorsClient.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/RpcClient.h>

class RobotHelper
{
    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    std::vector<std::string> m_axesList; /**< Vector containing the name of the controlled joints. */
    unsigned int m_actuatedDOFs; /**< Number of the actuated DoFs. */

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect *m_positionDirectInterface{nullptr}; /**< Direct position control interface. */
    yarp::dev::IPositionControl *m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl *m_velocityInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IControlMode *m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits *m_limitsInterface{nullptr}; /**< Encorders interface. */

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

    bool m_useExternalRobotBase; /**< True if an the base is provided by the extern. */
    bool m_useFloatingBaseEstimator; /**< True if an the base is provided by the base estimator. */
    bool m_usePelvisIMU; /**< True if an the pelvis imu will be used. */
    bool m_useHeadIMU; /**< True if an the head imu will be used. */
    bool m_useFeetIMUSimulation; /**< True if an the Feet imu will be used. */
    bool m_useFeetIMUExperiment; /**< True if an the Feet imu will be used. */



    iDynTree::Transform m_robotBaseTransform; /**< World_T_robot base. */
    iDynTree::Twist m_robotBaseTwist; /**< Robot twist base expressed in mixed representation. */

    iDynTree::Rotation m_headimuOrientation; /**< imu orientation data */
    iDynTree::LinAcceleration m_headimuAcceleration;/**< imu acceleration data */
    iDynTree::AngVelocity m_headimuAngularVelocity;/**< /**< imu angular velocity data */

    iDynTree::Rotation m_leftFootIMUOrientation; /**< left foot imu orientation data */
    iDynTree::LinAcceleration m_leftFootIMUAcceleration;/**< left foot imu acceleration data */
    iDynTree::AngVelocity m_leftFootIMUAngularVelocity;/**< left foot imu angular velocity data */

    iDynTree::Rotation m_rightFootIMUOrientation; /**< right foot imu orientation data */
    iDynTree::LinAcceleration m_rightFootIMUAcceleration;/**< right foot imu acceleration data */
    iDynTree::AngVelocity m_rightFootIMUAngularVelocity;/**< right foot imu angular velocity data */

    iDynTree::Rotation m_pelvisimuOrientation; /**< imu orientation data */
    iDynTree::LinAcceleration m_pelvisimuAcceleration;/**< imu acceleration data */
    iDynTree::AngVelocity m_pelvisimuAngularVelocity;/**< /**< imu angular velocity data */

    iDynTree::Transform m_robotEstimatedBaseTransform; /**< World_T_robot base that come from estimator. */
    iDynTree::Twist m_robotEstimatedBaseTwist; /**< Robot twist base expressed in mixed representation that come from estimator. */

//    iDynTree::Transform m_robotEstimatedBaseTransform; /**< World_T_robot base that come from estimator. */
//    iDynTree::Twist m_robotEstimatedBaseTwist; /**< Robot twist base expressed in mixed representation that come from estimator.

iDynTree::Rotation m_initialHeadIMUOrientation;
iDynTree::Rotation m_initialPelvisIMUOrientation;

    yarp::os::BufferedPort<yarp::sig::Vector> m_robotBasePort; /**< Robot base port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotBaseEstimatorPort; /**< Robot base port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_pelvisIMUPort; /**< Pelvis IMU  port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_headIMUPort; /**< Head IMU port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_leftFootIMUPort; /**< Left Foot IMU port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightFootIMUPort; /**< Right Foot IMU port. */
    yarp::dev::PolyDriver  m_masRemapperFeetIMU;
    yarp::os::Property m_masRemapperProperty;

    double m_heightOffset;


    yarp::dev::IThreeAxisLinearAccelerometers* m_accelerometers{nullptr};
    yarp::dev::IThreeAxisGyroscopes* m_gyros{nullptr};
    yarp::dev::IOrientationSensors* m_imu_orientation_sensors{nullptr};

    int m_controlMode{-1}; /**< Current position control mode */

    /**
     * Get the higher position error among all joints.
     * @param desiredJointPositionsRad desired joint position in radiants;
     * @param worstError is a pair containing the indices of the joint with the
     * worst error and its value.
     * @return true in case of success and false otherwise.
     */
    bool getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                       std::pair<std::string, double>& worstError);

    /**
     * Switch the control mode.
     * @param controlMode is the control mode.
     * @return true in case of success and false otherwise.
     */
    bool switchToControlMode(const int& controlMode);
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

    //bool getFeedbacksRaw(unsigned int maxAttempts = 1);
    bool getFeedbacksRaw(unsigned int maxAttempts = 1, bool useBaseEst = false);

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
    bool resetFilters(const iDynTree::Model modelLoader, const iDynTree::Rotation baseToWorldRotation);

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


    const iDynTree::Transform& getBaseTransform() const;

    const iDynTree::Twist& getBaseTwist() const;

    const iDynTree::Transform& getEstimatedBaseTransform() const;

    const iDynTree::Twist& getEstimatedBaseTwist() const;

    /**
     * Set the height of the offset coming from the base estimation
     * @param offset offset of the height of the base in meters
     */
    void setHeightOffset(const double& offset);

    /**
     * Return true if the base of the robot is provided by an external source
     */
    bool isExternalRobotBaseUsed();

    bool isFloatingBaseEstimatorUsed();
    const iDynTree::LinAcceleration &getPelvisIMUAcceleration() const;
    const iDynTree::Rotation &getPelvisIMUOreintation() const;
    const iDynTree::AngVelocity &getPelvisIMUAngularVelocity() const;
    bool getFirstPelvisIMUData(const iDynTree::Model modelLoader, const iDynTree::Rotation baseToWorldRotation, const int maxAttempts);
    const iDynTree::Rotation &getInitialPelvisIMUOreintation() const;

    const iDynTree::LinAcceleration &getHeadIMUAcceleration() const;
    const iDynTree::Rotation &getHeadIMUOreintation() const;
    const iDynTree::AngVelocity &getHeadIMUAngularVelocity() const;
    bool getFirstHeadIMUData(const iDynTree::Model modelLoader, const iDynTree::Rotation baseToWorldRotation,const iDynTree::Rotation headToBaseRotation, const int maxAttempts);
    const iDynTree::Rotation &getInitialHeadIMUOreintation() const;
    bool isPelvisIMUUsed();
    bool isHeadIMUUsed();
    bool isFeetIMUUsedSimulation();
    bool isFeetIMUUsedExperiment();
    const iDynTree::LinAcceleration &getLeftFootIMUAcceleration() const;
    const iDynTree::AngVelocity &getLeftFootIMUAngularVelocity() const;
    const iDynTree::Rotation &getLeftFootIMUOreintation() const;
    const iDynTree::LinAcceleration &getRightFootIMUAcceleration() const;
    const iDynTree::AngVelocity &getRightFootIMUAngularVelocity() const;
    const iDynTree::Rotation &getRightFootIMUOreintation() const;

};

#endif
