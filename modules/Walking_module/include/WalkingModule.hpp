/**
 * @file WalkingModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_MODULE_HPP
#define WALKING_MODULE_HPP

// std
#include <memory>
#include <deque>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IControlMode2.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RpcClient.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include "TrajectoryGenerator.hpp"
#include "WalkingController.hpp"
#include "WalkingDCMReactiveController.hpp"
#include "WalkingZMPController.hpp"
#include "WalkingInverseKinematics.hpp"
#include "WalkingQPInverseKinematics_osqp.hpp"
#include "WalkingQPInverseKinematics_qpOASES.hpp"
#include "WalkingForwardKinematics.hpp"
#include "StableDCMModel.hpp"
#include "WalkingPIDHandler.hpp"
#include "WalkingLogger.hpp"
#include "TimeProfiler.hpp"

// iCub-ctrl
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/minJerkCtrl.h>

#include "thrifts/WalkingCommands.h"

enum class WalkingFSM {Idle, Configured, Prepared, Walking, OnTheFly, Stance};

/**
 * RFModule of the 2D-DCM dynamics model.
 */
class WalkingModule:
    public yarp::os::RFModule,
    public WalkingCommands
{
    double m_dT; /**< RFModule period. */
    double m_time; /**< Current time. */
    std::string m_robot; /**< Robot name. */

    WalkingFSM m_robotState{WalkingFSM::Idle}; /**< State  of the WalkingFSM. */

    bool m_firstStep; /**< True if this is the first step. */
    bool m_useMPC; /**< True if the MPC controller is used. */
    bool m_useQPIK; /**< True if the QP-IK is used. */
    bool m_useOSQP; /**< True if osqp is used to QP-IK problem. */
    bool m_dumpData; /**< True if data are saved. */
    bool m_useVelocity; /**< True if real velocity control is used. */
    bool m_useVelocityControllerAsIK; /**< True the velocity controller is used as IK. */
    bool m_useLeftHand; /**< Use the left hand inside the  inverse kinematics. */
    bool m_useRightHand; /**< Use the right hand inside the  inverse kinematics. */
    bool m_useVelocityModulation; /**< Use velocity modulation. */

    std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator; /**< Pointer to the trajectory generator object. */
    std::unique_ptr<WalkingController> m_walkingController; /**< Pointer to the walking DCM MPC object. */
    std::unique_ptr<WalkingDCMReactiveController> m_walkingDCMReactiveController; /**< Pointer to the walking DCM reactive controller object. */
    std::unique_ptr<WalkingZMPController> m_walkingZMPController; /**< Pointer to the walking ZMP controller object. */
    std::unique_ptr<WalkingIK> m_IKSolver; /**< Pointer to the inverse kinematics solver. */
    std::unique_ptr<WalkingQPIK_osqp> m_QPIKSolver_osqp; /**< Pointer to the inverse kinematics solver (osqp). */
    std::unique_ptr<WalkingQPIK_qpOASES> m_QPIKSolver_qpOASES; /**< Pointer to the inverse kinematics solver (qpOASES). */
    std::unique_ptr<WalkingFK> m_FKSolver; /**< Pointer to the forward kinematics solver. */
    std::unique_ptr<StableDCMModel> m_stableDCMModel; /**< Pointer to the stable DCM dynamics. */
    std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */
    std::unique_ptr<WalkingLogger> m_walkingLogger; /**< Pointer to the Walking Logger object. */
    std::unique_ptr<TimeProfiler> m_profiler; /**< Time profiler. */

    // related to the onTheFly feature
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_jointsSmoother; /**< Minimum jerk trajectory for the joint during the
                                                                     onTheFly procedure. */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_heightSmoother; /**< Minimum jerk trajectory for the CoM height during
                                                                     the onTheFly procedure. */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_additionalRotationWeightSmoother; /**< Minimum jerk trajectory for the weight
                                                                                       of the additional rotation matrix during
                                                                                       the onTheFly procedure. */
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_desiredJointWeightSmoother; /**< Minimum jerk trajectory for the weight
                                                                                 of the desired joint position during the
                                                                                 onTheFly procedure. */

    double m_onTheFlySmoothingTime; /**< Duration of the on the fly procedure. */
    double m_additionalRotationWeightDesired; /**< Desired additional rotational weight matrix. */
    double m_desiredJointsWeight; /**< Desired joint weight matrix. */
    yarp::sig::Vector m_desiredJointInRadYarp; /**< Desired joint position (regularization task). */

    std::deque<iDynTree::Transform> m_leftTrajectory; /**< Deque containing the trajectory of the left foot. */
    std::deque<iDynTree::Transform> m_rightTrajectory; /**< Deque containing the trajectory of the right foot. */

    std::deque<iDynTree::Twist> m_leftTwistTrajectory; /**< Deque containing the twist trajectory of the left foot. */
    std::deque<iDynTree::Twist> m_rightTwistTrajectory; /**< Deque containing the twist trajectory of the right foot. */

    std::deque<iDynTree::Vector2> m_DCMPositionDesired; /**< Deque containing the desired DCM position. */
    std::deque<iDynTree::Vector2> m_DCMVelocityDesired; /**< Deque containing the desired DCM velocity. */
    std::deque<bool> m_leftInContact; /**< Deque containing the left foot state. */
    std::deque<bool> m_rightInContact; /**< Deque containing the right foot state. */
    std::deque<double> m_comHeightTrajectory; /**< Deque containing the CoM height trajectory. */
    std::deque<double> m_comHeightVelocity; /**< Deque containing the CoM height velocity. */
    std::deque<size_t> m_mergePoints; /**< Deque containing the time position of the merge points. */

    std::deque<bool> m_isLeftFixedFrame; /**< Deque containing when the main frame of the left foot is the fixed frame
                                            In general a main frame of a foot is the fix frame only during the
                                            stance and the switch out phases. */

    yarp::dev::PolyDriver m_robotDevice; /**< Main robot device. */
    std::vector<std::string> m_axesList; /**< Vector containing the name of the controlled joints. */
    int m_actuatedDOFs; /**< Number of the actuated DoFs. */

    iDynTree::ModelLoader m_loader; /**< Model loader class. */

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IEncodersTimed *m_encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect *m_positionDirectInterface{nullptr}; /**< Direct position control interface. */
    yarp::dev::IPositionControl2 *m_positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl2 *m_velocityInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IControlMode2 *m_controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IControlLimits2 *m_limitsInterface{nullptr}; /**< Encorders interface. */
    yarp::os::Bottle m_remoteControlBoards; /**< Contain all the name of the controlled joints. */

    yarp::sig::Vector m_positionFeedbackInDegrees; /**< Vector containing the current joint position [deg]. */
    yarp::sig::Vector m_velocityFeedbackInDegrees; /**< Vector containing the current joint velocity [deg/s]. */

    iDynTree::VectorDynSize m_qDesired; /**< Vector containing the results of the IK algorithm [rad]. */
    // todo
    iDynTree::VectorDynSize m_dqDesired_osqp; /**< Vector containing the results of the QP-IK algorithm [rad/s]. */
    iDynTree::VectorDynSize m_dqDesired_qpOASES; /**< Vector containing the results of the QP-IK algorithm [rad/s]. */
    iDynTree::VectorDynSize m_positionFeedbackInRadians; /**< Vector containing the current joint position [rad]. */
    iDynTree::VectorDynSize m_velocityFeedbackInRadians; /**< Vector containing the current joint velocity [rad/s]. */
    iDynTree::VectorDynSize m_toDegBuffer; /**< Vector containing the desired joint positions that will be sent to the robot [deg]. */

    iDynTree::VectorDynSize m_minJointsVelocity; /**< Vector containing the max negative velocity [rad/s]. */
    iDynTree::VectorDynSize m_maxJointsVelocity; /**< Vector containing the max positive velocity [rad/s]. */

    iDynTree::VectorDynSize m_minJointsPosition; /**< Vector containing the max negative position [rad]. */
    iDynTree::VectorDynSize m_maxJointsPosition; /**< Vector containing the max positive position [rad]. */

    yarp::sig::Vector m_positionFeedbackInDegreesFiltered;
    yarp::sig::Vector m_velocityFeedbackInDegreesFiltered; /**< Vector containing the filtered joint velocity [deg/s]. */
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_positionFilter; /**< Joint position low pass filter .*/
    std::unique_ptr<iCub::ctrl::FirstOrderLowPassFilter> m_velocityFilter; /**< Joint velocity low pass filter .*/
    bool m_useVelocityFilter; /**< True if the joint velocity filter is used. */
    bool m_usePositionFilter; /**< True if the joint position filter is used. */

    iDynTree::Rotation m_inertial_R_worldFrame; /**< Rotation between the inertial and the world frame. */

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

    yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */

    bool m_newTrajectoryRequired; /**< if true a new trajectory will be merged soon. (after m_newTrajectoryMergeCounter - 2 cycles). */
    size_t m_newTrajectoryMergeCounter; /**< The new trajectory will be merged after m_newTrajectoryMergeCounter - 2 cycles. */

    std::mutex m_mutex; /**< Mutex. */

    iDynTree::Vector2 m_desiredPosition;

    // debug
    std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};

    double m_normalForceThreshold;

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_desiredLeftHandSmoother; /**< Minimum jerk
                                                                              trajectory for the left
                                                                              hand. */

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_desiredRightHandSmoother; /**< Minimum jerk
                                                                               trajectory for the
                                                                               right hand. */
    yarp::sig::Vector m_desiredLeftHandPoseYarp;
    iDynTree::Transform m_desiredLeftHandToRootLinkTransform;
    yarp::sig::Vector m_desiredRightHandPoseYarp;
    iDynTree::Transform m_desiredRightHandToRootLinkTransform;
    yarp::os::BufferedPort<yarp::sig::Vector> m_desiredLeftHandPosePort; /**< Desired left hand
                                                                            port. */
    yarp::os::BufferedPort<yarp::sig::Vector> m_desiredRightHandPosePort; /**< Desired right hand
                                                                             port. */

    yarp::os::BufferedPort<yarp::os::Bottle> m_torsoOrientationPort; /**< Port contain the yaw angle
                                                                        of the torso. */

    // velocity modulation
    double m_minForwardVelocity; /**< Minimal forward velocity. */
    double m_maxForwardVelocity; /**< Maximal forward velocity. */

    double m_minStepDurationIni; /**< Min step duration associated to minForwardVelocity. */
    double m_minStepDurationFinal; /**< Min step duration associated to maxForwardVelocity. */

    double m_maxStepDurationIni; /**< Max step duration associated to minForwardVelocity. */
    double m_maxStepDurationFinal; /**< Max step duration associated to maxForwardVelocity. */

    double m_nominalStepDurationIni; /**< Nominal step duration associated to minForwardVelocity. */
    double m_nominalStepDurationFinal; /**< Nominal step duration associated to maxForwardVelocity. */

    /**
     * Configure the Force torque sensors. The FT ports are only opened please use yarpamanger
     * to connect them.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureForceTorqueSensors(const yarp::os::Searchable& config);

    /**
     * Configure the hand retargeting feature.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureHandRetargeting(const yarp::os::Searchable& config);

    /**
     * Configure the Velocity modulation feature.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureVelocityModulation(yarp::os::Searchable& config);

    /**
     * Configure the Robot.
     * @param config is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configureRobot(const yarp::os::Searchable& config);

    /**
     * Get the name of the controlled joints from the resource finder
     * and set its.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool setControlledJoints(const yarp::os::Searchable& rf);

    /**
     * Get the robot model from the resource finder and set it.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool setRobotModel(const yarp::os::Searchable& rf);

    /**
     * Propagate time.
     */
    void propagateTime();

    /**
     * Propagate the reference signal.
     * @return true in case of success and false otherwise.
     */
    bool propagateReferenceSignals();

    /**
     * Get all the feedback signal from the interfaces
     * @return true in case of success and false otherwise.
     */
    bool getFeedbacks(unsigned int maxAttempts = 1);

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

    /**
     * Set the desired position reference. (The position will be sent using PositionControl mode)
     * @param jointPositionsRadians desired final joint position;
     * @param positioningTimeSec minimum jerk trajectory duration.
     * @return true in case of success and false otherwise.
     */
    bool setPositionReferences(const iDynTree::VectorDynSize& jointPositionsRadians,
                               const double& positioningTimeSec);

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
     * Update the FK solver.
     * @return true in case of success and false otherwise.
     */
    bool updateFKSolver();

    /**
     * Set the QP-IK problem.
     * @param solver is the pointer to the solver (osqp or qpOASES)
     * @param desiredCoMPosition desired CoM position;
     * @param desiredCoMVelocity desired CoM velocity;
     * @param desiredNeckOrientation desired neck orientation (rotation matrix);
     * @param output is the output of the solver (i.e. the desired joint velocity)
     * @return true in case of success and false otherwise.
     */
    bool solveQPIK(auto& solver, const iDynTree::Position& desiredCoMPosition,
                   const iDynTree::Vector3& desiredCoMVelocity,
                   const iDynTree::Position& actualCoMPosition,
                   const iDynTree::Rotation& desiredNeckOrientation,
                   iDynTree::VectorDynSize &output);
    /**
     * Evaluate the position of CoM.
     * @param comPosition position of the center of mass;
     * @param comVelocity velocity of the center of mass.
     * @return true in case of success and false otherwise.
     */
    bool evaluateCoM(iDynTree::Position& comPosition, iDynTree::Vector3& comVelocity);

    /**
     * Evaluate the position of 2D-Divergent component of motion.
     * @param dcm 2d-Divergent component of motion.
     * @return true in case of success and false otherwise.
     */
    bool evaluateDCM(iDynTree::Vector2& dcm);

    /**
     * Evaluate the position of Zero momentum point.
     * @param zmp zero momentum point.
     * @return true in case of success and false otherwise.
     */
    bool evaluateZMP(iDynTree::Vector2& zmp);

    /**
     * Generate the first trajectory.
     * This method has to be called before updateTrajectories() method.
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories();

    /**
     * Generate the first trajectory. (onTheFly)
     * @param leftToRightTransform transformation between left and right feet.
     * @return true in case of success and false otherwise.
     */
    bool generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform);

    /**
     * Ask for a new trajectory (The trajectory will be evaluated by a thread).
     * @param initTime is the initial time of the trajectory;
     * @param isLeftSwinging todo wrong name?;
     * @param measuredTransform transformation between the world and the (stance/swing??) foot;
     * @param mergePoint is the instant at which the old and the new trajectory will be merged;
     * @param desiredPosition final desired position of the projection of the CoM.
     * @return true/false in case of success/failure.
     */
    bool askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                            const iDynTree::Transform& measuredTransform,
                            const size_t& mergePoint, const iDynTree::Vector2& desiredPosition);

    /**
     * Update the old trajectory.
     * This method has to be called only if the trajectory generator has finished to evaluate the new trajectory.
     * The old and the new trajectory will be merged at mergePoint.
     * @param mergePoint instant at which the old and the new trajectory will be merged
     * @return true/false in case of success/failure.
     */
    bool updateTrajectories(const size_t& mergePoint);

    /**
     * Update the desired hands trajectories
     */
    void updateDesiredHandsPose();

    /**
     * Linear interpolation.
     * @param x0
     * @param y0
     * @param xf
     * @param yf
     * @param x
     * @return y
     */
    double linearInterpolation(const double& x0, const double& y0,
                               const double& xf, const double& yf,
                               const double& x);
public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * Start the robot motion from a generic two-feet-standing position.
     * @return true in case of success and false otherwise.
     */
    bool onTheFlyStartWalking(const double smoothingTime = 2.0);

    /**
     * This allows you to put the robot in a home position for walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool prepareRobot(bool onTheFly = false);

    /**
     * Start walking.
     * @return true in case of success and false otherwise.
     */
    virtual bool startWalking();

    /**
     * set the desired final position of the CoM.
     * @param x desired x position of the CoM;
     * @param y desired y position of the CoM.
     * @return true in case of success and false otherwise.
     */
    virtual bool setGoal(double x, double y);
};
#endif
