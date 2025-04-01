// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_MODULE_HPP
#define WALKING_MODULE_HPP

// std
#include <memory>
#include <deque>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RpcClient.h>

// BipedalLocomotion
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/RK4.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>
#include <BipedalLocomotion/Contacts/GlobalCoPEvaluator.h>
#include <BipedalLocomotion/System/TimeProfiler.h>

// iDynTree
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Position.h>
#include <iDynTree/Rotation.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/VectorDynSize.h>

// WalkingControllers library
#include <WalkingControllers/RobotInterface/Helper.h>
#include <WalkingControllers/RobotInterface/PIDHandler.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>
#include <WalkingControllers/TrajectoryPlanner/StableDCMModel.h>
#include <WalkingControllers/TrajectoryPlanner/FreeSpaceEllipseManager.h>

#include <WalkingControllers/SimplifiedModelControllers/DCMModelPredictiveController.h>
#include <WalkingControllers/SimplifiedModelControllers/DCMReactiveController.h>
#include <WalkingControllers/SimplifiedModelControllers/ZMPController.h>

#include <WalkingControllers/WholeBodyControllers/InverseKinematics.h>
#include <WalkingControllers/WholeBodyControllers/BLFTSID.h>
#include <WalkingControllers/WholeBodyControllers/BLFIK.h>

#include <WalkingControllers/JointControllers/AdmittanceController.h>
#include <WalkingControllers/Integrators/jointIntegrators.h>

#include <WalkingControllers/KinDynWrapper/Wrapper.h>

#include <WalkingControllers/RetargetingHelper/Helper.h>

#include <WalkingControllers/YarpUtilities/TransformHelper.h>

// iCub-ctrl
#include <iCub/ctrl/filters.h>

#include <thrifts/WalkingCommands.h>

namespace WalkingControllers
{

/**
 * RFModule of the Walking controller
 */
    class WalkingModule: public yarp::os::RFModule, public WalkingCommands
    {
        enum class WalkingFSM {Idle, Configured, Preparing, Prepared, Walking, Paused, Stopped};
        WalkingFSM m_robotState{WalkingFSM::Idle}; /**< State  of the WalkingFSM. */

        double m_dT; /**< RFModule period. */
        double m_time; /**< Current time. */
        // double m_lastSetGoalTime; /**< Time of the last set goal. */
        // double m_maxTimeToWaitForGoal; /**< Maximum time to wait for a goal. */
        // std::string m_robot; /**< Robot name. */

        bool m_useMPC; /**< True if the MPC controller is used. */
        bool m_useQPIK; /**< True if the QP-IK is used. */
        bool m_useTSIDadmittance; /**< True if the TSID admittance controller is used. */
        bool m_dumpData; /**< True if data are saved. */
        bool m_firstRun; /**< True if it is the first run. */
        bool m_skipDCMController; /**< True if the desired ZMP should be used instead of the DCM controller. */
        bool m_removeZMPOffset{false}; /**< If true the offset between the ZMP and CoM is removed */

        double m_maxInitialCoMVelocity; /**< Bound on the initial CoM velocity to check if the robot is going to jump at startup. */
        iDynTree::Position m_zmpOffset; /** < Offset reading the zmp at the beginning*/
        iDynTree::Position m_zmpOffsetLocal; /** < Offset in the local frame*/

        std::unique_ptr<RobotInterface> m_robotControlHelper; /**< Robot control helper. */
        std::unique_ptr<TrajectoryGenerator> m_trajectoryGenerator; /**< Pointer to the trajectory generator object. */
        std::unique_ptr<FreeSpaceEllipseManager> m_freeSpaceEllipseManager; /**< Pointer to the free space ellipse manager. */
        std::unique_ptr<WalkingController> m_walkingController; /**< Pointer to the walking DCM MPC object. */
        std::unique_ptr<WalkingDCMReactiveController> m_walkingDCMReactiveController; /**< Pointer to the walking DCM reactive controller object. */
        std::unique_ptr<WalkingZMPController> m_walkingZMPController; /**< Pointer to the walking ZMP controller object. */
        std::unique_ptr<WalkingIK> m_IKSolver; /**< Pointer to the inverse kinematics solver. */
        std::unique_ptr<BLFIK> m_BLFIKSolver; /**< Pointer to the integration based ik. */
        std::unique_ptr<WalkingFK> m_FKSolver; /**< Pointer to the forward kinematics solver. */
        std::unique_ptr<BLFTSID> m_BLFTSIDSolver; /**< Pointer to the task space inverse dynamics. */
        std::unique_ptr<AdmittanceController> m_jointAdmittanceController; /**< Pointer to the joint Admittance Controller. */
        std::unique_ptr<JointAccelerationIntegrator> m_jointAccelerationIntegrator; /**< Pointer to the joint acceleration integrator. */
        std::unique_ptr<StableDCMModel> m_stableDCMModel; /**< Pointer to the stable DCM dynamics. */
        std::unique_ptr<WalkingPIDHandler> m_PIDHandler; /**< Pointer to the PID handler object. */
        std::unique_ptr<RetargetingClient> m_retargetingClient; /**< Pointer to the stable DCM dynamics. */
        std::unique_ptr<BipedalLocomotion::System::TimeProfiler> m_profiler; /**< Time profiler. */
        std::unique_ptr<YarpUtilities::TransformHelper> m_transformHelper; /**< Transform server/client helper. */
        BipedalLocomotion::Contacts::GlobalCoPEvaluator m_globalCoPEvaluator;

        struct JointsKinematics {
            std::shared_ptr<
                BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>
                dynamics;
            std::shared_ptr<BipedalLocomotion::ContinuousDynamicalSystem::RK4<
                BipedalLocomotion::ContinuousDynamicalSystem::
                    LinearTimeInvariantSystem>>
                integrator;
          };

          JointsKinematics m_jointsAccelerationIntegrator; /**< Double integrator of joint accelerations */

        std::vector<std::pair<iDynTree::FrameIndex, std::string>> m_framesToStream; /**< Frames to send to the transform server. */

        double m_additionalRotationWeightDesired; /**< Desired additional rotational weight matrix. */
        double m_desiredJointsWeight; /**< Desired joint weight matrix. */
        yarp::sig::Vector m_desiredJointInRadYarp; /**< Desired joint position (regularization task). */

        std::deque<iDynTree::Transform> m_leftTrajectory; /**< Deque containing the trajectory of the left foot. */
        std::deque<iDynTree::Transform> m_rightTrajectory; /**< Deque containing the trajectory of the right foot. */

        std::deque<iDynTree::Twist> m_leftTwistTrajectory; /**< Deque containing the twist trajectory of the left foot. */
        std::deque<iDynTree::Twist> m_rightTwistTrajectory; /**< Deque containing the twist trajectory of the right foot. */

        Eigen::Vector3d m_leftAngularVelocityCorrection;
        Eigen::Vector3d m_rightAngularVelocityCorrection;

        std::deque<iDynTree::SpatialAcc> m_leftAccelerationTrajectory; /**< Deque containing the spatial acceleration trajectory of the left foot. */
        std::deque<iDynTree::SpatialAcc> m_rightAccelerationTrajectory; /**< Deque containing the spatial acceleration trajectory of the left foot. */

        std::deque<iDynTree::Vector2> m_DCMPositionDesired; /**< Deque containing the desired DCM position. */
        std::deque<iDynTree::Vector2> m_DCMVelocityDesired; /**< Deque containing the desired DCM velocity. */
        std::deque<bool> m_leftInContact; /**< Deque containing the left foot state. */
        std::deque<bool> m_rightInContact; /**< Deque containing the right foot state. */
        std::deque<double> m_comHeightTrajectory; /**< Deque containing the CoM height trajectory. */
        std::deque<double> m_comHeightVelocity; /**< Deque containing the CoM height velocity. */
        std::deque<size_t> m_mergePoints; /**< Deque containing the time position of the merge points. */
        std::deque<bool> m_isStancePhase; /**< if true the robot is not walking */

        std::deque<bool> m_isLeftFixedFrame; /**< Deque containing when the main frame of the left foot is the fixed frame
                                                In general a main frame of a foot is the fix frame only during the
                                                stance and the switch out phases. */

        std::deque<iDynTree::Vector2> m_desiredZMP; /**< Deque containing the desired ZMP position. */

        std::deque<double> m_weightInLeftDesired; /**< Deque containing the desired weight percentage on the left foot */
        std::deque<double> m_weightInRightDesired; /**< Deque containing the desired weight percentage on the right foot */

        iDynTree::ModelLoader m_loader; /**< Model loader class. */

        iDynTree::VectorDynSize m_qDesiredIK; /**< Vector containing the results of the IK algorithm [rad]. */
        iDynTree::VectorDynSize m_dqDesiredIK; /**< Vector containing the results of the IK algorithm [rad]. */

        iDynTree::VectorDynSize m_qDesiredTSID; /**< Vector containing the desired joint position [rad]. */
        iDynTree::VectorDynSize m_dqDesiredTSID; /**< Vector containing the desired joint velocity [rad/s]. */
        iDynTree::VectorDynSize m_ddqDesiredTSID; /**< Vector containing the desired joint acceleration [rad/s^2]. */
        iDynTree::VectorDynSize m_desiredJointTorquesTSID; /**< Vector containing the desired joint torques for the low-level [Nm]. */
        iDynTree::VectorDynSize m_desiredJointTorquesAdmittance;   /**< Vector containing the desired joint torques for the low-level,
                                                                        computed by the joint admittance controller [Nm]. */

        iDynTree::Position m_CoMPositionTSID; /**< CoM position computed by the TSID controller. */
        iDynTree::Vector3 m_CoMVelocityTSID; /**< CoM velocity computed by the TSID controller. */
        iDynTree::Transform m_leftFootPoseTSID; /**< Left foot pose computed by the TSID controller. */
        iDynTree::Transform m_rightFootPoseTSID; /**< Right foot pose computed by the TSID controller. */
        iDynTree::Twist m_leftFootTwistTSID; /**< Left foot twist computed by the TSID controller. */
        iDynTree::Twist m_rightFootTwistTSID; /**< Right foot twist computed by the TSID controller. */

        iDynTree::Rotation m_inertial_R_worldFrame; /**< Rotation between the inertial and the world frame. */

        yarp::os::Port m_rpcPort; /**< Remote Procedure Call port. */
        yarp::os::BufferedPort<yarp::sig::Vector> m_desiredUnyciclePositionPort; /**< Desired robot position port. */

        bool m_newTrajectoryRequired; /**< if true a new trajectory will be merged soon. (after m_newTrajectoryMergeCounter - 2 cycles). */
        size_t m_newTrajectoryMergeCounter; /**< The new trajectory will be merged after m_newTrajectoryMergeCounter - 2 cycles. */

        bool m_useRootLinkForHeight;
        double m_comHeightOffset{0};

        std::mutex m_mutex; /**< Mutex. */

        iDynTree::VectorDynSize m_plannerInput, m_goalScaling;

        size_t m_plannerAdvanceTimeSteps; /** How many steps in advance the planner should be called. */

        size_t m_feedbackAttempts;
        double m_feedbackAttemptDelay;

        // debug
        std::unique_ptr<iCub::ctrl::Integrator> m_velocityIntegral{nullptr};

        BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorsCollectionServer; /**< Logger server. */

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
         * Advance the reference signal.
         * @return true in case of success and false otherwise.
         */
        bool advanceReferenceSignals();

        /**
         * Update the FK solver.
         * @return true in case of success and false otherwise.
         */
        bool updateFKSolver();

        /**
         * Set the QP-IK problem.
         * @param desiredCoMPosition desired CoM position;
         * @param desiredCoMVelocity desired CoM velocity;
         * @param desiredNeckOrientation desired neck orientation (rotation matrix);
         * @param output is the output of the solver (i.e. the desired joint velocity)
         * @return true in case of success and false otherwise.
         */
        bool solveBLFIK(const iDynTree::Position& desiredCoMPosition,
                        const iDynTree::Vector3& desiredCoMVelocity,
                        const iDynTree::Rotation& desiredNeckOrientation,
                        iDynTree::VectorDynSize &output);
        /**
         * Set and solve the TSID problem.
         * @param desiredCoMPosition desired CoM position;
         * @param desiredCoMVelocity desired CoM velocity;
         * @param desiredTorsoRotation desired torso rotation (rotation matrix);
         * @return true in case of success and false otherwise.
         */
        bool solveBLFTSID(const iDynTree::Position& desiredCoMPosition,
                          const iDynTree::Vector3& desiredCoMVelocity,
                          const iDynTree::Rotation& desiredTorsoRotation);

        /**
         * Get the desired joint acceleration and torque computed by the TSID controller.
         * @param jointDesiredAcceleration is the desired joint acceleration;
         * @param jointDesiredTorque is the desired joint torque.
         */
        void getBLFTSIDOutput(iDynTree::VectorDynSize &jointDesiredAcceleration, 
                              iDynTree::VectorDynSize &jointDesiredTorque);
        
        /**
        * Store the trajectories computed by the BLFTSID controller.
        * @return true in case of success and false otherwise.
        */
        bool storeBLFTSIDTrajectories();

        /**
        * Set the Admittance controller references, advance it, and return the output.
        * @param jointTorqueFeedforward joint torque feedforward term.
        * @param jointDesiredPosition joint desired position.
        * @param jointDesiredVelocity joint desired velocity.
        * @param jointMeasuredPosition joint measured position.
        * @param jointMeasuredVelocity joint measured velocity.
        * @param jointDesiredTorque joint desired torque, computed by the Admittance controller.
        * @return true if successful.
        */
        bool advanceJointAdmittanceController(const iDynTree::VectorDynSize & jointTorqueFeedforward,
                                              const iDynTree::VectorDynSize & jointDesiredPosition,
                                              const iDynTree::VectorDynSize & jointDesiredVelocity,
                                              iDynTree::VectorDynSize & jointDesiredTorque);

        /**
         * Compute Global CoP.
         * @param globalCoP is the global CoP.
         * @return true in case of success and false otherwise.
         */
        bool computeGlobalCoP(Eigen::Ref<Eigen::Vector2d> globalCoP);

        bool computeLocalCoPCorrection(Eigen::Ref<Eigen::Vector3d> leftCorrection, Eigen::Ref<Eigen::Vector3d> rightCorrection);

        /**
         * Given the two planned feet, it computes the average yaw rotation
         * @return the average Yaw rotation
         */
        iDynTree::Rotation computeAverageYawRotationFromPlannedFeet() const;

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
         * @param plannerDesiredInput The desired input to the planner.
         * @return true/false in case of success/failure.
         */
        bool askNewTrajectories(const double& initTime, const bool& isLeftSwinging,
                                const iDynTree::Transform& measuredTransform,
                                const size_t& mergePoint, const iDynTree::VectorDynSize &plannerDesiredInput);

        /**
         * Update the old trajectory.
         * This method has to be called only if the trajectory generator has finished to evaluate the new trajectory.
         * The old and the new trajectory will be merged at mergePoint.
         * @param mergePoint instant at which the old and the new trajectory will be merged
         * @return true/false in case of success/failure.
         */
        bool updateTrajectories(const size_t& mergePoint);

        /**
         * Set the input of the planner. The size of the input is different according to the
         * controller type of the planner.
         * If using the personFollowing controller, the input is a desired position is expressed using a
         * reference frame attached to the robot. The X axis points forward while the
         * Y axis points on the left.
         * If the controller is the direct one, the input sets the desired control inputs for the unicycle,
         * namely the forward, angular and lateral velocity.
         * @param plannerInput The input to the planner
         * @return true/false in case of success/failure.
         */
        bool setPlannerInput(const yarp::sig::Vector &plannerInput);

        /**
         * Reset the entire controller architecture
         */
        void reset();

        /**
         * @brief Apply the scaling on the input from goal port.
         * @param plannerInput The raw data read from the goal port.
         */
        void applyGoalScaling(yarp::sig::Vector &plannerInput);

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
         * This allows you to put the robot in a home position for walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool prepareRobot(bool onTheFly = false) override;

        /**
         * Start walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool startWalking() override;

        /**
         * Set the desired goal position to the planner.
         * @return true/false in case of success/failure;
         */
        virtual bool setGoal(const yarp::sig::Vector& plannerInput) override;

        /**
         * Pause walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool pauseWalking() override;

        /**
         * Stop walking.
         * @return true in case of success and false otherwise.
         */
        virtual bool stopWalking() override;
    };
};
#endif
