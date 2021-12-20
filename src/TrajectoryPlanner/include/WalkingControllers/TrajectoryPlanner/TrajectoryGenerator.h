/**
 * @file TrajectoryGenerator.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */


#ifndef WALKING_CONTROLLERS_TRAJECTORY_PLANNER_TRAJECTORY_GENERATOR_H
#define WALKING_CONTROLLERS_TRAJECTORY_PLANNER_TRAJECTORY_GENERATOR_H

// std
#include <thread>
#include <condition_variable>
#include <memory>

// YARP
#include <yarp/os/Searchable.h>

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>

#include <UnicycleGenerator.h>
#include <FreeSpaceEllipse.h>

namespace WalkingControllers
{

/**
 * Enumerator useful to track the state of the trajectory generator
 */
    enum class GeneratorState {NotConfigured, Configured, FirstStep, Called, Returned, Closing};

/**
 * TrajectoryGenerator class is used to handle the UnicycleTrajectoryGenerator library.
 */
    class TrajectoryGenerator
    {
        UnicycleGenerator m_trajectoryGenerator; /**< UnicycleTrajectoryGenerator object. */
        std::shared_ptr<DCMTrajectoryGenerator> m_dcmGenerator;
        std::shared_ptr<CoMHeightTrajectoryGenerator> m_heightGenerator;
        std::shared_ptr<FeetGenerator> m_feetGenerator;
        bool m_useMinimumJerk;

        bool m_swingLeft; /**< True if the first swing foot is the left. */

        double m_dT; /**< Sampling time of the planner. */
        double m_plannerHorizon; /**< Horizon of the planner. */
        std::size_t m_stancePhaseDelay; /**< Delay in ticks of the beginning of the stance phase. */

        double m_leftYawDeltaInRad; /**< Offset for the left foot rotation around the z axis. */
        double m_rightYawDeltaInRad; /**< Offset for the right foot rotation around the z axis. */
        double m_nominalWidth; /**< Nominal width between two feet. */
        double m_initTime; /**< Init time of the current trajectory. */

        iDynTree::Vector2 m_referencePointDistance; /**< Vector between the center of the unicycle and the point that has to be reach the goal. */

        GeneratorState m_generatorState{GeneratorState::NotConfigured}; /**< Useful to track the generator state. */

        std::thread m_generatorThread; /**< Main trajectory thread. */
        std::condition_variable m_conditionVariable; /**< Synchronizer. */

        bool m_correctLeft; /**< The left foot has to be corrected. */
        iDynTree::Transform m_measuredTransformLeft; /**< Measured transformation between the left foot and the world frame. (w_H_lf) */
        iDynTree::Transform m_measuredTransformRight; /**< Measured transformation between the right foot and the world frame. (w_H_rf) */

        iDynTree::Vector2 m_desiredPoint; /**< Desired final position of the x-y projection of the CoM. */

        iDynTree::Vector2 m_DCMBoundaryConditionAtMergePointPosition; /**< DCM position at the merge point. */
        iDynTree::Vector2 m_DCMBoundaryConditionAtMergePointVelocity; /**< DCM velocity at the merge point. */

        FreeSpaceEllipse m_freeSpaceEllipse; /**< The free space ellipse object. */
        bool m_newFreeSpaceEllipse; /**< Check if the free space ellipse has been updated. */

        std::mutex m_mutex; /**< Mutex. */

        /**
         * Main thread method.
         */
        void computeThread();

    public:

        /**
         * Deconstructor.
         */
        ~TrajectoryGenerator();

        /**
         * Initialize the trajectory generator
         * @param config yarp searchable object.
         * @return true/false in case of success/failure.
         */
        bool initialize(const yarp::os::Searchable& config);

        /**
         * Configure the planner.
         * @param config yarp searchable object.
         * @return true/false in case of success/failure.
         */
        bool configurePlanner(const yarp::os::Searchable& config);

        /**
         * Generate the first trajectory.
         * This method has to be called before updateTrajectories() method
         * @param initialPosition Intitial position of the base that will be recicved form gazebo base data
         * @return true/false in case of success/failure.
         */
        bool generateFirstTrajectories(const iDynTree::Position& initialBasePosition = iDynTree::Position::Zero());

        /**
         * Generate the first trajectory.
         * This method has to be called before only by the ontTheFly method.
         * @param leftToRightTransform transformation between from the left foot to the right foot;
         * @return true/false in case of success/failure.
         */
        bool generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform);
        // const iDynTree::Position &initialCOMPosition);

        /**
         * Update the trajectory.
         * The old trajectory will be deleted and a new one is evaluated. The boundary condition of the new trajectory is given by
         * the position and the velocity of the DCM at the merge point.
         * This method allows you to take into account the real position one foot at the beginning of the trajectory.
         * @param initTime is the initial time of the trajectory;
         * @param DCMBoundaryConditionAtMergePointPosition is the position of the DCM at the merge point;
         * @param DCMBoundaryConditionAtMergePointVelocity is the velocity of the DCM at the merge point;
         * @param correctLeft todo;
         * @param measured Measured transformation between the stance foot and the world frame. (w_H_{stancefoot});
         * @param desiredPosition final desired position of the projection of the CoM.
         * @return true/false in case of success/failure.
         */
        bool updateTrajectories(double initTime, const iDynTree::Vector2& DCMBoundaryConditionAtMergePointPosition,
                                const iDynTree::Vector2& DCMBoundaryConditionAtMergePointVelocity, bool correctLeft,
                                const iDynTree::Transform& measured, const iDynTree::Vector2& desiredPosition);

        /**
         * @brief Set the free space ellipse in which the robot can walk.
         * @note The ellipse is supposed to be defined in robot frame.
         * @param imageMatrix The image matrix mapping a unit circle to the ellipse
         * @param centerOffset The position of the center of the ellipse
         * @return true in case of success, false otherwise
         */
        bool setFreeSpaceEllipse(const iDynTree::MatrixFixSize<2,2>& imageMatrix, const iDynTree::VectorFixSize<2>& centerOffset);

        /**
         * Return if the trajectory was computed
         * @return true if the trajectory has been computed false otherwise.
         */
        bool isTrajectoryComputed();

        /**
         * Configure the planner in order to add or not the terminal step
         * @param terminalStep if it true the terminal step will be added
         */
        void addTerminalStep(bool terminalStep);

        /**
         * Return if a new trajectory is asked.
         * @return true if the trajectory has already asked.
         */
        bool isTrajectoryAsked();

        /**
         * Get the desired 2D-DCM position trajectory
         * @param DCMPositionTrajectory desired trajectory of the DCM.
         * @return true/false in case of success/failure.
         */
        bool getDCMPositionTrajectory(std::vector<iDynTree::Vector2>& DCMPositionTrajectory);

        /**
         * Get the desired 2D-DCM velocity trajectory
         * @param DCMVelocityTrajectory desired trajectory of the DCM.
         * @return true/false in case of success/failure.
         */
        bool getDCMVelocityTrajectory(std::vector<iDynTree::Vector2>& DCMVelocityTrajectory);

        /**
         * Get the feet trajectory
         * @param lFootTrajectory vector containing the left foot trajectory;
         * @param rFootTrajectory vector containing the right foot trajectory.
         * @return true/false in case of success/failure.
         */
        bool getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory,
                                 std::vector<iDynTree::Transform>& rFootTrajectory);


        /**
         * Get the feet twist
         * @param lFootTwist vector containing the left foot twists;
         * @param rFootTwist vector containing the right foot twists.
         * @return true/false in case of success/failure.
         */
        bool getFeetTwist(std::vector<iDynTree::Twist>& lFootTwist,
                          std::vector<iDynTree::Twist>& rFootTwist);
        /**
         * Get the when the main frame of the left foot is the fix frame.
         * @param isLeftFixedFrame vector containing when the main frame of
         * the left foot is the fix frame.
         * @return true/false in case of success/failure.
         */
        bool getWhenUseLeftAsFixed(std::vector<bool>& isLeftFixedFrame);

        /**
         * Get the feet phases
         * @param lFootContacts vector containing the state of the left foot (true = in contact);
         * @param rFootContacts vector containing the state of the right foot (true = in contact).
         * @return true/false in case of success/failure.
         */
        bool getFeetStandingPeriods(std::vector<bool>& lFootContacts,
                                    std::vector<bool>& rFootContacts);

        /**
         * Get the CoM height trajectory
         * @param CoMHeightTrajectory vector containing trajectory of the COM on the z axis.
         * @return true/false in case of success/failure.
         */
        bool getCoMHeightTrajectory(std::vector<double>& CoMHeightTrajectory);

        /**
         * Get the CoM height velocity
         * @param CoMHeightVelocity vector containing the velocity of the COM on the z axis.
         * @return true/false in case of success/failure.
         */
        bool getCoMHeightVelocity(std::vector<double>& CoMHeightVelocity);

        /**
         * Get the merge points along the trajectory
         * @param mergePoints vector containing all the merge points of the trajectory.
         * @return true/false in case of success/failure.
         */
        bool getMergePoints(std::vector<size_t>& mergePoints);

        /**
         * Get the weight percentage for the left and right foot
         * @param weightInLeft vector containing the weight on the left foot (0 in case in case of
         * stance foot during SS, 1 in case of swing foot)
         * @param weightInRight vector containing the weight on the right foot (0 in case in case of
         * stance foot during SS, 1 in case of swing foot)
         * @return true/false in case of success/failure.
         */
        bool getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight);

        /**
         * Get the desired current state of the robot. If the robot is not walking (i.e. the DCM
         * velocity is almost zero), it is considered in "stance" phase.
         * @param isStancePhase vector containing if the robot is in the stance phase during the
         * entire horizon.
         * @return true/false in case of success/failure.
         */
        bool getIsStancePhase(std::vector<bool>& isStancePhase);

        /**
         * Get the desired ZMP trajectory
         * @param desiredZMP vector containing the desired ZMP on the xy plane
         * @return true/false in case of success/failure.
         */
        bool getDesiredZMPPosition(std::vector<iDynTree::Vector2>& desiredZMP);

        /**
         * Reset the planner
         */
        void reset();
    };
};

#endif
