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

        double m_nominalWidth; /**< Nominal width between two feet. */
        double m_initTime; /**< Init time of the current trajectory. */

        double m_nominalCoMHeight; /**< Nominal CoM height during walking. */
        double m_switchOverSwingRatio;/**< Double support suration devided by single support duration. */

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
         * Reset the planner
         */
        void reset();

        /**
        * Get the desired 2D-DCM position trajectory
        * @param DCMPositionTrajectory desired trajectory of the DCM adjustment.
        * @return true/false in case of success/failure.
        */
        bool getDCMPositionTrajectoryAdjusted(std::vector<iDynTree::Vector2>& DCMPositionTrajectory);

        /**
         * Get the desired 2D-DCM velocity trajectory
         * @param DCMVelocityTrajectory desired trajectory of the DCM adjustment Velocity.
         * @return true/false in case of success/failure.
         */
        bool getDCMVelocityTrajectoryAdjusted(std::vector<iDynTree::Vector2>& DCMVelocityTrajectory);

        /**
        * Get the desired 2D-ZMP position trajectory
        * @param ZMPPositionTrajectory desired trajectory of the ZMP.
        * @return true/false in case of success/failure.
        */
        bool getZMPPositionTrajectory(std::vector<iDynTree::Vector2>& ZMPPositionTrajectory);

        /**
         * Get the desired trajectories
         * @param dcmSubTrajectories desired trajectories.
         * @return true/false in case of success/failure.
         */
        bool getDCMSubTrajectories(std::vector<std::shared_ptr<GeneralSupportTrajectory>>& dcmSubTrajectories);

        /**
         * Get the weight on the left on the right foot is a number that goes from 0 to 1
         * @param weightInLeft weight on the left foot
         * @param weightInRight weight on the right foot
         * @return true/false in case of success/failure.
         */
        bool getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight);

        /**
         * Get the feet acceleration
         * @param lFootAcceleration vector containing the left foot acceleration;
         * @param rFootAcceleration vector containing the right foot acceleration.
         * @return true/false in case of success/failure.
         */
        bool getFeetAcceleration(std::vector<iDynTree::SpatialAcc>& lFootAcceleration, std::vector<iDynTree::SpatialAcc>& rFootAcceleration);

        /**
         * Generate trajectories for a given footprints
         * @param left the left foot footprint;
         * @param right the right foot footprint;
         * @param initTime
         * @param initialState
         * @return true/false in case of success/failure.
         */
        bool generateTrajectoriesFromFootprintsStepAdjustment(std::shared_ptr<FootPrint> left, std::shared_ptr<FootPrint> right, const double &initTime, DCMInitialState initialState);

        /**
         * Get the phases of each foot during walking from unicycle
         * @param leftPhases vector containing all the phases that left foot experience.
         * @param rightPhases vector containing all the phases that right foot experience.
         * @return true/false in case of success/failure.
         */
        bool getStepPhases(std::vector<StepPhase> &leftPhases, std::vector<StepPhase> &rightPhases);

        /**
         * Get the left foot print
         * @param leftFootPrint pointer to the left footprint
         * @return true/false in case of success/failure.
         */
        bool getLeftFootprint(std::shared_ptr<FootPrint>& leftFootPrint);

        /**
         * Get the right foot print
         * @param rightFootPrint pointer to the right footprint
         * @return true/false in case of success/failure.
         */
        bool getRightFootprint(std::shared_ptr<FootPrint>& rightFootPrint);

         /**
          * Get the Nominal CoM height trajectory for omega calculation
          * @param nominalCoMHeight  nominal CoM height
          * @return true/false in case of success/failure.
          */
         bool getNominalCoMHeight(double & nominalCoMHeight);

         /**
          * Get the ratio of double support to single support
          * @param switchOverSwingRatio returning ratio of double support to single support
          * @return true/false in case of success/failure.
          */
         bool getSwitchOverSwingRatio(double &switchOverSwingRatio);

         /**
          * Get the DCM boundary condition at merge point
          * @param DCMBoundryConditionAtMergePoint is DCM boundary condition at merge point
          * @return true/false in case of success/failure.
          */
         bool getDCMBoundaryConditionAtMergePoint(DCMInitialState DCMBoundryConditionAtMergePoint);
    };
};

#endif
