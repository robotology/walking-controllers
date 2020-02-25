/**
 * @file StepAdaptationController.hpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef WALKING_CONTROLLERS_STEP_ADAPTATION_CONTROLLERS_H
#define WALKING_CONTROLLERS_STEP_ADAPTATION_CONTROLLERS_H


// std
#include <memory>

// eigen
#include <Eigen/Sparse>

// iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/ConvexHullHelpers.h>

// yarp
#include <yarp/os/Value.h>

#include <unordered_map>
#include <deque>

// solver
#include <WalkingControllers/StepAdaptationController/QPSolver.hpp>

//interpolation
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>


/**
 * StepAdaptationController class contains the controller instances.
 */
namespace WalkingControllers
{
    class StepAdaptationController
    {
        iDynTree::Vector2 m_zmpPositionNominal; /**< The next desired step position(The zmp position for next single support) .. */
        iDynTree::Vector2 m_dcmOffsetNominal; /**< The next desired dcm offset*/
        double m_sigmaNominal; /**< The exponential function of step duration multplied by the natural frequency of the LIPM.*/

        iDynTree::Vector2 m_zmpPositionWeight; /**< The wight of next step position term in the cost function.*/
        iDynTree::Vector2 m_dcmOffsetWeight;/**< The wight of dcm offset term in the cost function.*/
        double m_sigmaWeight;/**< The wight of step timing term in the cost function.*/

        iDynTree::Vector2 m_dcm_ErrorThreshold; /**< The threshold for activating the push recovery based on DCM error.*/
        iDynTree::Vector2 m_roll_pitch_ErrorThreshold; /**< The threshold for activating the pendulum estimator based on the foot orientation error.*/

        iDynTree::Vector2 m_currentZmpPosition; /**< The current step position(The zmp position of current stance foot). */
        iDynTree::Vector2 m_currentDcmPosition; /**< The current DCM position.*/
        iDynTree::Vector2 m_zmpPositionTollerance_x;/**< The tollerance of step position in x direction.*/
        iDynTree::Vector2 m_zmpPositionTollerance_yLeft;/**< The tollerance of step position in y direction(left).*/
        iDynTree::Vector2 m_zmpPositionTollerance_yRight;/**< The tollerance of step position in y direction(right).*/

        double m_stepTiming; /**< The remanined single support duration+(next double support duration)/2 .*/
        double m_stepDurationTolerance;/**< The tollerance of step timing with respect to the nominal value.*/
        double m_remainingSingleSupportDuration;/**< The remained single support duration.*/
        double m_omega;/**< The natural frequency of LIPM.*/

        double m_currentTime;/**< The  current time.*/
        double m_nextDoubleSupportDuration;/**< The timing of next double support.*/

        int m_inputSize;  /**< Size of the input vector.. */
        int m_numberOfConstraint;  /**< Size of the input vector.. */

        /**
         *The buffered vectors for the interpolation of the foot trajectory
         */
        iDynTree::VectorDynSize m_xPositionsBuffer, m_yPositionsBuffer, m_zPositionsBuffer,m_zzPositionsBuffer, m_yawsBuffer, m_timesBuffer, m_zTimesBuffer,m_zzTimesBuffer;

        iDynTree::ConvexHullProjectionConstraint m_convexHullComputer; /**< iDynTree convex hull helper. */
        std::vector<iDynTree::Polygon> m_feetExtendedPolygon;/**< convex hull of the allowable landing foot position. */
        iDynTree::Transform m_footTransform; /**< transform of the next foot position. */

        std::pair<bool, bool> m_feetStatus; /**< Current status of the feet. Left and Right. True is used
                                            if the foot is in contact. */

        bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

        /**
         * Pointer to the current QPSolver.
         * A new MPC solver is initialized when a new phase occurs.
         */
        std::shared_ptr<QPSolver> m_currentQPSolver;

    public:
        /**
         * Constructor of step adaptation controller.
         */
        StepAdaptationController();

        /**
         * Initialize the method
         * @param config yarp searchable configuration variable.
         * @return true/false in case of success/failure
         */
        bool initialize(const yarp::os::Searchable& config);


        /**
         * Solve the Optimization problem. If the QPSolver is not set It will be initialized.
         * @param isLeft is true if left foot is the swing foot
         * @return true/false in case of success/failure.
         */
        bool solve(bool isLeft);

        /**
         * Reset the controller
         */
        void reset();
        /**
         * Run the step adaptation and set constraints and gradient vector and solve the QP problem and find the adapted foot trajectory.
         * @param maxFootHeight maximum height of the swing foot.
         * @param takeOffTime take off time of the swing foot.
         * @param yawAngleAtImpact yaw angle of the swing foot at landing moment.
         * @param zmpOffset offset between zmp and the stance foot position.
         * @param currentFootTransform Transform of the current swing foot.
         * @param currentFootTwist Current twist of the swing foot.
         * @param adaptatedFootTransform  Adapted transform of the swing foot.
         * @param adaptedFootTwist Adapted twist of the swing foot.
         * @param adaptedFootAcceleration Adapted acceleration of the swing foot.
         * @return true/false in case of success/failure.
         */
        bool getAdaptatedFootTrajectory(double maxFootHeight, double dt, double takeOffTime, double yawAngleAtImpact,
                                        iDynTree::Vector2 zmpOffset, const iDynTree::Transform& currentFootTransform,
                                        const iDynTree::Twist& currentFootTwist, iDynTree::Transform& adaptatedFootTransform,
                                        iDynTree::Twist& adaptedFootTwist, iDynTree::SpatialAcc& adaptedFootAcceleration);

        /**
         * Set the nominal next step position and yaw angle
         * @param nominalZmpPosition Nominal next step position(with a constant offset)
         * @param angle yaw angle of the swing foot at landing moment.
         * @return true/false in case of success/failure.
         */
        void setNominalNextStepPosition(const iDynTree::Vector2& nominalZmpPosition, const double& angle);

        /**
         * Set the varibales related to the timing of a step
         * @param omega Nominal next step position(with a constant offset).
         * @param currentTime current time of walking.
         * @param nextImpactTime Next impact time
         * @param nextDoubleSupportDuration Double support duration of the next step.
         */
        void setTimings(const double & omega, const double & currentTime, const double& nextImpactTime,
                        const double &nextDoubleSupportDuration);

        /**
         * Set the nominal DCM offset.
         * @param nominalDcmOffset Nominal DCM offset.
         * @return true/false in case of success/failure.
         */
        void setNominalDcmOffset(const iDynTree::Vector2& nominalDcmOffset);

        /**
         * Set the current ZMP(or with a constant offset the next step) Position.
         * @param currentZmpPosition Current position of the zmp.
         * @return true/false in case of success/failure.
         */
        void setCurrentZmpPosition(const iDynTree::Vector2& currentZmpPosition);

        /**
         * Set the current Dcm Position.
         * @param currentDcmPosition Current position of the DCM.
         * @return true/false in case of success/failure.
         */
        void setCurrentDcmPosition(const iDynTree::Vector2& currentDcmPosition);

        /**
         * Get the output of the controller.
         * @param controllerOutput is the vector containing the output the controller.
         * @return true/false in case of success/failure.
         */
        bool getControllerOutput(iDynTree::Vector3& controllerOutput);

        /**
         * Get the adapted step timing.
         * @return The adapted step timing.
         */
        double getDesiredImpactTime();

        /**
         * Get the adapted zmp(in another words, the next step position).
         * @return The adapted zmp of the next step.
         */
        iDynTree::Vector2 getDesiredZmp();

        /**
         * Get the roll and pitch error threshold that has been set by the configuration file.
         * @return 2D vector of roll and pitch error threshold that will be used for push detection.
         */
        iDynTree::Vector2 getRollPitchErrorThreshold();

        /**
         * Get the DCM error threshold that has been set by the configuration file.
         * @return 2D vector of the DCM error threshold that will be used for push detection.
         */
        iDynTree::Vector2 getDCMErrorThreshold();
    };
};

#endif
