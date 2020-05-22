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

// iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>

// yarp
#include <yarp/os/Value.h>

#include <unordered_map>

#include <qpOASES.hpp>
#include <WalkingControllers/iDynTreeUtilities/Helper.h>
#include <WalkingControllers/StepAdaptationController/DCMSimpleEstimator.hpp>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>

struct FootTrajectoryGenerationInput
{
    double maxFootHeight;
    double discretizationTime ;
    double takeOffTime;
    double yawAngleAtImpact;
    iDynTree::Vector2 zmpToCenterOfFootPosition;
    iDynTree::Transform currentFootTransform;
    iDynTree::Twist currentFootTwist;
};

struct StepAdapterInput
{
    double time;
    double timeOffset;
    double dT;
    double omega;
    std::deque<bool> leftInContact;
    std::deque<bool> rightInContact;
    std::vector<std::shared_ptr<GeneralSupportTrajectory>> dcmSubTrajectories;
    std::shared_ptr<FootPrint> leftFootprints;
    std::shared_ptr<FootPrint> rightFootprints;
    iDynTree::Vector2 dcmPositionSmoothed;
    StepList rightStepList;
    StepList leftStepList;
};

struct StepAdapterOutput
{
    iDynTree::Transform adaptedFootLeftTransform;
    iDynTree::Transform adaptedFootRightTransform;
    iDynTree::Twist adaptedFootRightTwist;
    iDynTree::Twist adaptedFootLeftTwist;
    iDynTree::SpatialAcc adaptedFootLeftAcceleration;
    iDynTree::SpatialAcc adaptedFootRightAcceleration;
    iDynTree::Transform currentFootLeftTransform;
    iDynTree::Transform currentFootRightTransform;
    iDynTree::Twist currentFootLeftTwist;
    iDynTree::Twist currentFootRightTwist;
    iDynTree::SpatialAcc currentFootLeftAcceleration;
    iDynTree::SpatialAcc currentFootRightAcceleration;
    std::deque<iDynTree::Vector2> dcmPositionAdjusted;
    std::deque<iDynTree::Vector2> dcmVelocityAdjusted;
    iDynTree::Vector2 zmpNominal;
    iDynTree::Vector2 zmpAdjusted;
    double isPushActive;
    double isRollActive;
    double isPitchActive;
    int pushRecoveryActiveIndex;
    double kDCMSmoother;
    double kFootSmoother;
    int indexSmoother;
    int indexFootSmoother;
    int timeIndexAfterPushDetection;
    int FootTimeIndexAfterPushDetection;
    int indexPush;
    double impactTimeNominal;
    double impactTimeAdjusted;
};
/**
 * StepAdaptationController class contains the controller instances.
 */
namespace WalkingControllers
{

/**
 * Enumerator for understanding which foot is in  the swing phase.
 */
    enum class SwingFoot{Left,Right};

    class StepAdaptationController
    {
        /**
         * Pointer to the optimization solver
         */
        std::unique_ptr<qpOASES::SQProblem> m_QPSolver_qpOASES{nullptr}; /**< qpOASES Optimization solver. */

        iDynSparseMatrix m_hessianMatrix;/**< hessian matrix of cost function. */
        iDynSparseMatrix m_constraintsMatrix; /**< constraints matrix. */
        iDynTree::VectorDynSize m_gradient;/**< Gradient vector. */
        iDynTree::VectorDynSize m_lowerBound; /**< Lower bound vector. */
        iDynTree::VectorDynSize m_upperBound; /**< Upper bound vector. */
        std::unique_ptr<DCMSimpleEstimator> m_DCMEstimator; /**< Pointer to the simple pendulum estimator. */
        iDynTree::VectorDynSize m_solution;  /**< solution vector of the optimization. */

        int m_inputSize; /**< Size of the controlled input vector . */
        int m_numberOfConstraints; /**< Size of the constraint vector . */

        bool m_isFirstTime;/**< boolean  that indicates whether the solver has been already initilized? . */

        iDynTree::Vector2 m_zmpPositionNominal; /**< The next desired step position(The zmp position for next single support) .. */
        iDynTree::Vector2 m_dcmOffsetNominal; /**< The next desired dcm offset*/
        double m_sigmaNominal; /**< The exponential function of step duration multplied by the natural frequency of the LIPM.*/

        iDynTree::Vector2 m_zmpPositionWeight; /**< The wight of next step position term in the cost function.*/
        iDynTree::Vector2 m_dcmOffsetWeight;/**< The wight of dcm offset term in the cost function.*/
        double m_sigmaWeight;/**< The wight of step timing term in the cost function.*/
        int m_pushRecoveryActivationIndex;/**< A threshold index for activation of push recovery.*/

        std::vector<std::string> m_pushDetectionListRightArmX; /**< Vector containing the name of the right arm joints that will be used for push detection in X direction. */
        std::vector<std::string> m_pushDetectionListLeftArmX;/**< Vector containing the name of the left arm joints that will be used for push detection in X direction. */
        std::vector<std::string> m_pushDetectionListRightArmY;/**< Vector containing the name of the right arm joints that will be used for push detection in Y direction. */
        std::vector<std::string> m_pushDetectionListLeftArmY;/**< Vector containing the name of the left arm joints that will be used for push detection in Y direction. */

        iDynTree::Vector2 m_dcmErrorThreshold; /**< The threshold for activating the push recovery based on DCM error.*/
        iDynTree::Vector2 m_rollPitchErrorThreshold; /**< The threshold for activating the pendulum estimator based on the foot orientation error.*/
        iDynTree::Vector2 m_armRollPitchErrorOffset; /**< The offset for arm joints error to generate DCM error inside the simple DCM estimator to use in step adaptation*/

        iDynTree::Vector2 m_currentZmpPosition; /**< The current step position(The zmp position of current stance foot). */
        iDynTree::Vector2 m_currentDcmPosition; /**< The current DCM position.*/

        iDynTree::Vector2 m_zmpToCenterOfFootPositionLeft;
        iDynTree::Vector2 m_zmpToCenterOfFootPositionRight;

        double m_remainingSingleSupportDuration;/**< The remained single support duration.*/
        double m_stepTiming; /**< The remanined single support duration+(next double support duration)/2  that is used for optimization.*/
        double m_stepDurationTolerance;/**< The tolerance of step timing with respect to the nominal value.*/
        double m_stepHeight;/**< The maximum height of swing foot.*/

        double m_omega;/**< The natural frequency of LIPM.*/

        double m_currentTime;/**< The  current time.*/
        double m_nextDoubleSupportDuration;/**< The timing of next double support.*/

        bool m_isPitchActive=0;/**< The  boolean that shows whether push is detected with pitch arm joints or no.*/
        bool m_isRollActive=0;/**< The  boolean that shows whether push is detected with roll arm joints or no.*/

        double m_armRollError;/**< The  error related to the roll joints position of the arm .*/
        double m_armPitchError;/**< The  error related to the pitch joints position of the arm .*/

        /**
         *The buffered vectors for the interpolation of the foot trajectory
         */
        iDynTree::VectorDynSize m_xPositionsBuffer, m_yPositionsBuffer, m_zFirstPiecePositionsBuffer,m_zSecondPiecePositionsBuffer,
                                m_yawsBuffer, m_timesBuffer, m_zFirstPieceTimesBuffer,m_zSecondPieceTimesBuffer;

        iDynTree::ConvexHullProjectionConstraint m_convexHullComputer; /**< iDynTree convex hull helper. */
        std::vector<iDynTree::Polygon> m_feetExtendedPolygon;/**< convex hull of the allowable landing foot position. */
        iDynTree::Transform m_footTransform; /**< transform of the next foot position. */

        bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

        /**
         * Compute the hessian matrix.
         * Please do not call this function to update the hessian matrix! It can be set only once.
         * @return true/false in case of success/failure.
         */
        bool computeHessianMatrix();

        /**
         * Compute or update the linear constraints matrix(A) related to equality and inequality constraints(C<Ax<B)
         * If the solver is already set the linear constraints matrix is updated otherwise it is set for
         * the first time.
         * @return true/false in case of success/failure.
         */
        bool computeConstraintsMatrix();

        /**
         * Compute or update the gradient
         * @return true/false in case of success/failure.
         */
        bool computeGradientVector();

        /**
         * Set or update the lower and the upper bounds
         * @return true/false in case of success/failure.
         */
        bool computeBoundsVectorOfConstraints();

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
        bool configure(const yarp::os::Searchable& config);

        /**
         * Solve the Optimization problem. If the QPSolver is not set It will be initialized.
         * @param isLeft is true if left foot is the swing foot
         * @return true/false in case of success/failure.
         */
        bool solve(SwingFoot swingFoot);

        /**
         * Trigger the step adaptation by detecting the push by the arm joints in compliant mode.
         * @param numberOfActuatedDof The  number of the joints that is actuated.
         * @param qDesired The vector of th desired positions of the joints.
         * @param qActual The vector of th actual positions of the joints.
         * @param leftInContact The deque of boolean that shows left foot is in contact or no.
         * @param rightInContact The deque of boolean that shows right foot is in contact or no.
         * @return true/false in case of success/failure.
         */
        bool triggerStepAdapterByArmCompliant(const double& numberOfActuatedDof, const iDynTree::VectorDynSize& qDesired, const iDynTree::VectorDynSize& qActual,
                                              const std::deque<bool>& leftInContact, const std::deque<bool>& rightInContact, std::vector<std::string> jointsListVector);

        /**
         * Reset the controller
         */
        void reset();

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
         * Get the next DCM Offset
         * @return The DCM offset of the next step.
         */
        iDynTree::Vector2 getDesiredDCMOffset();

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

        /**
         * Replan the swing foot trajectory.
         * @param input Structure that includes data that we need as input for function.
         * @param adaptatedFootTransform  Adapted transform of the swing foot.
         * @param adaptedFootTwist Adapted twist of the swing foot.
         * @param adaptedFootAcceleration Adapted acceleration of the swing foot.
         * @return true/false in case of success/failure.
         */
        bool getAdaptatedFootTrajectory(const FootTrajectoryGenerationInput &input, iDynTree::Transform& adaptatedFootTransform,
                                        iDynTree::Twist& adaptedFootTwist, iDynTree::SpatialAcc& adaptedFootAcceleration);

        /**
         * Get the Value of the arm joints roll error ..
         * @return Value of the arm joints roll error .
         */
        const double& getArmRollError()const;

        /**
         * Get the Value of the arm joints pitch error ..
         * @return Value of the arm joints pitch error .
         */
        const double& getArmPitchError()const;

        /**
         * Get the threshold of push recovery activation index .
         * @return The integer threshold of push recovery activation index .
         */
        const int &getPushRecoveryActivationIndex() const;

        /**
         * Get the boolean to specify that the push in forward direction has been detected ..
         * @return True in case that the pitch joints of the arm detect the push .
         */
        bool isArmPitchActive();

        /**
         * Get the boolean to specify that the push in lateral direction has been detected ..
         * @return True in a case that the roll+yaw joints of the arm detect the push .
         */
        bool isArmRollActive();

        /**
         * Get the estimated position of the DCM.
         * @return ertimated position of the DCM.
         */
        const iDynTree::Vector2& getEstimatedDCM() const;

        /**
         * update the pendulum estimator
         * @param CoM2DPosition CoM2DPosition the 2D com position obtained as if the foot is not rotated.
         * @param CoMVelocity the vector of com velocity that is simple time derivative of the com position.
         * @param measuredZMP the vector of measured zmp position with respect to the inertial frame.
         * @param CoMHeight the CoM height.
         * @return true/false in case of success/failure
         */
        bool UpdateDCMEstimator(const iDynTree::Vector2 &CoM2DPosition, const iDynTree::Vector2 &CoMVelocity, const iDynTree::Vector2 &measuredZMP, const double &CoMHeight);


        /**
         * Run the step adaptation.
         * @param input Structure that includes data that we need as input for function.
         * @param output Structure that includes data that is output of step adjustment.
         * @return true/false in case of success/failure.
         */
        bool runStepAdaptation(const StepAdapterInput &input, StepAdapterOutput& output);

    };
};

#endif
