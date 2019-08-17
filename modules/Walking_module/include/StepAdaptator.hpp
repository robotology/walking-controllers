
#ifndef STEP_ADAPTATOR_HPP
#define STEP_ADAPTATOR_HPP

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
#include <QPSolver.hpp>

//interpolation
#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/EigenHelpers.h>


/**
 * StepAdaptator class contains the controller instances.
 */
class StepAdaptator
{
    /**
     * The value of change of the ZMP during single support phase
     */
    iDynTree::Vector2 m_zmpPositionNominal;
    iDynTree::Vector2 m_dcmOffsetNominal;
    double m_sigmaNominal;

    iDynTree::Vector2 m_zmpPositionWeight;
    iDynTree::Vector2 m_dcmOffsetWeight;
    double m_sigmaWeight;

    iDynTree::Vector2 m_currentZmpPosition;
    iDynTree::Vector2 m_currentDcmPosition;

    iDynTree::Vector2 m_zmpPositionTollerance_x;
    iDynTree::Vector2 m_zmpPositionTollerance_yLeft;
    iDynTree::Vector2 m_zmpPositionTollerance_yRight;


    iDynTree::Vector2 m_desiredZmp;

    double m_stepTiming;
    double m_stepDurationTolerance;
    double m_remainingSingleSupportDuration;
    double m_omega;

    double m_currentTime;
    double m_nextDoubleSupportDuration;

    int m_inputSize;  /**< Size of the input vector. It is equal to 3 now!!!!. */
    int m_numberOfConstraint;  /**< Size of the input vector. It is equal to 5 now!!!!. */

    iDynTree::VectorDynSize m_xPositionsBuffer, m_yPositionsBuffer, m_zPositionsBuffer,m_zzPositionsBuffer, m_yawsBuffer, m_timesBuffer, m_zTimesBuffer,m_zzTimesBuffer;

    iDynTree::ConvexHullProjectionConstraint m_convexHullComputer; /**<iDynTree convex hull helper. */
    std::vector<iDynTree::Polygon> m_feetExtendedPolygon;
    iDynTree::Transform m_footTransform;

    std::pair<bool, bool> m_feetStatus; /**< Current status of the feet. Left and Right. True is used
                                           if the foot is in contact. */

    bool m_isSolutionEvaluated{false}; /**< True if the solution is evaluated. */

    /**
     * Pointer to the current QPSolver.
     * A new MPC solver is initialized when a new phase occurs.
     */

    std::shared_ptr<QPSolver> m_currentQPSolver;

public:

    StepAdaptator();

    /**
     * Initialize the method
     * @param config yarp searchable configuration variable.
     * @return true/false in case of success/failure
     */
    bool initialize(const yarp::os::Searchable& config);


    /**
     * Solve the Optimization problem. If the QPSolver is not set It will be initialized.
     * @return true/false in case of success/failure.
     */
    bool solve(bool isLeft);

    /**
     * Reset the controller
     */
    void reset();
    /**
     * Run the step adaptation and set constraints and gradient vector and solve the QP problem
     * @param nominalValuesVector Vector that includes the Desired Value of DCM at the landing moment of foot, StepTiming and next StepPosition and next DCM Offset;
     * @param currentValuesVector This vector includes the current value of real ZMP, real DCM and delta(the distance that ZMP moves in the SS phase)   ;
     * @return true/false in case of success/failure.
     */


    bool getAdaptatedFootTrajectory(double maxFootHeight, double dt, double takeOffTime, double yawAngleAtImpact, iDynTree::Vector2 zmpOffset,
                                    const iDynTree::Transform& currentFootTransform, const iDynTree::Twist& currentFootTwist,
                                    iDynTree::Transform& adaptatedFootTransform, iDynTree::Twist& adaptedFootTwist,
                                    iDynTree::SpatialAcc& adaptedFootAcceleration);

    void setNominalNextStepPosition(const iDynTree::Vector2& nominalZmpPosition, const double& angle);

    void setTimings(const double & omega, const double & currentTime, const double& nextImpactTime,
                    const double &nextDoubleSupportDuration);

    void setNominalDcmOffset(const iDynTree::Vector2& nominalDcmOffset);

    void setCurrentZmpPosition(const iDynTree::Vector2& currentZmpPosition);

    void setCurrentDcmPosition(const iDynTree::Vector2& currentDcmPosition);

    /**
     * Get the output of the controller.
     * @param controllerOutput is the vector containing the output the controller.
     * @return true/false in case of success/failure.
     */
    bool getControllerOutput(iDynTree::Vector3& controllerOutput);

    double getDesiredImpactTime();

    iDynTree::Vector2 getDesiredZmp();
};

#endif
