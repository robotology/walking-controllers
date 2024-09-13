// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// std
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <algorithm>

// yarp
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/EigenSparseHelpers.h>
#include <iDynTree/Direction.h>

#include <WalkingControllers/iDynTreeUtilities/Helper.h>
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/SimplifiedModelControllers/DCMModelPredictiveController.h>

using namespace WalkingControllers;

iDynSparseMatrix WalkingController::evaluateThetaMatrix()
{
    // set the submatrix dimension
    int submatrixDimension = m_inputSize * m_controllerHorizon;

    iDynTree::Triplets thetaTriplets;
    thetaTriplets.addDiagonalMatrix(0, 0, 1, m_inputSize * m_controllerHorizon);
    thetaTriplets.addDiagonalMatrix(m_inputSize, 0, -1, m_inputSize * (m_controllerHorizon -1));

    iDynSparseMatrix thetaMatrix(submatrixDimension, submatrixDimension);
    thetaMatrix.setFromConstTriplets(thetaTriplets);

    return thetaMatrix;
}

iDynTree::Triplets WalkingController::evaluateInputWeightStackedMatrix(const iDynTree::Triplets& inputWeightMatrix)
{
    iDynTree::Triplets inputWeightStackedMatrix;

    // populate triplets adding submatrices
    //  \tilde{R} = diag(R, R, ..., R)
    for(int i = 0; i < m_controllerHorizon; i++)
        iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(i * m_inputSize, i * m_inputSize,
                                                          inputWeightMatrix,
                                                          inputWeightStackedMatrix);
    return inputWeightStackedMatrix;
}

iDynTree::Triplets WalkingController::evaluateStateWeightStackedMatrix(const iDynTree::Triplets& stateWeightMatrix)
{
    iDynTree::Triplets stateWeightStackedMatrix;

    // populate triplets adding submatices
    //  \tilde{Q} = diag(Q, Q, ..., Q)
    for(int i = 0; i < (m_controllerHorizon + 1); i++)
        iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(i * m_inputSize, i * m_inputSize,
                                                          stateWeightMatrix,
                                                          stateWeightStackedMatrix);
    return stateWeightStackedMatrix;
}


iDynSparseMatrix WalkingController::evaluateHessianInputSubmatrix(const iDynTree::Triplets& inputWeightStackedTriplets,
                                                                  const iDynSparseMatrix& thetaMatrix)
{
    int submatrixDimension = m_inputSize * m_controllerHorizon;

    iDynSparseMatrix inputWeightStackedMatrix(submatrixDimension, submatrixDimension);
    inputWeightStackedMatrix.setFromConstTriplets(inputWeightStackedTriplets);

    return iDynTreeUtilities::SparseMatrix::fromEigen((Eigen::SparseMatrix<double>(iDynTree::toEigen(thetaMatrix).transpose())
                                                    * iDynTree::toEigen(inputWeightStackedMatrix) *
                                                    iDynTree::toEigen(thetaMatrix)).pruned(0.00001));

}

iDynTree::Triplets WalkingController::evaluateEqualConstraintsMatrix(const iDynTree::Triplets& stateDynamicsTriplets,
                                                                     const iDynTree::Triplets& inputDynamicsTriplets)
{
    // evaluate submatrices
    iDynTree::Triplets equalConstraintsStateSubmatrixTriplets = evaluateEqualConstraintsStateSubmatrix(stateDynamicsTriplets);
    iDynTree::Triplets equalConstraintsInputSubmatrixTriplets = evaluateEqualConstraintsInputSubmatrix(inputDynamicsTriplets);

    // evaluate equal constraints matrix
    iDynTree::Triplets equalConstraintsTriplets;
    iDynTreeUtilities::Triplets::pushTriplets(equalConstraintsStateSubmatrixTriplets,
                                           equalConstraintsTriplets);
    iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(0, m_inputSize * (m_controllerHorizon + 1),
                                                      equalConstraintsInputSubmatrixTriplets,
                                                      equalConstraintsTriplets);

    return equalConstraintsTriplets;
}

iDynTree::Triplets WalkingController::evaluateEqualConstraintsStateSubmatrix(const iDynTree::Triplets& stateDynamicsMatrix)
{
    // evaluate equal constraint state triplets
    // [See here](https://github.com/loc2/element_capture-point-walking/issues/9).
    iDynTree::Triplets equalConstraintsStateSubmatrix;
    equalConstraintsStateSubmatrix.addDiagonalMatrix(0, 0, -1,
                                                     m_inputSize * (m_controllerHorizon + 1));

    for(int i = 0; i < m_controllerHorizon; i++)
        iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(i * m_stateSize + m_stateSize,
                                                          i * m_stateSize,
                                                          stateDynamicsMatrix,
                                                          equalConstraintsStateSubmatrix);

    return equalConstraintsStateSubmatrix;
}

iDynTree::Triplets WalkingController::evaluateEqualConstraintsInputSubmatrix(const iDynTree::Triplets& inputDynamicsMatrix)
{
    // evaluate equal constraints input triplets
    iDynTree::Triplets equalConstraintsInputSubmatrix;
    for(int i = 0; i < m_controllerHorizon; i++)
        iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(i * m_stateSize + m_stateSize,
                                                          i * m_stateSize,
                                                          inputDynamicsMatrix,
                                                          equalConstraintsInputSubmatrix);
    return equalConstraintsInputSubmatrix;
}

iDynSparseMatrix WalkingController::evaluateHessianMatrix(const iDynTree::Triplets& stateSubmatrix,
                                                          const iDynSparseMatrix& inputSubmatrix)
{
    int matrixDimension = m_stateSize * (m_controllerHorizon + 1) +
        m_inputSize * m_controllerHorizon;

    // add submatrix to the triplets
    iDynTree::Triplets hessianMatrixTriplet;
    iDynTreeUtilities::Triplets::pushTriplets(stateSubmatrix, hessianMatrixTriplet);
    hessianMatrixTriplet.addSubMatrix(m_stateSize * (m_controllerHorizon + 1),
                                      m_stateSize * (m_controllerHorizon + 1),
                                      inputSubmatrix);

    // evaluate the hessian matrix
    iDynSparseMatrix hessianMatrix(matrixDimension, matrixDimension);
    hessianMatrix.setFromConstTriplets(hessianMatrixTriplet);

    return hessianMatrix;
}



iDynSparseMatrix WalkingController::evaluateGradientSubmatrix(const iDynTree::Triplets& inputWeightStackedTriplets,
                                                              const iDynSparseMatrix& thetaMatrix)
{
    // evaluate e1 matrix
    // e1 = [I, 0, 0, 0, ..., 0]'
    iDynSparseMatrix e1Matrix(m_inputSize * m_controllerHorizon,
                              m_inputSize);

    iDynTree::Triplets e1Triplets;
    e1Triplets.addDiagonalMatrix(0, 0, 1, m_inputSize);
    e1Matrix.setFromConstTriplets(e1Triplets);

    int submatrixDimension = m_inputSize * m_controllerHorizon;
    iDynSparseMatrix inputWeightStackedMatrix(submatrixDimension, submatrixDimension);
    inputWeightStackedMatrix.setFromConstTriplets(inputWeightStackedTriplets);

    // return gradient submatrix
    return iDynTreeUtilities::SparseMatrix::fromEigen((-Eigen::SparseMatrix<double>(iDynTree::toEigen(thetaMatrix).transpose())
                                                    * iDynTree::toEigen(inputWeightStackedMatrix)
                                                    * iDynTree::toEigen(e1Matrix)).pruned(0.00001));
}

bool WalkingController::initializeMatrices(const yarp::os::Searchable& config)
{
    yarp::os::Value tempValue;

    // check if the config is empty
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for walking controller.";
        return false;
    }

    // get sampling time
    double dT = config.check("sampling_time", yarp::os::Value(0.016)).asFloat64();

    // evaluate the controller horizon
    double controllerHorizonSeconds = config.check("controllerHorizon",
                                                   yarp::os::Value(2.0)).asFloat64();
    m_controllerHorizon = static_cast<int>(std::round(controllerHorizonSeconds / dT));

    // get the state weight matrix
    tempValue = config.find("stateWeightTriplets");
    iDynTree::Triplets stateWeightMatrix;
    if(!iDynTreeUtilities::Triplets::getTripletsFromValues(tempValue, m_stateSize, stateWeightMatrix))
    {
        yError() << "Initialization failed while reading stateWeightTriplets vector.";
        return false;
    }
    // the sparse matrix
    m_stateWeightMatrix.resize(m_stateSize, m_stateSize);
    m_stateWeightMatrix.setFromConstTriplets(stateWeightMatrix);

    // get the input weight matrix
    tempValue = config.find("inputWeightTriplets");
    iDynTree::Triplets inputWeightMatrix;
    if(!iDynTreeUtilities::Triplets::getTripletsFromValues(tempValue, m_inputSize, inputWeightMatrix))
    {
        yError() << "Initialization failed while reading inputWeightTriplets vector.";
        return false;
    }

    // evaluate submatrices
    iDynSparseMatrix thetaMatrix = evaluateThetaMatrix();
    iDynTree::Triplets inputWeightStackedMatrix = evaluateInputWeightStackedMatrix(inputWeightMatrix);
    iDynTree::Triplets stateWeightStackedMatrix = evaluateStateWeightStackedMatrix(stateWeightMatrix);
    iDynSparseMatrix hessianInputSubmatrix = evaluateHessianInputSubmatrix(inputWeightStackedMatrix,
                                                                           thetaMatrix);

    // evaluate hessian matrix
    m_hessianMatrix = evaluateHessianMatrix(stateWeightStackedMatrix, hessianInputSubmatrix);

    // evaluate gradient submatrix
    m_gradientSubmatrix = evaluateGradientSubmatrix(inputWeightStackedMatrix, thetaMatrix);

    // get model parameters
    double comHeight;
    if(!YarpUtilities::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get the double from searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asFloat64();
    double omega = sqrt(gravityAcceleration / comHeight);

    // evaluate dynamics matrix
    iDynTree::Triplets stateDynamicsTriplets;
    iDynTree::Triplets inputDynamicsTriplets;
    stateDynamicsTriplets.addDiagonalMatrix(0, 0, exp(omega * dT), m_stateSize);
    inputDynamicsTriplets.addDiagonalMatrix(0, 0, 1 - exp(omega * dT), m_inputSize);

    // evaluate equal constraints matrix
    m_equalConstraintsMatrixTriplets = evaluateEqualConstraintsMatrix(stateDynamicsTriplets,
                                                                      inputDynamicsTriplets);
    return true;
}

bool WalkingController::initializeConstraints(const yarp::os::Searchable& config)
{
    yarp::os::Value feetDimensions = config.find("foot_size");
    if(feetDimensions.isNull() || !feetDimensions.isList())
    {
        yError() << "Please set the foot_size in the configuration file.";
        return false;
    }

    yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
    if(!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
    {
        yError() << "Error while reading the feet dimensions. Wrong number of elements.";
        return false;
    }

    yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
    if(xLimits.isNull() || !xLimits.isList())
    {
        yError() << "Error while reading the X limits.";
        return false;
    }

    yarp::os::Bottle *xLimitsPtr = xLimits.asList();
    if(!xLimitsPtr || xLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the X limits. Wrong dimensions.";
        return false;
    }

    double xlimit1 = xLimitsPtr->get(0).asFloat64();
    double xlimit2 = xLimitsPtr->get(1).asFloat64();

    yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
    if(yLimits.isNull() || !yLimits.isList())
    {
        yError() << "Error while reading the Y limits.";
        return false;
    }

    yarp::os::Bottle *yLimitsPtr = yLimits.asList();
    if(!yLimitsPtr || yLimitsPtr->size() != 2)
    {
        yError() << "Error while reading the Y limits. Wrong dimensions.";
        return false;
    }

    double ylimit1 = yLimitsPtr->get(0).asFloat64();
    double ylimit2 = yLimitsPtr->get(1).asFloat64();

    // evaluate the foot polygon
    iDynTree::Polygon foot;
    foot = iDynTree::Polygon::XYRectangleFromOffsets(std::abs(std::max(xlimit1, xlimit2)),
                                                     std::abs(std::min(xlimit1, xlimit2)),
                                                     std::abs(std::max(ylimit1, ylimit2)),
                                                     std::abs(std::min(ylimit1, ylimit2)));
    m_feetPolygons.resize(2);
    m_feetPolygons[0] = foot;
    m_feetPolygons[1] = foot;

    // set the tolerance of the convex hull
    m_convexHullTolerance = config.check("convex_hull_tolerance", yarp::os::Value(0.01)).asFloat64();

    return true;
}

bool WalkingController::initialize(const yarp::os::Searchable& config)
{
    // initialize the state and in input vectors size.
    m_stateSize = 2;
    m_inputSize = 2;

    yarp::os::Value input = config.find("initial_zmp_position");
    if(input.isNull())
    {
        yError() << "[initialize] Empty initial zmp position.";
        return false;
    }
    if(!input.isList() || !input.asList())
    {
        yError() << "[initialize] Unable to read zmp position.";
        return false;
    }
    yarp::os::Bottle *inputPtr = input.asList();

    if(inputPtr->size() != 2)
    {
        yError() << "[initialize] The dimension set in the configuration file is not 2.";
        return false;
    }

    for (int i = 0; i < inputPtr->size(); ++i)
    {
        if(!inputPtr->get(i).isFloat64())
        {
            yError() << "[initialize] The zmp position is expected to be a double";
            return false;
        }
        m_output(i) = inputPtr->get(i).asFloat64();
    }

    if(!initializeMatrices(config))
    {
        yError() << "[initialize] Error while the matrices are initialized";
        return false;
    }

    if(!initializeConstraints(config))
    {
        yError() << "[initialize] Error while the constraints are initialized";
        return false;
    }

    // reset the solver
    reset();

    return true;
}

bool WalkingController::setConvexHullConstraint(const std::deque<iDynTree::Transform>& leftFoot,
                                                const std::deque<iDynTree::Transform>& rightFoot,
                                                const std::deque<bool>& leftInContact,
                                                const std::deque<bool>& rightInContact)
{
    auto feetStatus = std::make_pair(leftInContact.front(), rightInContact.front());

    // the status of the feet is the same of the previous iteration
    // the convexHull is already evaluated: do nothing
    if(m_feetStatus == feetStatus)
        return true;

    m_feetStatus = feetStatus;

    // evaluate the convex hull
    if(feetStatus == std::make_pair(true, true))
    {
        if(!buildConvexHull(leftFoot.front(), rightFoot.front()))
        {
            yError() << "[setConvexHullConstraint] Error while the contraints are evaluated.";
            return false;
        }
    }

    if(feetStatus == std::make_pair(true, false))
    {
        if(!buildConvexHull(leftFoot.front()))
        {
            yError() << "[setConvexHullConstraint] Error while the contraints are evaluated.";
            return false;
        }
    }

    if(feetStatus == std::make_pair(false, true))
    {
        if(!buildConvexHull(rightFoot.front()))
        {
            yError() << "[setConvexHullConstraint] Error while the contraints are evaluated.";
            return false;
        }
    }

    if(feetStatus == std::make_pair(false, false))
    {
        yError() << "[setConvexHullConstraint] None foot is in contact How is it possible?.";
        return false;
    }

    int numberOfConstraints = static_cast<int>(m_convexHullComputer.A.rows());

    // is it possible to reuse the old solver??
    m_currentController = std::make_shared<MPCSolver>(m_stateSize, m_inputSize,
                                                      m_controllerHorizon,
                                                      numberOfConstraints,
                                                      m_equalConstraintsMatrixTriplets,
                                                      m_gradientSubmatrix,
                                                      m_stateWeightMatrix);
    // the hessian matrix is set only once
    if(!m_currentController->setHessianMatrix(m_hessianMatrix))
    {
        yError() << "[addNewController] Unable to set the hessian matrix.";
        return false;
    }

    if(!m_currentController->setConstraintsMatrix(m_convexHullComputer.A))
    {
        yError() << "[setConvexHullConstraint] Unable to add set constraints Matrix.";
        return false;
    }

    return true;
}

bool WalkingController::setFeedback(const iDynTree::Vector2& currentState)
{
    return m_currentController->setBounds(currentState, m_convexHullComputer.b);
}

bool WalkingController::setReferenceSignal(const std::deque<iDynTree::Vector2>& referenceSignal,
                                           const bool& resetTrajectory)
{
    return m_currentController->setGradient(referenceSignal, m_output, resetTrajectory);
}

bool WalkingController::buildConvexHull(const iDynTree::Transform& leftFootTransform,
                                        const iDynTree::Transform& rightFootTransform)
{
    // initilialize axes direction
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;
    yAxis.zero();
    yAxis(1) = 1;

    // initilize plane origin
    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    std::vector<iDynTree::Transform> feetTransforms;
    feetTransforms.push_back(leftFootTransform);
    feetTransforms.push_back(rightFootTransform);

    return m_convexHullComputer.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                m_feetPolygons, feetTransforms);
}

bool WalkingController::buildConvexHull(const iDynTree::Transform& footTransform)
{
    // initilialize axes direction
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;
    yAxis.zero();
    yAxis(1) = 1;

    // initilize plane origin
    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    std::vector<iDynTree::Transform> feetTransforms;
    feetTransforms.push_back(footTransform);

    return m_convexHullComputer.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                std::vector<iDynTree::Polygon>(1, m_feetPolygons[0]),
                                                feetTransforms);
}

bool WalkingController::solve()
{
    if(!m_currentController->isInitialized())
    {
        if(!m_currentController->initialize())
        {
            yError() << "[solve] Unable to initialize the solver.";
            return false;
        }
    }

    if(!m_currentController->solve())
    {
        yError() << "[solve] Unable to solve the problem.";
        return false;
    }

    iDynTree::VectorDynSize solution = m_currentController->getSolution();
    m_output(0) = solution(m_stateSize * (m_controllerHorizon + 1));
    m_output(1) = solution(m_stateSize * (m_controllerHorizon + 1) + 1);

    if(m_convexHullComputer.computeMargin(m_output) < -m_convexHullTolerance)
    {
        yError() << "[solve] The evaluated ZMP is outside the convexHull.";
        return false;
    }

    return true;
}

const iDynTree::Vector2& WalkingController::getControllerOutput() const
{
    return m_output;
}

void WalkingController::reset()
{
    // used to indicate the first step.
    m_feetStatus = std::make_pair<bool, bool>(false, false);
}
