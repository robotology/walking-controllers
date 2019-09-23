
// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <QPSolver.hpp>
#include <Utils.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;

QPSolver::QPSolver(const int& inputSize, const int& numberOfConstraints)
    :m_inputSize(inputSize), m_numberOfConstraints(numberOfConstraints)
{
    // instantiate the solver class
    m_QPSolver = std::make_unique<OsqpEigen::Solver>();

    //set the number of deceision variables of QP problem
    m_QPSolver->data()->setNumberOfVariables(inputSize);

    // set the number of all constraints includes inequality and equality constraints
    m_QPSolver->data()->setNumberOfConstraints(numberOfConstraints);

    m_QPSolver->settings()->setVerbosity(false);
    m_QPSolver->settings()->setPolish(true);

    m_constraintsMatrix.resize(numberOfConstraints, inputSize);
    m_upperBound.resize(numberOfConstraints);
    m_lowerBound.resize(numberOfConstraints);
    m_gradient.resize(inputSize);

    // set the constant elements of the constraint matrix
    m_constraintsMatrix(0, 0) = 1;
    m_constraintsMatrix(1, 1) = 1;

    m_constraintsMatrix(0, 3) = 1;
    m_constraintsMatrix(1, 4) = 1;

    m_constraintsMatrix(6, 2) = 1;

    m_hessianMatrix.resize(m_inputSize, m_inputSize);

    m_solution.resize(m_inputSize);

    // qpoases
    m_QPSolver_qpOASES = std::make_unique<qpOASES::SQProblem>(inputSize,
                                                              m_numberOfConstraints);

    m_QPSolver_qpOASES->setPrintLevel(qpOASES::PL_LOW);
    m_isFirstTime = true;
}

bool QPSolver::setHessianMatrix(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight)
{
    m_hessianMatrix(0,0) = zmpWeight(0);
    m_hessianMatrix(1,1) = zmpWeight(1);

    m_hessianMatrix(2,2) = sigmaWeight;

    m_hessianMatrix(3,3) = dcmOffsetWeight(0);
    m_hessianMatrix(4,4) = dcmOffsetWeight(1);
    // if (m_QPSolver->isInitialized())
    // {
    //     yWarning()<<"[QPslover::setHessianMatrix] The Hessian Matrix should be set just one time! In step adaptation the hessian matrix is constant and just depend on the gains of cost funtion.";
    //     //        return  false;
    // }
    // else
    // {
    //     if (!(m_QPSolver->data()->setHessianMatrix(iDynTree::toEigen(m_hessianMatrix))))
    //     {
    //         yError()<<"[QPslover::setHessianMatrix]Unable to set first time the hessian matrix.";
    //         return false;
    //     };
    // }
    return true;
}

bool QPSolver::setGradientVector(const iDynTree::Vector2& zmpWeight, const iDynTree::Vector2& dcmOffsetWeight, const double& sigmaWeight,
                                 const iDynTree::Vector2& zmpNominal, const iDynTree::Vector2& dcmOffsetNominal, const double& sigmaNominal)
{
    iDynTree::toEigen(m_gradient).segment(0, 2) = -(iDynTree::toEigen(zmpWeight).asDiagonal() * iDynTree::toEigen(zmpNominal));
    m_gradient(2) = -sigmaWeight * sigmaNominal;
    iDynTree::toEigen(m_gradient).segment(3, 2)  = -(iDynTree::toEigen(dcmOffsetWeight).asDiagonal() * iDynTree::toEigen(dcmOffsetNominal));

    if(m_QPSolver->isInitialized())
    {
        if(!m_QPSolver->updateGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to update the Gradient Vector";
            return false;
        }
    }
    else
    {
        if(!m_QPSolver->data()->setGradient(iDynTree::toEigen(m_gradient)))
        {
            yError()<<"[QPSolver::setGradientVector]:unable to set the Gradient Vector for the first time";
            return false;
        }
    }
    return true;
}

bool QPSolver::setConstraintsMatrix(const iDynTree::Vector2& currentDcmPosition, const iDynTree::Vector2& currentZmpPosition,
                                    const iDynTree::MatrixDynSize& convexHullMatrix,const iDynTree::Vector2& delta2,const iDynTree::Vector2& finalZMPPosition)
{
    if(convexHullMatrix.rows() != 4 || convexHullMatrix.cols() != 2)
    {
        yError() << "QPSolver::setConstraintsMatrix the convex hull matrix it is strange " << convexHullMatrix.toString();
        return false;
    }

    iDynTree::Vector2 temp;
    iDynTree::toEigen(temp) = iDynTree::toEigen(finalZMPPosition)-iDynTree::toEigen(delta2)/2 - iDynTree::toEigen(currentDcmPosition);

    m_constraintsMatrix(0, 2) = temp(0);
    m_constraintsMatrix(1, 2) = temp(1);
    m_constraintsMatrix(2, 0) = convexHullMatrix(0, 0);
    m_constraintsMatrix(2, 1) = convexHullMatrix(0, 1);
    m_constraintsMatrix(3, 0) = convexHullMatrix(1, 0);
    m_constraintsMatrix(3, 1) = convexHullMatrix(1, 1);
    m_constraintsMatrix(4, 0) = convexHullMatrix(2, 0);
    m_constraintsMatrix(4, 1) = convexHullMatrix(2, 1);
    m_constraintsMatrix(5, 0) = convexHullMatrix(3, 0);
    m_constraintsMatrix(5, 1) = convexHullMatrix(3, 1);

    m_constraintsMatrix(1, 1) = 1;

    m_constraintsMatrix(0, 3) = 1;
    m_constraintsMatrix(1, 4) = 1;

    m_constraintsMatrix(6, 2) = 1;

    m_constraintsMatrix(7, 3) = 1;



    // if(m_QPSolver->isInitialized())
    // {
    //     if(!m_QPSolver->updateLinearConstraintsMatrix(iDynTree::toEigen(m_constraintsMatrix)))
    //     {
    //         yError()<<"[setConstraintsMatrix] unable to update the linear constraints matrix of QPSolver corresponding to step adaptator!";
    //         return false;
    //     }
    // }
    // else
    // {
    //     if (!m_QPSolver->data()->setLinearConstraintsMatrix(iDynTree::toEigen(m_constraintsMatrix)))
    //     {
    //         yError()<<"[setConstraintsMatrix] unable to set the the linear constraints matrix of QPSolver corresponding to step adaptator for the first time ";
    //         return false;
    //     }
    // }
    return true;
}

bool QPSolver::setBoundsVectorOfConstraints(const iDynTree::Vector2& zmpPosition, const iDynTree::VectorDynSize& convexHullVector,
                                            const double& stepDuration, const double& stepDurationTollerance, const double& remainingSingleSupportDuration, const double& omega,const iDynTree::Vector2 &Delta1)
{
    if(convexHullVector.size() != 4)
    {
        yError() << "QPSolver::setConstraintsMatrix the convex hull matrix it is strange " << convexHullVector.toString();
        return  false;
    }


// Two equality constraints and three inequality constraint

    // equality constraint
    iDynTree::toEigen(m_upperBound).segment(0, 2) = (iDynTree::toEigen(zmpPosition)+iDynTree::toEigen(Delta1)/2);
    iDynTree::toEigen(m_lowerBound).segment(0, 2) = (iDynTree::toEigen(zmpPosition)+iDynTree::toEigen(Delta1)/2);
    iDynTree::toEigen(m_upperBound).segment(2, 4) = iDynTree::toEigen(convexHullVector);
//    m_upperBound(0) = qpOASES::INFTY;
//    m_upperBound(1) = qpOASES::INFTY;
//    m_upperBound(2) = qpOASES::INFTY;
//    m_upperBound(3) = qpOASES::INFTY;
//    m_upperBound(4) = qpOASES::INFTY;
//    m_upperBound(5) = qpOASES::INFTY;
//    m_lowerBound(0) = -qpOASES::INFTY;
//    m_lowerBound(1) = -qpOASES::INFTY;
    m_lowerBound(2) = -qpOASES::INFTY;
    m_lowerBound(3) = -qpOASES::INFTY;
    m_lowerBound(4) = -qpOASES::INFTY;
    m_lowerBound(5) = -qpOASES::INFTY;
//    m_lowerBound(6) = -qpOASES::INFTY;
//    m_upperBound(6) = qpOASES::INFTY;

    // iDynTree::toEigen(m_upperBound).segment(2, 2) = iDynTree::toEigen(zmpPositionNominal) + iDynTree::toEigen(zmpPositionTollerance);
    // iDynTree::toEigen(m_lowerBound).segment(2, 2) = iDynTree::toEigen(zmpPositionNominal) - iDynTree::toEigen(zmpPositionTollerance);

    m_upperBound(6) = std::exp((stepDuration + stepDurationTollerance) * omega);
    m_lowerBound(6) = std::exp((stepDuration - std::min(stepDurationTollerance, remainingSingleSupportDuration)) * omega);
m_upperBound(7)=qpOASES::INFTY;


m_lowerBound(7)=0;

    // std::cerr << "u = [" << m_upperBound << "];";
    // std::cerr << "l = [" << m_lowerBound << "];";

    // if (m_QPSolver->isInitialized())
    // {
    //     if (!m_QPSolver->updateBounds(iDynTree::toEigen(m_lowerBound),iDynTree::toEigen(m_upperBound)))
    //     {
    //         yError()<<"[setBoundsVectorOfConstraints]Unable to update the bounds of constraints in QP problem in step adaptation";
    //         return false;
    //     }
    // }
    // else
    // {
    //     if (!m_QPSolver->data()->setLowerBound(iDynTree::toEigen(m_lowerBound)))
    //     {
    //         yError()<<"[setBoundsVectorOfConstraints] Unable to set the lower bounds of constraints in QP problem in step adaptation ";
    //         return false;
    //     }
    //     if (!m_QPSolver->data()->setUpperBound(iDynTree::toEigen(m_upperBound)))
    //     {
    //         yError()<<"[setBoundsVectorOfConstraints] Unable to set the  upper bounds of constraints in QP problem in step adaptation";
    //         return false;
    //     }
    // }
    return true;
}

bool QPSolver::isInitialized()
{
    return m_QPSolver->isInitialized();
}

bool QPSolver::initialize()
{
    return m_QPSolver->initSolver();
}

bool QPSolver::solve()
{
    // if (!m_QPSolver->isInitialized())
    // {
    //     yError()<<"[solve in QPSolver.cpp] The solver has not initilialized";
    //     return false;
    // }
    // if(!m_QPSolver->solve())
    // {
    //     yError() << "[QPSolver::solve] Unable to solve the problem";
    //     return false;
    // }


        // convert sparse matrix into a dense matrix
    MatrixXd constraintMatrix = MatrixXd(iDynTree::toEigen(m_constraintsMatrix));
    MatrixXd hessianMatrix = MatrixXd(iDynTree::toEigen(m_hessianMatrix));

    int nWSR = 100;
    if(!m_isFirstTime)
    {
        if(m_QPSolver_qpOASES->hotstart(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                                 nullptr, nullptr,
                                 m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }
    }
    else
    {
        if(m_QPSolver_qpOASES->init(hessianMatrix.data(), m_gradient.data(), constraintMatrix.data(),
                             nullptr, nullptr,
                             m_lowerBound.data(), m_upperBound.data(), nWSR, 0)
           != qpOASES::SUCCESSFUL_RETURN)
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }

        m_isFirstTime = false;
    }

    m_QPSolver_qpOASES->getPrimalSolution(m_solution.data());

    //iDynTree::toEigen(m_solution) = m_QPSolver->getSolution();

    return true;
}

iDynTree::VectorDynSize QPSolver::getSolution()
{
    return m_solution;
}
