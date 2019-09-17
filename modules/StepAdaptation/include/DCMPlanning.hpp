#ifndef  DCM_PLANNING_HPP
#define  DCM_PLANNING_HPP
// std
#include <deque>

// iDynTree
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

// osqp-eigen
#include <OsqpEigen/OsqpEigen.h>

#include <qpOASES.hpp>
#include <Utils.hpp>



class DCMPlanning{
private:
double m_nominalStepLength;
double m_omega;

double m_nominalStepTiming;
double m_nominalStepWidth;
iDynTree::Vector2 m_DCMPosition;
iDynTree::Vector2 m_DCMVelocity;
iDynTree::Vector2 m_CoMPosition;
iDynTree::Vector2 m_CoMVelocity;
iDynTree::Vector2 m_initialZMPPosition;
iDynTree::Vector2 m_finalZMPPosition;
iDynTree::Vector2 m_initialDCMPosition;
iDynTree::Vector2 m_finalDCMPosition;



public:
DCMPlanning(const double omega);
bool getNominalStepParameters(double& stepLength, double& stepWidth, double& stepTiming);
bool setNominalStepParameters(const double& stepLength,const double& stepWidth,const double & stepTiming);
bool solveDCMDynamics(const double &time);
bool solveCoMDynamics(const double& time, const iDynTree::Vector2& initialCoM, const iDynTree::Vector2& DCM);
bool getDCMPosition(iDynTree::Vector2 & DCMPosition);
bool getDCMVelocity(iDynTree::Vector2 & DCMVelocity);
bool getCoMPosition(iDynTree::Vector2 & CoMPosition);
bool getCoMVelocity(iDynTree::Vector2 & CoMVelocity);
bool setBoundryZMPPosition(const iDynTree::Vector2 & initialZMPPosition,const iDynTree::Vector2 & finalZMPPosition);
bool setInitialDCMPosition(const iDynTree::Vector2 & initialDCMPosition);
};

#endif
