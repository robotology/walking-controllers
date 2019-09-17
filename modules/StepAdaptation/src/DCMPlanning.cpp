
// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <DCMPlanning.hpp>
#include <Utils.hpp>

DCMPlanning::DCMPlanning(const double omega){
    m_omega=omega;
};



bool DCMPlanning::solveCoMDynamics(const double& time, const iDynTree::Vector2& initialCoM, const iDynTree::Vector2& DCM){
    //  CoM=(InitialCoM()-DCM())*exp(-m_omega*time)+_DCM(_DCM.rows()-1,1);
    iDynTree::Vector2 CoMOffset;
    iDynTree::toEigen(CoMOffset)=iDynTree::toEigen(initialCoM)-iDynTree::toEigen(DCM);
    iDynTree::toEigen(m_CoMPosition)=iDynTree::toEigen(CoMOffset)*exp(-m_omega*time)+iDynTree::toEigen(DCM);

    return true;
}



bool DCMPlanning::solveDCMDynamics(const double& time){
    iDynTree::Vector2 CoeffA;
    iDynTree::Vector2 CoeffB;
    double landa=exp(-m_omega*m_nominalStepTiming);
    iDynTree::toEigen( CoeffA)=(iDynTree::toEigen(m_finalZMPPosition)-iDynTree::toEigen(m_initialZMPPosition))/(landa-1);
    iDynTree::toEigen( CoeffB)=(landa*iDynTree::toEigen(m_initialZMPPosition)-iDynTree::toEigen(m_finalZMPPosition))/(landa-1);

    iDynTree::Vector2 CoeffC;

    iDynTree::toEigen(CoeffC)=iDynTree::toEigen(m_initialDCMPosition)-iDynTree::toEigen(CoeffA)/2-iDynTree::toEigen(CoeffB);
    iDynTree::toEigen(m_DCMPosition)=(iDynTree::toEigen(CoeffA)/2)*exp(-m_omega*time)+iDynTree::toEigen(CoeffB)+(iDynTree::toEigen(CoeffC))*exp(m_omega*time);

    return true;
}


bool DCMPlanning::getCoMPosition(iDynTree::Vector2 & CoMPosition){
    CoMPosition=m_CoMPosition;

    return true;
}

bool DCMPlanning::getDCMPosition(iDynTree::Vector2 & DCMPosition){
    DCMPosition=m_DCMPosition;
    return true;
}

bool DCMPlanning::setNominalStepParameters(const double& stepLength,const double& stepWidth,const double & stepTiming){
    m_nominalStepLength= stepLength;
    m_nominalStepTiming= stepTiming;
    m_nominalStepWidth= stepWidth;

    return true;
}

bool DCMPlanning::setBoundryZMPPosition(const iDynTree::Vector2 & initialZMPPosition,const iDynTree::Vector2 & finalZMPPosition){
    m_initialZMPPosition=initialZMPPosition;
    m_finalZMPPosition=finalZMPPosition;
    return true;
}

bool DCMPlanning::setInitialDCMPosition(const iDynTree::Vector2 & initialDCMPosition){
    m_initialDCMPosition=initialDCMPosition;

    return true;
}
