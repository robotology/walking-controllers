/**
 * @file DCMEstimator.cpp
 * @authors
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <math.h>

// YARP
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

//iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

#include <DCMEstimator.hpp>
#include <Utils.hpp>
#include <iDynTree/yarp/YARPEigenConversions.h>
#include <iDynTree/Model/Model.h>


bool DCMEstimator::initialize(const yarp::os::Searchable& config,const iDynTree::Model modelLoader)
{
    if(config.isNull())
    {
        yError() << "[initialize] Empty configuration for ZMP controller.";
        return false;
    }

    double comHeight;
    if(!YarpHelper::getNumberFromSearchable(config, "com_height", comHeight))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }
    double gravityAcceleration = config.check("gravity_acceleration", yarp::os::Value(9.81)).asDouble();

    m_omega = sqrt(gravityAcceleration / comHeight);
    m_mass=modelLoader.getTotalMass();

    // set the sampling time
    double samplingTime;
    if(!YarpHelper::getNumberFromSearchable(config, "sampling_time", samplingTime))
    {
        yError() << "[initialize] Unable to get a double from a searchable.";
        return false;
    }

    yarp::sig::Vector buffer;
    buffer.resize(2, 0.0);

    // instantiate Integrator object
    m_dcmVelocityIntegrator = std::make_unique<iCub::ctrl::Integrator>(samplingTime, buffer);

    return true;
}

bool DCMEstimator::integrateDCMVelocity(iDynTree::Vector2 zmp, iDynTree::Vector2 dcm){

    if(m_dcmVelocityIntegrator == nullptr)
    {
        yError() << "[integrateDCMVelocity] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }
    yarp::sig::Vector dcmVelocityYarp(2);
    iDynTree::toEigen(dcmVelocityYarp) = m_omega * (iDynTree::toEigen(dcm)-iDynTree::toEigen(zmp));

    // integrate DCM velocities
    yarp::sig::Vector  dcmPositionYarp;
    dcmPositionYarp = m_dcmVelocityIntegrator->integrate(dcmVelocityYarp);

    // convert YARP vector into iDynTree vector
    iDynTree::toiDynTree(dcmVelocityYarp, m_dcmVelocity);
    iDynTree::toiDynTree(dcmPositionYarp, m_dcmEstimatedPosition);

    return true;

}

bool DCMEstimator::integrateDCMVelocity(iDynTree::AngVelocity omegaIMU, iDynTree::LinAcceleration accelerationIMU, iDynTree::Position baseToCoMPosition, iDynTree::LinVelocity CoMVelocity3d){
    if(m_dcmVelocityIntegrator == nullptr)
    {
        yError() << "[integrateDCMVelocity] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }
iDynTree::Vector3 comAcceleration3d;
iDynTree::Vector3 tempAcceleration;
iDynTree::toEigen(tempAcceleration)=iDynTree::toEigen(omegaIMU).cross(iDynTree::toEigen(baseToCoMPosition));
iDynTree::toEigen(comAcceleration3d)=iDynTree::toEigen(accelerationIMU)+iDynTree::toEigen(omegaIMU).cross(iDynTree::toEigen(tempAcceleration));


iDynTree::Vector2 comAcceleration;
comAcceleration(0)=comAcceleration3d(0);
comAcceleration(1)=comAcceleration3d(1);

iDynTree::Vector2 CoMVelocity;
CoMVelocity(0)=CoMVelocity3d(0);
CoMVelocity(1)=CoMVelocity3d(1);

yarp::sig::Vector dcmVelocityYarp(2);
iDynTree::toEigen(dcmVelocityYarp) =(iDynTree::toEigen(CoMVelocity)+(iDynTree::toEigen(comAcceleration)/m_omega));

// integrate DCM velocities
yarp::sig::Vector  dcmPositionYarp;
dcmPositionYarp = m_dcmVelocityIntegrator->integrate(dcmVelocityYarp);

// convert YARP vector into iDynTree vector
iDynTree::toiDynTree(dcmVelocityYarp, m_dcmVelocity);
iDynTree::toiDynTree(dcmPositionYarp, m_dcmEstimatedPosition);

return true;
}




bool DCMEstimator::integrateDCMVelocity(iDynTree::LinearForceVector3 contactForce3d, iDynTree::LinVelocity CoMVelocity3d){
    if(m_dcmVelocityIntegrator == nullptr)
    {
        yError() << "[integrateDCMVelocity] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }
    iDynTree::Vector2 contactForce2d;
    contactForce2d(0)=contactForce3d(0);
    contactForce2d(1)=contactForce3d(1);

    iDynTree::Vector2 CoMVelocity;
    CoMVelocity(0)=CoMVelocity3d(0);
    CoMVelocity(1)=CoMVelocity3d(1);

    iDynTree::Vector2 comAcceleration;
    iDynTree::toEigen(comAcceleration)=iDynTree::toEigen (contactForce2d)/m_mass;
    yarp::sig::Vector dcmVelocityYarp(2);

    iDynTree::toEigen(dcmVelocityYarp) =(iDynTree::toEigen(CoMVelocity)+(iDynTree::toEigen(comAcceleration)/m_omega));

    // integrate DCM velocities
    yarp::sig::Vector  dcmPositionYarp;
    dcmPositionYarp = m_dcmVelocityIntegrator->integrate(dcmVelocityYarp);

    // convert YARP vector into iDynTree vector
    iDynTree::toiDynTree(dcmVelocityYarp, m_dcmVelocity);
    iDynTree::toiDynTree(dcmPositionYarp, m_dcmEstimatedPosition);

    return true;
}


//void DCMEstimator::setInput(const iDynTree::Vector2& input)
//{
//    m_dcmPosition = input;
//}

//bool DCMEstimator::integrateDCMVelocity()
//{
//    if(m_comIntegrator == nullptr)
//    {
//        yError() << "[integrateModel] The dcm integrator object is not ready. "
//                 << "Please call initialize method.";
//        return false;
//    }

//    // evaluate the velocity of the CoM
//    yarp::sig::Vector comVelocityYarp(2);
//    iDynTree::toEigen(comVelocityYarp) = -m_omega * (iDynTree::toEigen(m_comPosition) -
//                                                     iDynTree::toEigen(m_dcmPosition));

//    // integrate velocities
//    yarp::sig::Vector comPositionYarp(2);
//    comPositionYarp = m_comIntegrator->integrate(comVelocityYarp);

//    // convert YARP vector into iDynTree vector
//    iDynTree::toiDynTree(comVelocityYarp, m_comVelocity);
//    iDynTree::toiDynTree(comPositionYarp, m_comPosition);

//    return true;
//}

const iDynTree::Vector2& DCMEstimator::getDCMPosition() const
{
    return m_dcmEstimatedPosition;
}

//const iDynTree::Vector2& DCMEstimator::getCoMVelocity() const
//{
//    return m_comVelocity;
//}

bool DCMEstimator::reset(const iDynTree::Vector2& initialValue)
{
    if(m_dcmVelocityIntegrator == nullptr)
    {
        yError() << "[reset] The dcm integrator object is not ready. "
                 << "Please call initialize method.";
        return false;
    }

    yarp::sig::Vector buffer;
    iDynTree::toYarp(initialValue, buffer);

    m_dcmVelocityIntegrator->reset(buffer);
    m_dcmEstimatedPosition = initialValue;
    return true;
}
