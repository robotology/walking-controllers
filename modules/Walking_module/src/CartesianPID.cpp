/**
 * @file CartesianPID.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <CartesianPID.hpp>
#include <Utils.hpp>

const iDynTree::Vector3& CartesianPID::getControllerOutput() const
{
    return m_controllerOutput;
}

void RotationalPID::setGains(const double& c0, const double& c1, const double& c2)
{
    m_c0 = c0;
    m_c1 = c1;
    m_c2 = c2;
}

void RotationalPID::setDesiredTrajectory(const iDynTree::Vector3 &desiredAcceleration,
                                         const iDynTree::Vector3 &desiredVelocity,
                                         const iDynTree::Rotation &desiredOrientation)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredOrientation = desiredOrientation;
}

void RotationalPID::setFeedback(const iDynTree::Vector3 &velocity,
                                const iDynTree::Rotation &orientation)
{
    m_velocity = velocity;
    m_orientation = orientation;
}

void RotationalPID::evaluateControl()
{
    iDynTree::Matrix3x3 errorAttitude;
    Eigen::Vector3d error;
    errorAttitude = iDynTreeHelper::Rotation::skewSymmetric(m_orientation * m_desiredOrientation.inverse());
    error = iDynTree::unskew(iDynTree::toEigen(errorAttitude));

    Eigen::Vector3d dotError;
    iDynTree::Matrix3x3 dotErrorAttitude;

    Eigen::Matrix3d skewAngularVelocity = iDynTree::skew(iDynTree::toEigen(m_velocity));
    Eigen::Matrix3d skewDesiredAngularVelocity = iDynTree::skew(iDynTree::toEigen(m_desiredVelocity));

    iDynTree::toEigen(dotErrorAttitude) = skewAngularVelocity *
        iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse())
        -iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse()) *
        skewDesiredAngularVelocity;

    dotError = iDynTree::unskew(iDynTree::toEigen(iDynTreeHelper::Rotation::skewSymmetric(dotErrorAttitude)));

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_desiredAcceleration)
                - m_c0 * dotError
        - m_c1 * (iDynTree::toEigen(m_velocity) - iDynTree::toEigen(m_desiredVelocity))
        - m_c2 * error;
}

void LinearPID::setGains(const double& kp, const double& kd)
{
    for(int i = 0; i < 3; i++)
    {
        m_kp(i) = kp;
        m_kd(i) = kd;
    }
}

void LinearPID::setGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd)
{
    m_kp = kp;
    m_kd = kd;
}

void LinearPID::setDesiredTrajectory(const iDynTree::Vector3 &desiredAcceleration,
                                     const iDynTree::Vector3 &desiredVelocity,
                                     const iDynTree::Vector3 &desiredPosition)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredPosition = desiredPosition;
}

void LinearPID::setFeedback(const iDynTree::Vector3 &velocity,
                            const iDynTree::Vector3 &position)
{
    m_velocity = velocity;
    m_position = position;
}

void LinearPID::evaluateControl()
{
    iDynTree::toEigen(m_error) = iDynTree::toEigen(m_desiredPosition) - iDynTree::toEigen(m_position);
    iDynTree::toEigen(m_dotError) = iDynTree::toEigen(m_desiredVelocity) - iDynTree::toEigen(m_velocity);

    iDynTree::toEigen(m_controllerOutput) = iDynTree::toEigen(m_desiredAcceleration)
        + iDynTree::toEigen(m_kp).asDiagonal() * iDynTree::toEigen(m_error)
        + iDynTree::toEigen(m_kd).asDiagonal() * iDynTree::toEigen(m_dotError);
}
