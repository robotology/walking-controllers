/**
 * @file DCMSimpleEstimator.hpp
 * @authors Milad Shafiee <milad.shafiee@iit.it>
 * @copyright 2020 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2020
 */

#ifndef WALKING_CONTROLLERS_DCM_SIMPLE_ESTIMATOR_H
#define WALKING_CONTROLLERS_DCM_SIMPLE_ESTIMATOR_H

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

//iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/LinearMotionVector3.h>


namespace WalkingControllers
{
/**
 * The theory related to this simple estimator has been discussed in the following github issue:
 *https://github.com/robotology/walking-controllers/issues/61
*/
    class DCMSimpleEstimator
    {
        double m_omega; /**< Inverted time constant of the 3D-LIPM. */
        double m_mass; /**< Mass of the robot. */
        iDynTree::Vector2 m_dcmEstimatedPosition; /**< Position of the estimated DCM. */
        iDynTree::Vector2 m_dcmPosition; /**< Position of the DCM. */
        iDynTree::Vector2 m_dcmVelocity; /**< Velocity of the dcm. */

    public:

        /**
         * Config the DCMEstimator.
         * @param config config of the simple DCM estimator;
         * @return true on success, false otherwise.
         */
        bool configure(const yarp::os::Searchable& config);

        /**
         * Get the position of the DCM.
         * @return position of the DCM.
         */
        const iDynTree::Vector2& getDCMPosition() const;

        /**
         * run the pendulum estimator
         * @param footOrientation the orientation of stance foot.
         * @param zmp the vector of zmp position with respect to the inertial frame.
         * @param com the vector of com position with respect to the inertial frame.
         * @param CoMVelocity3d the vector of com velocity that is simple time derivative of the com position.
         * @return true/false in case of success/failure
         */
        bool update(const iDynTree::Rotation& footOrientation,const iDynTree::Vector3& zmp,const iDynTree::Vector3& com,const iDynTree::LinVelocity& CoMVelocity3d);
    };
};

#endif
