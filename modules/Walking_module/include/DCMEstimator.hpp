/**
 * @file LinearInvertedPendulumModel.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef DCM_ESTIMATOR_HPP
#define DCM_ESTIMATOR_HPP

// std
#include <memory>

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/sig/Vector.h>

// iCub-ctrl
#include <iCub/ctrl/pids.h>

//iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

/**
 * StableDCMModel linear inverted pendulum model.
 */
class DCMEstimator
{
    double m_omega; /**< Inverted time constant of the 3D-LIPM. */

    std::unique_ptr<iCub::ctrl::Integrator> m_dcmVelocityIntegrator{nullptr}; /**< CoM integrator object. */
    iDynTree::Vector2 m_dcmEstimatedPosition; /**< Position of the DCM. */
    iDynTree::Vector2 m_dcmPosition; /**< Position of the DCM. */
    iDynTree::Vector2 m_comPosition; /**< Position of the CoM. */
    iDynTree::Vector2 m_dcmVelocity; /**< Velocity of the dcm. */

public:

bool integrateDCMVelocity(iDynTree::Vector2 zmp,iDynTree::Vector2 dcm);

bool integrateDCMVelocity(iDynTree::AngVelocity omegaIMU, iDynTree::LinAcceleration accelerationIMU, iDynTree::Position baseToCoMPosition, iDynTree::LinVelocity CoMVelocity3d);

bool integrateDCMVelocity(iDynTree::LinearForceVector3 contactForce3d, iDynTree::LinVelocity CoMVelocity3d);


    /**
     * Initialize the 3D-LIPM.
     * @param config config of the 3D-LIPM;
     * @return true on success, false otherwise.
     */
    bool initialize(const yarp::os::Searchable& config);
    /**
     * Set the controlled input.
     * @param controlledInput of the 3D-LIPM (i.e. Position of the ZMP).
     */
    void setInput(const iDynTree::Vector2& input);

    /**
     * Integrate the model.
     * @return true on success, false otherwise.
     */
    bool integrateModel();

    /**
     * Get the position of the CoM.
     * @return position of the CoM.
     */
    const iDynTree::Vector2& getDCMPosition() const;

    /**
     * Get the velocity of the CoM.
     * @return velocity of the CoM.
     */
    const iDynTree::Vector2& getCoMVelocity() const;

    /**
     * Reset the Model
     * @param initialValue initial position of the CoM
     * @return true/false in case of success/failure
     */
    bool reset(const iDynTree::Vector2& initialValue);
};

#endif
