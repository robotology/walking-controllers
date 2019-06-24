/**
 * @file WalkingAdmittanceController.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <OsqpEigen/OsqpEigen.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingConstraint.hpp>
#include <WalkingAdmittanceController.hpp>


class WalkingAdmittanceController::Implementation
{
    friend WalkingAdmittanceController;

    int m_actuatedDoFs;

    iDynTree::Wrench m_desiredLeftWrench;
    iDynTree::Wrench m_desiredRightWrench;

    iDynTree::MatrixDynSize m_leftFootJacobian;
    iDynTree::MatrixDynSize m_rightFootJacobian;
    iDynTree::MatrixDynSize m_neckJacobian;

    iDynTree::VectorDynSize m_leftFootBiasAcceleration;
    iDynTree::VectorDynSize m_rightFootBiasAcceleration;
    iDynTree::VectorDynSize m_neckBiasAcceleration;

    iDynTree::Rotation m_additionalRotation;

    iDynTree::MatrixDynSize m_massMatrix;
    iDynTree::VectorDynSize m_generalizedBiasForces;

    // QP Quantities
    std::unique_ptr<OsqpEigen::Solver> m_optimizer{nullptr};
    Eigen::SparseMatrix<double> m_hessianEigen;
    iDynTree::VectorDynSize m_gradient;
    Eigen::SparseMatrix<double>  m_constraintMatrix;
    iDynTree::VectorDynSize m_upperBound;
    iDynTree::VectorDynSize m_lowerBound;
    int m_numberOfConstraints;
    int m_numberOfVariables;
    iDynTree::VectorDynSize m_desiredJointTorqueOutput;
    iDynTree::VectorDynSize m_desiredJointAccelerationOutput;

    std::map<std::string, std::shared_ptr<Constraint>> m_constraints;
    std::map<std::string, std::shared_ptr<CostFunctionElement>> m_costFunctions;

    std::map<std::string, Eigen::SparseMatrix<double>> m_hessianMatrices;
    std::map<std::string, iDynTree::VectorDynSize> m_gradientVectors;

    // TODO MOVE ME
    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;
    iDynTree::VectorDynSize m_desiredJointAcceleration;
    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;

    void instantiateTorqueCostFunction(const iDynTree::VectorDynSize &weight, const int& numberOfDofs)
    {
        auto ptr = std::make_shared<InputRegularizationTerm>(numberOfDofs);
        ptr->setSubMatricesStartingPosition(numberOfDofs + 6, numberOfDofs + 6);
        ptr->setWeight(weight);

        m_costFunctions.insert({"torque_costFunction", ptr});
        m_hessianMatrices["torque_costFunction"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["torque_costFunction"].resize(m_numberOfVariables);
        m_gradientVectors["torque_costFunction"].zero();

        return;
    }

    void instantiateJointCostFunction(const iDynTree::VectorDynSize &weight, const iDynTree::VectorDynSize &kp,
                                      const iDynTree::VectorDynSize &kd, const int& numberOfDofs)
    {
        auto ptr = std::make_shared<JointRegularizationCostFunction>(numberOfDofs);
        ptr->setSubMatricesStartingPosition(6, 6);
        ptr->setWeight(weight);

        ptr->setProportionalGains(kp);
        ptr->setDerivativeGains(kd);

        ptr->setDesiredJointPosition(m_desiredJointPosition);
        ptr->setDesiredJointVelocity(m_desiredJointVelocity);
        ptr->setDesiredJointAcceleration(m_desiredJointAcceleration);
        ptr->setJointPosition(m_jointPosition);
        ptr->setJointVelocity(m_jointVelocity);

        m_costFunctions.insert({"joint_regularization_costFunction", ptr});
        m_hessianMatrices["joint_regularization_costFunction"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["joint_regularization_costFunction"].resize(m_numberOfVariables);
        m_gradientVectors["joint_regularization_costFunction"].zero();

        return;
    }

    void instantiateNeckCostFunction(const double& c0, const double& kpAngular, const double& kdAngular, const iDynTree::VectorDynSize weight)
    {
        std::shared_ptr<CartesianCostFunction> ptr;
        ptr = std::make_shared<CartesianCostFunction>(CartesianElement::Type::ORIENTATION);
        ptr->setControlMode(CartesianElement::ControlMode::POSITION);
        ptr->setSubMatricesStartingPosition(0, 0);

        ptr->setWeight(weight);
        ptr->setBiasAcceleration(m_neckBiasAcceleration);
        ptr->setRoboticJacobian(m_neckJacobian);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        m_costFunctions.insert({"neck", ptr});
        m_hessianMatrices["neck"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["neck"].resize(m_numberOfVariables);
        m_gradientVectors["neck"].zero();
    }

    void instantiateSystemDynamicsConstraint(const int& numberOfDofs)
    {
        auto ptr = std::make_shared<SystemDynamicConstraint>(numberOfDofs);
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

        // TODO desired or measured?
        ptr->setContactWrench(m_desiredLeftWrench);
        ptr->setContactWrench(m_desiredRightWrench);

        ptr->setContactWrenchJacobian(m_leftFootJacobian);
        ptr->setContactWrenchJacobian(m_rightFootJacobian);

        ptr->setMassMatrix(m_massMatrix);
        ptr->setGeneralizedBiasForces(m_generalizedBiasForces);

        // add to the constraints
        m_constraints.insert({"system_dynamics", ptr});

        m_numberOfConstraints += ptr->getNumberOfConstraints();

        return;
    }

    void instantiateFeetConstraint(const iDynTree::Vector3& kpLinear, const iDynTree::Vector3& kdLinear,
                                   const double& c0, const double& kpAngular, const double& kdAngular,
                                   const iDynTree::Vector3& kpForce, const double& kpCouple)
    {
        std::shared_ptr<CartesianConstraint> ptr;

        // Left foot
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        ptr->forceController()->setGains(kpForce);
        ptr->coupleController()->setGains(kpCouple);

        ptr->setRoboticJacobian(m_leftFootJacobian);
        ptr->setBiasAcceleration(m_leftFootBiasAcceleration);

        // we suppose that the robot will start in double support phase
        ptr->setControlMode(CartesianElement::ControlMode::FORCE);

        m_constraints.insert({"left_foot", ptr});
        m_numberOfConstraints += ptr->getNumberOfConstraints();

        // Right foot
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        ptr->forceController()->setGains(kpForce);
        ptr->coupleController()->setGains(kpCouple);

        ptr->setRoboticJacobian(m_rightFootJacobian);
        ptr->setBiasAcceleration(m_rightFootBiasAcceleration);

        // we suppose that the robot will start in double support phase
        ptr->setControlMode(CartesianElement::ControlMode::FORCE);

        m_constraints.insert({"right_foot", ptr});
        m_numberOfConstraints += ptr->getNumberOfConstraints();
        return;
    }

    void setFootState(const iDynTree::Transform& footToWorldTransform, const iDynTree::Twist& footTwist, const iDynTree::Wrench& footWrench, bool footInContact, std::shared_ptr<CartesianConstraint>& foot)
    {
        // kinematics
        foot->positionController()->setFeedback(footTwist.getLinearVec3(),
                                                footToWorldTransform.getPosition());
        foot->orientationController()->setFeedback(footTwist.getAngularVec3(),
                                                   footToWorldTransform.getRotation());

        // statics
        foot->forceController()->setFeedback(footWrench.getLinearVec3());
        foot->coupleController()->setFeedback(footWrench.getAngularVec3());

        if(footInContact)
            foot->setControlMode(CartesianElement::ControlMode::FORCE);
        else
            foot->setControlMode(CartesianElement::ControlMode::POSITION);

        return;
    }

    bool setFeetState(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::Wrench& leftFootWrench, bool leftInContact,
                      const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::Wrench& rightFootWrench, bool rightInContact)
    {
        auto constraint = m_constraints.find("left_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setFeetState] unable to find the left foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setFootState(leftFootToWorldTransform, leftFootTwist, leftFootWrench, leftInContact, ptr);

        constraint = m_constraints.find("right_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setFeetState] unable to find the right foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setFootState(rightFootToWorldTransform, rightFootTwist, rightFootWrench, rightInContact, ptr);

        return true;
    }

    void setDesiredFootTrajectory(const iDynTree::Transform& footToWorldTransform, const iDynTree::Twist& footTwist, const iDynTree::SpatialAcc& footAcceleration,
                                  const iDynTree::Wrench& footWrench, std::shared_ptr<CartesianConstraint>& foot)
    {
        // Kinematcs
        foot->positionController()->setDesiredTrajectory(footAcceleration.getLinearVec3(), footTwist.getLinearVec3(), footToWorldTransform.getPosition());
        foot->orientationController()->setDesiredTrajectory(footAcceleration.getAngularVec3(), footTwist.getAngularVec3(), footToWorldTransform.getRotation());

        // static
        foot->forceController()->setDesiredForce(footWrench.getLinearVec3());
        foot->coupleController()->setDesiredForce(footWrench.getAngularVec3());

        return;
    }

    bool setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::SpatialAcc& leftFootAcceleration,
                                  const iDynTree::Wrench& leftFootWrench,
                                  const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::SpatialAcc& rightFootAcceleration,
                                  const iDynTree::Wrench& rightFootWrench)
    {

        m_desiredLeftWrench = leftFootWrench;
        m_desiredRightWrench = rightFootWrench;

        auto constraint = m_constraints.find("left_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setDesiredFootTrajectory(leftFootToWorldTransform, leftFootTwist, leftFootAcceleration, leftFootWrench, ptr);

        constraint = m_constraints.find("right_foot");
        if(constraint == m_constraints.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the stance foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setDesiredFootTrajectory(rightFootToWorldTransform, rightFootTwist, rightFootAcceleration, rightFootWrench, ptr);

        return true;
    }

    bool setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation)
    {
        iDynTree::Vector3 dummy;
        dummy.zero();
        auto cost = m_costFunctions.find("neck");
        if(cost == m_costFunctions.end())
        {
            yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianCostFunction>(cost->second);
        ptr->orientationController()->setDesiredTrajectory(dummy, dummy,
                                                           desiredNeckOrientation * m_additionalRotation);
        return true;
    }

    bool setNeckState(const iDynTree::Rotation& neckOrientation, const iDynTree::Twist& neckVelocity)
    {
        auto cost = m_costFunctions.find("neck");
        if(cost == m_costFunctions.end())
        {
            yError() << "[setNeckState] unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianCostFunction>(cost->second);
        ptr->orientationController()->setFeedback(neckVelocity.getAngularVec3(), neckOrientation);

        return true;
    }

    bool setHessianMatrix()
    {
        std::string key;
        Eigen::SparseMatrix<double> hessianEigen(m_numberOfVariables, m_numberOfVariables);
        for(const auto& element: m_costFunctions)
        {
            key = element.first;

            element.second->evaluateHessian(m_hessianMatrices.at(key));
            hessianEigen += m_hessianMatrices.at(key);
        }

        if(m_optimizer->isInitialized())
        {
            if(!m_optimizer->updateHessianMatrix(hessianEigen))
            {
                yError() << "[setHessianMatrix] Unable to update the hessian matrix.";
                return false;
            }
        }
        else
        {
            if(!m_optimizer->data()->setHessianMatrix(hessianEigen))
            {
                yError() << "[setHessianMatrix] Unable to set first time the hessian matrix.";
                return false;
            }
        }

        return true;
    }

    bool setGradientVector()
    {
        std::string key;
        m_gradient.zero();
        for(const auto& element: m_costFunctions)
        {
            key = element.first;
            element.second->evaluateGradient(m_gradientVectors.at(key));
            iDynTree::toEigen(m_gradient) += iDynTree::toEigen(m_gradientVectors.at(key));
        }

        if(m_optimizer->isInitialized())
        {
            if(!m_optimizer->updateGradient(iDynTree::toEigen(m_gradient)))
            {
                yError() << "[setGradient] Unable to update the gradient.";
                return false;
            }
        }
        else
        {
            if(!m_optimizer->data()->setGradient(iDynTree::toEigen(m_gradient)))
            {
                yError() << "[setGradient] Unable to set first time the gradient.";
                return false;
            }
        }

        return true;
    }

    bool setLinearConstraintMatrix()
    {
        for(const auto& constraint: m_constraints)
            constraint.second->evaluateJacobian(m_constraintMatrix);

        if(m_optimizer->isInitialized())
        {
            if(!m_optimizer->updateLinearConstraintsMatrix(m_constraintMatrix))
            {
                yError() << "[setLinearConstraintsMatrix] Unable to update the constraints matrix.";
                return false;
            }
        }
        else
        {
            if(!m_optimizer->data()->setLinearConstraintsMatrix(m_constraintMatrix))
            {
                yError() << "[setLinearConstraintsMatrix] Unable to set the constraints matrix.";
                return false;
            }
        }

        return true;
    }

    bool setBounds()
    {
        auto lowerBound(iDynTree::toEigen(m_lowerBound));
        auto upperBound(iDynTree::toEigen(m_upperBound));

        for(const auto& constraint: m_constraints)
            constraint.second->evaluateBounds(m_upperBound, m_lowerBound);

        if(m_optimizer->isInitialized())
        {
            if(!m_optimizer->updateBounds(lowerBound, upperBound))
            {
                yError() << "[setBounds] Unable to update the bounds.";
                return false;
            }
        }
        else
        {
            if(!m_optimizer->data()->setLowerBound(lowerBound))
            {
                yError() << "[setBounds] Unable to set the first time the lower bound.";
                return false;
            }

            if(!m_optimizer->data()->setUpperBound(upperBound))
            {
                yError() << "[setBounds] Unable to set the first time the upper bound.";
                return false;
            }
        }
        return true;
    }

    bool solve()
    {

        if(!setHessianMatrix())
        {
            yError() << "[solve] Unable to set the hessian matrix.";
            return false;
        }

        if(!setGradientVector())
        {
            yError() << "[solve] Unable to set the gradient vector matrix.";
            return false;
        }

        if(!setLinearConstraintMatrix())
        {
            yError() << "[solve] Unable to set the linear constraint matrix.";
            return false;
        }

        if(!setBounds())
        {
            yError() << "[solve] Unable to set the bounds.";
            return false;
        }

        if(!m_optimizer->isInitialized())
        {
            if(!m_optimizer->initSolver())
            {
                yError() << "[solve] Unable to initialize the solver";
                return false;
            }
        }

        if(!m_optimizer->solve())
        {
            yError() << "[solve] Unable to solve the problem.";
            return false;
        }

        iDynTree::toEigen(m_desiredJointTorqueOutput) = m_optimizer->getSolution().segment(m_actuatedDoFs + 6, m_actuatedDoFs);
        iDynTree::toEigen(m_desiredJointAccelerationOutput) = m_optimizer->getSolution().segment(6, m_actuatedDoFs);

        return true;
    }
};

WalkingAdmittanceController::WalkingAdmittanceController():
    m_pimpl(new WalkingAdmittanceController::Implementation)
{
    assert(m_pimpl);
}

WalkingAdmittanceController::~WalkingAdmittanceController()
{}

bool WalkingAdmittanceController::initialize(yarp::os::Searchable &config, const int& actuatedDoFs)
{
    m_pimpl->m_numberOfConstraints = 0;
    m_pimpl->m_numberOfVariables = 6 + 2 * actuatedDoFs;

    m_pimpl->m_actuatedDoFs = actuatedDoFs;

    // instantiate cost and constraints
    {
        yarp::os::Bottle& option = config.findGroup("TORQUE");
        iDynTree::VectorDynSize weight(actuatedDoFs);

        if(!YarpHelper::getVectorFromSearchable(option, "weight", weight))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        m_pimpl->instantiateTorqueCostFunction(weight, actuatedDoFs);
    }

    {
        yarp::os::Bottle& option = config.findGroup("JOINT_REGULARIZATION");
        iDynTree::VectorDynSize weight(actuatedDoFs);

        if(!YarpHelper::getVectorFromSearchable(option, "weight", weight))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        iDynTree::VectorDynSize kp(actuatedDoFs);
        if(!YarpHelper::getVectorFromSearchable(option, "kp", kp))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        iDynTree::VectorDynSize kd(actuatedDoFs);
        if(option.check("use_default_kd", yarp::os::Value("False")).asBool())
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }
            iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
        }
        else
        {
            if(!YarpHelper::getVectorFromSearchable(option, "kd", kd))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
                return false;
            }
        }

        m_pimpl->instantiateJointCostFunction(weight, kp, kd, actuatedDoFs);
    }

    {
        m_pimpl->instantiateSystemDynamicsConstraint(actuatedDoFs);
    }

    {
        yarp::os::Bottle& option = config.findGroup("FEET");
        iDynTree::Vector3 kpLinear, kdLinear;

        if(!YarpHelper::getVectorFromSearchable(option, "kpLinear", kpLinear))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        bool useDefaultKd = option.check("use_default_kd", yarp::os::Value("False")).asBool();
        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }
            iDynTree::toEigen(kdLinear) = 2 / scaling * iDynTree::toEigen(kpLinear).array().sqrt();
        }
        else
        {
            if(!YarpHelper::getVectorFromSearchable(option, "kd", kdLinear))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
                return false;
            }
        }

        double c0, kpAngular, kdAngular;
        if(!YarpHelper::getNumberFromSearchable(option, "c0", c0))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(option, "kpAngular", kpAngular))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
            return false;
        }

        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }

            kdAngular = 2 / scaling * std::sqrt(kpAngular);
        }
        else
        {
            if(!YarpHelper::getNumberFromSearchable(option, "kdAngular", kdAngular))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }
        }

        iDynTree::Vector3 kpForce;
        if(!YarpHelper::getVectorFromSearchable(option, "kpForce", kpForce))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        double kpCouple;
        if(!YarpHelper::getNumberFromSearchable(option, "kpCouple", kpCouple))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
            return false;
        }


        m_pimpl->instantiateFeetConstraint(kpLinear, kdLinear,
                                           c0, kpAngular, kdAngular,
                                           kpForce, kpCouple);
    }

    {
        yarp::os::Bottle& option = config.findGroup("NECK");
        bool useDefaultKd = option.check("use_default_kd", yarp::os::Value("False")).asBool();
        double c0, kpAngular, kdAngular;
        if(!YarpHelper::getNumberFromSearchable(option, "c0", c0))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(option, "kpAngular", kpAngular))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
            return false;
        }

        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }

            kdAngular = 2 / scaling * std::sqrt(kpAngular);
        }
        else
        {
            if(!YarpHelper::getNumberFromSearchable(option, "kdAngular", kdAngular))
            {
                yError() << "[WalkingAdmittanceController::initialize] Unable to get number.";
                return false;
            }
        }

        iDynTree::VectorDynSize weight(3);
        if(!YarpHelper::getVectorFromSearchable(option, "weight", weight))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to find a vector";
            return false;
        }

        if(!iDynTree::parseRotationMatrix(option, "additional_rotation", m_pimpl->m_additionalRotation))
        {
            yError() << "[WalkingAdmittanceController::initialize] Unable to set the additional rotation.";
            return false;
        }


        m_pimpl->instantiateNeckCostFunction(c0, kpAngular, kdAngular, weight);
    }

    // resize variables
    m_pimpl->m_hessianEigen.resize(m_pimpl->m_numberOfVariables, m_pimpl->m_numberOfVariables);
    m_pimpl->m_constraintMatrix.resize(m_pimpl->m_numberOfConstraints, m_pimpl->m_numberOfVariables);
    m_pimpl->m_gradient.resize(m_pimpl->m_numberOfVariables);
    m_pimpl->m_gradient.zero();
    m_pimpl->m_lowerBound.resize(m_pimpl->m_numberOfConstraints);
    m_pimpl->m_lowerBound.zero();
    m_pimpl->m_upperBound.resize(m_pimpl->m_numberOfConstraints);
    m_pimpl->m_upperBound.zero();
    m_pimpl->m_desiredJointTorqueOutput.resize(actuatedDoFs);
    m_pimpl->m_desiredJointAccelerationOutput.resize(actuatedDoFs);

    m_pimpl->m_massMatrix.resize(actuatedDoFs + 6, actuatedDoFs + 6);
    m_pimpl->m_generalizedBiasForces.resize(actuatedDoFs + 6);
    m_pimpl->m_leftFootJacobian.resize(6, actuatedDoFs + 6);
    m_pimpl->m_rightFootJacobian.resize(6, actuatedDoFs + 6);
    m_pimpl->m_leftFootBiasAcceleration.resize(6);
    m_pimpl->m_rightFootBiasAcceleration.resize(6);
    m_pimpl->m_neckJacobian.resize(3, actuatedDoFs + 6);
    m_pimpl->m_neckBiasAcceleration.resize(3);

    // TODO move me
    m_pimpl->m_desiredJointPosition.resize(actuatedDoFs);
    m_pimpl->m_desiredJointVelocity.resize(actuatedDoFs);
    m_pimpl->m_desiredJointAcceleration.resize(actuatedDoFs);
    m_pimpl->m_jointPosition.resize(actuatedDoFs);
    m_pimpl->m_jointVelocity.resize(actuatedDoFs);

    // instantiate the optimization problem
    m_pimpl->m_optimizer = std::make_unique<OsqpEigen::Solver>();
    m_pimpl->m_optimizer->data()->setNumberOfVariables(m_pimpl->m_numberOfVariables);
    m_pimpl->m_optimizer->data()->setNumberOfConstraints(m_pimpl->m_numberOfConstraints);

    m_pimpl->m_optimizer->settings()->setVerbosity(false);
    m_pimpl->m_optimizer->settings()->setLinearSystemSolver(0);

    // set constant element of the cost function
    for(const auto& element: m_pimpl->m_costFunctions)
    {
        std::string key = element.first;
        element.second->setHessianConstantElements(m_pimpl->m_hessianMatrices.at(key));
        element.second->setGradientConstantElements(m_pimpl->m_gradientVectors.at(key));
    }

    // set constant element of the constraint
    for(const auto& constraint: m_pimpl->m_constraints)
    {
        constraint.second->setJacobianConstantElements(m_pimpl->m_constraintMatrix);
        constraint.second->setBoundsConstantElements(m_pimpl->m_upperBound, m_pimpl->m_lowerBound);
    }

    // print some usefull information
    yInfo() << "Total number of constraints " << m_pimpl->m_numberOfConstraints;
    for(const auto& constraint: m_pimpl->m_constraints)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();


    yInfo() << "Total number of cost function " << m_pimpl->m_hessianMatrices.size();
    for(const auto& costFunction: m_pimpl->m_costFunctions)
        yInfo() << costFunction.first;


    return true;
}

void WalkingAdmittanceController::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                            const iDynTree::VectorDynSize& desiredJointVelocity,
                                                            const iDynTree::VectorDynSize& desiredJointAcceleration)
{
    m_pimpl->m_desiredJointPosition = desiredJointPosition;
    m_pimpl->m_desiredJointVelocity = desiredJointVelocity;
    m_pimpl->m_desiredJointAcceleration = desiredJointAcceleration;
}

void WalkingAdmittanceController::setJointState(const iDynTree::VectorDynSize& jointPosition,
                                                const iDynTree::VectorDynSize& jointVelocity)
{
    m_pimpl->m_jointPosition = jointPosition;
    m_pimpl->m_jointVelocity = jointVelocity;
}

void WalkingAdmittanceController::setMassMatrix(const iDynTree::MatrixDynSize& massMatrix)
{
    m_pimpl->m_massMatrix = massMatrix;
}

void WalkingAdmittanceController::setGeneralizedBiasForces(const iDynTree::VectorDynSize& generalizedBiasForces)
{
    m_pimpl->m_generalizedBiasForces = generalizedBiasForces;
}

void WalkingAdmittanceController::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                  const iDynTree::MatrixDynSize& rightFootJacobian)
{
    m_pimpl->m_rightFootJacobian = rightFootJacobian;
    m_pimpl->m_leftFootJacobian = leftFootJacobian;
}

void WalkingAdmittanceController::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                                          const iDynTree::Vector6 &rightFootBiasAcceleration)
{

    iDynTree::toEigen(m_pimpl->m_leftFootBiasAcceleration) = iDynTree::toEigen(leftFootBiasAcceleration);
    iDynTree::toEigen(m_pimpl->m_rightFootBiasAcceleration) = iDynTree::toEigen(rightFootBiasAcceleration);
}

bool WalkingAdmittanceController::setFeetState(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::Wrench& leftFootWrench, bool leftInContact,
                                               const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::Wrench& rightFootWrench, bool rightInContact)
{
    return m_pimpl->setFeetState(leftFootToWorldTransform, leftFootTwist, leftFootWrench, leftInContact,
                                 rightFootToWorldTransform, rightFootTwist, rightFootWrench, rightInContact);
}

bool WalkingAdmittanceController::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::SpatialAcc& leftFootAcceleration,
                                                           const iDynTree::Wrench& leftFootWrench,
                                                           const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::SpatialAcc& rightFootAcceleration,
                                                           const iDynTree::Wrench& rightFootWrench)
{
    return m_pimpl->setDesiredFeetTrajectory(leftFootToWorldTransform, leftFootTwist,
                                             leftFootAcceleration, leftFootWrench,
                                             rightFootToWorldTransform, rightFootTwist,
                                             rightFootAcceleration, rightFootWrench);
}

bool WalkingAdmittanceController::setNeckState(const iDynTree::Rotation& neckOrientation,
                                               const iDynTree::Twist& neckVelocity)
{
    return m_pimpl->setNeckState(neckOrientation, neckVelocity);
}

bool WalkingAdmittanceController::setDesiredNeckTrajectory(const iDynTree::Rotation& neckOrientation)
{
    return m_pimpl->setDesiredNeckTrajectory(neckOrientation);
}

void WalkingAdmittanceController::setNeckJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    iDynTree::toEigen(m_pimpl->m_neckJacobian) = iDynTree::toEigen(jacobian).block(3, 0,
                                                                                   3, m_pimpl->m_actuatedDoFs + 6);
}

void WalkingAdmittanceController::setNeckBiasAcceleration(const iDynTree::Vector6 &biasAcceleration)
{
    // get only the angular part
    iDynTree::toEigen(m_pimpl->m_neckBiasAcceleration) = iDynTree::toEigen(biasAcceleration).block(3, 0, 3, 1);
}

bool WalkingAdmittanceController::solve()
{
    return m_pimpl->solve();
}

const iDynTree::VectorDynSize& WalkingAdmittanceController::desiredJointTorque() const
{
    return m_pimpl->m_desiredJointTorqueOutput;
}
