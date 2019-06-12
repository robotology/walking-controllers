/**
 * @file ContactWrenchMapping.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/EigenSparseHelpers.h>

#include <OsqpEigen/OsqpEigen.h>

#include <ContactWrenchMapping.hpp>
#include <Utils.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;


class ContactWrenchMapping::Implementation
{
    friend ContactWrenchMapping;

    // options
    bool m_useLinearMomentumConstraint{false};
    bool m_useLinearMomentumCostFunction{false};

    bool m_useAngularMomentumConstraint{false};
    bool m_useAngularMomentumCostFunction{false};

    // QP Quantities
    std::unique_ptr<OsqpEigen::Solver> m_optimizer{nullptr};
    Eigen::SparseMatrix<double> m_hessianEigen;
    iDynTree::VectorDynSize m_gradient;
    Eigen::SparseMatrix<double>  m_constraintMatrix;
    iDynTree::VectorDynSize m_upperBound;
    iDynTree::VectorDynSize m_lowerBound;
    int m_numberOfConstraints;
    int m_numberOfVariables;
    iDynTree::VectorDynSize m_solution;

    iDynTree::Transform m_leftFootToWorldTransform;
    iDynTree::Transform m_rightFootToWorldTransform;

    std::map<std::string, std::shared_ptr<Constraint>> m_constraints;
    std::map<std::string, std::shared_ptr<CostFunctionElement>> m_costFunctions;

    std::map<std::string, Eigen::SparseMatrix<double>> m_hessianMatrices;
    std::map<std::string, iDynTree::VectorDynSize> m_gradientVectors;

    double m_regularizationForceScale;
    double m_regularizationForceOffset;



    void instantiateLinearMomentumConstraint(const iDynTree::VectorDynSize& kp,
                                             const iDynTree::VectorDynSize& kd)
    {
        m_useLinearMomentumConstraint = true;

        // memory allocation
        // Notice here we re using DoubleSupport even if the robot is in single support because in case of SS the wrench
        // acting on the swing foot is equal to zero

        auto ptr = std::make_shared<LinearMomentumConstraint>(LinearMomentumConstraint::Type::DOUBLE_SUPPORT);

        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);

        m_constraints.insert(std::make_pair("linear_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();

        return;
    }

    void instantiateLinearMomentumCostFunction(const iDynTree::VectorDynSize weight)
    {
        m_useLinearMomentumCostFunction = true;

        // memory allocation

        // Notice here we re using DoubleSupport even if the robot is in single support because in case of SS the wrench
        // acting on the swing foot is equal to zero
        auto ptr = std::make_shared<LinearMomentumCostFunction>(LinearMomentumCostFunction::Type::DOUBLE_SUPPORT);
        // only the forces are used to control the linear momentum
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);

        m_costFunctions.insert({"linear_momentum_costFunction", ptr});
        m_hessianMatrices["linear_momentum_costFunction"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["linear_momentum_costFunction"].resize(m_numberOfVariables);
        m_gradientVectors["linear_momentum_costFunction"].zero();

        return;
    }

    void instantiateAngularMomentumConstraint(const double& kp, const double& ki)
    {
        m_useAngularMomentumConstraint = true;

        auto ptr = std::make_shared<AngularMomentumConstraint>();
        ptr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
        ptr->setKp(kp);
        ptr->setKi(ki);


        ptr->setFootToWorldTransform(m_leftFootToWorldTransform);
        ptr->setFootToWorldTransform(m_rightFootToWorldTransform);

        m_constraints.insert(std::make_pair("angular_momentum_constraint", ptr));
        m_numberOfConstraints += ptr->getNumberOfConstraints();

        return;
    }


    void instantiateAngularMomentumCostFunction(const double& kp, const double& ki, const iDynTree::VectorDynSize& weight)
    {
        m_useAngularMomentumCostFunction = true;

        auto ptr = std::make_shared<AngularMomentumCostFunction>();
        ptr->setSubMatricesStartingPosition(0, 0);
        ptr->setWeight(weight);
        ptr->setKp(kp);
        ptr->setKi(ki);

        ptr->setFootToWorldTransform(m_leftFootToWorldTransform);
        ptr->setFootToWorldTransform(m_rightFootToWorldTransform);


        m_costFunctions.insert(std::make_pair("angular_momentum_costFunction", ptr));

        m_hessianMatrices["angular_momentum_costFunction"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["angular_momentum_costFunction"].resize(m_numberOfVariables);
        return;
    }

    void instantiateInputRegularizationTerm(const double& regularizationForceScale, const double& regularizationForceOffset)
    {

        m_regularizationForceScale = regularizationForceScale;
        m_regularizationForceOffset = regularizationForceOffset;

        // left foot
        std::shared_ptr<InputRegularizationTerm> ptr;
        ptr = std::make_shared<InputRegularizationTerm>(6);
        ptr->setSubMatricesStartingPosition(0, 0);
        m_costFunctions.insert(std::make_pair("regularization_left_force", ptr));
        m_hessianMatrices["regularization_left_force"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["regularization_left_force"].resize(m_numberOfVariables);
        m_gradientVectors["regularization_left_force"].zero();

        // right foot
        ptr = std::make_shared<InputRegularizationTerm>(6);
        ptr->setSubMatricesStartingPosition(6, 0);
        m_costFunctions.insert(std::make_pair("regularization_right_force", ptr));
        m_hessianMatrices["regularization_right_force"].resize(m_numberOfVariables, m_numberOfVariables);
        m_gradientVectors["regularization_right_force"].resize(m_numberOfVariables);
        m_gradientVectors["regularization_right_force"].zero();

        return;
    }

    void instantiateContactForcesConstraint(const double& staticFrictionCoefficient, const int& numberOfPoints,
                                            const double& torsionalFrictionCoefficient,
                                            const iDynTree::Vector2& footLimitX, const iDynTree::Vector2& footLimitY,
                                            const double& minimalNormalForce)
    {
        // left foot
        auto leftForcePtr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                                              torsionalFrictionCoefficient, minimalNormalForce,
                                                              footLimitX, footLimitY);

        leftForcePtr->setSubMatricesStartingPosition(m_numberOfConstraints, 0);
        leftForcePtr->setFootToWorldTransform(m_leftFootToWorldTransform);

        m_constraints.insert(std::make_pair("left_force", leftForcePtr));
        m_numberOfConstraints += leftForcePtr->getNumberOfConstraints();


        // right foot
        auto rightForcePtr = std::make_shared<ForceConstraint>(numberOfPoints, staticFrictionCoefficient,
                                                               torsionalFrictionCoefficient, minimalNormalForce,
                                                               footLimitX, footLimitY);

        rightForcePtr->setSubMatricesStartingPosition(m_numberOfConstraints, 6);
        rightForcePtr->setFootToWorldTransform(m_rightFootToWorldTransform);

        m_constraints.insert(std::make_pair("right_force", rightForcePtr));
        m_numberOfConstraints += rightForcePtr->getNumberOfConstraints();

        return;
    }

    bool setCoMState(const iDynTree::Position& comPosition, const iDynTree::Vector3& comVelocity)
    {
        if(m_useLinearMomentumConstraint)
        {
            // save com desired trajectory
            auto constraint = m_constraints.find("linear_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setCoMState] unable to find the linear momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
            ptr->setCoMPosition(comPosition);
            // ptr->setCoMVelocity(comVelocity);
        }

        if(m_useLinearMomentumCostFunction)
        {
            // save com desired trajectory
            auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
            if(costFunction == m_costFunctions.end())
            {
                yError() << "[setCoMState] unable to find the linear momentum costFunction. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
            ptr->setCoMPosition(comPosition);
        }

        if(m_useAngularMomentumConstraint)
        {
            auto constraint = m_constraints.find("angular_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setCoMState] unable to find the angular momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
            ptr->setCoMPosition(comPosition);
        }

        if(m_useAngularMomentumCostFunction)
        {
            auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
            if(costFunction == m_costFunctions.end())
            {
                yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
            ptr->setCoMPosition(comPosition);
        }

        return true;
    }

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition, const iDynTree::Vector3& comVelocity,
                                 const iDynTree::Vector3& comAcceleration)
    {
        // if(m_useLinearMomentumConstraint)
        // {
        //     // save com desired trajectory
        //     auto constraint = m_constraints.find("linear_momentum_constraint");
        //     if(constraint == m_constraints.end())
        //     {
        //         yError() << "[setCoMState] unable to find the linear momentum constraint. "
        //                  << "Please call 'initialize()' method";
        //         return false;
        //     }

        //     auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
        //     ptr->setDesiredCoMPosition(comPosition);
        //     ptr->setDesiredCoMVelocity(comVelocity);
        //     ptr->setDesiredCoMAcceleration(comAcceleration);
        // }
        return true;
    }


    bool setCentroidalMomentum(const iDynTree::SpatialMomentum& centroidalTotalMomentum)
    {
        if(m_useAngularMomentumConstraint)
        {
            auto constraint = m_constraints.find("angular_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setLinearAngularMomentum] unable to find the angular momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(constraint->second);
            ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
        }

        if(m_useAngularMomentumCostFunction)
        {
            auto costFunction = m_costFunctions.find("angular_momentum_costFunction");
            if(costFunction == m_costFunctions.end())
            {
                yError() << "[setCoMState] unable to find the angular momentum costFunction. "
                         << "Please call 'initialize()' method";
                return false;
            }

            auto ptr = std::dynamic_pointer_cast<AngularMomentumElement>(costFunction->second);
            ptr->setAngularMomentum(centroidalTotalMomentum.getAngularVec3());
        }
        return true;
    }


    bool setDesiredVRP(const iDynTree::Vector3 &vrp)
    {
        if(m_useLinearMomentumConstraint)
        {
            auto constraint = m_constraints.find("linear_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setDesiredVRP] Unable to find the linear_momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }
            auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
            ptr->setDesiredVRP(vrp);
        }

        if(m_useLinearMomentumCostFunction)
        {
            auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
            if(costFunction == m_costFunctions.end())
            {
                yError() << "[setDesiredVRP] Unable to find the linear_momentum costFunction. "
                         << "Please call 'initialize()' method";
                return false;
            }
            auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
            ptr->setDesiredVRP(vrp);
        }
        return true;
    }

    bool setRobotMass(const double &mass)
    {
        if(m_useLinearMomentumConstraint)
        {
            auto constraint = m_constraints.find("linear_momentum_constraint");
            if(constraint == m_constraints.end())
            {
                yError() << "[setRobotMass] Unable to find the linear_momentum constraint. "
                         << "Please call 'initialize()' method";
                return false;
            }
            auto ptr = std::static_pointer_cast<LinearMomentumConstraint>(constraint->second);
            ptr->setRobotMass(mass);
        }

        if(m_useLinearMomentumCostFunction)
        {
            auto costFunction = m_costFunctions.find("linear_momentum_costFunction");
            if(costFunction == m_costFunctions.end())
            {
                yError() << "[setRobotMass] Unable to find the linear_momentum costFunction. "
                         << "Please call 'initialize()' method";
                return false;
            }
            auto ptr = std::static_pointer_cast<LinearMomentumCostFunction>(costFunction->second);
            ptr->setRobotMass(mass);
        }

        return true;
    }

    bool setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight)
    {
        iDynTree::VectorDynSize weightLeft(6), weightRight(6);

        for(int i = 0; i < 6; i++)
        {
            weightLeft(i) = m_regularizationForceScale * std::fabs(weightInLeft)
                + m_regularizationForceOffset;

            weightRight(i) = m_regularizationForceScale * std::fabs(weightInRight)
                + m_regularizationForceOffset;
        }

        auto cost = m_costFunctions.find("regularization_left_force");
        if(cost == m_costFunctions.end())
        {
            yError() << "[setFeetWeightPercentage] Unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
        ptr->setWeight(weightLeft);

        cost = m_costFunctions.find("regularization_right_force");
        if(cost == m_costFunctions.end())
        {
            yError() << "[setFeetWeightPercentage] Unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }
        ptr = std::static_pointer_cast<InputRegularizationTerm>(cost->second);
        ptr->setWeight(weightRight);

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

        iDynTree::toEigen(m_solution) = m_optimizer->getSolution();

        return true;
    }

};

ContactWrenchMapping::ContactWrenchMapping():
    m_pimpl(new ContactWrenchMapping::Implementation)
{
    assert(m_pimpl);
}

ContactWrenchMapping::~ContactWrenchMapping()
{

}

bool ContactWrenchMapping::initialize(yarp::os::Searchable& config)
{
    m_pimpl->m_numberOfConstraints = 0;
    m_pimpl->m_numberOfVariables = 6 + 6;
    {
        yarp::os::Bottle& linearMomentumOptions = config.findGroup("LINEAR_MOMENTUM");

        if(linearMomentumOptions.check("use_as_cost_function", yarp::os::Value("False")).asBool())
        {
            iDynTree::VectorDynSize weight(3);

            if(!YarpHelper::getVectorFromSearchable(linearMomentumOptions, "weight", weight))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }

            m_pimpl->instantiateLinearMomentumCostFunction(weight);
        }

        if(linearMomentumOptions.check("use_as_constraint", yarp::os::Value("False")).asBool())
        {
            iDynTree::VectorDynSize kp(3);
            iDynTree::VectorDynSize kd(3);

            if(!YarpHelper::getVectorFromSearchable(linearMomentumOptions, "kp", kp))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }

            if(!YarpHelper::getVectorFromSearchable(linearMomentumOptions, "kd", kd))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }

            m_pimpl->instantiateLinearMomentumConstraint(kp, kd);
        }
    }

    {
        yarp::os::Bottle& angularMomentumOptions = config.findGroup("ANGULAR_MOMENTUM");

        if(angularMomentumOptions.check("use_as_cost_function", yarp::os::Value("False")).asBool())
        {
            double kp, ki;
            iDynTree::VectorDynSize weight(3);

            if(!YarpHelper::getNumberFromSearchable(angularMomentumOptions, "kp", kp))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }


            if(!YarpHelper::getNumberFromSearchable(angularMomentumOptions, "ki", ki))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }

            if(!YarpHelper::getVectorFromSearchable(angularMomentumOptions, "weight", weight))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }


            m_pimpl->instantiateAngularMomentumCostFunction(kp, ki, weight);
        }

        if(angularMomentumOptions.check("use_as_constraint", yarp::os::Value("False")).asBool())
        {
            double kp, ki;

            if(!YarpHelper::getNumberFromSearchable(angularMomentumOptions, "kp", kp))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }


            if(!YarpHelper::getNumberFromSearchable(angularMomentumOptions, "ki", ki))
            {
                yError() << "[ContactWrenchMapping::initialize] Unable to find a vector";
                return false;
            }

            m_pimpl->instantiateAngularMomentumConstraint(kp, ki);
        }

    }

    {
        yarp::os::Bottle& contactForcesOption = config.findGroup("CONTACT_FORCES");

        double staticFrictionCoefficient;
        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "staticFrictionCoefficient", staticFrictionCoefficient))
        {
            yError() << "[initialize] Unable to get the number from searchable.";
            return false;
        }

        int numberOfPoints;
        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "numberOfPoints", numberOfPoints))
        {
            yError() << "[initialize] Unable to get the number from searchable.";
            return false;
        }

        double torsionalFrictionCoefficient;
        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "torsionalFrictionCoefficient",
                                                torsionalFrictionCoefficient))
        {
            yError() << "[initialize] Unable to get the number from searchable.";
            return false;
        }

        // feet dimensions
        // TODO try to use YarpHelper
        yarp::os::Value feetDimensions = contactForcesOption.find("foot_size");
        if(feetDimensions.isNull() || !feetDimensions.isList())
        {
            yError() << "Please set the foot_size in the contactForcesOptionconfiguration file.";
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

        iDynTree::Vector2 footLimitX;
        footLimitX(0) = xLimitsPtr->get(0).asDouble();
        footLimitX(1) = xLimitsPtr->get(1).asDouble();

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

        iDynTree::Vector2 footLimitY;
        footLimitY(0) = yLimitsPtr->get(0).asDouble();
        footLimitY(1) = yLimitsPtr->get(1).asDouble();

        double minimalNormalForce;
        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "minimalNormalForce", minimalNormalForce))
        {
            yError() << "[initialize] Unable to get the number from searchable.";
            return false;
        }


        m_pimpl->instantiateContactForcesConstraint(staticFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient,
                                                    footLimitX, footLimitY, minimalNormalForce);


        double regularizationForceScale, regularizationForceOffset;

        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "regularization_force_scale", regularizationForceScale))
        {
            yError() << "[ContactWrenchMapping::initialize] Unable to find the number";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(contactForcesOption, "regularization_force_offset", regularizationForceOffset))
        {
            yError() << "[ContactWrenchMapping::initialize] Unable to find the number";
            return false;
        }

        m_pimpl->instantiateInputRegularizationTerm(regularizationForceScale, regularizationForceOffset);
    }

    // resize
    // sparse matrix
    m_pimpl->m_hessianEigen.resize(m_pimpl->m_numberOfVariables, m_pimpl->m_numberOfVariables);
    m_pimpl->m_constraintMatrix.resize(m_pimpl->m_numberOfConstraints, m_pimpl->m_numberOfVariables);

    // dense vectors
    m_pimpl->m_gradient.resize(m_pimpl->m_numberOfVariables);
    m_pimpl->m_gradient.zero();
    m_pimpl->m_lowerBound.resize(m_pimpl->m_numberOfConstraints);
    m_pimpl->m_lowerBound.zero();
    m_pimpl->m_upperBound.resize(m_pimpl->m_numberOfConstraints);
    m_pimpl->m_upperBound.zero();

    m_pimpl->m_solution.resize(m_pimpl->m_numberOfVariables);

    // initialize the optimization problem
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

    // // print some usefull information
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

bool ContactWrenchMapping::setRobotMass(const double& mass)
{
    return m_pimpl->setRobotMass(mass);
}

void ContactWrenchMapping::setFeetState(const bool &leftInContact, const bool &rightInContact)
{
    std::shared_ptr<ForceConstraint> constraintLeft = std::static_pointer_cast<ForceConstraint>(m_pimpl->m_constraints["left_force"]);
    std::shared_ptr<ForceConstraint> constraintRight = std::static_pointer_cast<ForceConstraint>(m_pimpl->m_constraints["right_force"]);
    if(leftInContact)
        constraintLeft->enableConstraint();
    else
        constraintLeft->disableConstraint();

    if(rightInContact)
        constraintRight->enableConstraint();
    else
        constraintRight->disableConstraint();
}

bool ContactWrenchMapping::setCentroidalMomentum(const iDynTree::SpatialMomentum& centroidalMomentum)
{
    return m_pimpl->setCentroidalMomentum(centroidalMomentum);
}

void ContactWrenchMapping::setFeetState(const iDynTree::Transform& leftFootToWorldTransform,
                                       const iDynTree::Transform& rightFootToWorldTransform)
{
    m_pimpl->m_leftFootToWorldTransform = leftFootToWorldTransform;
    m_pimpl->m_rightFootToWorldTransform = rightFootToWorldTransform;
}

bool ContactWrenchMapping::setCoMState(const iDynTree::Position& comPosition,
                                      const iDynTree::Vector3& comVelocity)
{
    return m_pimpl->setCoMState(comPosition, comVelocity);
}

bool ContactWrenchMapping::setDesiredVRP(const iDynTree::Vector3 &vrp)
{
    return m_pimpl->setDesiredVRP(vrp);
}

bool ContactWrenchMapping::setDesiredCoMTrajectory(const iDynTree::Position& comPosition,
                                                  const iDynTree::Vector3& comVelocity,
                                                  const iDynTree::Vector3& comAcceleration)
{
    return m_pimpl->setDesiredCoMTrajectory(comPosition, comVelocity,comAcceleration);
}

bool ContactWrenchMapping::setFeetWeightPercentage(const double &weightInLeft, const double &weightInRight)
{
    return m_pimpl->setFeetWeightPercentage(weightInLeft, weightInRight);
}

bool ContactWrenchMapping::solve()
{
    return m_pimpl->solve();
}

const iDynTree::VectorDynSize& ContactWrenchMapping::solution() const
{
    return m_pimpl->m_solution;
}
