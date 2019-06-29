/**
 * @file WalkingTaskPriority.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <WalkingConstraint.hpp>
#include <WalkingTaskPriority.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixXd;
class WalkingTaskPriority::Implementation
{
    friend WalkingTaskPriority;

    int m_actuatedDoFs;

    iDynTree::MatrixDynSize m_leftFootJacobian;
    iDynTree::MatrixDynSize m_rightFootJacobian;
    iDynTree::MatrixDynSize m_neckJacobian;
    iDynTree::MatrixDynSize m_comJacobian;
    iDynTree::MatrixDynSize m_centroidalAngularJacobian;

    iDynTree::VectorDynSize m_leftFootBiasAcceleration;
    iDynTree::VectorDynSize m_rightFootBiasAcceleration;
    iDynTree::VectorDynSize m_neckBiasAcceleration;
    iDynTree::VectorDynSize m_comBiasAcceleration;
    iDynTree::VectorDynSize m_centroidalAngularBiasAcceleration;

    iDynTree::Rotation m_additionalRotation;

    // QP Quantities
    std::vector<Eigen::SparseMatrix<double>>  m_constraintMatrices;
    std::vector<iDynTree::VectorDynSize> m_bounds;

    iDynTree::VectorDynSize m_desiredJointAccelerationOutput;
    iDynTree::VectorDynSize m_desiredRobotAccelerationOutput;
    std::map<std::string, std::shared_ptr<Constraint>> m_task1;
    std::map<std::string, std::shared_ptr<Constraint>> m_task2;
    std::map<std::string, std::shared_ptr<Constraint>> m_task3;

    int m_task1Size;
    int m_task2Size;
    int m_task3Size;

    std::vector<iDynTree::MatrixDynSize> m_nullProjection;
    std::vector<iDynTree::VectorDynSize> m_desiredRobotAccelerations;

    // TODO MOVE ME
    iDynTree::VectorDynSize m_desiredJointPosition;
    iDynTree::VectorDynSize m_desiredJointVelocity;
    iDynTree::VectorDynSize m_desiredJointAcceleration;
    iDynTree::VectorDynSize m_jointPosition;
    iDynTree::VectorDynSize m_jointVelocity;


    void instantiateJointConstraint(const iDynTree::VectorDynSize &kp, const iDynTree::VectorDynSize &kd, const int& numberOfDofs)
    {
        auto ptr = std::make_shared<JointRegularizationConstraint>(numberOfDofs);
        ptr->setSubMatricesStartingPosition(m_task3Size, 6);

        ptr->setProportionalGains(kp);
        ptr->setDerivativeGains(kd);

        ptr->setDesiredJointPosition(m_desiredJointPosition);
        ptr->setDesiredJointVelocity(m_desiredJointVelocity);
        ptr->setDesiredJointAcceleration(m_desiredJointAcceleration);
        ptr->setJointPosition(m_jointPosition);
        ptr->setJointVelocity(m_jointVelocity);

        m_task3.insert({"joint_regularization", ptr});
        m_task3Size += ptr->getNumberOfConstraints();

        return;
    }

    void instantiateNeckConstraint(const double& c0, const double& kpAngular, const double& kdAngular)
    {
        std::shared_ptr<CartesianConstraint> ptr;
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::ORIENTATION);
        ptr->setControlMode(CartesianElement::ControlMode::POSITION);
        ptr->setSubMatricesStartingPosition(m_task2Size, 0);

        ptr->setBiasAcceleration(m_neckBiasAcceleration);
        ptr->setRoboticJacobian(m_neckJacobian);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        m_task2.insert({"neck", ptr});
        m_task2Size += ptr->getNumberOfConstraints();
    }

    void instantiateAngularMomentumConstraint(const double kp)
    {
        std::shared_ptr<CartesianConstraint> ptr;
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSITION);
        ptr->setControlMode(CartesianElement::ControlMode::POSITION);
        ptr->setSubMatricesStartingPosition(m_task2Size, 0);

        ptr->setBiasAcceleration(m_centroidalAngularBiasAcceleration);
        ptr->setRoboticJacobian(m_centroidalAngularJacobian);
        ptr->positionController()->setGains(kp, 0);
        iDynTree::Vector3 dummy;
        dummy.zero();
        ptr->positionController()->setDesiredTrajectory(dummy, dummy, dummy);

        m_task2.insert({"angular_momentum", ptr});
        m_task2Size += ptr->getNumberOfConstraints();
    }


    void instantiateCoMConstraint()
    {
        std::shared_ptr<CartesianConstraint> ptr;

        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSITION);
        ptr->setSubMatricesStartingPosition(m_task1Size, 0);

        ptr->positionController()->setGains(0, 0);

        ptr->setRoboticJacobian(m_comJacobian);
        ptr->setBiasAcceleration(m_comBiasAcceleration);

        // we suppose that the robot will start in double support phase
        ptr->setControlMode(CartesianElement::ControlMode::POSITION);

        m_task1.insert({"com", ptr});

        m_task1Size += ptr->getNumberOfConstraints();
        return;
    }

    void instantiateFeetConstraint(const iDynTree::Vector3& kpLinear, const iDynTree::Vector3& kdLinear,
                                   const double& c0, const double& kpAngular, const double& kdAngular,
                                   const iDynTree::Vector3& kpForce, const double& kpCouple)
    {
        std::shared_ptr<CartesianConstraint> ptr;

        // Left foot
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->setSubMatricesStartingPosition(m_task1Size, 0);

        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        ptr->forceController()->setGains(kpForce);
        ptr->coupleController()->setGains(kpCouple);

        ptr->setRoboticJacobian(m_leftFootJacobian);
        ptr->setBiasAcceleration(m_leftFootBiasAcceleration);

        // we suppose that the robot will start in double support phase
        ptr->setControlMode(CartesianElement::ControlMode::FORCE);

        m_task1.insert({"left_foot", ptr});
        m_task1Size += ptr->getNumberOfConstraints();

        // Right foot
        ptr = std::make_shared<CartesianConstraint>(CartesianElement::Type::POSE);
        ptr->setSubMatricesStartingPosition(m_task1Size, 0);

        ptr->positionController()->setGains(kpLinear, kdLinear);
        ptr->orientationController()->setGains(c0, kdAngular, kpAngular);

        ptr->forceController()->setGains(kpForce);
        ptr->coupleController()->setGains(kpCouple);

        ptr->setRoboticJacobian(m_rightFootJacobian);
        ptr->setBiasAcceleration(m_rightFootBiasAcceleration);

        // we suppose that the robot will start in double support phase
        ptr->setControlMode(CartesianElement::ControlMode::FORCE);

        m_task1.insert({"right_foot", ptr});
        m_task1Size += ptr->getNumberOfConstraints();
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
        auto constraint = m_task1.find("left_foot");
        if(constraint == m_task1.end())
        {
            yError() << "[setFeetState] unable to find the left foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }
        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setFootState(leftFootToWorldTransform, leftFootTwist, leftFootWrench, leftInContact, ptr);

        constraint = m_task1.find("right_foot");
        if(constraint == m_task1.end())
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
        auto constraint = m_task1.find("left_foot");
        if(constraint == m_task1.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the swing foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setDesiredFootTrajectory(leftFootToWorldTransform, leftFootTwist, leftFootAcceleration, leftFootWrench, ptr);

        constraint = m_task1.find("right_foot");
        if(constraint == m_task1.end())
        {
            yError() << "[setDesiredFeetTrajectory] unable to find the stance foot constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);
        this->setDesiredFootTrajectory(rightFootToWorldTransform, rightFootTwist, rightFootAcceleration, rightFootWrench, ptr);

        return true;
    }

    bool setDesiredCoMTrajectory(const iDynTree::Position& comPosition, const iDynTree::Vector3 &vrp)
    {
        iDynTree::Vector3 desiredCoMAcceleration;
        // TODO remove magic numbers
        iDynTree::toEigen(desiredCoMAcceleration) = 9.81/0.53 * (iDynTree::toEigen(comPosition) - iDynTree::toEigen(vrp));

        iDynTree::Vector3 dummy;
        dummy.zero();

        auto constraint = m_task1.find("com");
        if(constraint == m_task1.end())
        {
            yError() << "[setDesiredCoMTrajectory] unable to find the com constraint. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(constraint->second);

        ptr->positionController()->setDesiredTrajectory(desiredCoMAcceleration, dummy, dummy);
        ptr->positionController()->setFeedback(dummy, dummy);

        return true;
    }

    bool setDesiredNeckTrajectory(const iDynTree::Rotation& desiredNeckOrientation)
    {
        iDynTree::Vector3 dummy;
        dummy.zero();
        auto cost = m_task2.find("neck");
        if(cost == m_task2.end())
        {
            yError() << "[setDesiredNeckTrajectory] unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(cost->second);
        ptr->orientationController()->setDesiredTrajectory(dummy, dummy,
                                                           desiredNeckOrientation * m_additionalRotation);
        return true;
    }

    bool setNeckState(const iDynTree::Rotation& neckOrientation, const iDynTree::Twist& neckVelocity)
    {
        auto cost = m_task2.find("neck");
        if(cost == m_task2.end())
        {
            yError() << "[setNeckState] unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }

        auto ptr = std::static_pointer_cast<CartesianConstraint>(cost->second);
        ptr->orientationController()->setFeedback(neckVelocity.getAngularVec3(), neckOrientation);

        return true;
    }

    bool setAngularMomentum(const iDynTree::Vector3& angularMomentum)
    {
        auto cost = m_task2.find("angular_momentum");
        if(cost == m_task2.end())
        {
            yError() << "[setNeckState] unable to find the neck trajectory element. "
                     << "Please call 'initialize()' method";
            return false;
        }

        iDynTree::Vector3 dummy;
        dummy.zero();
        auto ptr = std::static_pointer_cast<CartesianConstraint>(cost->second);
        ptr->positionController()->setFeedback(dummy, angularMomentum);

        return true;
    }

    bool setLinearConstraintMatrix()
    {
        for(const auto& constraint: m_task1)
            constraint.second->evaluateJacobian(m_constraintMatrices[0]);

        for(const auto& constraint: m_task2)
            constraint.second->evaluateJacobian(m_constraintMatrices[1]);

        for(const auto& constraint: m_task3)
            constraint.second->evaluateJacobian(m_constraintMatrices[2]);

        return true;
    }

    bool setBounds()
    {
        for(const auto& constraint: m_task1)
        {
            constraint.second->evaluateBounds(m_bounds[0], m_bounds[0]);
            yInfo() << constraint.first << " " << m_bounds[0].toString();
        }

        for(const auto& constraint: m_task2)
        {
            constraint.second->evaluateBounds(m_bounds[1], m_bounds[1]);
            yInfo() << constraint.first << " " << m_bounds[1].toString();
        }

        for(const auto& constraint: m_task3)
            constraint.second->evaluateBounds(m_bounds[2], m_bounds[2]);

        return true;
    }

    void evaluateNullProjector(const iDynTree::MatrixDynSize& projectorPrevious, const Eigen::SparseMatrix<double>& constraint, iDynTree::MatrixDynSize& projectorOutput)
    {

        MatrixXd temp = constraint * iDynTree::toEigen(projectorPrevious);
        MatrixXd tempPseudo = temp.completeOrthogonalDecomposition().pseudoInverse();

        iDynTree::toEigen(projectorOutput) = iDynTree::toEigen(projectorPrevious)
            - tempPseudo * constraint * iDynTree::toEigen(projectorPrevious);
    }

    void evaluateRobotAcceleration(const iDynTree::VectorDynSize& robotAccelerationPrevious, const iDynTree::MatrixDynSize& projectorPrevious, const Eigen::SparseMatrix<double>& constraintMatrix, const iDynTree::VectorDynSize& constraintVector, iDynTree::VectorDynSize& robotAcceleration)
    {

        MatrixXd temp = constraintMatrix * iDynTree::toEigen(projectorPrevious);
        MatrixXd tempPseudo = temp.completeOrthogonalDecomposition().pseudoInverse();

        iDynTree::toEigen(robotAcceleration) = iDynTree::toEigen(robotAccelerationPrevious) +
            tempPseudo
            * (iDynTree::toEigen(constraintVector) - constraintMatrix * iDynTree::toEigen(robotAccelerationPrevious));
    }

    bool solve()
    {

        // update constraint matrix
        setLinearConstraintMatrix();

        setBounds();
        // update bounds


        // evaluate solution
        for(int i = 1; i < 3; i++)
        {
            evaluateRobotAcceleration(m_desiredRobotAccelerations[i-1], m_nullProjection[i-1], m_constraintMatrices[i-1], m_bounds[i-1], m_desiredRobotAccelerations[i]);
            evaluateNullProjector(m_nullProjection[i-1], m_constraintMatrices[i-1], m_nullProjection[i]);
        }

        MatrixXd temp = m_constraintMatrices[1] * iDynTree::toEigen(m_nullProjection[1]);
        MatrixXd tempPseudo = temp.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd tempProduct = tempPseudo * temp;


        iDynTree::toEigen(m_desiredRobotAccelerations[3]) = iDynTree::toEigen(m_desiredRobotAccelerations[2])
            + iDynTree::toEigen(m_nullProjection[1]) * (MatrixXd::Identity(tempProduct.rows(), tempProduct.cols()) - tempProduct).rightCols(m_actuatedDoFs) * iDynTree::toEigen(m_bounds[2]);


        iDynTree::toEigen(m_desiredJointAccelerationOutput) = iDynTree::toEigen(m_desiredRobotAccelerations.back()).segment(6, m_actuatedDoFs);
        m_desiredRobotAccelerationOutput = m_desiredRobotAccelerations.back();

        return true;
    }
};

WalkingTaskPriority::WalkingTaskPriority():
    m_pimpl(new WalkingTaskPriority::Implementation)
{
    assert(m_pimpl);
}

WalkingTaskPriority::~WalkingTaskPriority()
{}

bool WalkingTaskPriority::initialize(yarp::os::Searchable &config, const int& actuatedDoFs)
{
    m_pimpl->m_task1Size = 0;
    m_pimpl->m_task2Size = 0;
    m_pimpl->m_task3Size = 0;

    m_pimpl->m_actuatedDoFs = actuatedDoFs;

    // instantiate cost and constraints
    {
        yarp::os::Bottle& option = config.findGroup("JOINT_REGULARIZATION");

        iDynTree::VectorDynSize kp(actuatedDoFs);
        if(!YarpHelper::getVectorFromSearchable(option, "kp", kp))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to find a vector";
            return false;
        }

        iDynTree::VectorDynSize kd(actuatedDoFs);
        if(option.check("use_default_kd", yarp::os::Value("False")).asBool())
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }
            iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
        }
        else
        {
            if(!YarpHelper::getVectorFromSearchable(option, "kd", kd))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to find a vector";
                return false;
            }
        }

        m_pimpl->instantiateJointConstraint(kp, kd, actuatedDoFs);
    }

    {
        yarp::os::Bottle& option = config.findGroup("FEET");
        iDynTree::Vector3 kpLinear, kdLinear;

        if(!YarpHelper::getVectorFromSearchable(option, "kpLinear", kpLinear))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to find a vector";
            return false;
        }

        bool useDefaultKd = option.check("use_default_kd", yarp::os::Value("False")).asBool();
        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }
            iDynTree::toEigen(kdLinear) = 2 / scaling * iDynTree::toEigen(kpLinear).array().sqrt();
        }
        else
        {
            if(!YarpHelper::getVectorFromSearchable(option, "kd", kdLinear))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to find a vector";
                return false;
            }
        }

        double c0, kpAngular, kdAngular;
        if(!YarpHelper::getNumberFromSearchable(option, "c0", c0))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(option, "kpAngular", kpAngular))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }

        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }

            kdAngular = 2 / scaling * std::sqrt(kpAngular);
        }
        else
        {
            if(!YarpHelper::getNumberFromSearchable(option, "kdAngular", kdAngular))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }
        }

        iDynTree::Vector3 kpForce;
        if(!YarpHelper::getVectorFromSearchable(option, "kpForce", kpForce))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to find a vector";
            return false;
        }

        double kpCouple;
        if(!YarpHelper::getNumberFromSearchable(option, "kpCouple", kpCouple))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }


        m_pimpl->instantiateFeetConstraint(kpLinear, kdLinear,
                                           c0, kpAngular, kdAngular,
                                           kpForce, kpCouple);
    }

    {
        m_pimpl->instantiateCoMConstraint();
    }

    {
        yarp::os::Bottle& option = config.findGroup("NECK");
        bool useDefaultKd = option.check("use_default_kd", yarp::os::Value("False")).asBool();
        double c0, kpAngular, kdAngular;
        if(!YarpHelper::getNumberFromSearchable(option, "c0", c0))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }

        if(!YarpHelper::getNumberFromSearchable(option, "kpAngular", kpAngular))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }

        if(useDefaultKd)
        {
            double scaling;
            if(!YarpHelper::getNumberFromSearchable(option, "scaling", scaling))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }

            kdAngular = 2 / scaling * std::sqrt(kpAngular);
        }
        else
        {
            if(!YarpHelper::getNumberFromSearchable(option, "kdAngular", kdAngular))
            {
                yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
                return false;
            }
        }


        if(!iDynTree::parseRotationMatrix(option, "additional_rotation", m_pimpl->m_additionalRotation))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to set the additional rotation.";
            return false;
        }


        m_pimpl->instantiateNeckConstraint(c0, kpAngular, kdAngular);
    }

    {
        yarp::os::Bottle& option = config.findGroup("ANGULAR_MOMENTUM");
        double kp;

        if(!YarpHelper::getNumberFromSearchable(option, "kp", kp))
        {
            yError() << "[WalkingTaskPriority::initialize] Unable to get number.";
            return false;
        }

        m_pimpl->instantiateAngularMomentumConstraint(kp);
    }


    // resize variables
    m_pimpl->m_bounds.resize(3);
    m_pimpl->m_bounds[0].resize(m_pimpl->m_task1Size);
    m_pimpl->m_bounds[0].zero();
    m_pimpl->m_bounds[1].resize(m_pimpl->m_task2Size);
    m_pimpl->m_bounds[1].zero();
    m_pimpl->m_bounds[2].resize(m_pimpl->m_task3Size);
    m_pimpl->m_bounds[2].zero();
    m_pimpl->m_constraintMatrices.resize(3);
    m_pimpl->m_constraintMatrices[0].resize(m_pimpl->m_task1Size, actuatedDoFs + 6);
    m_pimpl->m_constraintMatrices[1].resize(m_pimpl->m_task2Size, actuatedDoFs + 6);
    m_pimpl->m_constraintMatrices[2].resize(m_pimpl->m_task3Size, actuatedDoFs + 6);

    yInfo() << m_pimpl->m_task3Size << "**********************";

    m_pimpl->m_nullProjection.resize(4);
    for(int i = 0; i < 4; i++)
    {
        m_pimpl->m_nullProjection[i].resize(actuatedDoFs + 6, actuatedDoFs + 6);
        m_pimpl->m_nullProjection[i].zero();
    }

    for(int i = 0; i < actuatedDoFs + 6; i++)
        m_pimpl->m_nullProjection[0](i, i) = 1;

    m_pimpl->m_desiredRobotAccelerations.resize(4);
    for(int i = 0; i < 4; i++)
    {
        m_pimpl->m_desiredRobotAccelerations[i].resize(actuatedDoFs + 6);
        m_pimpl->m_desiredRobotAccelerations[i].zero();
    }

    m_pimpl->m_desiredJointAccelerationOutput.resize(actuatedDoFs);
    m_pimpl->m_desiredRobotAccelerationOutput.resize(actuatedDoFs + 6);

    m_pimpl->m_leftFootJacobian.resize(6, actuatedDoFs + 6);
    m_pimpl->m_rightFootJacobian.resize(6, actuatedDoFs + 6);
    m_pimpl->m_leftFootBiasAcceleration.resize(6);
    m_pimpl->m_rightFootBiasAcceleration.resize(6);
    m_pimpl->m_neckJacobian.resize(3, actuatedDoFs + 6);
    m_pimpl->m_neckBiasAcceleration.resize(3);
    m_pimpl->m_comJacobian.resize(3, actuatedDoFs + 6);
    m_pimpl->m_comBiasAcceleration.resize(3);
    m_pimpl->m_centroidalAngularJacobian.resize(3, actuatedDoFs + 6);
    m_pimpl->m_centroidalAngularBiasAcceleration.resize(3);

    // TODO as soon as iDynTree will be implemented
    m_pimpl->m_centroidalAngularBiasAcceleration.zero();

    // TODO move me
    m_pimpl->m_desiredJointPosition.resize(actuatedDoFs);
    m_pimpl->m_desiredJointVelocity.resize(actuatedDoFs);
    m_pimpl->m_desiredJointAcceleration.resize(actuatedDoFs);
    m_pimpl->m_jointPosition.resize(actuatedDoFs);
    m_pimpl->m_jointVelocity.resize(actuatedDoFs);

    // set constant element of the constraint
    for(const auto& constraint: m_pimpl->m_task1)
    {
        constraint.second->setJacobianConstantElements(m_pimpl->m_constraintMatrices[0]);
        constraint.second->setBoundsConstantElements(m_pimpl->m_bounds[0], m_pimpl->m_bounds[0]);
    }

    for(const auto& constraint: m_pimpl->m_task2)
    {
        constraint.second->setJacobianConstantElements(m_pimpl->m_constraintMatrices[1]);
        constraint.second->setBoundsConstantElements(m_pimpl->m_bounds[1], m_pimpl->m_bounds[1]);
    }

    for(const auto& constraint: m_pimpl->m_task3)
    {
        constraint.second->setJacobianConstantElements(m_pimpl->m_constraintMatrices[2]);
        constraint.second->setBoundsConstantElements(m_pimpl->m_bounds[2], m_pimpl->m_bounds[2]);
    }


    // print some usefull information
    yInfo() << "Total number of constraints " << m_pimpl->m_task1Size;
    for(const auto& constraint: m_pimpl->m_task1)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();

    yInfo() << "Total number of constraints " << m_pimpl->m_task2Size;
    for(const auto& constraint: m_pimpl->m_task2)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();


        yInfo() << "Total number of constraints " << m_pimpl->m_task3Size;
    for(const auto& constraint: m_pimpl->m_task3)
        yInfo() << constraint.first << ": " << constraint.second->getNumberOfConstraints()
                << constraint.second->getJacobianStartingRow()
                << constraint.second->getJacobianStartingColumn();


    return true;
}

void WalkingTaskPriority::setDesiredJointTrajectory(const iDynTree::VectorDynSize& desiredJointPosition,
                                                    const iDynTree::VectorDynSize& desiredJointVelocity,
                                                    const iDynTree::VectorDynSize& desiredJointAcceleration)
{
    m_pimpl->m_desiredJointPosition = desiredJointPosition;
    m_pimpl->m_desiredJointVelocity = desiredJointVelocity;
    m_pimpl->m_desiredJointAcceleration = desiredJointAcceleration;
}

void WalkingTaskPriority::setJointState(const iDynTree::VectorDynSize& jointPosition,
                                        const iDynTree::VectorDynSize& jointVelocity)
{
    m_pimpl->m_jointPosition = jointPosition;
    m_pimpl->m_jointVelocity = jointVelocity;
}

void WalkingTaskPriority::setFeetJacobian(const iDynTree::MatrixDynSize& leftFootJacobian,
                                                  const iDynTree::MatrixDynSize& rightFootJacobian)
{
    m_pimpl->m_rightFootJacobian = rightFootJacobian;
    m_pimpl->m_leftFootJacobian = leftFootJacobian;
}

void WalkingTaskPriority::setFeetBiasAcceleration(const iDynTree::Vector6 &leftFootBiasAcceleration,
                                                  const iDynTree::Vector6 &rightFootBiasAcceleration)
{

    iDynTree::toEigen(m_pimpl->m_leftFootBiasAcceleration) = iDynTree::toEigen(leftFootBiasAcceleration);
    iDynTree::toEigen(m_pimpl->m_rightFootBiasAcceleration) = iDynTree::toEigen(rightFootBiasAcceleration);
}

bool WalkingTaskPriority::setFeetState(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::Wrench& leftFootWrench, bool leftInContact,
                                               const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::Wrench& rightFootWrench, bool rightInContact)
{
    return m_pimpl->setFeetState(leftFootToWorldTransform, leftFootTwist, leftFootWrench, leftInContact,
                                 rightFootToWorldTransform, rightFootTwist, rightFootWrench, rightInContact);
}

bool WalkingTaskPriority::setDesiredFeetTrajectory(const iDynTree::Transform& leftFootToWorldTransform, const iDynTree::Twist& leftFootTwist, const iDynTree::SpatialAcc& leftFootAcceleration,
                                                           const iDynTree::Wrench& leftFootWrench,
                                                           const iDynTree::Transform& rightFootToWorldTransform, const iDynTree::Twist& rightFootTwist, const iDynTree::SpatialAcc& rightFootAcceleration,
                                                           const iDynTree::Wrench& rightFootWrench)
{
    return m_pimpl->setDesiredFeetTrajectory(leftFootToWorldTransform, leftFootTwist,
                                             leftFootAcceleration, leftFootWrench,
                                             rightFootToWorldTransform, rightFootTwist,
                                             rightFootAcceleration, rightFootWrench);
}

bool WalkingTaskPriority::setNeckState(const iDynTree::Rotation& neckOrientation,
                                               const iDynTree::Twist& neckVelocity)
{
    return m_pimpl->setNeckState(neckOrientation, neckVelocity);
}

bool WalkingTaskPriority::setDesiredNeckTrajectory(const iDynTree::Rotation& neckOrientation)
{
    return m_pimpl->setDesiredNeckTrajectory(neckOrientation);
}

void WalkingTaskPriority::setNeckJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    iDynTree::toEigen(m_pimpl->m_neckJacobian) = iDynTree::toEigen(jacobian).block(3, 0,
                                                                                   3, m_pimpl->m_actuatedDoFs + 6);
}

void WalkingTaskPriority::setNeckBiasAcceleration(const iDynTree::Vector6 &biasAcceleration)
{
    // get only the angular part
    iDynTree::toEigen(m_pimpl->m_neckBiasAcceleration) = iDynTree::toEigen(biasAcceleration).block(3, 0, 3, 1);
}

void WalkingTaskPriority::setCoMJacobian(const iDynTree::MatrixDynSize& jacobian)
{
    iDynTree::toEigen(m_pimpl->m_comJacobian) = iDynTree::toEigen(jacobian);
}

void WalkingTaskPriority::setCoMBiasAcceleration(const iDynTree::Vector3 &biasAcceleration)
{
    m_pimpl->m_neckBiasAcceleration = biasAcceleration;
}

bool WalkingTaskPriority::setDesiredCoMTrajectory(const iDynTree::Position& comPosition, const iDynTree::Vector3& vrpPosition)
{
    return m_pimpl->setDesiredCoMTrajectory(comPosition, vrpPosition);
}

bool WalkingTaskPriority::setAngularMomentum(const iDynTree::Vector3& angularMomentum)
{
    return m_pimpl->setAngularMomentum(angularMomentum);
}

void WalkingTaskPriority::setAngularMomentumJacobian(const iDynTree::MatrixDynSize& angularMomentumJacobian)
{
    yInfo() << angularMomentumJacobian.rows() << " " << angularMomentumJacobian.cols();
    iDynTree::toEigen(m_pimpl->m_centroidalAngularJacobian) = iDynTree::toEigen(angularMomentumJacobian).bottomRows(3);
}


bool WalkingTaskPriority::solve()
{
    return m_pimpl->solve();
}

const iDynTree::VectorDynSize& WalkingTaskPriority::desiredJointAcceleration() const
{
    return m_pimpl->m_desiredJointAccelerationOutput;
}

const iDynTree::VectorDynSize& WalkingTaskPriority::desiredRobotAcceleration() const
{
    return m_pimpl->m_desiredRobotAccelerationOutput;
}
