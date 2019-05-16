/**
 * @file WalkingInverseKinematics.cpp
 * @authors Stefano Dafarra
 *          Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>

//iDynTree
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/KinDynComputations.h>

// Eigen
#include <Eigen/Core>

#include <WalkingInverseKinematics.hpp>

WalkingIK::WalkingIK()
    : m_verbose(false)
    , m_lFootFrame("l_sole")
    , m_rFootFrame("r_sole")
    , m_inertial_R_world(iDynTree::Rotation::Identity())
    , m_prepared(false)
    , m_additionalRotationWeight(1.0)
    , m_jointRegularizationWeight(0.5)
{}

WalkingIK::~WalkingIK()
{
}

void WalkingIK::setVerboseMode(bool verboseMode) { m_verbose = verboseMode; }

bool WalkingIK::initialize(yarp::os::Searchable& ikOption, const iDynTree::Model& model, const std::vector<std::string>& jointList)
{
    solverVerbosity = ikOption.check("solver-verbosity",yarp::os::Value(0)).asInt();
    maxCpuTime = ikOption.check("max-cpu-time",yarp::os::Value(0.2)).asDouble();
    m_jointRegularizationWeight = ikOption.check("joint_regularization_weight", yarp::os::Value(0.5)).asDouble();
    std::string lFootFrame = ikOption.check("left_foot_frame", yarp::os::Value("l_sole")).asString();
    std::string rFootFrame = ikOption.check("right_foot_frame", yarp::os::Value("r_sole")).asString();
    std::string solverName = ikOption.check("solver_name", yarp::os::Value("mumps")).asString();
    m_additionalFrame = ikOption.check("additional_frame", yarp::os::Value("")).asString();
    if(m_additionalFrame.size()!=0)
    {
        if(!iDynTree::parseRotationMatrix(ikOption, "additional_rotation", m_additionalRotation))
        {
            m_additionalRotation = iDynTree::Rotation::Identity();
            yInfo() << "Using the identity as desired rotation for the additional frame";
        }
        if(m_verbose)
            yInfo() << "Desired rotation for the additional frame:\n" << m_additionalRotation.toString();
    }
    yarp::os::Value jointRegularization = ikOption.find("jointRegularization");

    if(!setModel(model, jointList))
    {
        yError()<<"Error while loading the model.";
        return false;
    }

    if(m_verbose)
    {
        double joint_min;
        double joint_max;

        //for each joint, ask the limits
        yInfo() << "Joint Limits:";
        for (iDynTree::JointIndex jointIdx = 0; jointIdx < static_cast<int>(m_ik.reducedModel().getNrOfJoints()); ++jointIdx) {
            iDynTree::IJointConstPtr joint = m_ik.reducedModel().getJoint(jointIdx);
            //if the joint does not have limits skip it
            if (!joint->hasPosLimits())
                continue;
            //for each DoF modelled by the joint get the limits
            for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof)
            {
                if (!joint->getPosLimits(dof,
                                         joint_min,
                                         joint_max))
                    continue;
                yInfo() << m_ik.reducedModel().getJointName(jointIdx) << ": " <<  iDynTree::rad2deg(joint_min) << " to " << iDynTree::rad2deg(joint_max);
            }
        }

        m_baseTransform = m_ik.fullModel().getFrameTransform( m_ik.fullModel().getFrameIndex(m_lFootFrame) ).inverse();

        iDynTree::LinkIndex baseDebug = m_ik.reducedModel().getFrameLink(m_ik.reducedModel().getFrameIndex(m_lFootFrame));
        lchecker.loadRobotModel(m_ik.reducedModel());
        if(!lchecker.setFloatingBase(m_ik.reducedModel().getLinkName(baseDebug)))
            return false;

        dummyVel.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
        dummyVel.zero();
        dummyBaseVel.zero();
        dummygrav.zero();
        lchecker.setRobotState(m_baseTransform, dummyVel, dummyBaseVel, dummyVel, dummygrav);
        if(m_additionalFrame.size()!=0)
        {
            yInfo() << "Relative rotation between" << m_lFootFrame << " and " << m_additionalFrame
                    << ":\n" << lchecker.getRelativeTransform(m_lFootFrame, m_additionalFrame).getRotation().toString();
        }
        yInfo() << "Position of the COM in zero position for all the joints: "
                << lchecker.getCenterOfMassPosition().toString();
    }

    if(!(jointRegularization.isNull()))
    {
        if (!jointRegularization.isList() || !jointRegularization.asList())
        {
            yError()<<"Unable to read the jointRegularization list.";
            return false;
        }
        yarp::os::Bottle *guessValue = jointRegularization.asList();

        if(guessValue->size() > 0 && static_cast<size_t>(guessValue->size()) != m_ik.reducedModel().getNrOfDOFs())
        {
            yError("[ERROR WALKIK] The jointRegularization list should have the same dimension of the number of DoFs of the provided model. Model = %lu, Guess = %lu.",
                   m_ik.reducedModel().getNrOfDOFs(), guessValue->size());
            return false;
        }

        for(int i = 0; i < guessValue->size(); ++i)
        {
            if(!guessValue->get(i).isDouble() && !guessValue->get(i).isInt())
            {
                yError("The jointRegularization value is expected to be a double");
            }
            m_jointRegularization(i) = guessValue->get(i).asDouble()*iDynTree::deg2rad(1);
        }
        m_guess = m_jointRegularization;
    }

    if(!this->setFootFrame("left",lFootFrame))
    {
        yError() << "Unable to select frame:" << lFootFrame;
        return false;
    }

    if(!this->setFootFrame("right",rFootFrame))
    {
        yError() << "Unable to select frame:" << rFootFrame;
        return false;
    }

    m_ik.setMaxCPUTime(maxCpuTime);
    m_ik.setVerbosity(solverVerbosity);
    m_ik.setLinearSolverName(solverName);

    if (m_verbose)
    {
        yInfo() << "Solver verbosity: " << solverVerbosity;
        yInfo() << "Max CPU time: " << maxCpuTime;
        yInfo() << "Joint Regularization (RAD): " << m_jointRegularization.toString();
    }

    return prepareIK();
}


bool WalkingIK::setModel(const iDynTree::Model& model, const std::vector< std::string >& consideredJoints)
{
    if(!(m_ik.setModel(model,consideredJoints)))
        return false;

    m_feedback.resize(static_cast<unsigned int>(m_ik.fullModel().getNrOfDOFs()));
    m_feedback.zero();

    m_jointRegularization.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_jointRegularization.zero();

    m_guess.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_guess.zero();

    m_qResult.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_qResult.zero();

    m_prepared = false;

    return true;
}

bool WalkingIK::setFootFrame(const std::string& foot, const std::string& footFrame)
{
    if (foot == "left")
    {
        m_lFootFrame = footFrame;
    }
    else if (foot == "right")
    {
        m_rFootFrame = footFrame;
    }
    else
    {
        yError() << "WalkingIK: Allowed foot parameters: \"left\", \"right\".";
        return false;
    }

    m_prepared = false;

    return true;
}

bool WalkingIK::updateIntertiaToWorldFrameRotation(const iDynTree::Rotation &inertial_R_worldFrame)
{
    if(m_additionalFrame.size() == 0)
    {
        yError() << "WalkingIK: Cannot update the inertia to left frame rotation if no additional frame is provided.";
        return false;
    }
    m_inertial_R_world = inertial_R_worldFrame;
    return true;
}

bool WalkingIK::setFullModelFeedBack(const iDynTree::VectorDynSize& feedback)
{
    if(feedback.size() != m_ik.fullModel().getNrOfDOFs())
    {
        yError("WalkingIK: The feedback is expected to have the same dimension of the total number of degrees of freedom of the model. Input -> %d != %lu <- Model.",
               feedback.size(), m_ik.fullModel().getNrOfDOFs());
        return false;
    }
    m_feedback = feedback;
    return true;
}


bool WalkingIK::prepareIK()
{
    if(m_ik.reducedModel().getNrOfDOFs() == 0)
    {
        yError() << "WalkingIK: First you have to load a model.";
        return false;
    }

    m_ik.clearProblem();

    m_ik.setMaxCPUTime(maxCpuTime);
    m_ik.setVerbosity(solverVerbosity);

    m_ik.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
    m_ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintFull);

    iDynTree::LinkIndex base = m_ik.fullModel().getFrameLink(m_ik.fullModel().getFrameIndex(m_lFootFrame));

    m_baseTransform = m_ik.fullModel().getFrameTransform( m_ik.fullModel().getFrameIndex(m_lFootFrame) ).inverse();

    if(!m_ik.setFloatingBaseOnFrameNamed(m_ik.fullModel().getLinkName(base)))
    {
        yError() << "WalkingIK: Invalid frame selected for the left foot: "<< m_lFootFrame;
        return false;
    }

    m_ik.addFrameConstraint(m_lFootFrame,iDynTree::Transform::Identity());

    m_ik.setCOMAsConstraint(true);

    bool ok;
    ok = m_ik.addTarget(m_rFootFrame, iDynTree::Transform::Identity());
    if(!ok)
    {
        yError() << "WalkingIK: Unable to add a constraint for "<< m_rFootFrame << ".";
        return false;
    }
    m_ik.setTargetResolutionMode(m_rFootFrame, iDynTree::InverseKinematicsTreatTargetAsConstraintFull);

    if(m_additionalFrame.size() != 0){
        ok = m_ik.addRotationTarget(m_additionalFrame, m_additionalRotation, m_additionalRotationWeight);
        if(!ok){
            yError() << "WalkingIK: Unable to add a rotation target on "<< m_additionalFrame << ".";
            return false;
        }
        m_ik.setTargetResolutionMode(m_additionalFrame, iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    }

    m_ik.setCostTolerance(1e-4);
    m_ik.setConstraintsTolerance(1e-4);
    m_ik.setCOMAsConstraintTolerance(1e-4);

    ///DEBUG
    iDynTree::LinkIndex baseDebug = m_ik.reducedModel().getFrameLink(m_ik.reducedModel().getFrameIndex(m_lFootFrame));
    lchecker.loadRobotModel(m_ik.reducedModel());
    if(!lchecker.setFloatingBase(m_ik.reducedModel().getLinkName(baseDebug)))
        return false;

    dummyVel.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    dummyVel.zero();
    dummyBaseVel.zero();
    dummygrav.zero();

    m_prepared = true;

    return true;
}

bool WalkingIK::updateAdditionalRotation(const iDynTree::Rotation& additionalRotation)
{
    if(m_additionalFrame.size() == 0)
    {
        yError() << "WalkingIK: Cannot update the additional rotation if no frame is provided.";
        return false;
    }
    m_additionalRotation = additionalRotation;
    return true;
}

bool WalkingIK::setInitialGuess(const iDynTree::VectorDynSize& guess)
{
    if(guess.size() != m_ik.reducedModel().getNrOfDOFs())
    {
        yError("WalkingIK: The guess is expected to have the same dimension of the reduced number of degrees of freedom. Input -> %d != %lu <- Model.",guess.size(), m_ik.reducedModel().getNrOfDOFs());
        return false;
    }

    m_guess = guess;

    return true;
}

bool WalkingIK::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration)
{
    if(desiredJointConfiguration.size() != m_ik.reducedModel().getNrOfDOFs())
    {
        yError("WalkingIK: The desiredJointConfiguration is expected to have the same dimension of the reduced number of degrees of freedom. Input -> %d != %lu <- Model.",desiredJointConfiguration.size(), m_ik.reducedModel().getNrOfDOFs());
        return false;
    }

    m_jointRegularization = desiredJointConfiguration;

    return true;
}



bool WalkingIK::computeIK(const iDynTree::Transform& leftTransform, const iDynTree::Transform& rightTransform, const iDynTree::Position& comPosition, iDynTree::VectorDynSize& result)
{
    if(!m_prepared){
        if(!prepareIK()){
            yError()<<"WalkingIK: Error in the preparation phase.";
            return false;
        }
    }

    iDynTree::Transform desiredRightTransform;
    iDynTree::Position desiredCoMPosition;
    iDynTree::Transform baseTransform;
    bool ok = true;

    desiredRightTransform = leftTransform.inverse()*rightTransform;

    if (m_verbose) {
        yInfo() << "Desired right Transform: ";
        yInfo() << desiredRightTransform.toString();
    }

    m_ik.updateTarget(m_rFootFrame, desiredRightTransform);

    if(m_additionalFrame.size() != 0){
        m_ik.updateRotationTarget(m_additionalFrame, leftTransform.getRotation().inverse() * m_inertial_R_world.inverse() * m_additionalRotation, m_additionalRotationWeight);
    }

    desiredCoMPosition = leftTransform.inverse() * comPosition;
    if (m_verbose) {
        yInfo() << "Input CoM position: ";
        yInfo() << comPosition.toString();
        yInfo() << "Desired CoM position: ";
        yInfo() << desiredCoMPosition.toString();
    }

    m_ik.setCOMTarget(desiredCoMPosition, 100.0);

    ok = m_ik.setCurrentRobotConfiguration(m_baseTransform,m_feedback);
    if(!ok){
        yError() << "WalkingIK: Error while setting the feedback.";
        return false;
    }

    ok = m_ik.setReducedInitialCondition(&m_baseTransform, &m_guess);
    if(!ok){
        yError() << "WalkingIK: Error while setting the guess.";
        return false;
    }

    ok = m_ik.setDesiredReducedJointConfiguration(m_jointRegularization, m_jointRegularizationWeight);
    if(!ok){
        yError() << "WalkingIK: Error while setting the desired joint configuration.";
        return false;
    }

    ok = m_ik.solve();

    if(!ok){
        yError() << "WalkingIK: Failed in finding a solution.";
        return false;
    }

    m_ik.getReducedSolution(baseTransform, m_qResult);

    lchecker.setRobotState(m_baseTransform, m_qResult, dummyBaseVel, dummyVel,dummygrav);

    iDynTree::Position comError = desiredCoMPosition - lchecker.getCenterOfMassPosition();
    iDynTree::Position footError = desiredRightTransform.getPosition() - lchecker.getRelativeTransform(m_lFootFrame, m_rFootFrame).getPosition();

    if (m_verbose) {
        yInfo() << "CoM error position: "<< comError.toString();
        yInfo() << "Foot position error: "<<footError.toString();
    }

    result = m_qResult;
    m_guess = m_qResult;

    return true;
}

const std::string WalkingIK::getLeftFootFrame() const
{
    return m_lFootFrame;
}

const std::string WalkingIK::getRightFootFrame() const
{
    return m_rFootFrame;
}

bool WalkingIK::usingAdditionalRotationTarget()
{
    return (m_additionalFrame.size() != 0);
}

const iDynTree::VectorDynSize &WalkingIK::desiredJointConfiguration() const
{
    return m_jointRegularization;
}

bool WalkingIK::setAdditionalRotationWeight(double weight)
{
    if (weight < 0){
        yError() << "The additionalRotationWeight is supposed to be non-negative.";
        return false;
    }
    m_additionalRotationWeight = weight;
    return true;
}

double WalkingIK::additionalRotationWeight()
{
    return m_additionalRotationWeight;
}

bool WalkingIK::setDesiredJointsWeight(double weight)
{
    if (weight < 0){
        yError() << "The desiredJointWeight is supposed to be non-negative.";
        return false;
    }
    m_jointRegularizationWeight = weight;
    return true;
}

double WalkingIK::desiredJointWeight()
{
    return m_jointRegularizationWeight;
}
