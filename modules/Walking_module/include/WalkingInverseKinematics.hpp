/**
 * @file WalkingInverseKinematics.hpp
 * @authors Stefano Dafarra
 *          Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ICUB_WALKINGIKV2_H
#define ICUB_WALKINGIKV2_H

// iDynTree
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/InverseKinematics.h>
#include <string>

namespace yarp {
    namespace os {
        class Searchable;
        class Value;
    }
}

namespace iDynTree {
    class Transform;
    class VectorDynSize;
    class Model;
    class Position;
}

/**
 * Computes the inverse kinematics for a walking problem.
 * The optimisation problem this class solves is the following:
 * \f[
 *
 * \f]
 */
class WalkingIK
{

    bool m_verbose;
    iDynTree::InverseKinematics m_ik;

    std::string m_lFootFrame;
    std::string m_rFootFrame;
    std::string m_additionalFrame;

    iDynTree::Transform m_baseTransform;
    iDynTree::Rotation  m_additionalRotation;
    iDynTree::Rotation  m_inertial_R_world;

    iDynTree::VectorDynSize m_jointRegularization, m_guess, m_feedback, m_qResult;

    //DEBUG
    iDynTree::KinDynComputations lchecker;
    iDynTree::VectorDynSize dummyVel;
    iDynTree::Twist dummyBaseVel;
    iDynTree::Vector3 dummygrav;

    int solverVerbosity;
    double maxCpuTime;

    bool m_prepared;

    double m_additionalRotationWeight, m_jointRegularizationWeight;

    bool prepareIK();

public:

    /**
     * Constructor
     */
    WalkingIK();

    /**
     * Destructor
     */
    ~WalkingIK();

    /**
     * Set the verbosity level
     *
     * @param verboseMode the verbosity mode
     */
    void setVerboseMode(bool verboseMode);

    /**
     * Initialize the walking IK
     * @param ikOption the options for the IK
     * @param jointList list of joints to be considered for the inverse kinematics
     * @return true on success, false otherwise
     */
    bool initialize(yarp::os::Searchable& ikOption, const iDynTree::Model& model, const std::vector<std::string>& jointList);

    /**
     * Sets the URDF model
     * @param URDFfilename path to the URDF file
     * @param consideredJoints joints to be considered. If empty all joints in the model will be used
     * @return true on success, false otherwise
     */
    bool setModel(const iDynTree::Model& model,
                  const std::vector< std::string > & consideredJoints = std::vector<std::string>());

    bool setFootFrame(const std::string& foot,
                      const std::string& footFrame);

    bool updateIntertiaToWorldFrameRotation(const iDynTree::Rotation& inertial_R_worldFrame);

    bool updateAdditionalRotation(const iDynTree::Rotation& additionalRotation); //Defined in a inertial frame, with the z pointing upwards

    bool setFullModelFeedBack(const iDynTree::VectorDynSize& feedback); //IMPROVEMENT: We should consider that the tracking of feet trajectory may not be perfect. This may lead the robot to fall since the loop is not closed over the position of the feet.

    bool setInitialGuess(const iDynTree::VectorDynSize& guess);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    /**
     * Compute the inverse kinematics
     * @param startingFoot the starting stance foot
     * @param guess the initial guess for the solution
     * @return true on success, false otherwise
     */
    bool computeIK(const iDynTree::Transform& leftTransform,
                   const iDynTree::Transform& rightTransform,
                   const iDynTree::Position& comPosition,
                   iDynTree::VectorDynSize& result);


    const std::string getLeftFootFrame() const;

    const std::string getRightFootFrame() const;

    bool usingAdditionalRotationTarget();

    const iDynTree::VectorDynSize& desiredJointConfiguration() const;

    bool setAdditionalRotationWeight(double weight);

    double additionalRotationWeight();

    bool setDesiredJointsWeight(double weight);

    double desiredJointWeight();

    /**
     * Get desired neck orientation.
     * @param output
     * @return true/false in case of success/failure.
     */
    bool getDesiredNeckOrientation(iDynTree::Vector3& output);
};

#endif // end of ICUB_WALKINGIK_H
