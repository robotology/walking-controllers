/**
 * @file WalkingRetargetingClient.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef RETARGETING_CLIENT_HPP
#define RETARGETING_CLIENT_HPP

#include <memory>

#include <iDynTree/Core/Transform.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/minJerkCtrl.h>

class RetargetingClient
{
private:

    iDynTree::Transform m_leftHandTransform;
    iDynTree::Transform m_rightHandTransform;

    yarp::sig::Vector m_leftHandTransformYarp;
    yarp::sig::Vector m_rightHandTransformYarp;

    yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandTransformPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandTransformPort;

    bool m_useHandRetargeting;

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_leftHandSmoother; /**< Minimum jerk trajectory
                                                                       for the left hand. */

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_rightHandSmoother; /**< Minimum jerk trajectory
                                                                        for the right hand. */


    void convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                            iDynTree::Transform& transform);

public:

    bool initialize(const yarp::os::Searchable &config,
                    const std::string &name,
                    const double &period);

    void reset(const iDynTree::Transform& leftHandTransform,
               const iDynTree::Transform& rightHandTransform);

    void getFeedback();

    const iDynTree::Transform& leftHandTransform() const;

    const iDynTree::Transform& rightHandTransform() const;
};

#endif
