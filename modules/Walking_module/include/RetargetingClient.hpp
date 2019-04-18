/**
 * @file WalkingRetargetingClient.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

#ifndef RETARGETING_CLIENT_HPP
#define RETARGETING_CLIENT_HPP

#include <iDynTree/Core/Transform.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

class RetargetingClient
{
private:

    iDynTree::Transform m_leftHandTransform;
    iDynTree::Transform m_rightHandTransform;

    yarp::os::BufferedPort<yarp::sig::Vector> m_leftHandTransformPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_rightHandTransformPort;

    bool m_useHandRetargeting;

    void convertYarpVectorPoseIntoTransform(const yarp::sig::Vector& vector,
                                            iDynTree::Transform& transform);

public:

    bool initialize(const yarp::os::Searchable &config,
                    const std::string &name);

    void reset(const iDynTree::Transform& leftHandTransform,
               const iDynTree::Transform& rightHandTransform);

    void getFeedback();

    const iDynTree::Transform& leftHandTransform() const;

    const iDynTree::Transform& rightHandTransform() const;
};

#endif
