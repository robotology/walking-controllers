// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_TF_HELPER_H
#define WALKING_CONTROLLERS_TF_HELPER_H

#include <string>
#include <vector>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/Searchable.h>

#include <iDynTree/Transform.h>

namespace WalkingControllers
{

namespace YarpUtilities
{
    class TransformHelper
    {
        std::string m_rootFrame;
        std::string m_baseFrameName;
        std::string m_joystickFrameName;

        yarp::dev::IFrameTransform* m_tfPublisher;
        yarp::dev::PolyDriver m_clientDriver;
        yarp::dev::PolyDriver m_serverDriver;

        std::vector<std::string> m_frames;

        yarp::sig::Matrix m_buffer;

        void convertTransform(const iDynTree::Transform& transform, yarp::sig::Matrix& buffer);

    public:

        /**
         * Destructor.
         */
        ~TransformHelper();
        /**
         * Configure the transform helper.
         * @param config the configuration parameters;
         * @return true/false in case of success/failure.
         */
        bool configure(const yarp::os::Searchable& config);
        /**
         * Set the base transform in the transform server.
         * @param transform is the transform to set.
         * @return true/false in case of success/failure.
         */
        bool setBaseTransform(const iDynTree::Transform & transform);

        /**
         * Set the transform used for interpreting the joystick value in the transform server.
         * @param transform is the transform to set.
         * @return true/false in case of success/failure.
         */
        bool setJoystickTransform(const iDynTree::Transform& transform);

        /**
         * Set a generic transform in the transform server.
         * @param name is the name of the transform;
         * @param transform is the transform to set.
         * @return true/false in case of success/failure.
         */
        bool setTransform(const std::string& name, const iDynTree::Transform& transform);

        /**
         * Get the list of additional frames to publish, obtained from configuration file
         * @return the list of additional frames.
         */
        const std::vector<std::string>& getAdditionalFrames();

        /**
        * Closes the transform helper.
        */
        void close();

    };
}
}

#endif //WALKING_CONTROLLERS_TF_HELPER_H
