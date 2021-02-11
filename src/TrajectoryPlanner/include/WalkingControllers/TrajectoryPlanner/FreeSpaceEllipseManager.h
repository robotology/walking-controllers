/**
 * @file FreeSpaceEllipseManager.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#ifndef WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H
#define WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

// iDynTree
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorFixSize.h>

//UnicyclePlanner
#include <FreeSpaceEllipse.h>

//STD
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

namespace WalkingControllers
{
    class FreeSpaceEllipseManager
    {
    public:

        struct Ellipse {
            iDynTree::MatrixFixSize<2,2> imageMatrix;
            iDynTree::VectorFixSize<2> centerOffset;
        };

        ~FreeSpaceEllipseManager();

        bool initialize(const yarp::os::Searchable& generalConfig, const yarp::os::Searchable& specificConfig);

        bool isNewEllipseAvailable() const;

        Ellipse getEllipse();

    private:

        std::thread m_referenceThread;
        std::atomic<bool> m_isThreadRunning, m_newEllipseReady;
        std::mutex m_mutex;
        yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort;

        FreeSpaceEllipse m_inputEllipse;

        void referenceThread();

    };

}

#endif // WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H
