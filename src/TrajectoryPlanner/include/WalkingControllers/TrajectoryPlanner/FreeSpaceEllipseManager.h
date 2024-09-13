// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H
#define WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H

// YARP
#include <yarp/os/Searchable.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>

// iDynTree
#include <iDynTree/MatrixFixSize.h>
#include <iDynTree/VectorFixSize.h>

//UnicyclePlanner
#include <FreeSpaceEllipse.h>

//STD
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

namespace WalkingControllers
{
    /**
     * The FreeSpaceEllipseManager class handles the retrieval of a free space ellipse from an external source (a port for the time being).
     */
    class FreeSpaceEllipseManager
    {
    public:

        /**
         * Internal struct to describe an ellipse.
         */
        struct Ellipse {
            iDynTree::MatrixFixSize<2,2> imageMatrix; /**< The matrix mapping a unit circle to an ellipse. **/
            iDynTree::VectorFixSize<2> centerOffset; /**< The position of the center of the ellipse. **/
        };

        /**
         * Destructor
         */
        ~FreeSpaceEllipseManager();

        /**
         * @brief Initialize the free space ellipse manager.
         * At the moment it requires a parameter called name to define the prefix of the input port.
         * The remaining part of the input port can be set via the parameter port_name, that by default is "freeSpaceEllipse:in".
         * @param config The configuration parameters
         * @return True in case of success, false otherwise.
         */
        bool initialize(const yarp::os::Searchable& config);

        /**
         * @brief Check if a new ellipse is available.
         * @return True if a new ellipse is available, false otherwise
         */
        bool isNewEllipseAvailable() const;

        /**
         * @brief Get the ellipse
         * @note isNewEllipseAvailable will return false after calling this method, until a new ellipse is available.
         * @return An Ellipse struct defining the input ellipse.
         */
        Ellipse getEllipse();

    private:

        std::thread m_referenceThread; /** The thread in which the port is read. **/
        std::atomic<bool> m_isThreadRunning; /** Internal flag to control the thread running. **/
        std::atomic<bool> m_newEllipseReady; /** Internal flag to check if a new ellipse is available. **/
        std::mutex m_mutex; /** Internal mutex. **/
        yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort; /** Input port. **/

        FreeSpaceEllipse m_inputEllipse; /** The input ellipse. **/

        void referenceThread(); /** The method reading new ellipses from the port in a separate thread. **/

    };

}

#endif // WALKING_CONTROLLERS_TRAJECTORY_PLANNER_FREESPACEELLIPSEMANAGER_H
