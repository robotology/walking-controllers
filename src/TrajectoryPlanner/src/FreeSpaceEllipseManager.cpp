/**
 * @file FreeSpaceEllipseManager.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2021 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2021
 */

#include <WalkingControllers/TrajectoryPlanner/FreeSpaceEllipseManager.h>
#include <vector>
#include <tuple>
#include <yarp/os/LogStream.h>

void WalkingControllers::FreeSpaceEllipseManager::referenceThread()
{
    while (m_isThreadRunning)
    {
        yarp::os::Bottle* bottleInput = m_inputPort.read(true);

        FreeSpaceEllipse inputEllipse;

        if(bottleInput->size() == 5)
        {
            double a = 0, b = 0, theta = 0, centerX = 0, centerY = 0;

            std::vector<std::tuple<size_t, double*> >inputs = {{0, &a},
                                                               {1, &b},
                                                               {2, &theta},
                                                               {3, &centerX},
                                                               {4, &centerY}};

            for (auto i : inputs)
            {
                if (!bottleInput->get(std::get<0>(i)).isDouble())
                {
                    yError() << "[FreeSpaceEllipseManager::referenceThread] The input number " << std::get<0>(i) << "(0 based) is not a double.";
                    continue;
                }

                *(std::get<1>(i)) = bottleInput->get(std::get<0>(i)).asDouble();
            }

            if (!inputEllipse.setEllipse(a, b, theta, centerX, centerY))
            {
                yError() << "[FreeSpaceEllipseManager::referenceThread] Failed to set the new ellipse.";
                continue;
            }
        //TODOOOOOOOOOO Deal with the matrix case
        }
        else
        {
            yError() << "[FreeSpaceEllipseManager::referenceThread] The number of inputs written on the port are wrong.";
            continue;
        }


    }
}

WalkingControllers::FreeSpaceEllipseManager::~FreeSpaceEllipseManager()
{
    m_isThreadRunning = false;

    if (m_referenceThread.joinable())
    {
        m_referenceThread.join();
        m_referenceThread = std::thread();
    }
}

bool WalkingControllers::FreeSpaceEllipseManager::initialize(const yarp::os::Searchable &generalConfig, const yarp::os::Searchable &specificConfig)
{
    //TODOOOOOOOOOOOOO to be filled
    //Also I need to connect these parts in the walking module
}

bool WalkingControllers::FreeSpaceEllipseManager::isNewEllipseAvailable() const
{
    return m_newEllipseReady;
}

WalkingControllers::FreeSpaceEllipseManager::Ellipse WalkingControllers::FreeSpaceEllipseManager::getEllipse()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    WalkingControllers::FreeSpaceEllipseManager::Ellipse output;

    output.imageMatrix = m_inputEllipse.imageMatrix();
    output.centerOffset = m_inputEllipse.centerOffset();

    return output;
}
