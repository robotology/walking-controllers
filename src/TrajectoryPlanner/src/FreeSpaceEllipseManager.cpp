// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <WalkingControllers/TrajectoryPlanner/FreeSpaceEllipseManager.h>
#include <WalkingControllers/YarpUtilities/Helper.h>
#include <vector>
#include <tuple>
#include <yarp/os/LogStream.h>
#include <chrono>

void WalkingControllers::FreeSpaceEllipseManager::referenceThread()
{
    while (m_isThreadRunning)
    {
        if (m_inputPort.getPendingReads())
        {
            yarp::os::Bottle* bottleInput = m_inputPort.read(false);

            if (!bottleInput)
            {
                continue;
            }

            FreeSpaceEllipse inputEllipse;

            if(bottleInput->size() == 5)
            {
                double a = 10, b = 10, theta = 0, centerX = 0, centerY = 0;

                std::vector<std::tuple<size_t, double&> >inputs = {{0, a},
                                                                   {1, b},
                                                                   {2, theta},
                                                                   {3, centerX},
                                                                   {4, centerY}};

                for (const auto& i : inputs)
                {
                    if (!bottleInput->get(std::get<0>(i)).isFloat64())
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        yError() << "[FreeSpaceEllipseManager::referenceThread] The input number " << std::get<0>(i) << "(0 based) is not a double.";
                        continue;
                    }

                    std::get<1>(i) = bottleInput->get(std::get<0>(i)).asFloat64();
                }

                if (!inputEllipse.setEllipse(a, b, theta, centerX, centerY))
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    yError() << "[FreeSpaceEllipseManager::referenceThread] Failed to set the new ellipse.";
                    continue;
                }
            //We can also deal with the matrix case eventually
            }
            else
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                yError() << "[FreeSpaceEllipseManager::referenceThread] The number of inputs written on the port are wrong.";
                continue;
            }

            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_inputEllipse = inputEllipse;
                m_newEllipseReady = true;
            }
        }
        else
        {
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(1ms);
        }

    }
}

WalkingControllers::FreeSpaceEllipseManager::~FreeSpaceEllipseManager()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_isThreadRunning = false;

    if (m_referenceThread.joinable())
    {
        m_referenceThread.join();
        m_referenceThread = std::thread();
    }

    m_inputPort.close();
}

bool WalkingControllers::FreeSpaceEllipseManager::initialize(const yarp::os::Searchable &config)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_referenceThread.joinable())
    {
        yError() << "[FreeSpaceEllipseManager::initialize] The internal thread is already running. Have you called this method twice?";
        return false;
    }

    std::string name;
    if(!YarpUtilities::getStringFromSearchable(config, "name", name))
    {
        yError() << "[FreeSpaceEllipseManager::initialize] Unable to get the name from the general config.";
        return false;
    }

    yarp::os::Bottle settingsGroup = config.findGroup("ELLIPSE_MANAGER_SETTINGS");

    yarp::os::Value portName = settingsGroup.check("port_name", yarp::os::Value("freeSpaceEllipse:in"));

    if (!portName.isString())
    {
        yError() << "[FreeSpaceEllipseManager::initialize] Found port_name but it is not a string.";
        return false;
    }

    std::string fullPortName = "/" + name + "/" + portName.toString();

    if (!m_inputPort.open(fullPortName))
    {
        yError() << "[FreeSpaceEllipseManager::initialize] Failed to open port with name " << fullPortName <<".";
        return false;
    }

    m_newEllipseReady = settingsGroup.check("use_initial_ellipse", yarp::os::Value(false)).asBool();

    if (m_newEllipseReady)
    {
        yarp::os::Bottle initialEllipseGroup = config.findGroup("INITIAL_ELLIPSE");

        if (initialEllipseGroup.isNull())
        {
            yError() << "[FreeSpaceEllipseManager::initialize] use_initial_ellipse is set to true, but no group named INITIAL_ELLIPSE has been found.";
            return false;
        }

        double a = 10, b = 10, theta = 0, centerX = 0, centerY = 0;

        std::vector<std::tuple<std::string, double&> >inputs = {{"semi_major_axis", a},
                                                                {"semi_minor_axis", b},
                                                                {"angle", theta},
                                                                {"center_x", centerX},
                                                                {"center_y", centerY}};

        for (auto i : inputs)
        {
            yarp::os::Value input;
            if (!YarpUtilities::getNumberFromSearchable(initialEllipseGroup, std::get<0>(i), std::get<1>(i)))
            {
                yError() << "[FreeSpaceEllipseManager::initialize] The group INITIAL_ELLIPSE is missing some fields.";
                return false;
            }
        }

        if (!m_inputEllipse.setEllipse(a, b, theta, centerX, centerY))
        {
            yError() << "[FreeSpaceEllipseManager::initialize] Failed to set the initial ellipse.";
            return false;
        }
    }

    m_isThreadRunning = true;

    m_referenceThread = std::thread(&FreeSpaceEllipseManager::referenceThread, this);

    return true;
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

    m_newEllipseReady = false;

    return output;
}
