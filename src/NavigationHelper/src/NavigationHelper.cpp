/**
 * @file NavigationHelper.cpp
 * @authors Simone Micheletti <simone.micheletti@iit.it>
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#include <iostream>
#include <thread>
#include <WalkingControllers/NavigationHelper/NavigationHelper.h>

#include <WalkingControllers/YarpUtilities/Helper.h>


using namespace WalkingControllers;

NavigationHelper::NavigationHelper()
{
}

NavigationHelper::~NavigationHelper()
{
}

bool NavigationHelper::closeThreads()
{
    //Close parallel threads
    if (m_runThreads)
    {
        m_runThreads = false;

        yInfo() << "Closing m_navigationTriggerThread";
        if(m_navigationTriggerThread.joinable())
        {
            m_navigationTriggerThread.join();
            m_navigationTriggerThread = std::thread();
        }
    }
    return true;
}

bool NavigationHelper::closeHelper()
{
    try
    {
        //close threads first
        if (m_runThreads)
        {
            closeThreads();
        }
        //close ports
        if(!m_replanningTriggerPort.isClosed())
            m_replanningTriggerPort.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

bool NavigationHelper::init(const yarp::os::Searchable& config)
{
    m_navigationReplanningDelay = config.check("navigationReplanningDelay", yarp::os::Value(0.9)).asFloat64();
    m_navigationTriggerLoopRate = config.check("navigationTriggerLoopRate", yarp::os::Value(100)).asInt32();
    m_publishInfo = config.check("publishNavigationInfo", yarp::os::Value(false)).asBool();
    m_simulationMode = config.check("simulationMode", yarp::os::Value(false)).asBool();
    if (config.check("plannerMode", yarp::os::Value("manual")).asString() == "navigation")
    {
        //if in navigation mode we need this parameter to be true
        m_publishInfo = true;
    }
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        yInfo() << "[NavigationHelper::init] Configuring NavigationHelper without publishing infos on ports ";
        return true;
    }

    // format check
    if (m_navigationTriggerLoopRate<=0)
    {
        yError() << "[configure] navigationTriggerLoopRate must be strictly positive, instead is: " << m_navigationTriggerLoopRate;
        return false;
    }
    if (m_navigationReplanningDelay<0)
    {
        yError() << "[configure] navigationTriggerLoopRate must be positive, instead is: " << m_navigationReplanningDelay;
        return false;
    }

    std::string replanningTriggerPortPortName = m_portPrefix + "/replanning_trigger:o";
    if (!m_replanningTriggerPort.open(replanningTriggerPortPortName))
    {
        yError() << "[NavigationHelper::init] Could not open" << replanningTriggerPortPortName << " port.";
        return false;
    }

    m_runThreads = true;
    m_navigationTriggerThread = std::thread(&NavigationHelper::computeNavigationTrigger, this);
    return true;
}

// This thread synchronizes the walking-controller with the navigation stack.
// Writes on a port a boolean value when to replan the path
void NavigationHelper::computeNavigationTrigger()
{
    if (!m_publishInfo)
    {   //exit the function if the infos are not needed
        return;
    }
    bool trigger = false;   //flag used to fire the wait for sending the navigation replanning trigger
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Starting Thread";
    yarp::os::NetworkClock myClock;
    if (m_simulationMode)
    {
        myClock.open("/clock", "/navigationTriggerClock");
    }
    bool enteredDoubleSupport = false, exitDoubleSupport = true;
    while (m_runThreads)
    {
        {
            std::unique_lock<std::mutex> lock(m_updateFeetMutex);
            if (m_leftInContact.size()>0 && m_rightInContact.size()>0)  //external consistency check
            {
                if (m_leftInContact.at(0) && m_rightInContact.at(0))    //double support check
                {
                    if (exitDoubleSupport)
                    {
                        enteredDoubleSupport = true;
                        exitDoubleSupport = false;
                    }
                }
                else
                {
                    if (! exitDoubleSupport)
                    {
                        trigger = true; //in this way we have only one trigger each exit of double support
                    }
                    exitDoubleSupport = true;
                }
            }
            else
            {
                trigger = false;
            }
        }
        
        //send the replanning trigger after a certain amount of seconds
        if (trigger)
        {
            trigger = false;
            //waiting -> could make it dependant by the current swing step duration
            //if in simulation we need to sync the clock, we can't use the system clock
            if (m_simulationMode)
            {
                myClock.delay(m_navigationReplanningDelay);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(m_navigationReplanningDelay*1000));
            }
            yInfo() << "[NavigationHelper::computeNavigationTrigger] Triggering navigation replanning";
            auto& b = m_replanningTriggerPort.prepare();
            b.clear();
            b.add((yarp::os::Value)true);   //send the planning trigger
            m_replanningTriggerPort.write();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
    }
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Terminated thread";
}


bool NavigationHelper::updateFeetDeques(const std::deque<bool> &left, const std::deque<bool> &right)
{
    std::unique_lock<std::mutex> lock(m_updateFeetMutex);
    m_leftInContact = left;
    m_rightInContact = right;
    return true;
}
