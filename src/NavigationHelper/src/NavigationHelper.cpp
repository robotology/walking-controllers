/**
 * @file NavigationHelper.cpp
 * @authors Simone Micheletti <simone.micheletti@iit.it>
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#include <iostream>
#include <WalkingControllers/NavigationHelper/NavigationHelper.h>

#include <yarp/os/Stamp.h>
#include <yarp/os/NetworkClock.h>

#include <WalkingControllers/YarpUtilities/Helper.h>


using namespace WalkingControllers;

NavigationHelper::NavigationHelper(std::deque<bool> &leftInContact, std::deque<bool> &rightInContact)
{
    m_leftInContact = &leftInContact;
    m_rightInContact = &rightInContact;
}
    
NavigationHelper::~NavigationHelper()
{
    
}

bool NavigationHelper::setThreads(bool status)
{
    m_runThreads = status;
    return true;
}

bool NavigationHelper::closeThreads()
{
    //Close parallel threads
    if (m_runThreads)
    {
        m_runThreads = false;
        yInfo() << "Closing m_virtualUnicyclePubliserThread";
        if(m_virtualUnicyclePubliserThread.joinable())
        {
            m_virtualUnicyclePubliserThread.join();
            m_virtualUnicyclePubliserThread = std::thread();
        }

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
        if(!m_unicyclePort.isClosed())
            m_unicyclePort.close();
        if(!m_replanningTriggerPort.isClosed())
            m_replanningTriggerPort.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return true;
}

bool NavigationHelper::init(const yarp::os::Searchable& config)
{
    //ports for navigation integration
    std::string unicyclePortPortName = "/virtual_unicycle_states:o";
    if (!m_unicyclePort.open(unicyclePortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open virtual_unicycle_states port";
        return false;
    }

    std::string replanningTriggerPortPortName = "/replanning_trigger:o";
    if (!m_replanningTriggerPort.open(replanningTriggerPortPortName))
    {
        yError() << "[NavigationHelper::init] Could not open" << replanningTriggerPortPortName << " port.";
        return false;
    }

    yarp::os::Bottle& trajectoryPlannerOptions = config.findGroup("NAVIGATION");
    m_navigationReplanningDelay = trajectoryPlannerOptions.check("navigationReplanningDelay", yarp::os::Value(0.9)).asFloat64();
    m_navigationTriggerLoopRate = trajectoryPlannerOptions.check("navigationTriggerLoopRate", yarp::os::Value(100)).asInt32();
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

    m_runThreads = true;
    //m_virtualUnicyclePubliserThread = std::thread(&computeVirtualUnicycleThread, this);
    m_navigationTriggerThread = std::thread(&NavigationHelper::computeNavigationTrigger, this);
    return true;
}

// This thread synchronizes the walking-controller with the navigation stack.
// Writes on a port a boolean value when to replan the path
void NavigationHelper::computeNavigationTrigger()
{
    bool trigger = false;   //flag used to fire the wait for sending the navigation replanning trigger
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Starting Thread";
    yarp::os::NetworkClock myClock;
    myClock.open("/clock", "/navigationTriggerClock");
    bool enteredDoubleSupport = false, exitDoubleSupport = true;
    while (m_runThreads)
    {
        // Block the thread until the robot is in the walking state
        if (m_leftInContact == nullptr || m_rightInContact == nullptr)
        {
            //kill thread
            yError() << "[NavigationHelper::computeNavigationTrigger] null pointer to m_leftInContact or m_rightInContact. Killing thread.";
            return;
        }

        //double support check
        if (m_leftInContact->size()>0 && m_rightInContact->size()>0)  //external consistency check
        {
            if (m_leftInContact->at(0) && m_rightInContact->at(0))
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

        //send the replanning trigger after a certain amount of seconds
        if (trigger)
        {
            trigger = false;
            //waiting -> could make it dependant by the current swing step duration
            myClock.delay(m_navigationReplanningDelay);
            yDebug() << "[NavigationHelper::computeNavigationTrigger] Triggering navigation replanning";
            auto& b = m_replanningTriggerPort.prepare();
            b.clear();
            b.add((yarp::os::Value)true);   //send the planning trigger
            m_replanningTriggerPort.write();
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
    }
    yInfo() << "[NavigationHelper::computeNavigationTrigger] Terminated thread";
}