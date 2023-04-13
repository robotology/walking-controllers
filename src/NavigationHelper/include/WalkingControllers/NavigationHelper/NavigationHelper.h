/**
 * @file NavigationHelper.h
 * @authors Simone Micheletti <simone.micheletti@iit.it>
 * @copyright 2023 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2023
 */

#ifndef NAVIGATION_HELPER__HPP
#define NAVIGATION_HELPER__HPP

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <atomic>
#include <thread>
#include <deque>

//#include <WalkingControllers/WalkingModule/Module.h>


namespace WalkingControllers
{
    class NavigationHelper
    {
    private:
        yarp::os::BufferedPort<yarp::os::Bottle> m_unicyclePort; /**< Port that streams odometry info of the virtual unicycle. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_replanningTriggerPort; /**< Publishes the flag triggering the navigation's global planner. */
        std::atomic<bool> m_runThreads{false};
        std::deque<bool>* m_leftInContact;
        std::deque<bool>* m_rightInContact;
        double m_navigationReplanningDelay;   /**< Delay in seconds of how much to wait before sending the trigger to the navigation stack after exiting double support. */
        int m_navigationTriggerLoopRate;    /**< Loop rate for the thread computing the navigation trigger*/

        std::thread m_virtualUnicyclePubliserThread; /**< Thread for publishing the state of the unicycle used in the TrajectoryGenerator. */
        std::thread m_navigationTriggerThread; /**< Thread for publishing the flag triggering the navigation's global planner. */

    public:
        NavigationHelper(std::deque<bool> &leftInContact, std::deque<bool> &rightInContact);
        ~NavigationHelper();

        bool setThreads(bool status);
        bool closeThreads();
        bool closeHelper();
        bool init(const yarp::os::Searchable& config);
        void computeNavigationTrigger();
    };
    
    
    

}

#endif
