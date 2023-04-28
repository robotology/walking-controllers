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
#include <mutex>

namespace WalkingControllers
{
    class NavigationHelper
    {
    private:
        yarp::os::BufferedPort<yarp::os::Bottle> m_replanningTriggerPort; /**< Publishes the flag triggering the navigation's global planner. */
        std::atomic<bool> m_runThreads{false};      /**< Global flag that allows the looping of the threads. */
        std::deque<bool> m_leftInContact;           /**< Copy of the deques in Module of the left feet contacts status. */
        std::deque<bool> m_rightInContact;          /**< Copy of the deques in Module of the left feet contacts status. */
        double m_navigationReplanningDelay;         /**< Delay in seconds of how much to wait before sending the trigger to the navigation stack after exiting double support. */
        int m_navigationTriggerLoopRate;            /**< Loop rate for the thread computing the navigation trigger*/
        bool m_publishInfo;                         /**< Flag to whether publish information. */
        std::thread m_navigationTriggerThread;      /**< Thread for publishing the flag triggering the navigation's global planner. */
        std::mutex m_updateFeetMutex;    
        bool m_simulationMode{false};               /**< Flag that syncs the trigger delay with the external clock if in simulation. */  

        const std::string m_portPrefix = "/navigation_helper";
        
        /**
         * Function launched by the looping thread
         */
        void computeNavigationTrigger();

    public:
        NavigationHelper();
        ~NavigationHelper();

        /**
         * Close the Navigation Helper Threads and ports
         * @return true/false in case of success/failure.
         */
        bool closeThreads();

        /**
         * Close the Navigation Helper Threads and ports
         * @return true/false in case of success/failure.
         */
        bool closeHelper();

        /**
         * Initialize the Navigation Helper
         * @param config yarp searchable object of the configuration.
         * @return true/false in case of success/failure.
         */
        bool init(const yarp::os::Searchable& config);

        bool updateFeetDeques(const std::deque<bool> &left, const std::deque<bool> &right);
    };
}

#endif
