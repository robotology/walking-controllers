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

namespace WalkingControllers
{
    class NavigationHelper
    {
    private:
        yarp::os::BufferedPort<yarp::os::Bottle> m_replanningTriggerPort; /**< Publishes the flag triggering the navigation's global planner. */
        std::atomic<bool> m_runThreads{false};  /**< Global flag that allows the looping of the threads. */
        std::deque<bool>* m_leftInContact;      /**< Pointer to the deques in Module of the left feet contacts status. */
        std::deque<bool>* m_rightInContact;     /**< Pointer to the deques in Module of the left feet contacts status. */
        double m_navigationReplanningDelay;     /**< Delay in seconds of how much to wait before sending the trigger to the navigation stack after exiting double support. */
        int m_navigationTriggerLoopRate;        /**< Loop rate for the thread computing the navigation trigger*/
        bool m_publishInfo;
        std::thread m_navigationTriggerThread;  /**< Thread for publishing the flag triggering the navigation's global planner. */

        const std::string m_portPrefix = "/navigation_helper";
        
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
         * @param leftInContact deque in Module.cpp of the left feet contacts status.
         * @param rightInContact deque in Module.cpp of the right feet contacts status.
         * @return true/false in case of success/failure.
         */
        bool init(const yarp::os::Searchable& config, 
                    std::deque<bool> &leftInContact, 
                    std::deque<bool> &rightInContact
                    );

        /**
         * Function launched by the looping thread
         */
        void computeNavigationTrigger();
    };
}

#endif
