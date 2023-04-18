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
#include <WalkingControllers/KinDynWrapper/Wrapper.h>
#include <WalkingControllers/TrajectoryPlanner/StableDCMModel.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>

namespace WalkingControllers
{
    class NavigationHelper
    {
    private:
        yarp::os::BufferedPort<yarp::os::Bottle> m_unicyclePort; /**< Port that streams odometry info of the virtual unicycle. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_replanningTriggerPort; /**< Publishes the flag triggering the navigation's global planner. */
        yarp::os::BufferedPort<yarp::os::Bottle> m_feetPort; /**< Feet port vector of feet positions (left, right). */
        yarp::os::BufferedPort<yarp::sig::Vector> m_plannedCoMPositionPort; /**< Desired CoM position port for naviagation purposes. */
        std::atomic<bool> m_runThreads{false};
        std::deque<bool>* m_leftInContact;
        std::deque<bool>* m_rightInContact;
        double m_navigationReplanningDelay;   /**< Delay in seconds of how much to wait before sending the trigger to the navigation stack after exiting double support. */
        int m_navigationTriggerLoopRate;    /**< Loop rate for the thread computing the navigation trigger*/
        bool m_wasInDoubleSupport;
        bool m_publishInfo;
        std::deque<iDynTree::Vector3> m_desiredCoM_Trajectory;  /**< Deque containing the desired CoM trajectory projected on the ground in pose x, y, theta. */

        std::thread m_virtualUnicyclePubliserThread; /**< Thread for publishing the state of the unicycle used in the TrajectoryGenerator. */
        std::thread m_navigationTriggerThread; /**< Thread for publishing the flag triggering the navigation's global planner. */

        const std::string m_portPrefix = "/navigation_helper";

    public:
        NavigationHelper();
        ~NavigationHelper();

        bool setThreads(bool status);
        bool closeThreads();
        bool closeHelper();
        bool init(const yarp::os::Searchable& config, std::deque<bool> &leftInContact, std::deque<bool> &rightInContact,
                    std::unique_ptr<WalkingFK> &FKSolver, 
                    std::unique_ptr<StableDCMModel> &stableDCMModel, 
                    std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                    );
        void computeNavigationTrigger();
        void computeVirtualUnicycleThread(std::unique_ptr<WalkingFK> &FKSolver, 
                                            std::unique_ptr<StableDCMModel> &stableDCMModel, 
                                            std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                                            );

        bool publishPlannedFootsteps(std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator);
        bool publishCoM(bool newTrajectoryMerged, 
                        std::deque<iDynTree::Vector2> &m_DCMPositionDesired, 
                        std::unique_ptr<StableDCMModel> &m_stableDCMModel,
                        std::deque<iDynTree::Transform> &m_leftTrajectory,
                        std::deque<iDynTree::Transform> &m_rightTrajectory);
    };
}

#endif
