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

#include <yarp/os/Stamp.h>
#include <yarp/os/NetworkClock.h>

#include <WalkingControllers/YarpUtilities/Helper.h>
#include <WalkingControllers/TrajectoryPlanner/TrajectoryGenerator.h>


using namespace WalkingControllers;

NavigationHelper::NavigationHelper()
{
    m_wasInDoubleSupport = false;
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
        yInfo() << "Closing m_virtualUnicyclePubliserThread";
        if(m_virtualUnicyclePubliserThread.joinable())
        {
            m_virtualUnicyclePubliserThread.join();
            m_virtualUnicyclePubliserThread = std::thread();
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
        if(!m_feetPort.isClosed())
            m_feetPort.close();
        if(!m_plannedCoMPositionPort.isClosed())
            m_plannedCoMPositionPort.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return false;
    }
    return true;
}

bool NavigationHelper::init(const yarp::os::Searchable& config, std::deque<bool> &leftInContact, std::deque<bool> &rightInContact,
                        std::unique_ptr<WalkingFK> &FKSolver, 
                        std::unique_ptr<StableDCMModel> &stableDCMModel, 
                        std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator)
{
    m_leftInContact = &leftInContact;
    m_rightInContact = &rightInContact;

    m_navigationTriggerLoopRate = config.check("navigationTriggerLoopRate", yarp::os::Value(100)).asInt32();
    m_publishInfo = config.check("publishNavigationInfo", yarp::os::Value(false)).asBool();
    if (config.check("plannerMode", yarp::os::Value("manual")).asString() == "navigation")
    {
        //if in navigation mode we need this parameter to be true
        m_publishInfo = true;
    }
    
    // format check
    if (m_navigationTriggerLoopRate<=0)
    {
        yError() << "[configure] navigationTriggerLoopRate must be strictly positive, instead is: " << m_navigationTriggerLoopRate;
        return false;
    }
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        yInfo() << "[NavigationHelper::init] Configuring NavigationHelper without publishing infos on ports ";
        return true;
    }
    
    //ports for navigation integration
    std::string unicyclePortPortName = m_portPrefix + "/virtual_unicycle_states:o";
    if (!m_unicyclePort.open(unicyclePortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open virtual_unicycle_states port";
        return false;
    }

    std::string feetPositionsPortPortName = m_portPrefix + "/feet_positions:o";
    if (!m_feetPort.open(feetPositionsPortPortName))
    {
        yError() << "[WalkingModule::configure] Could not open feet_positions port";
        return false;
    }

    // open CoM planned trajectory port for navigation integration
    std::string plannedCoMPositionPortName = m_portPrefix + "/planned_CoM/data:o";
    if (!m_plannedCoMPositionPort.open(plannedCoMPositionPortName))
    {
        yError() << "[WalkingModule::configure] Could not open " << plannedCoMPositionPortName << " port.";
        return false;
    }

    m_runThreads = true;
    m_virtualUnicyclePubliserThread = std::thread(&NavigationHelper::computeVirtualUnicycleThread, this, std::ref(FKSolver), std::ref(stableDCMModel), std::ref(trajectoryGenerator));
    return true;
}

// This thread publishes the internal informations of the virtual unicycle extracted from the feet and the CoM speed
// It's the internal odometry
void NavigationHelper::computeVirtualUnicycleThread(std::unique_ptr<WalkingFK> &FKSolver, 
                                                    std::unique_ptr<StableDCMModel> &stableDCMModel, 
                                                    std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator
                                                    )
{
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        return;
    }
    
    yInfo() << "[NavigationHelper::computeVirtualUnicycleThread] Starting Thread";
    bool inContact = false;
    while (m_runThreads)
    {
        //if (m_robotState != 4)
        //{
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
        //    continue;
        //}

        iDynTree::Vector3 virtualUnicyclePose, virtualUnicycleReference;
        std::string stanceFoot;
        iDynTree::Transform footTransformToWorld, root_linkTransform;
        //Stance foot check
        if (m_leftInContact->size() > 0)
        {
            if (m_leftInContact->at(0))
            {
                stanceFoot = "left";
                footTransformToWorld = FKSolver->getLeftFootToWorldTransform();
            }
            else
            {
                stanceFoot = "right";
                footTransformToWorld = FKSolver->getRightFootToWorldTransform();
            }
            inContact = true;
        }
        else if (m_rightInContact->size() > 0)
        {
            if (m_rightInContact->at(0))
            {
                stanceFoot = "right";
                footTransformToWorld = FKSolver->getRightFootToWorldTransform();
            }
            else
            {
                stanceFoot = "left";
                footTransformToWorld = FKSolver->getLeftFootToWorldTransform();
            }
            inContact = true;
        }
        else
        {
            inContact = false;
        }
        root_linkTransform = FKSolver->getRootLinkToWorldTransform();   //world -> rootLink transform
        //Data conversion and port writing
        if (inContact)
        {
            if (trajectoryGenerator->getUnicycleState(stanceFoot, virtualUnicyclePose, virtualUnicycleReference))
            {
                //send data
                yarp::os::Stamp stamp (0, yarp::os::Time::now());   //move to private member of the class
                m_unicyclePort.setEnvelope(stamp);
                auto& data = m_unicyclePort.prepare();
                data.clear();
                auto& unicyclePose = data.addList();
                unicyclePose.addFloat64(virtualUnicyclePose(0));
                unicyclePose.addFloat64(virtualUnicyclePose(1));
                unicyclePose.addFloat64(virtualUnicyclePose(2));
                auto& referencePose = data.addList();
                referencePose.addFloat64(virtualUnicycleReference(0));
                referencePose.addFloat64(virtualUnicycleReference(1));
                referencePose.addFloat64(virtualUnicycleReference(2));
                data.addString(stanceFoot);

                //add root link stransform: X, Y, Z, r, p, yaw,
                auto& transform = data.addList();
                transform.addFloat64(root_linkTransform.getPosition()(0));
                transform.addFloat64(root_linkTransform.getPosition()(1));
                transform.addFloat64(root_linkTransform.getPosition()(2));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(0));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(1));
                transform.addFloat64(root_linkTransform.getRotation().asRPY()(2));

                //planned velocity of the CoM
                auto comVel = stableDCMModel->getCoMVelocity();
                auto& velData = data.addList();
                velData.addFloat64(comVel(0));
                velData.addFloat64(comVel(1));
                m_unicyclePort.write();
            }
            else
            {
                yError() << "[NavigationHelper::computeVirtualUnicycleThread] Could not getUnicycleState from m_trajectoryGenerator (no data sent).";
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/m_navigationTriggerLoopRate));
    }
    yInfo() << "[NavigationHelper::computeVirtualUnicycleThread] Terminated thread";
}

bool NavigationHelper::publishPlannedFootsteps(std::unique_ptr<TrajectoryGenerator> &trajectoryGenerator)
{
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        return true;
    }
    if (m_feetPort.isClosed())
    {
        yError() << "[NavigationHelper::publishPlannedFootsteps] Feet port closed (no data sent).";
        return false;
    }
    
    //Send footsteps info on port anyway (x, y, yaw) wrt root_link
    std::vector<TrajectoryGenerator::Pose> leftFootprints, rightFootprints;
    if (trajectoryGenerator->getFootprints(leftFootprints, rightFootprints))
    {
        if (leftFootprints.size()>0 && rightFootprints.size()>0)
        {
            auto& feetData = m_feetPort.prepare();
            feetData.clear();
            auto& rightFeet = feetData.addList();
            auto& leftFeet = feetData.addList();
            //left foot
            for (size_t i = 0; i < leftFootprints.size(); ++i)
            {
                auto& pose = leftFeet.addList();
                pose.addFloat64(leftFootprints[i].x);   //x
                pose.addFloat64(leftFootprints[i].y);   //y
                pose.addFloat64(leftFootprints[i].theta);   //yaw
            }
            //right foot
            for (size_t j = 0; j < rightFootprints.size(); ++j)
            {
                auto& pose = rightFeet.addList();
                pose.addFloat64(rightFootprints[j].x);   //x
                pose.addFloat64(rightFootprints[j].y);   //y
                pose.addFloat64(rightFootprints[j].theta);   //yaw
            }
            m_feetPort.write();
        }
    }
    return true;
}

bool NavigationHelper::publishCoM(bool newTrajectoryMerged, std::deque<iDynTree::Vector2> &m_DCMPositionDesired, 
                                std::unique_ptr<StableDCMModel> &m_stableDCMModel,
                                std::deque<iDynTree::Transform> &m_leftTrajectory,
                                std::deque<iDynTree::Transform> &m_rightTrajectory)
{
    if (!m_publishInfo)
    {   //exit the funnction if the infos are not needed
        return true;
    }
    
    // streaming CoM desired position and heading for Navigation algorithms         
    //if(!m_leftTrajectory.size() == m_DCMPositionDesired.size())
    //{
    //    yWarning() << "[WalkingModule::updateModule] Inconsistent dimenstions. CoM planned poses will be augmented" ;
    //}

    double yawLeftp, yawRightp, meanYawp;
    yarp::sig::Vector& planned_poses = m_plannedCoMPositionPort.prepare();
    planned_poses.clear();

    bool saveCoM = false;   //flag to whether save the CoM trajectory only once each double support
    if (m_leftInContact->size()>0 && m_rightInContact->size()>0)  //consistency check
    {
        //evaluate the state of the contacts
        if (m_leftInContact->at(0) && m_rightInContact->at(0))  //double support
        {
            if (!m_wasInDoubleSupport)
            {
                saveCoM = true;
                m_wasInDoubleSupport = true;
            }
            else    //I still need to assign a value to the flag
            {
                saveCoM = false;
            }
                
            //override the previous state check if there has been a merge of a new trajectory on this thread cycle
            //in this way we have the latest updated trajectory
            if (newTrajectoryMerged)
            {
                saveCoM = true;
            }
        }
        else
        {
            saveCoM = false;
            if (m_wasInDoubleSupport)
            {
                m_wasInDoubleSupport = false;
            }
        }
    }
        
    if (saveCoM)
    {
        m_desiredCoM_Trajectory.clear();
    }
    for (auto i = 0; i < m_DCMPositionDesired.size(); i++)
    {
        m_stableDCMModel->setInput(m_DCMPositionDesired[i]);
        m_stableDCMModel->integrateModel();
        yawLeftp =  m_leftTrajectory[i].getRotation().asRPY()(2);
        yawLeftp =  m_rightTrajectory[i].getRotation().asRPY()(2);
        meanYawp = std::atan2(std::sin(yawLeftp) + std::sin(yawRightp),
                                std::cos(yawLeftp) + std::cos(yawRightp));
        
        planned_poses.push_back(m_stableDCMModel->getCoMPosition().data()[0]);
        planned_poses.push_back(m_stableDCMModel->getCoMPosition().data()[1]);
        planned_poses.push_back(meanYawp);
        //saving data also internally -> should do this only once per double support
        if (saveCoM)
        {
            iDynTree::Vector3 pose;
            pose(0) = m_DCMPositionDesired[i](0);
            pose(1) = m_DCMPositionDesired[i](1);;
            pose(2) = meanYawp;
            m_desiredCoM_Trajectory.push_back(pose);
        }
    }
    //m_newTrajectoryMerged = false;  //reset flag
    m_plannedCoMPositionPort.write();
    m_stableDCMModel->reset(m_DCMPositionDesired.front());
    return true;
}