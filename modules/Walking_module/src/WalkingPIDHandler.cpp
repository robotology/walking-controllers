/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file WalkingPIDHandler.cpp
 * @authors: Stefano Dafarra  <stefano.dafarra@iit.it>
 */

#include "WalkingPIDHandler.hpp"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>

#include <sstream>
#include <cmath>

WalkingPIDHandler::WalkingPIDHandler()
    :m_useGainScheduling(false)
    ,m_pidInterface(nullptr)
    ,m_phaseInitTime(0.0)
    ,m_previousPhase(PIDPhase::Default)
    ,m_currentPIDIndex(-1) //DEFAULT
    ,m_desiredPIDIndex(-1)
    ,m_firmwareDelay(0.0)
    ,m_smoothingTime(1.0)
{
}

WalkingPIDHandler::~WalkingPIDHandler()
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_useGainScheduling = false;
        m_conditionVariable.notify_one();
    }

    if (m_handlerThread.joinable()){
        m_handlerThread.join();
        m_handlerThread = std::thread();
    }
}

bool WalkingPIDHandler::parsePIDGroup(const yarp::os::Bottle *group, PIDmap& pidMap)
{
    if (!group){
        yError() << "Empty group pointer.";
        return false;
    }

    pidMap.clear();
    for (int i = 1; i < group->size(); ++i){
        yarp::os::Value subgroup = group->get(i);
        if (isPIDElement(subgroup)){
            std::string jointName;
            yarp::dev::Pid desiredPID;
            if (!parsePIDElement(subgroup, jointName, desiredPID))
                return false;

            PIDmap::const_iterator original = m_originalPID.find(jointName);

            yarp::dev::Pid safePID;
            if(original != m_originalPID.end()){
                safePID = original->second;
                safePID.setKp(desiredPID.kp);
                safePID.setKi(desiredPID.ki);
                safePID.setKd(desiredPID.kd);
            } else {
                yWarning() << "No joint found with name " << jointName;
                safePID = desiredPID;
            }

            std::pair<PIDmap::iterator, bool> result = pidMap.insert(PIDmap::value_type(jointName, safePID));
            if (!(result.second)) {
                yError() << "Error while inserting item "<< jointName <<" in the map";
                return false;
            }
        }
    }
    return true;
}

bool WalkingPIDHandler::parseDefaultPIDGroup(const yarp::os::Bottle *defaultGroup)
{
    if (!defaultGroup){
        yError() << "Empty default group pointer.";
        return false;
    }

    m_defaultPID = m_originalPID;
    for (int i = 1; i < defaultGroup->size(); ++i){
        yarp::os::Value subgroup = defaultGroup->get(i);
        if (isPIDElement(subgroup)){
            std::string jointName;
            yarp::dev::Pid desiredPID;
            if (!parsePIDElement(subgroup, jointName, desiredPID))
                return false;

            PIDmap::iterator result = m_defaultPID.find(jointName);
            if (result != m_originalPID.end()) {
                result->second.setKp(desiredPID.kp);
                result->second.setKi(desiredPID.ki);
                result->second.setKd(desiredPID.kd);
            } else {
                yWarning() << "No joint found with the name " << jointName <<". Skipping the insertion of the default PID.";
            }
        }
    }
    return true;

}

bool WalkingPIDHandler::parsePIDElement(const yarp::os::Value &groupElement, std::string &jointName, yarp::dev::Pid &pid)
{
    if (groupElement.isList()){
        yarp::os::Bottle* element;
        element = groupElement.asList();
        if ((element->get(0).isString()) && (element->size() == 4)) {
            jointName = element->get(0).toString();
        } else {
            yError() << "Invalid element in the PID file: " << element->toString();
            return false;
        }

        for (int g = 1; g < 4; ++g) {
            yarp::os::Value gain = element->get(g);
            if(!(gain.isDouble())&& !(gain.isInt())){
                yError() << "The gains are supposed to be numeric. " << jointName << ": " <<  element->get(g).toString();
                return false;
            }
        }
        pid.setKp(element->get(1).asDouble());
        pid.setKi(element->get(2).asDouble());
        pid.setKd(element->get(3).asDouble());
    } else {
        return false;
    }
    return true;
}

bool WalkingPIDHandler::parsePIDConfigurationFile(const yarp::os::Bottle &PIDSettings)
{
    m_useGainScheduling = PIDSettings.check("useGainScheduling", yarp::os::Value(false)).asBool();
    m_firmwareDelay = PIDSettings.check("firmwareDelay", yarp::os::Value(0.0)).asDouble();
    double smoothingTime = PIDSettings.check("smoothingTime", yarp::os::Value(1.0)).asDouble();

    if (smoothingTime <= 0.0) {
        yError() << "The smoothing time is supposed to be positive.";
        return false;
    }
    m_smoothingTime = smoothingTime;


    if (m_firmwareDelay < 0){
        yError() << "The firmwareDelay field is expected to be non-negative.";
        return false;
    }

    bool defaultFound = false;
    for (int g = 1; g < PIDSettings.size(); ++g){
        if (PIDSettings.get(g).isList()){
            yarp::os::Bottle *group = PIDSettings.get(g).asList();
            if (group->get(0).toString() == "DEFAULT"){
                yInfo() << "Parsing DEFAULT PID group.";

                if(!parseDefaultPIDGroup(group))
                    return false;

                defaultFound = true;
            } else if (group->check("activationPhase")){
                std::string name = group->get(0).toString();
                yInfo() << "Parsing " << name << " PID group.";

                yarp::os::Value phaseInput = group->find("activationPhase");
                PIDPhase phase;
                if (!fromStringToPIDPhase(phaseInput.asString(), phase))
                    return false;

                double activationOffset = group->check("activationOffset", yarp::os::Value(0.0)).asDouble();
//                double smoothingTime = group->check("smoothingTime", yarp::os::Value(1.0)).asDouble(); //For the time being we use a common smoothingTime

                PIDmap groupMap;
                if (!parsePIDGroup(group, groupMap))
                    return false;

                m_PIDs.insert(m_PIDs.end(), PIDSchedulingObject(name, phase, activationOffset, groupMap));

                if (!(m_PIDs.back().setSmoothingTime(m_smoothingTime))){ //For the time being we use a common smoothingTime
                    yError() << "Failed is setting smoothingTime for group " << name <<".";
                    m_PIDs.pop_back();
                    return false;
                }
            }
        }
    }
    if (!defaultFound){
        yError() << "DEFAULT PID group not found.";
        return false;
    }
    return true;
}

bool WalkingPIDHandler::fromStringToPIDPhase(const std::string &input, PIDPhase &output)
{
    if (input == "SWING_LEFT"){
        output = PIDPhase::SwingLeft;
    } else if (input == "SWING_RIGHT"){
        output = PIDPhase::SwingRight;
    } else if (input == "SWITCH"){
        output = PIDPhase::Switch;
    } else {
        yError() << "Unrecognized PID Phase " << input;
        return false;
    }
    return true;
}

bool WalkingPIDHandler::guessPhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed)
{
    if (leftIsFixed.size() != rightIsFixed.size()){
        yError() << "Incongruous dimension of the leftIsFixed and rightIsFixed vectors.";
        return false;
    }

    if (m_phases.size() != leftIsFixed.size())
        m_phases.resize(leftIsFixed.size());

    for (size_t instant = 0; instant < leftIsFixed.size(); ++instant){
        if (leftIsFixed[instant] && rightIsFixed[instant]){
            m_phases[instant] = PIDPhase::Switch;
        } else if (leftIsFixed[instant]) {
            m_phases[instant] = PIDPhase::SwingRight;
        } else if (rightIsFixed[instant]) {
            m_phases[instant] = PIDPhase::SwingLeft;
        } else {
            yError() << "Unrecognized phase.";
            return false;
        }
    }
    return true;
}

void WalkingPIDHandler::setPIDThread()
{
    double smoothingTime = 1.0;
    std::string name;
    PIDmap oldPIDs, desiredPIDs, defaultPIDs;
    bool previousWasDefault = false;
    AxisMap axisMap;

    while (m_useGainScheduling){
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_conditionVariable.wait(lock, [&]{return (((m_desiredPIDIndex != -1) && (m_desiredPIDIndex != m_currentPIDIndex)) || !m_useGainScheduling);});
            if (m_useGainScheduling){

                if (m_currentPIDIndex == -1){
                    previousWasDefault = true;
                    oldPIDs = m_defaultPID;
                } else {
                    oldPIDs = m_PIDs[static_cast<size_t>(m_currentPIDIndex)].getDesiredGains();
                    previousWasDefault = false;
                }

                m_currentPIDIndex = m_desiredPIDIndex;
                defaultPIDs = m_defaultPID;
                axisMap = m_axisMap;
                smoothingTime = m_PIDs[static_cast<size_t>(m_desiredPIDIndex)].smoothingTime();
                desiredPIDs = m_PIDs[static_cast<size_t>(m_desiredPIDIndex)].getDesiredGains();
                name = m_PIDs[static_cast<size_t>(m_desiredPIDIndex)].name();

                yInfo() << "Inserting " << name << " PID group.";
            }
        }
        if (!m_useGainScheduling)
            break;

        if (previousWasDefault){
            if (!setPID(desiredPIDs, axisMap, smoothingTime)){
                yError() << "Unable to set the PIDs for group " << name;
            }
        } else {
            if (!setAndRestorePIDs(desiredPIDs, oldPIDs, defaultPIDs, axisMap, smoothingTime)){
                yError() << "Unable to set the PIDs for group " << name;
            }
        }
    }
}

/*
  bool WalkingPIDHandler::getSmoothingTimes(yarp::os::Bottle &defaultSmoothingTime)
  {
  defaultSmoothingTime.clear();

  if (!(m_remoteVariables->getRemoteVariable("posPidSlopeTime", defaultSmoothingTime))) {
  yError() << "Unable to get the posPidSlopeTime remote variable.";
  return false;
  }

  yInfo() << "Read smoothing times: " << defaultSmoothingTime.toString();

  return true;
  }

  bool WalkingPIDHandler::setSmoothingTimes(yarp::os::Bottle &desiredSmoothingTime)
  {
  if (desiredSmoothingTime.isNull()){
  yError() << "The bottle containing the desired smoothing times is empty";
  return false;
  }

  yInfo() << "Set smoothing times: " << desiredSmoothingTime.toString();


  if (!(m_remoteVariables->setRemoteVariable("posPidSlopeTime", desiredSmoothingTime))){
  yError() << "Error while setting the posPidSlopeTime remote variable.";
  return false;
  }
  return true;
  }

  bool WalkingPIDHandler::setGeneralSmoothingTime(double smoothingTime)
  {
  if (m_originalSmoothingTimesInMs.isNull()){
  yError() << "The bottle containing the original smoothing times is empty";
  return false;
  }

  yarp::os::Bottle newRemoteVariable;
  int smoothingTimeinMs = static_cast<int>(std::round(smoothingTime*1000));

  for (int i = 0; i < m_originalSmoothingTimesInMs.size(); ++i){

  yarp::os::Bottle &newList = newRemoteVariable.addList();

  if (m_originalSmoothingTimesInMs.get(i).isList()) {
  for (int j = 0; j < m_originalSmoothingTimesInMs.get(i).asList()->size(); ++j){
  newList.addInt(smoothingTimeinMs);
  }
  } else {
  yError() << "The structure of the original smoothing times is not the expected one.";
  }
  }

  return setSmoothingTimes(newRemoteVariable);
  } */

bool WalkingPIDHandler::setGeneralSmoothingTime(double smoothingTime)
{
    int smoothingTimeinMs = static_cast<int>(std::round(smoothingTime*1000));

    if (!(m_remoteControlBoards.get(0).isList())){
        yError() << "The remoteControlBoards variable does not contain any list.";
        return false;
    }

    yarp::os::Bottle &remoteControlBoardsList = *(m_remoteControlBoards.get(0).asList());

    yarp::dev::PolyDriver tempDriver;
    yarp::dev::IRemoteVariables* tempRemoteVariable;

    yarp::os::Property tempOptions;
    tempOptions.put("local", "/pidHandler/temp");
    tempOptions.put("device", "remote_controlboard");

    for (int rcb = 0; rcb < remoteControlBoardsList.size(); ++rcb) {
        tempOptions.put("remote", remoteControlBoardsList.get(rcb));
        if (!tempDriver.open(tempOptions)) {
            yError() << "Error while opening " << remoteControlBoardsList.get(rcb).asString() << " control board.";
            return false;
        }

        if (!tempDriver.view(tempRemoteVariable) || !tempRemoteVariable) {
            yError() << "Cannot obtain IRemoteVariables interface in control board "<< remoteControlBoardsList.get(rcb).asString();
            return false;
        }

        yarp::os::Bottle input;

        if (!(tempRemoteVariable->getRemoteVariable("posPidSlopeTime", input))) {
            yError() << "Unable to get the posPidSlopeTime remote variable in control board "<< remoteControlBoardsList.get(rcb).asString();
            return false;
        }

        yarp::os::Bottle output;

        for (int i = 0; i < input.size(); ++i) {
            if (input.get(i).isList()){
                yarp::os::Bottle &innerOutput = output.addList();
                yarp::os::Bottle *innerInput = input.get(i).asList();
                for (int j = 0; j < innerInput->size(); ++j)
                    innerOutput.addInt(smoothingTimeinMs);
            } else {
                output.addInt(smoothingTimeinMs);
            }
        }

        if (!(tempRemoteVariable->setRemoteVariable("posPidSlopeTime", output))){
            yError() << "Error while setting the posPidSlopeTime remote variable in control board "<< m_remoteControlBoards.get(rcb).asString();
            return false;
        }

        tempDriver.close();
        tempRemoteVariable = nullptr;
    }
    return true;
}

bool WalkingPIDHandler::getAxisMap()
{
    m_axisMap.clear();

    int axes;
    if (!m_encodersInterface->getAxes(&axes)) {
        yError("Error while retrieving the number of axes");
    }
    for (int ax = 0; ax < axes; ++ax) {
        std::string axisName;
        yarp::os::ConstString yarpAxisName;
        if (m_axisInfo->getAxisName(ax, yarpAxisName)) {
            axisName = yarpAxisName.c_str();
            std::pair<AxisMap::iterator, bool> result = m_axisMap.insert(AxisMap::value_type(axisName, ax));
            if (!result.second) {
                yError("Error while inserting an item in the axis map");
                return false;
            }
        } else {
            yError() << "Error while retrieving the name of axis " << ax;
            return false;
        }
    }
    return true;
}

bool WalkingPIDHandler::getPID(PIDmap& output)
{
    int axes;
    output.clear();
    if (!m_encodersInterface->getAxes(&axes)) {
        yError("Error while retrieving the number of axes");
    }
    for (int ax = 0; ax < axes; ++ax) {
        std::string axisName;
        yarp::dev::Pid pid;
        yarp::os::ConstString yarpAxisName;

        if (m_axisInfo->getAxisName(ax, yarpAxisName)) {
            axisName = yarpAxisName.c_str();
            if (m_pidInterface->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION, ax, &pid)) {
                std::pair<PIDmap::iterator, bool> result = output.insert(PIDmap::value_type(axisName, pid));
                if (!result.second) {
                    yError("Error while inserting an item in the output map");
                    return false;
                }
            } else {
                yError() << "Error while retrieving the PID of " << axisName;
                return false;
            }
        } else {
            yError() << "Error while retrieving the name of axis " << ax;
            return false;
        }
    }
    return true;
}

bool WalkingPIDHandler::setPID(const PIDmap &pidMap)
{
    if (pidMap.empty()) {
        yInfo("Empty PID list");
        return true;
    }

    if (m_axisMap.empty()) {
        yError("Empty axis map. Cannot setPIDs.");
        return false;
    }

    for (PIDmap::const_iterator pid = pidMap.cbegin(); pid != pidMap.cend(); ++pid){
        AxisMap::const_iterator axis = m_axisMap.find(pid->first);

        if (axis != m_axisMap.cend()){
            if (!m_pidInterface->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, axis->second, pid->second)) {
                yError() << "Error while setting the PID on " << axis->first;
                return false;
            }
        }
    }
    return true;
}

bool WalkingPIDHandler::setPID(const PIDmap &pidMap, const AxisMap &axisMap, double smoothingTime)
{
    //For the time being we use a common smoothingTime

    if (pidMap.empty()) {
        return true;
    }

    if (axisMap.empty()) {
        yError("Empty axis map. Cannot setPIDs.");
        return false;
    }

    for (PIDmap::const_iterator pid = pidMap.cbegin(); pid != pidMap.cend(); ++pid){
        AxisMap::const_iterator axis = axisMap.find(pid->first);

        if (axis != axisMap.cend()){
            if (!m_pidInterface->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, axis->second, pid->second)) {
                yError() << "Error while setting the PID on " << axis->first;
                return false;
            }
        }
    }
    return true;
}

bool WalkingPIDHandler::setAndRestorePIDs(const PIDmap &newPIDmap, const PIDmap &oldPIDmap, const PIDmap &defaultPIDmap, const AxisMap &axisMap, double smoothingTime)
{
    //For the time being we use a common smoothingTime

    if (!setPID(newPIDmap, axisMap, smoothingTime)){
        yError() << "Failed in setting new PIDs";
        return false;
    }

    for (PIDmap::const_iterator oldPID = oldPIDmap.cbegin(); oldPID != oldPIDmap.cend(); ++oldPID){
        PIDmap::const_iterator newPID = newPIDmap.find(oldPID->first);
        if (newPID == newPIDmap.cend()) {
            PIDmap::const_iterator defaultPID = defaultPIDmap.find(oldPID->first);

            if (defaultPID != defaultPIDmap.cend()){
                AxisMap::const_iterator axis = axisMap.find(oldPID->first);

                if (axis != axisMap.cend()){
                    if (!m_pidInterface->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, axis->second, defaultPID->second)) {
                        yError() << "Error while setting the default PID on " << axis->first;
                        return false;
                    }
                }
            } else {
                yError() << "Unable to restore the default PID for joint " << oldPID->first;
                return false;
            }
        }
    }
    return true;
}

bool WalkingPIDHandler::isPIDElement(const yarp::os::Value &groupElement)
{
    bool yes = false;
    yes = groupElement.isList();
    yes = yes && groupElement.asList()->get(0).isString();
    yes = yes && (groupElement.asList()->get(0).asString() != "activationPhase");
    yes = yes && (groupElement.asList()->get(0).asString() != "activationOffset");
//    yes = yes && (groupElement.asList()->get(0).asString() != "smoothingTime"); //For the time being we use a common smoothingTime
    return yes;
}

bool WalkingPIDHandler::initialize(const yarp::os::Bottle &PIDSettings, yarp::dev::PolyDriver &robotDriver, yarp::os::Bottle& remoteControlBords)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    m_remoteControlBoards = remoteControlBords;

    m_originalPID.clear();
    m_defaultPID.clear();

    if (!PIDSettings.isNull()) {
        yInfo("Loading custom PIDs");

        if (!robotDriver.isValid()){
            yError() << "Invalid robot driver. Cannot load PIDS.";
            return false;
        }

        if (!robotDriver.view(m_pidInterface) || !m_pidInterface) {
            yError("PID I/F is not implemented");
            return false;
        }

        if (!robotDriver.view(m_axisInfo) || !m_axisInfo){
            yError("Cannot obtain IAxisInfo interface");
            return false;
        }

        if (!robotDriver.view(m_encodersInterface) || !m_encodersInterface)
        {
            yError("Cannot obtain IEncoders interface");
            return false;
        }

        if (!robotDriver.view(m_remoteVariables) || !m_remoteVariables)
        {
            yError("Cannot obtain IRemoteVariables interface");
            return false;
        }

        if (!getPID(m_originalPID)){
            yError("Error while reading the original PIDs.");
            return false;
        }

        if (!getAxisMap()){
            yError("Failed in populating the axis map.");
            return false;
        }

        if (!parsePIDConfigurationFile(PIDSettings)){
            yError() << "Failed in parsing the PID configuration file.";
            return false;
        }

        if (!(setPID(m_defaultPID))){
            yError("Error while setting the desired PIDs.");
        } else{
            yInfo("DEFAULT PID successfully loaded");
        }

        if (m_useGainScheduling) {
            /*if (!getSmoothingTimes(m_originalSmoothingTimesInMs)) { //to be restored once the gain scheduling has a proper interface to get the smoothing times
              yError() << "Error while retrieving the original smoothing times. Deactivating gain scheduling.";
              m_useGainScheduling = false;
              } else */if (!setGeneralSmoothingTime(m_smoothingTime)) {
                yError() << "Error while setting the default smoothing time. Deactivating gain scheduling.";
                m_useGainScheduling = false;
            } else {
                m_handlerThread = std::thread(&WalkingPIDHandler::setPIDThread, this);
            }
        }

    }
    return true;
}

bool WalkingPIDHandler::restorePIDs()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!m_originalPID.empty()) {
        if (!(setPID(m_originalPID))) {
            yError("Error while restoring the original PIDs.");
            return false;
        }
        m_originalPID.clear();
    }

    if (m_useGainScheduling && !(m_originalSmoothingTimesInMs.isNull())) {
        if (!setGeneralSmoothingTime(0.0)) { //use the ones from the get once the gain scheduling has a proper interface to get the smoothing times
            yError("Error while restoring the original smoothing times.");
            return false;
        }
    }

    return true;
}

bool WalkingPIDHandler::usingGainScheduling()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_useGainScheduling;
}

bool WalkingPIDHandler::updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, double time)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!guessPhases(leftIsFixed, rightIsFixed))
        return false;

    if (m_phases.front() != m_previousPhase){
        m_phaseInitTime = time;
        m_previousPhase = m_phases.front();
    }

    std::vector<size_t> desiredPIDs;
    for (size_t pid = 0; pid < m_PIDs.size(); ++pid){
        if (!m_PIDs[pid].computeInitTime(time, m_phases, m_phaseInitTime))
            return false;

        if (m_PIDs[pid].initTime() <= (time + m_firmwareDelay)){
            desiredPIDs.push_back(pid);
        }
    }

    if (desiredPIDs.size() > 1){
        std::ostringstream message;
        message << "The following PID groups would be activated at the same time: ";
        for (size_t pid = 0; pid < desiredPIDs.size()-1; ++pid)
            message << m_PIDs[desiredPIDs[pid]].name() << ", ";
        message << m_PIDs[desiredPIDs.back()].name() << ".";
        message << "Only the first will be set to the robot.";
        yWarning("%s", message.str().c_str());
    }

    if (desiredPIDs.size() > 0)
        m_desiredPIDIndex = static_cast<int>(desiredPIDs[0]);

    m_conditionVariable.notify_one();

    return true;
}

bool WalkingPIDHandler::reset()
{
    if (m_useGainScheduling && (m_desiredPIDIndex != -1)) {
        if (!(setPID(m_defaultPID))) {
            yError("Error while setting the original PIDs during reset.");
            return false;
        }
    }

    m_phaseInitTime = 0.0;
    m_previousPhase = PIDPhase::Default;
    m_currentPIDIndex = -1; //DEFAULT
    m_desiredPIDIndex = -1;

    return true;
}

PIDSchedulingObject::PIDSchedulingObject(const std::string &name, const PIDPhase &activationPhase, double activationOffset, const PIDmap &desiredPIDs)
    :m_name(name)
    ,m_desiredPIDs(desiredPIDs)
    ,m_activationPhase(activationPhase)
    ,m_activationOffset(activationOffset)
    ,m_smoothingTime(1.0)
    ,m_computedInitTime(0.0)
    ,m_dT(0.01)
{

}

bool PIDSchedulingObject::setSmoothingTime(double smoothingTime)
{
    if (smoothingTime <= 0){
        yError() << "The smoothing time is supposed to be positive.";
        return false;
    }
    m_smoothingTime = smoothingTime;
    return true;
}

bool PIDSchedulingObject::setPeriod(double dT)
{
    if (m_dT <= 0){
        yError() << "The dT is supposed to be positive";
        return false;
    }
    m_dT = dT;
    return true;
}

const PIDmap &PIDSchedulingObject::getDesiredGains()
{
    return m_desiredPIDs;
}

bool PIDSchedulingObject::computeInitTime(double time, const std::vector<PIDPhase> &phases, double currentPhaseInitTime)
{
    if (currentPhaseInitTime > time){
        yError() << "The initial time of the current phase cannot be greater than the current time.";
        return false;
    }

    if (phases.size() == 0){
        yError() << "Empty phase vector.";
        return false;
    }

    if (phases.front() == m_activationPhase){
        m_computedInitTime = currentPhaseInitTime + m_activationOffset;
        return true;
    }

    size_t k = 0;
    while ((k < phases.size()) && (phases[k] != m_activationPhase)){
        ++k;
    }
    m_computedInitTime = time + k*m_dT + m_activationOffset;
    return true;
}

double PIDSchedulingObject::initTime()
{
    return m_computedInitTime;
}

double PIDSchedulingObject::smoothingTime()
{
    return m_smoothingTime;
}

std::string PIDSchedulingObject::name()
{
    return m_name;
}
