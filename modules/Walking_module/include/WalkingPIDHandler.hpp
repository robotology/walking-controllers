/******************************************************************************
 *                                                                            *
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file WalkingPIDHandler.h
 * @authors: Stefano Dafarra  <stefano.dafarra@iit.it>
 */


#ifndef ICUB_WALKINGPIDHANDLER_H
#define ICUB_WALKINGPIDHANDLER_H

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <yarp/dev/ControlBoardPid.h>
#include <yarp/os/Bottle.h>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace yarp{
    namespace os{
        class Searchable;
        class Value;
    }

    namespace dev {
        class PolyDriver;
        class IAxisInfo;
        class IPidControl;
        class IEncodersTimed;
        class IRemoteVariables;
    }
}

typedef std::map<std::string, int> AxisMap;

typedef std::map<std::string, yarp::dev::Pid> PIDmap;

enum class PIDPhase {
    Default,
        SwingLeft,
        SwingRight,
        Switch
        };

class PIDSchedulingObject {
    std::string m_name;
    PIDmap m_desiredPIDs;
    PIDPhase m_activationPhase;
    double m_activationOffset;
    double m_smoothingTime;
    double m_computedInitTime;
    double m_dT;

public:
    PIDSchedulingObject(const std::string &name, const PIDPhase &activationPhase, double activationOffset, const PIDmap &desiredPIDs);

    bool setSmoothingTime(double smoothingTime);

    bool setPeriod(double dT);

    const PIDmap& getDesiredGains();

    bool computeInitTime(double time, const std::vector<PIDPhase> &phases, double currentPhaseInitTime);

    double initTime();

    double smoothingTime();

    std::string name();

};

class WalkingPIDHandler {

    bool m_useGainScheduling;
    AxisMap m_axisMap;
    PIDmap m_originalPID;
    PIDmap m_defaultPID;
    yarp::dev::IPidControl *m_pidInterface;
    yarp::dev::IAxisInfo *m_axisInfo;
    yarp::dev::IEncodersTimed *m_encodersInterface;
    yarp::dev::IRemoteVariables *m_remoteVariables;
    std::vector<PIDSchedulingObject> m_PIDs;
    std::vector<PIDPhase> m_phases;
    yarp::os::Bottle m_originalSmoothingTimesInMs;
    double m_phaseInitTime;
    PIDPhase m_previousPhase;
    int m_currentPIDIndex;
    int m_desiredPIDIndex;
    double m_firmwareDelay, m_smoothingTime, m_maximumContactDelay;
    yarp::os::Bottle m_remoteControlBoards; //to be removed when the gain scheduling has a proper interface to set the smoothing times.
    std::deque<bool> m_leftIsFixedModified, m_rightIsFixedModified;
    double m_dT;

    std::mutex m_mutex;
    std::condition_variable m_conditionVariable;
    std::thread m_handlerThread;

    bool getAxisMap();

    bool getPID(PIDmap& output);

    bool setPID(const PIDmap& pidMap);

    bool setPID(const PIDmap &pidMap, const AxisMap &axisMap, double smoothingTime);

    bool setAndRestorePIDs(const PIDmap &newPIDmap, const PIDmap &oldPIDmap, const PIDmap &defaultPIDmap, const AxisMap &axisMap, double smoothingTime);

    bool isPIDElement(const yarp::os::Value &groupElement);

    bool parsePIDGroup(const yarp::os::Bottle* group, PIDmap& pidMap);

    bool parseDefaultPIDGroup(const yarp::os::Bottle* defaultGroup);

    bool parsePIDElement(const yarp::os::Value &groupElement, std::string &jointName, yarp::dev::Pid &pid);

    bool parsePIDConfigurationFile(const yarp::os::Bottle& PIDSettings);

    bool fromStringToPIDPhase(const std::string &input, PIDPhase &output);

    bool guessPhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed);

    void setPIDThread();

    //bool getSmoothingTimes(yarp::os::Bottle &defaultSmoothingTime); //to be restored when the gain scheduling has a proper interface to set the smoothing times.

    //bool setSmoothingTimes(yarp::os::Bottle &desiredSmoothingTime); //to be restored when the gain scheduling has a proper interface to set the smoothing times.

    bool setGeneralSmoothingTime(double smoothingTime);

    bool modifyFixedLists(std::deque<bool> &leftIsFixed, std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed);

public:
    WalkingPIDHandler();

    ~WalkingPIDHandler();

    bool initialize(const yarp::os::Bottle& PIDSettings, yarp::dev::PolyDriver& robotDriver, yarp::os::Bottle& remoteControlBords, double dT);

    bool setMaximumContactDelay(double maxContactDelay);

    bool restorePIDs();

    bool usingGainScheduling();

    bool updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, double time);

    bool updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed, double time);

    bool reset();

};

#endif // ICUB_WALKINGPIDHANDLER_H
