// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <WalkingControllers/JoypadModule/Module.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

#include <vector>

using namespace WalkingControllers;

bool JoypadModule::configure(yarp::os::ResourceFinder &rf)
{
    // check if the configuration file is empty
    if(rf.isNull())
    {
        yError() << "[configure] Empty configuration for the force torque sensors.";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asFloat64();

    // set the module name
    std::string name;
    if(!YarpUtilities::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    // set the deadzone interval
    if(!YarpUtilities::getNumberFromSearchable(rf, "deadzone", m_deadzone))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set the maximum value measured by the joypad
    if(!YarpUtilities::getNumberFromSearchable(rf, "fullscale", m_fullscale))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    const std::string connectPortsTag = "connectPortsSeparately";
    //connectPortsSeparately is present or set to true
    m_connectPortsSeparately = rf.check(connectPortsTag) && (rf.find(connectPortsTag).isNull() || rf.find(connectPortsTag).asBool());

    std::vector<std::pair<std::string, int&>> joypadInputs = {{"forwardAxis", m_forwardAxis},
                                                              {"rotationAxis", m_rotationAxis},
                                                              {"sideAxis", m_sideAxis},
                                                              {"prepareButton", m_prepareButton},
                                                              {"startButton", m_startButton},
                                                              {"pauseButton", m_pauseButton},
                                                              {"stopButton", m_stopButton},
                                                              {"connectGoalButton", m_connectGoalButton},
                                                              {"connectRpcButton", m_connectRpcButton},
                                                              {"disconnectButton", m_disconnectButton}};
    for (auto& input : joypadInputs)
    {
        if (!YarpUtilities::getNumberFromSearchable(rf, input.first, input.second))
        {
            yError() << "[configure] Unable to get " << input.first << " from searchable";
            return false;
        }
    }

    yarp::os::Bottle& conf = rf.findGroup("JOYPAD_DEVICE");

    if (conf.isNull())
    {
        yError() << "[configure] Unable to get the group JOYPAD_DEVICE from the configuration file.";
        return false;
    }

    // Open joypad polydriver
    if(!m_joypad.open(conf))
    {
        yError() << "[configure] Unable to open the polydriver.";
        return false;
    }

    // get the interface
    if (!m_joypad.view(m_joypadController) || !m_joypadController)
    {
        yError() << "[configure] Unable to attach JoypadController interface to the "
                 << "PolyDriver object";
        return false;
    }

    std::string portName;
    if(!YarpUtilities::getStringFromSearchable(rf, "rpcClientPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcClientPortName = "/" + name + portName;
    m_rpcClientPort.open(m_rpcClientPortName);

    if(!YarpUtilities::getStringFromSearchable(rf, "rpcServerPort_name", m_rpcServerPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);

    if(!YarpUtilities::getStringFromSearchable(rf, "robotGoalOutputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_robotGoalOutputPortName = "/" + name + portName;
    m_robotGoalPort.open(m_robotGoalOutputPortName);

    if(!YarpUtilities::getStringFromSearchable(rf, "robotGoalInputPort_name", m_robotGoalInputPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(m_robotGoalOutputPortName, m_robotGoalInputPortName);

    return true;
}

double JoypadModule::getPeriod()
{
    return m_dT;
}

bool JoypadModule::close()
{
    if (m_joypad.isValid())
    {
        m_joypadController = nullptr;
        m_joypad.close();
    }

    // close the ports
    m_rpcClientPort.close();
    m_robotGoalPort.close();

    return true;
}

bool JoypadModule::updateModule()
{
    yarp::os::Bottle cmd, outcome;
    float prepare{ 0.0 }, start{ 0.0 },
          stop{ 0.0 }, pause{ 0.0 },
          connectGoal{ 0.0 }, connectRpc{ 0.0 }, disconnect { 0.0 };

    m_joypadController->getButton(m_prepareButton, prepare);
    m_joypadController->getButton(m_startButton, start);
    m_joypadController->getButton(m_stopButton, stop);
    m_joypadController->getButton(m_pauseButton, pause);
    m_joypadController->getButton(m_connectGoalButton, connectGoal);
    m_joypadController->getButton(m_connectRpcButton, connectRpc);
    m_joypadController->getButton(m_disconnectButton, disconnect);

    // get the values from stick
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
    m_joypadController->getAxis(0, y);
    m_joypadController->getAxis(1, x);
    m_joypadController->getAxis(2, z);

    x = -deadzone(x);
    y = -deadzone(y);
    z = -deadzone(z);

    if(prepare > 0)
    {
        yInfo() << "[updateModule] Prepare robot";
        cmd.addString("prepareRobot");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(start > 0)
    {
        yInfo() << "[updateModule] Start walking";
        cmd.addString("startWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(pause > 0)
    {
        yInfo() << "[updateModule] Pause walking";
        cmd.addString("pauseWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(stop > 0)
    {
        yInfo() << "[updateModule] Stop walking";
        cmd.addString("stopWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    // connect the ports
    else if (connectGoal > 0 && connectRpc > 0)
    {
        yInfo() << "[updateModule] Connecting both ports";
        if (!yarp::os::Network::isConnected(m_rpcClientPortName, m_rpcServerPortName))
            yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);
        if (!yarp::os::Network::isConnected(m_robotGoalOutputPortName, m_robotGoalInputPortName))
            yarp::os::Network::connect(m_robotGoalOutputPortName, m_robotGoalInputPortName);
    }
    else if (m_connectPortsSeparately && connectGoal > 0)
    {
        yInfo() << "[updateModule] Connecting goal port";
        if (!yarp::os::Network::isConnected(m_robotGoalOutputPortName, m_robotGoalInputPortName))
            yarp::os::Network::connect(m_robotGoalOutputPortName, m_robotGoalInputPortName);
    }
    else if (m_connectPortsSeparately && connectRpc > 0)
    {
        yInfo() << "[updateModule] Connecting RPC port";
        if (!yarp::os::Network::isConnected(m_rpcClientPortName, m_rpcServerPortName))
            yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);
    }
    // disconnect the ports
    else if (disconnect > 0)
    {
        yInfo() << "[updateModule] Disconnecting ports";
        if (yarp::os::Network::isConnected(m_rpcClientPortName, m_rpcServerPortName))
            yarp::os::Network::disconnect(m_rpcClientPortName, m_rpcServerPortName);
        if (yarp::os::Network::isConnected(m_robotGoalOutputPortName, m_robotGoalInputPortName))
            yarp::os::Network::disconnect(m_robotGoalOutputPortName, m_robotGoalInputPortName);
    }
    else
    {
        yarp::sig::Vector& goal= m_robotGoalPort.prepare();
        goal.clear();
        goal.push_back(x);
        goal.push_back(y);
        goal.push_back(z);
        m_robotGoalPort.write();
    }
    return true;
}

double JoypadModule::deadzone(const double &input)
{
    if (input >= 0)
    {
        if (input > m_deadzone)
            return (input - m_deadzone)/(m_fullscale - m_deadzone);
        else return 0.0;
    }
    else
    {
        if (input < -m_deadzone)
            return (input + m_deadzone)/(m_fullscale - m_deadzone);
        else return 0.0;
    }
}
