/**
 * @file JoypadModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include <WalkingControllers/JoypadModule/Module.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

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
    float aButton{ 0.0 }, bButton{ 0.0 },
          xButton{ 0.0 }, yButton{ 0.0 },
          l1Button{ 0.0 }, r1Button{ 0.0 };

    // prepare robot (A button)
    m_joypadController->getButton(0, aButton);

    // start walking (B button)
    m_joypadController->getButton(1, bButton);

    // stop walking (X button)
    m_joypadController->getButton(3, xButton);

    // pause walking (Y button)
    m_joypadController->getButton(4, yButton);

    // reset connection (L1 + R1)
    m_joypadController->getButton(6, l1Button);
    m_joypadController->getButton(7, r1Button);

    // get the values from stick
    double x{ 0.0 }, y{ 0.0 }, z{ 0.0 };
    m_joypadController->getAxis(0, y);
    m_joypadController->getAxis(1, x);
    m_joypadController->getAxis(2, z);

    x = -deadzone(x);
    y = -deadzone(y);
    z = -deadzone(z);

    if(aButton > 0)
    {
        yInfo() << "[updateModule] Prepare robot";
        cmd.addString("prepareRobot");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(bButton > 0)
    {
        yInfo() << "[updateModule] Start walking";
        cmd.addString("startWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(yButton > 0)
    {
        yInfo() << "[updateModule] Pause walking";
        cmd.addString("pauseWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(xButton > 0)
    {
        yInfo() << "[updateModule] Stop walking";
        cmd.addString("stopWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    // connect the ports
    else if(l1Button > 0 && r1Button > 0)
    {
        yInfo() << "[updateModule] Connecting";
        if(!yarp::os::Network::isConnected(m_rpcClientPortName, m_rpcServerPortName))
            yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);
        if(!yarp::os::Network::isConnected(m_robotGoalOutputPortName, m_robotGoalInputPortName))
            yarp::os::Network::connect(m_robotGoalOutputPortName, m_robotGoalInputPortName);
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
