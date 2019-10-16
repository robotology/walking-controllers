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
#include <WalkingControllers/YarpHelper/Helper.h>

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
    m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if(!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    // set the deadzone interval
    if(!YarpHelper::getNumberFromSearchable(rf, "deadzone", m_deadzone))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set the maximum value measured by the joypad
    if(!YarpHelper::getNumberFromSearchable(rf, "fullscale", m_fullscale))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set scaling factors
    if(!YarpHelper::getNumberFromSearchable(rf, "scale_x", m_scaleX))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    if(!YarpHelper::getNumberFromSearchable(rf, "scale_y", m_scaleY))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set the polydriver
    yarp::os::Property conf;
    std::string deviceName;
    if(!YarpHelper::getStringFromSearchable(rf, "device", deviceName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }

    conf.put("device", deviceName);

    if(deviceName == "JoypadControlClient")
    {
        std::string local;
        if(!YarpHelper::getStringFromSearchable(rf, "local", local))
        {
            yError() << "[configure] Unable to get a string from searchable";
            return false;
        }

        std::string remote;
        if(!YarpHelper::getStringFromSearchable(rf, "remote", remote))
        {
            yError() << "[configure] Unable to get a string from searchable";
            return false;
        }

        conf.put("local", local);
        conf.put("remote", remote);
    }
    else
    {
        int sticks;
        if(!YarpHelper::getNumberFromSearchable(rf, "sticks", sticks))
        {
            yError() << "[configure] Unable to get a number from searchable";
            return false;
        }

        conf.put("sticks", sticks);
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
    if(!YarpHelper::getStringFromSearchable(rf, "rpcClientPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcClientPortName = "/" + name + portName;
    m_rpcClientPort.open(m_rpcClientPortName);

    if(!YarpHelper::getStringFromSearchable(rf, "rpcServerPort_name", m_rpcServerPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    yarp::os::Network::connect(m_rpcClientPortName, m_rpcServerPortName);

    if(!YarpHelper::getStringFromSearchable(rf, "robotGoalOutputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_robotGoalOutputPortName = "/" + name + portName;
    m_robotGoalPort.open(m_robotGoalOutputPortName);

    if(!YarpHelper::getStringFromSearchable(rf, "robotGoalInputPort_name", m_robotGoalInputPortName))
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
    float aButton, bButton, xButton, yButton, l1Button, r1Button;

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
    double x, y;
    m_joypadController->getAxis(0, x);
    m_joypadController->getAxis(1, y);

    x = -m_scaleX * deadzone(x);
    y = -m_scaleY * deadzone(y);

    std::swap(x,y);

    if(aButton > 0)
    {
        cmd.addString("prepareRobot");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(bButton > 0)
    {
        cmd.addString("startWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(yButton > 0)
    {
        cmd.addString("pauseWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    else if(xButton > 0)
    {
        cmd.addString("stopWalking");
        m_rpcClientPort.write(cmd, outcome);
    }
    // connect the ports
    else if(l1Button > 0 && r1Button > 0)
    {
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
