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

#include <JoypadModule.hpp>
#include <Utils.hpp>

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
    if(!YarpHelper::getDoubleFromSearchable(rf, "deadzone", m_deadzone))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set the maximum value measured by the joypad
    if(!YarpHelper::getDoubleFromSearchable(rf, "fullscale", m_fullscale))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set scaling factors
    if(!YarpHelper::getDoubleFromSearchable(rf, "scale_x", m_scaleX))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    if(!YarpHelper::getDoubleFromSearchable(rf, "scale_y", m_scaleY))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // set the polydriver
    std::string deviceName;
    if(!YarpHelper::getStringFromSearchable(rf, "device", deviceName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }

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

    yarp::os::Property conf;
    conf.put("device", deviceName);
    conf.put("local", local);
    conf.put("remote", remote);

    // Open joypad polydriver
    if(!m_joypad.open(conf))
    {
        yError() << "[configure] Unable to open the polydriver.";
        return false;
    }

    // get the interface
    if (!m_joypad.view(m_joypadController))
    {
        yError() << "[configure] Unable to attach JoypadController interface to the "
                 << "PolyDriver object";
        return false;
    }

    std::string portName;
    if(!YarpHelper::getStringFromSearchable(rf, "JoypadOutputPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_joypadOutputPortName = "/" + name + portName;

    if(!YarpHelper::getStringFromSearchable(rf, "JoypadInputPort_name", m_joypadInputPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }

    m_rpcPort.open(m_joypadOutputPortName);
    if(!yarp::os::Network::connect(m_joypadOutputPortName, m_joypadInputPortName))
        yInfo() << "Unable to connect to port " << m_joypadOutputPortName << " to "
                << m_joypadInputPortName
                << " I'll try to connect the port in the updateModule";

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
    m_rpcPort.close();

    return true;
}

bool JoypadModule::updateModule()
{
    if(yarp::os::Network::isConnected(m_joypadOutputPortName,
                                      m_joypadInputPortName))
    {
        double x, y;
        m_joypadController->getAxis(0, x);
        m_joypadController->getAxis(1, y);

        x = -m_scaleX * deadzone(x);
        y = -m_scaleY * deadzone(y);

        std::swap(x,y);

        yarp::os::Bottle cmd, outcome;
        cmd.addString("setGoal");
        cmd.addDouble(x);
        cmd.addDouble(y);
        m_rpcPort.write(cmd, outcome);
    }
    else
    {
        // try to connect the ports
        yarp::os::Network::connect(m_joypadOutputPortName,
                                   m_joypadInputPortName);
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
