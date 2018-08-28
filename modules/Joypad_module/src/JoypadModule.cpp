/**
 * @file JoypadModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include "yarp/os/LogStream.h"
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include "JoypadModule.hpp"

bool JoypadModule::configure(yarp::os::ResourceFinder &rf)
{
    yarp::os::Value* value;

    // check if the configuration file is empty
    if(rf.isNull())
    {
        yError() << "[configure] Empty configuration for the force torque sensors.";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();

    // set the module name
    if(!rf.check("name", value))
    {
        yError() << "[configure] Unable to find the module name.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] The module name is not a string.";
        return false;
    }
    std::string name = value->asString();
    setName(name.c_str());

    // set the deadzone interval
    if(!rf.check("deadzone", value))
    {
        yError() << "[configure] Unable to find the deadzone.";
        return false;
    }
    if(!value->isDouble())
    {
        yError() << "[configure] The deadzone is not a double.";
        return false;
    }
    m_deadzone = value->asDouble();

    // set the maximum value measured by the joypad
    if(!rf.check("fullscale", value))
    {
        yError() << "[configure] Unable to find the fullscale.";
        return false;
    }
    if(!value->isDouble())
    {
        yError() << "[configure] fullscale is not a double.";
        return false;
    }
    m_fullscale = value->asDouble();

    // set the maximum value measured by the joypad
    if(!rf.check("scale_x", value))
    {
        yError() << "[configure] Unable to find the fullscale.";
        return false;
    }
    if(!value->isDouble())
    {
        yError() << "[configure] fullscale is not a double.";
        return false;
    }
    m_scaleX = value->asDouble();

    // set the maximum value measured by the joypad
    if(!rf.check("scale_y", value))
    {
        yError() << "[configure] Unable to find the fullscale.";
        return false;
    }
    if(!value->isDouble())
    {
        yError() << "[configure] fullscale is not a double.";
        return false;
    }
    m_scaleY = value->asDouble();

    // set the polydriver
    if(!rf.check("device", value))
    {
        yError() << "[configure] Unable to find the device name.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] The device name is not a string.";
        return false;
    }
    std::string deviceName = value->asString();

    if(!rf.check("local", value))
    {
        yError() << "[configure] Unable to find local.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] The local is not a string.";
        return false;
    }
    std::string local = value->asString();

    if(!rf.check("remote", value))
    {
        yError() << "[configure] Unable to find remote.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] The remote is not a string.";
        return false;
    }
    std::string remote = value->asString();

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

    // open the communication with the WalkingModule
    if(!rf.check("JoypadOutputPort_name", value))
    {
        yError() << "[configure] Unable to find JoypadOutputPort_name.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] JoypadOutputPort_name is not a string.";
        return false;
    }
    m_joypadOutputPortName = "/" + name + value->asString();

    if(!rf.check("JoypadInputPort_name", value))
    {
        yError() << "[configure] Unable to find JoypadInputPort_name.";
        return false;
    }
    if(!value->isString())
    {
        yError() << "[configure] JoypadInputPort_name is not a string.";
        return false;
    }
    m_joypadInputPortName = value->asString();

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
