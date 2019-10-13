/**
 * @file LoggerClient.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// YARP
#include <yarp/os/LogStream.h>

#include <WalkingControllers/YarpHelper/Helper.h>
#include <WalkingControllers/LoggerClient/LoggerClient.h>

using namespace WalkingControllers;

bool LoggerClient::configure(const yarp::os::Searchable& config, const std::string& name)
{
    std::string portInput, portOutput;

    // check if the config file is empty
    if(config.isNull())
    {
        yError() << "[configureLogger] Empty configuration for the force torque sensors.";
        return false;
    }

    // open the connect the data logger port
    if(!YarpHelper::getStringFromSearchable(config, "dataLoggerOutputPort_name", portOutput))
    {
        yError() << "[configureLogger] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "dataLoggerInputPort_name", portInput))
    {
        yError() << "[configureLogger] Unable to get the string from searchable.";
        return false;
    }
    m_dataPort.open("/" + name + portOutput);
    if(!yarp::os::Network::connect("/" + name + portOutput,  portInput))
    {
        yError() << "Unable to connect to port " << "/" + name + portOutput;
        return false;
    }

    // open the connect the rpc logger port
    if(!YarpHelper::getStringFromSearchable(config, "dataLoggerRpcOutputPort_name", portOutput))
    {
        yError() << "[configureLogger] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "dataLoggerRpcInputPort_name", portInput))
    {
        yError() << "[configureLogger] Unable to get the string from searchable.";
        return false;
    }
    m_rpcPort.open("/" + name + portOutput);
    if(!yarp::os::Network::connect("/" + name + portOutput,  portInput))
    {
        yError() << "Unable to connect to port " << "/" + name + portOutput;
        return false;
    }
    return true;
}

bool LoggerClient::startRecord(const std::initializer_list<std::string>& strings)
{
    yarp::os::Bottle cmd, outcome;

    YarpHelper::populateBottleWithStrings(cmd, strings);

    m_rpcPort.write(cmd, outcome);
    if(outcome.get(0).asInt() != 1)
    {
        yError() << "[startWalking] Unable to store data";
        return false;
    }
    return true;
}

void LoggerClient::quit()
{
    // stop recording
    yarp::os::Bottle cmd, outcome;
    cmd.addString("quit");
    m_rpcPort.write(cmd, outcome);
    if(outcome.get(0).asInt() != 1)
        yInfo() << "[close] Unable to close the stream.";

    // close ports
    m_dataPort.close();
    m_rpcPort.close();
}
