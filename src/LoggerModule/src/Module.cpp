/**
 * @file LoggerModule.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#include <iomanip>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>

#include <WalkingControllers/LoggerModule/Module.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

double WalkingLoggerModule::getPeriod()
{
    return m_dT;
}

bool WalkingLoggerModule::close()
{
    // close the stream (if it is open)
    if(m_stream.is_open())
        m_stream.close();

    // close the ports
    m_dataPort.close();
    m_rpcPort.close();

    return true;
}

bool WalkingLoggerModule::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString() == "quit")
    {
        if(!m_stream.is_open())
        {
            yError() << "[RPC Server] The stream is not open.";
            reply.addInt32(0);
            return true;
        }
        m_stream.close();
        reply.addInt32(1);

        yInfo() << "[RPC Server] The stream is closed.";
        return true;
    }
    else if (command.get(0).asString() == "record")
    {
        if(m_stream.is_open())
        {
            yError() << "[RPC Server] The stream is already open.";
            reply.addInt32(0);
            return false;
        }

        m_numberOfValues = command.size() - 1;

        std::string head{"time "};
        for(int i = 0; i < m_numberOfValues; i++)
            head += command.get(i + 1).asString() + " ";

        yInfo() << "[RPC Server] The following data will be stored: "
                << head;

        // get the current time
        m_time0 = yarp::os::Time::now();

        // set the file name
        std::time_t t = std::time(nullptr);
        std::tm tm = *std::localtime(&t);

        std::stringstream fileName;
        fileName << "Dataset_" << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S")
                 << ".txt";

        m_stream.open(fileName.str().c_str());

        // write the head of the table
        m_stream << head << std::endl;

        reply.addInt32(1);
        return true;
    }
    else
    {
        yError() << "[RPC Server] Unknown command.";
        reply.addInt32(0);
        return false;
    }
    return true;
}

bool WalkingLoggerModule::configure(yarp::os::ResourceFinder &rf)
{
    // check if the configuration file is empty
    if(rf.isNull())
    {
        yError() << "[configure] Empty configuration for the force torque sensors.";
        return false;
    }

    std::string name;
    if(!YarpUtilities::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    setName(name.c_str());

    // set data port name
    std::string dataPortName;
    if(!YarpUtilities::getStringFromSearchable(rf, "data_port_name", dataPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_dataPort.open("/" + getName() + dataPortName);

    // set rpc port name
    std::string rpcPortName;
    if(!YarpUtilities::getStringFromSearchable(rf, "rpc_port_name", rpcPortName))
    {
        yError() << "[configure] Unable to get a string from searchable";
        return false;
    }
    m_rpcPort.open("/" + getName() + rpcPortName);
    attach(m_rpcPort);

    // set the RFModule period
    m_dT = rf.check("sampling_time", yarp::os::Value(0.005)).asFloat64();

    return true;
}

bool WalkingLoggerModule::updateModule()
{
    yarp::sig::Vector *data = NULL;

    // try to read data from port
    data = m_dataPort.read(false);

    if (data != NULL)
    {
        if(!m_stream.is_open())
        {
            yError() << "[updateModule] No stream is open. I cannot store your data.";
            return false;
        }

        if(data->size() != m_numberOfValues)
        {
            yError() << "[updateModule] The size of the vector is not the one expected. Expected: "
                     << m_numberOfValues << " received: " << data->size();
            return false;
        }

        // write into the file
        double time = yarp::os::Time::now() - m_time0;
        m_stream << time << " ";
        for(int i = 0; i < m_numberOfValues; i++)
            m_stream << (*data)[i] << " ";

        m_stream << std::endl;
    }
    return true;
}
