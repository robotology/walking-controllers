// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <yarp/os/LogStream.h>

#include <WalkingControllers/YarpUtilities/TransformHelper.h>

void WalkingControllers::YarpUtilities::TransformHelper::convertTransform(const iDynTree::Transform& transform, yarp::sig::Matrix& buffer)
{
    buffer.resize(4, 4);
    buffer.zero();
    iDynTree::Matrix4x4 homTrans = transform.asHomogeneousTransform();

    // Here we explot the fact that both YARP and iDynTree store
    // Matrices in row major form.
    memcpy(buffer.data(), homTrans.data(), 4 * 4 * sizeof(double));
}

WalkingControllers::YarpUtilities::TransformHelper::~TransformHelper()
{
    close();
}

bool WalkingControllers::YarpUtilities::TransformHelper::configure(const yarp::os::Searchable& config)
{
    //From the GENERAL options
    double period = config.check("sampling_time", yarp::os::Value(0.01)).asFloat64();
    std::string prefix = "/" + config.check("name", yarp::os::Value("walking-coordinator")).asString();


    m_rootFrame = config.check("rootFrame", yarp::os::Value("world")).asString();
    m_baseFrameName = config.check("baseFrame", yarp::os::Value("root_link")).asString();
    m_joystickFrameName = config.check("joystickFrame", yarp::os::Value("joystick")).asString();

    if (config.check("runServer", yarp::os::Value(false)).asBool())
    {
        yarp::os::Bottle& conf = config.findGroup("SERVER");

        if (conf.isNull())
        {
            yError() << "[TransformHelper::initialize] Unable to get the group SERVER from the configuration file.";
            return false;
        }
        if (!m_serverDriver.open(conf))
        {
            yError() << "[TransformHelper::initialize] Unable to open the server device with the following options:" << conf.toString();
            return false;
        }
    }

    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", config.check("device", yarp::os::Value("frameTransformClient")).asString());
    tfClientCfg.put("filexml_option", config.check("filexml_option", yarp::os::Value("ftc_yarp_only.xml")).asString());
    tfClientCfg.put("ft_client_prefix", prefix + "/transforms");
    if (config.check("tfRemote"))
    {
        tfClientCfg.put("ft_server_prefix", config.find("ft_server_prefix").asString());
    }

    tfClientCfg.put("period", period);
    tfClientCfg.put("local_rpc", prefix + "/transforms/client_rpc");

    if (!m_clientDriver.open(tfClientCfg))
    {
        yError() << "[TransformHelper::initialize] Unable to open transform client with the following options:" << tfClientCfg.toString();
        return false;
    }

    if (!m_clientDriver.view(m_tfPublisher) || m_tfPublisher == nullptr)
    {
        yError() << "[TransformHelper::initialize] Unable to view IFrameTransform interface.";
        return false;
    }

    m_buffer.resize(4, 4);
    m_buffer.zero();
    //TODO: configuration file
    //TODO: add possibility to stream additional frames

    return true;
}

bool WalkingControllers::YarpUtilities::TransformHelper::setBaseTransform(const iDynTree::Transform& transform)
{
    convertTransform(transform, m_buffer);
    return m_tfPublisher->setTransform(m_baseFrameName, m_rootFrame, m_buffer);
}



bool WalkingControllers::YarpUtilities::TransformHelper::setJoystickTransform(const iDynTree::Transform& transform)
{
    convertTransform(transform, m_buffer);
    return m_tfPublisher->setTransform(m_joystickFrameName, m_rootFrame, m_buffer);
}

void WalkingControllers::YarpUtilities::TransformHelper::close()
{
    m_clientDriver.close();
    m_tfPublisher = nullptr;
    m_serverDriver.close();
}
