// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <yarp/os/LogStream.h>

#include <WalkingControllers/YarpUtilities/TransformServerHelper.h>

void WalkingControllers::YarpUtilities::TransformServerHelper::convertTransform(const iDynTree::Transform& transform, yarp::sig::Matrix& buffer)
{
    buffer.resize(4, 4);
    buffer.zero();
    iDynTree::Matrix4x4 homTrans = transform.asHomogeneousTransform();

    // Here we explot the fact that both YARP and iDynTree store
    // Matrices in row major form.
    memcpy(buffer.data(), homTrans.data(), 4 * 4 * sizeof(double));
}

WalkingControllers::YarpUtilities::TransformServerHelper::~TransformServerHelper()
{
    close();
}

bool WalkingControllers::YarpUtilities::TransformServerHelper::configure(const yarp::os::Searchable& config)
{
    //From the GENERAL options
    double period = config.check("sampling_time", yarp::os::Value(0.01)).asFloat64();
    std::string prefix = "/" + config.check("name", yarp::os::Value("walking-coordinator")).asString();


    m_rootFrame = config.check("tf_root_frame", yarp::os::Value("world")).asString();
    m_baseFrameName = config.check("tf_base_frame", yarp::os::Value("root_link")).asString();
    m_joystickFrameName = config.check("tf_joystick_frame", yarp::os::Value("joystick")).asString();

    yarp::os::Property tfClientCfg;
    tfClientCfg.put("device", config.check("tfDevice", yarp::os::Value("frameTransformClient")).asString());
    tfClientCfg.put("filexml_option", config.check("tfFile", yarp::os::Value("ftc_yarp_only.xml")).asString());
    tfClientCfg.put("ft_client_prefix", config.check("tfLocal", yarp::os::Value(prefix + "/tf")).asString());
    if (config.check("tfRemote"))
    {
        tfClientCfg.put("ft_server_prefix", config.find("tfRemote").asString());
    }

    tfClientCfg.put("period", period);
    tfClientCfg.put("local_rpc", prefix + "/tf/local_rpc");

    if (!m_driver.open(tfClientCfg))
    {
        yError() << "[TransformServerHelper::initialize] Unable to open polydriver with the following options:" << tfClientCfg.toString();
        return false;
    }

    if (!m_driver.view(m_tfPublisher) || m_tfPublisher == nullptr)
    {
        yError() << "[TransformServerHelper::initialize] Unable to view IFrameTransform interface.";
        return false;
    }

    m_buffer.resize(4, 4);
    m_buffer.zero();
    //TODO: configuration file
    //TODO: add possibility to stream additional frames

    return true;
}

bool WalkingControllers::YarpUtilities::TransformServerHelper::setBaseTransform(const iDynTree::Transform& transform)
{
    convertTransform(transform, m_buffer);
    return m_tfPublisher->setTransform(m_baseFrameName, m_rootFrame, m_buffer);
}



bool WalkingControllers::YarpUtilities::TransformServerHelper::setJoystickTransform(const iDynTree::Transform& transform)
{
    convertTransform(transform, m_buffer);
    return m_tfPublisher->setTransform(m_joystickFrameName, m_rootFrame, m_buffer);
}

void WalkingControllers::YarpUtilities::TransformServerHelper::close()
{
    m_driver.close();
    m_tfPublisher = nullptr;
}
