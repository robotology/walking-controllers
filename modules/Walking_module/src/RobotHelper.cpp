#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <RobotHelper.hpp>
#include <Utils.hpp>
#include <iDynTree/Model/Model.h>

bool RobotHelper::getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                std::pair<std::string, double>& worstError)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotHelper::getWorstError] The encoder I/F is not ready";
        return false;
    }

    // clear the std::pair
    worstError.first = "";
    worstError.second = 0.0;
    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        if(absoluteJointErrorRad > worstError.second)
        {
            worstError.first = m_axesList[i];
            worstError.second = absoluteJointErrorRad;
        }
    }
    return true;
}

bool RobotHelper::getFirstPelvisIMUData(const iDynTree::Model modelLoader,const iDynTree::Rotation baseToWorldRotation,const int maxAttempts){
    int attempt=0;
    do {
        if (m_usePelvisIMU) {
            yarp::sig::Vector *pelvisIMU = NULL;
            pelvisIMU=m_pelvisIMUPort.read(false);

            if (pelvisIMU!=NULL) {
                yInfo()<<"i'm there";
                //yInfo()<<(*pelvisIMU)(0)<<iDynTree::rad2deg((*pelvisIMU)(1))<<(*pelvisIMU)(2);
                auto root_imu_idx = modelLoader.getFrameIndex("root_link_imu_frame");
                auto base_R_imu = modelLoader.getFrameTransform(root_imu_idx).getRotation();

                yInfo()<<"root_R_imu"<<base_R_imu.asRPY().toString();
                // iDynTree::Rotation InitialIMUOrientation;
                iDynTree::Rotation imuOrientationtoIMUWorld;
                imuOrientationtoIMUWorld=imuOrientationtoIMUWorld.RPY(iDynTree::deg2rad((*pelvisIMU)(0)),iDynTree::deg2rad((*pelvisIMU)(1)),iDynTree::deg2rad((*pelvisIMU)(2) ));
                iDynTree::toEigen(m_initialPelvisIMUOrientation)=iDynTree::toEigen(baseToWorldRotation)*iDynTree::toEigen(base_R_imu)*iDynTree::toEigen(imuOrientationtoIMUWorld.inverse());
                //  *imuOrientation.RPY(iDynTree::deg2rad((*pelvisIMU)(0)),iDynTree::deg2rad((*pelvisIMU)(1)),iDynTree::deg2rad((*pelvisIMU)(2) ));
                // yInfo()<<"imubefore"<<m_imuOrientation.asRPY().toString();
                //m_imuOrientation=root_R_imu.inverse()*m_imuOrientation;
                yInfo()<<"imu init orientation"<<m_initialPelvisIMUOrientation.asRPY().toString();


                //yInfo()<<"imu"<<root_R_imu.asRPY().toString();
                //                            yInfo()<<"imuafter"<<m_imuOrientation.asRPY().toString();
                //m_imuOrientation
                m_pelvisimuAcceleration(0)=(*pelvisIMU)(3) ;
                m_pelvisimuAcceleration(1)=(*pelvisIMU)(4) ;
                m_pelvisimuAcceleration(2)=(*pelvisIMU)(5) ;

                m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(6) ;
                m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(7) ;
                m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(8) ;
            }
        }
        yarp::os::Time::delay(0.001);
        attempt++;
    } while (attempt < maxAttempts);
    return true;
}

bool RobotHelper::getFirstHeadIMUData(const iDynTree::Model modelLoader,const iDynTree::Rotation baseToWorldRotation,const iDynTree::Rotation headToBaseRotation,const int maxAttempts){
    int attempt=0;
    do {
        if (m_useHeadIMU) {
            yarp::sig::Vector *headIMU = NULL;
            headIMU=m_headIMUPort.read(false);

            if (headIMU!=NULL) {
                yInfo()<<"i'm there";
                //yInfo()<<(*pelvisIMU)(0)<<iDynTree::rad2deg((*pelvisIMU)(1))<<(*pelvisIMU)(2);
                auto head_imu_idx = modelLoader.getFrameIndex("imu_frame");
                auto head_R_imu = modelLoader.getFrameTransform(head_imu_idx).getRotation();
                auto root_R_imu=headToBaseRotation*head_R_imu;

                yInfo()<<"head root_R_imu"<<root_R_imu.asRPY().toString();
                // iDynTree::Rotation InitialIMUOrientation;
                iDynTree::Rotation imuOrientationtoIMUWorld;
                imuOrientationtoIMUWorld=imuOrientationtoIMUWorld.RPY(iDynTree::deg2rad((*headIMU)(0)),iDynTree::deg2rad((*headIMU)(1)),iDynTree::deg2rad((*headIMU)(2) ));
                iDynTree::toEigen(m_initialHeadIMUOrientation)=iDynTree::toEigen(baseToWorldRotation)*iDynTree::toEigen(root_R_imu)*iDynTree::toEigen(imuOrientationtoIMUWorld.inverse());
                //  *imuOrientation.RPY(iDynTree::deg2rad((*pelvisIMU)(0)),iDynTree::deg2rad((*pelvisIMU)(1)),iDynTree::deg2rad((*pelvisIMU)(2) ));
                // yInfo()<<"imubefore"<<m_imuOrientation.asRPY().toString();
                //m_imuOrientation=root_R_imu.inverse()*m_imuOrientation;
                yInfo()<<"head imu init orientation"<<m_initialHeadIMUOrientation.asRPY().toString();


                //yInfo()<<"imu"<<root_R_imu.asRPY().toString();
                //                            yInfo()<<"imuafter"<<m_imuOrientation.asRPY().toString();
                //m_imuOrientation
                m_headimuAcceleration(0)=(*headIMU)(3) ;
                m_headimuAcceleration(1)=(*headIMU)(4) ;
                m_headimuAcceleration(2)=(*headIMU)(5) ;

                m_headimuAngularVelocity(0)=(*headIMU)(6) ;
                m_headimuAngularVelocity(0)=(*headIMU)(7) ;
                m_headimuAngularVelocity(0)=(*headIMU)(8) ;
            }
        }
        yarp::os::Time::delay(0.001);
        attempt++;
    } while (attempt < maxAttempts);
        return true;
}


bool RobotHelper::getFeedbacksRaw(const iDynTree::Model modelLoader,const iDynTree::Rotation intialPelvisIMUOrientation,const iDynTree::Rotation intialHeadIMUOrientation,const iDynTree::Rotation headToBaseRotation,unsigned int maxAttempts, bool getBaseEst)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotHelper::getFeedbacksRaw] Encoders I/F is not ready";
        return false;
    }

    bool okPosition = false;
    bool okVelocity = false;
    bool okTorque = false;
    bool okLeftWrench = false;
    bool okRightWrench = false;

    bool okBaseEstimation = true;
    bool okPelvisIMU = true;
    bool okHeadIMU = true;
    bool okFloatingBaseEstimation = true;
    if(getBaseEst){

        if(m_useExternalRobotBase)
            okBaseEstimation = !m_useExternalRobotBase;

        if(m_useFloatingBaseEstimator)
            okFloatingBaseEstimation = !m_useFloatingBaseEstimator;

    }

    if(m_usePelvisIMU)
        okPelvisIMU = !m_usePelvisIMU;

    if(m_useHeadIMU)
        okHeadIMU = !m_useHeadIMU;

    unsigned int attempt = 0;
    do
    {
        if(!okPosition)
            okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());

        if(!okVelocity)
            okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okLeftWrench)
        {
            yarp::sig::Vector *leftWrenchRaw = NULL;
            leftWrenchRaw = m_leftWrenchPort.read(false);
            if(leftWrenchRaw != NULL)
            {
                m_leftWrenchInput = *leftWrenchRaw;
                okLeftWrench = true;
            }
        }

        if(!okRightWrench)
        {
            yarp::sig::Vector *rightWrenchRaw = NULL;
            rightWrenchRaw = m_rightWrenchPort.read(false);
            if(rightWrenchRaw != NULL)
            {
                m_rightWrenchInput = *rightWrenchRaw;
                okRightWrench = true;
            }
        }


        if(m_useFloatingBaseEstimator)
        {
            if(!okFloatingBaseEstimation)
            {
                yarp::sig::Vector *baseEstimated = NULL;
                baseEstimated = m_robotBaseEstimatorPort.read(false);
                if(baseEstimated != NULL)
                {
                    m_robotEstimatedBaseTransform.setPosition(iDynTree::Position((*baseEstimated)(0),
                                                                                 (*baseEstimated)(1),
                                                                                 (*baseEstimated)(2) - m_heightOffset));

                    m_robotEstimatedBaseTransform.setRotation(iDynTree::Rotation::RPY((*baseEstimated)(3),
                                                                                      (*baseEstimated)(4),
                                                                                      (*baseEstimated)(5)));

                    m_robotEstimatedBaseTwist.setLinearVec3(iDynTree::Vector3(baseEstimated->data() + 6, 3));
                    m_robotEstimatedBaseTwist.setAngularVec3(iDynTree::Vector3(baseEstimated->data() + 6 + 3, 3));
                    okFloatingBaseEstimation = true;
                }
            }
        }

        // if(m_use)


        if(m_useExternalRobotBase)
        {
            if(!okBaseEstimation)
            {
                yarp::sig::Vector *base = NULL;
                base = m_robotBasePort.read(false);
                if(base != NULL)
                {
                    m_robotBaseTransform.setPosition(iDynTree::Position((*base)(0),
                                                                        (*base)(1),
                                                                        (*base)(2) - m_heightOffset));

                    m_robotBaseTransform.setRotation(iDynTree::Rotation::RPY((*base)(3),
                                                                             (*base)(4),
                                                                             (*base)(5)));

                    m_robotBaseTwist.setLinearVec3(iDynTree::Vector3(base->data() + 6, 3));
                    m_robotBaseTwist.setAngularVec3(iDynTree::Vector3(base->data() + 6 + 3, 3));
                    okBaseEstimation = true;
                }
            }
        }

        if (m_usePelvisIMU) {
            if(!okPelvisIMU){
                yarp::sig::Vector *pelvisIMU = NULL;
                pelvisIMU=m_pelvisIMUPort.read(false);
                if (pelvisIMU!=NULL) {
                    //yInfo()<<(*pelvisIMU)(0)<<iDynTree::rad2deg((*pelvisIMU)(1))<<(*pelvisIMU)(2);
                    auto root_imu_idx = modelLoader.getFrameIndex("root_link_imu_frame");
                    auto root_R_imu = modelLoader.getFrameTransform(root_imu_idx).getRotation();

                    // yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();
                    //yInfo()<<"m_imuOrientation"<<m_imuOrientation.asRPY().toString();
                    // yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();

                    m_pelvisimuOrientation=m_pelvisimuOrientation.RPY(iDynTree::deg2rad((*pelvisIMU)(0)),iDynTree::deg2rad((*pelvisIMU)(1)),iDynTree::deg2rad((*pelvisIMU)(2) ));
                    yInfo()<<"imubefore"<<m_pelvisimuOrientation.asRPY().toString();
                    //                m_imuOrientation=root_R_imu.inverse()*m_imuOrientation;

                    iDynTree::toEigen(m_pelvisimuOrientation)=iDynTree::toEigen(intialPelvisIMUOrientation)*iDynTree::toEigen(m_pelvisimuOrientation)*iDynTree::toEigen(root_R_imu.inverse());
                    yInfo()<<"inital"<<intialPelvisIMUOrientation.asRPY().toString();
                    //                                yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();
                    yInfo()<<"imuafter"<<m_pelvisimuOrientation.asRPY().toString();
                    //m_imuOrientation
                    m_pelvisimuAcceleration(0)=(*pelvisIMU)(3) ;
                    m_pelvisimuAcceleration(1)=(*pelvisIMU)(4) ;
                    m_pelvisimuAcceleration(2)=(*pelvisIMU)(5) ;

                    m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(6) ;
                    m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(7) ;
                    m_pelvisimuAngularVelocity(0)=(*pelvisIMU)(8) ;
                    okPelvisIMU=true;
                }
            }
        }


        if (m_useHeadIMU) {
            if(!okHeadIMU){
                yarp::sig::Vector *headIMU = NULL;
                headIMU=m_headIMUPort.read(false);
                if (headIMU!=NULL) {
                    //yInfo()<<(*pelvisIMU)(0)<<iDynTree::rad2deg((*pelvisIMU)(1))<<(*pelvisIMU)(2);
                    auto head_imu_idx = modelLoader.getFrameIndex("imu_frame");
                    auto head_R_imu = modelLoader.getFrameTransform(head_imu_idx).getRotation();
                    auto root_R_imu=headToBaseRotation*head_R_imu;
                    // yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();
                    //yInfo()<<"m_imuOrientation"<<m_imuOrientation.asRPY().toString();
                    // yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();

                    m_headimuOrientation=m_headimuOrientation.RPY(iDynTree::deg2rad((*headIMU)(0)),iDynTree::deg2rad((*headIMU)(1)),iDynTree::deg2rad((*headIMU)(2) ));
                    yInfo()<<"head imubefore"<<m_headimuOrientation.asRPY().toString();
                    //                m_imuOrientation=root_R_imu.inverse()*m_imuOrientation;

                    iDynTree::toEigen(m_headimuOrientation)=iDynTree::toEigen(intialHeadIMUOrientation)*iDynTree::toEigen(m_headimuOrientation)*iDynTree::toEigen( root_R_imu.inverse());
                    yInfo()<<"head inital"<<intialHeadIMUOrientation.asRPY().toString();
                    //                                yInfo()<<"root_R_imu"<<root_R_imu.asRPY().toString();
                    yInfo()<<"head imuafter"<<m_headimuOrientation.asRPY().toString();
                    //m_imuOrientation
                    m_headimuAcceleration(0)=(*headIMU)(3) ;
                    m_headimuAcceleration(1)=(*headIMU)(4) ;
                    m_headimuAcceleration(2)=(*headIMU)(5) ;

                    m_headimuAngularVelocity(0)=(*headIMU)(6) ;
                    m_headimuAngularVelocity(0)=(*headIMU)(7) ;
                    m_headimuAngularVelocity(0)=(*headIMU)(8) ;
                    okHeadIMU=true;
                }
            }
        }

        if(okPosition && okVelocity && okLeftWrench && okRightWrench && okHeadIMU && okPelvisIMU &&  (okBaseEstimation || okFloatingBaseEstimation))

        {
            for(unsigned j = 0 ; j < m_actuatedDOFs; j++)
            {
                m_positionFeedbackRad(j) = iDynTree::deg2rad(m_positionFeedbackDeg(j));
                m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDeg(j));
            }

            if(!iDynTree::toiDynTree(m_leftWrenchInput, m_leftWrench))
            {
                yError() << "[RobotHelper::getFeedbacksRaw] Unable to convert left foot wrench.";
                return false;
            }
            if(!iDynTree::toiDynTree(m_rightWrenchInput, m_rightWrench))
            {
                yError() << "[RobotHelper::getFeedbacksRaw] Unable to convert right foot wrench.";
                return false;
            }
            return true;
        }
        yarp::os::Time::delay(0.001);
        attempt++;
    } while (attempt < maxAttempts);

    yError() << "[RobotHelper::getFeedbacksRaw] The following readings failed:";
    if(!okPosition)
        yError() << "\t - Position encoders";

    if(!okVelocity)
        yError() << "\t - Velocity encoders";

    if(!okLeftWrench)
        yError() << "\t - Left wrench";

    if(!okRightWrench)
        yError() << "\t - Right wrench";

    if(!okBaseEstimation)
        yError() << "\t - Base estimation";

    if(!okPelvisIMU)
        yError() << "\t - Pelvis imu data";

    if(!okHeadIMU)
        yError() << "\t - Head imu data";
    return false;
}

bool RobotHelper::configureRobot(const yarp::os::Searchable& config)
{
    // robot name: used to connect to the robot
    std::string robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    double sampligTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    std::string name;
    if(!YarpHelper::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotHelper::configureRobot] Unable to get the string from searchable.";
        return false;
    }

    yarp::os::Value *axesListYarp;
    if(!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotHelper::configureRobot] Unable to find joints_list into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotHelper::configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value *iCubPartsYarp;
    if(!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[configureRobot] Unable to find remote_control_boards into config file.";
        return false;
    }
    if(!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[configureRobot] Unable to convert yarp list into a vector of strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    m_remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = m_remoteControlBoards.addList();
    for(auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", m_remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = m_axesList.size();

    // open the device
    if(!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    // obtain the interfaces
    if(!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[configureRobot] Cannot obtain IEncoders interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionControl interface";
        return false;
    }

    if(!m_robotDevice.view(m_velocityInterface) || !m_velocityInterface)
    {
        yError() << "[configureRobot] Cannot obtain IVelocityInterface interface";
        return false;
    }

    if(!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[configureRobot] Cannot obtain IPositionDirect interface";
        return false;
    }

    if(!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    if(!m_robotDevice.view(m_limitsInterface) || !m_controlModeInterface)
    {
        yError() << "[configureRobot] Cannot obtain IControlMode interface";
        return false;
    }

    // resize the buffers
    m_positionFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_velocityFeedbackDeg.resize(m_actuatedDOFs, 0.0);
    m_positionFeedbackRad.resize(m_actuatedDOFs);
    m_velocityFeedbackRad.resize(m_actuatedDOFs);
    m_desiredJointPositionRad.resize(m_actuatedDOFs);
    m_desiredJointValueDeg.resize(m_actuatedDOFs);
    m_jointVelocitiesBounds.resize(m_actuatedDOFs);
    m_jointPositionsUpperBounds.resize(m_actuatedDOFs);
    m_jointPositionsLowerBounds.resize(m_actuatedDOFs);

    m_velocityFeedbackDegFiltered.resize(m_actuatedDOFs);
    m_velocityFeedbackDegFiltered.zero();

    // check if the robot is alive
    bool okPosition = false;
    bool okVelocity = false;
    for (int i=0; i < 10 && !okPosition && !okVelocity; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackDeg.data());
        okVelocity = m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackDeg.data());

        if(!okPosition || !okVelocity)
            yarp::os::Time::delay(0.1);
    }
    if(!okPosition)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    if(!okVelocity)
    {
        yError() << "[configure] Unable to read encoders.";
        return false;
    }

    m_useVelocityFilter = config.check("use_joint_velocity_filter", yarp::os::Value("False")).asBool();
    if(m_useVelocityFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getNumberFromSearchable(config, "joint_velocity_cut_frequency", cutFrequency))
        {
            yError() << "[configure] Unable get double from searchable.";
            return false;
        }

        // set filters
        m_velocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                 sampligTime);


        m_velocityFilter->init(m_velocityFeedbackDeg);
    }

    // get the limits
    double maxVelocity, minAngle, maxAngle, dummy;
    for(unsigned int i = 0; i < m_actuatedDOFs; i++)
    {
        if(!m_limitsInterface->getVelLimits(i, &dummy, &maxVelocity))
        {
            yError() << "[configure] Unable get the velocity limits of the joint: "
                     << m_axesList[i];
            return false;
        }

        m_jointVelocitiesBounds(i) = iDynTree::deg2rad(maxVelocity);


        if(!m_limitsInterface->getLimits(i, &minAngle, &maxAngle))
        {
            yError() << "[configure] Unable get the position limits of the joint: "
                     << m_axesList[i];
            return false;
        }

        m_jointPositionsUpperBounds(i) = iDynTree::deg2rad(maxAngle);
        m_jointPositionsLowerBounds(i) = iDynTree::deg2rad(minAngle);

    }

    m_useFloatingBaseEstimator=config.check("use_floating_base_estimator", yarp::os::Value("False")).asBool();
    if(m_useFloatingBaseEstimator)
    {
        m_robotBaseEstimatorPort.open("/" + name + "/robotBaseEstimator:i");
        // connect port

        std::string floatingBaseEstimatorPortName;
        if(!YarpHelper::getStringFromSearchable(config, "floating_base_estimator_port_name", floatingBaseEstimatorPortName))
        {
            yError() << "[RobotHelper::useFloatingBaseEstimator] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(floatingBaseEstimatorPortName, "/" + name + "/robotBaseEstimator:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/robotBaseEstimator:i";
            return false;
        }
    }


    m_usePelvisIMU=config.check("m_use_pelvis_imu", yarp::os::Value("False")).asBool();
    if (m_usePelvisIMU) {
        m_pelvisIMUPort.open("/" + name + "/pelvisIMU:i");
        // connect port

        std::string pelvisIMUPortName;
        if(!YarpHelper::getStringFromSearchable(config, "imu_pelvis_port_name", pelvisIMUPortName))
        {
            yError() << "[RobotHelper::pelvisIMUPort] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(pelvisIMUPortName,"/" + name + "/pelvisIMU:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/pelvisIMU::i";
            return false;
        }
    }


    m_useHeadIMU=config.check("m_use_head_imu", yarp::os::Value("False")).asBool();
    if (m_useHeadIMU) {
        m_headIMUPort.open("/" + name + "/headIMU:i");
        // connect port

        std::string headIMUPortName;
        if(!YarpHelper::getStringFromSearchable(config, "imu_head_port_name", headIMUPortName))
        {
            yError() << "[RobotHelper::headIMUPort] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(headIMUPortName,"/" + name + "/headIMU:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/headIMU::i";
            return false;
        }
    }


    m_useExternalRobotBase = config.check("use_external_robot_base", yarp::os::Value("False")).asBool();
    if(m_useExternalRobotBase)
    {
        m_robotBasePort.open("/" + name + "/robotBase:i");
        // connect port

        std::string floatingBasePortName;
        if(!YarpHelper::getStringFromSearchable(config, "floating_base_port_name", floatingBasePortName))
        {
            yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(floatingBasePortName, "/" + name + "/robotBase:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/robotBase:i";
            return false;
        }
    }


    m_heightOffset = 0;


    return true;
}

bool RobotHelper::configureForceTorqueSensors(const yarp::os::Searchable& config)
{
    std::string portInput, portOutput;

    // check if the config file is empty
    if(config.isNull())
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Empty configuration for the force torque sensors.";
        return false;
    }

    std::string name;
    if(!YarpHelper::getStringFromSearchable(config, "name", name))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }

    double sampligTime = config.check("sampling_time", yarp::os::Value(0.016)).asDouble();

    // open and connect left foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchInputPort_name", portInput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get "
                    "the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "leftFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_leftWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to connect to port "
                 << portOutput << " to " << "/" + name + portInput;
        return false;
    }

    // open and connect right foot wrench
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchInputPort_name", portInput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    if(!YarpHelper::getStringFromSearchable(config, "rightFootWrenchOutputPort_name", portOutput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to get the string from searchable.";
        return false;
    }
    // open port
    m_rightWrenchPort.open("/" + name + portInput);
    // connect port
    if(!yarp::os::Network::connect(portOutput, "/" + name + portInput))
    {
        yError() << "[RobotHelper::configureForceTorqueSensors] Unable to connect to port "
                 << portOutput << " to " << "/" + name + portInput;
        return false;
    }

    m_useWrenchFilter = config.check("use_wrench_filter", yarp::os::Value("False")).asBool();
    if(m_useWrenchFilter)
    {
        double cutFrequency;
        if(!YarpHelper::getNumberFromSearchable(config, "wrench_cut_frequency", cutFrequency))
        {
            yError() << "[RobotHelper::configureForceTorqueSensors] Unable get double from searchable.";
            return false;
        }

        m_leftWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                   sampligTime);
        m_rightWrenchFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency,
                                                                                    sampligTime);
    }
    return true;
}

bool RobotHelper::configurePIDHandler(const yarp::os::Bottle& config)
{
    m_PIDHandler = std::make_unique<WalkingPIDHandler>();
    return m_PIDHandler->initialize(config, m_robotDevice, m_remoteControlBoards);
}


bool RobotHelper::resetFilters(const iDynTree::Model modelLoader,const iDynTree::Rotation baseToWorldRotation)
{
    //if(!getFeedbacksRaw())
    if(!getFeedbacksRaw(modelLoader,m_initialPelvisIMUOrientation,m_initialHeadIMUOrientation,baseToWorldRotation,100, true))
    {
        yError() << "[RobotHelper::resetFilters] Unable to get the feedback from the robot";
        return false;
    }


    if(m_useVelocityFilter)
        m_velocityFilter->init(m_velocityFeedbackDeg);

    if(m_useWrenchFilter)
    {
        m_leftWrenchFilter->init(m_leftWrenchInput);
        m_rightWrenchFilter->init(m_rightWrenchInput);
    }

    return true;
}

bool RobotHelper::getFeedbacks(const iDynTree::Model modelLoader,iDynTree::Rotation baseToWorldRotation, unsigned int maxAttempts)
{
    // by default we consider the presence of a base estimation block
    bool useBaseEst = true;

    if(!getFeedbacksRaw(modelLoader,m_initialPelvisIMUOrientation,m_initialHeadIMUOrientation,baseToWorldRotation,maxAttempts, useBaseEst))
    {
        yError() << "[RobotHelper::getFeedbacks] Unable to get the feedback from the robot";
        return false;
    }

    if(m_useVelocityFilter)
    {
        // filter the joint position and the velocity
        m_velocityFeedbackDegFiltered = m_velocityFilter->filt(m_velocityFeedbackDeg);
        for(unsigned j = 0; j < m_actuatedDOFs; ++j)
            m_velocityFeedbackRad(j) = iDynTree::deg2rad(m_velocityFeedbackDegFiltered(j));
    }
    if(m_useWrenchFilter)
    {
        m_leftWrenchInputFiltered = m_leftWrenchFilter->filt(m_leftWrenchInput);
        m_rightWrenchInputFiltered = m_rightWrenchFilter->filt(m_rightWrenchInput);

        if(!iDynTree::toiDynTree(m_leftWrenchInputFiltered, m_leftWrench))
        {
            yError() << "[RobotHelper::getFeedbacks] Unable to convert left foot wrench.";
            return false;
        }
        if(!iDynTree::toiDynTree(m_rightWrenchInputFiltered, m_rightWrench))
        {
            yError() << "[RobotHelper::getFeedbacks] Unable to convert right foot wrench.";
            return false;
        }
    }
    return true;
}

bool RobotHelper::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if(!m_controlModeInterface)
    {
        yError() << "[RobotHelper::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if(!m_controlModeInterface->setControlModes(controlModes.data()))
    {
        yError() << "[RobotHelper::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotHelper::setPositionReferences(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                        const double& positioningTimeSec)
{
    if(m_controlMode != VOCAB_CM_POSITION)
    {
        if(!switchToControlMode(VOCAB_CM_POSITION))
        {
            yError() << "[RobotHelper::setPositionReferences] Unable to switch in position control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_POSITION;
    }

    m_positioningTime = positioningTimeSec;
    m_positionMoveSkipped = false;
    if(m_positionInterface == nullptr)
    {
        yError() << "[RobotHelper::setPositionReferences] Position I/F is not ready.";
        return false;
    }

    m_desiredJointPositionRad = desiredJointPositionsRad;

    std::pair<std::string, double> worstError("", 0.0);

    if(!getWorstError(desiredJointPositionsRad, worstError))
    {
        yError() << "[RobotHelper::setPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second < 0.03)
    {
        m_positionMoveSkipped = true;
        return true;
    }

    if(positioningTimeSec < 0.01)
    {
        yError() << "[RobotHelper::setPositionReferences] The positioning time is too short.";
        return false;
    }

    if(!m_encodersInterface->getEncoders(m_positionFeedbackDeg.data()))
    {
        yError() << "[RobotHelper::setPositionReferences] Error while reading encoders.";
        return false;
    }

    std::vector<double> refSpeeds(m_actuatedDOFs);

    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for (int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        refSpeeds[i] = std::max(3.0, iDynTree::rad2deg(absoluteJointErrorRad) / positioningTimeSec);
    }

    if(!m_positionInterface->setRefSpeeds(refSpeeds.data()))
    {
        yError() << "[RobotHelper::setPositionReferences] Error while setting the desired speed of joints.";
        return false;
    }

    // convert a radians vector into a degree vector
    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(m_desiredJointPositionRad(i)) ;

    if(!m_positionInterface->positionMove(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotHelper::setPositionReferences] Error while setting the desired positions.";
        return false;
    }

    m_startingPositionControlTime = yarp::os::Time::now();
    return true;
}

bool RobotHelper::checkMotionDone(bool& motionDone)
{
    // if the position move is skipped the motion is implicitly done
    if(m_positionMoveSkipped)
    {
        motionDone = true;
        return true;
    }

    bool checkMotionDone = false;
    m_positionInterface->checkMotionDone(&checkMotionDone);

    std::pair<std::string, double> worstError;
    if (!getWorstError(m_desiredJointPositionRad, worstError))
    {
        yError() << "[RobotHelper::checkMotionDone] Unable to get the worst error.";
        return false;
    }

    double now = yarp::os::Time::now();
    double timeThreshold = 1;
    if (now - m_startingPositionControlTime > m_positioningTime + timeThreshold)
    {
        yError() << "[RobotHelper::checkMotionDone] The timer is expired but the joint "
                 << worstError.first << " has an error of " << worstError.second
                 << " radians";
        return false;
    }

    motionDone = checkMotionDone && worstError.second < 0.1;
    return true;
}

bool RobotHelper::setDirectPositionReferences(const iDynTree::VectorDynSize& desiredPositionRad)
{
    if(m_positionDirectInterface == nullptr)
    {
        yError() << "[RobotHelper::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[RobotHelper::setDirectPositionReferences] Encoders I/F not ready.";
        return false;
    }

    if(m_controlMode != VOCAB_CM_POSITION_DIRECT)
    {
        if(!switchToControlMode(VOCAB_CM_POSITION_DIRECT))
        {
            yError() << "[RobotHelper::setDirectPositionReferences] Unable to switch in position-direct control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_POSITION_DIRECT;
    }

    if(desiredPositionRad.size() != m_actuatedDOFs)
    {
        yError() << "[RobotHelper::setDirectPositionReferences] Dimension mismatch between desired position "
                 << "vector and the number of controlled joints.";
        return false;
    }

    std::pair<std::string, double> worstError("", 0.0);

    if(!getWorstError(desiredPositionRad, worstError))
    {
        yError() << "[RobotHelper::setDirectPositionReferences] Unable to get the worst error.";
        return false;
    }

    if(worstError.second > 0.5)
    {
        yError() << "[RobotHelper::setDirectPositionReferences] The worst error between the current and the "
                 << "desired position of the " << worstError.first
                 << "-th joint is greater than 0.5 rad.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredPositionRad(i));

    if(!m_positionDirectInterface->setPositions(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotHelper::setDirectPositionReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotHelper::setVelocityReferences(const iDynTree::VectorDynSize& desiredVelocityRad)
{
    if(m_velocityInterface == nullptr)
    {
        yError() << "[RobotHelper::setVelocityReferences] PositionDirect I/F not ready.";
        return false;
    }

    if(m_encodersInterface == nullptr)
    {
        yError() << "[RobotHelper::setVelocityReferences] Encoders I/F not ready.";
        return false;
    }

    if(m_controlMode != VOCAB_CM_VELOCITY)
    {
        if(!switchToControlMode(VOCAB_CM_VELOCITY))
        {
            yError() << "[RobotHelper::setVelocityReferences] Unable to switch in velocity control mode.";
            return false;
        }
        m_controlMode = VOCAB_CM_VELOCITY;
    }

    if(desiredVelocityRad.size() != m_actuatedDOFs)
    {
        yError() << "[RobotHelper::setVelocityReferences] Dimension mismatch between desired velocity "
                 << "vector and the number of controlled joints.";
        return false;
    }

    for(unsigned i = 0; i < m_actuatedDOFs; i++)
        m_desiredJointValueDeg(i) = iDynTree::rad2deg(desiredVelocityRad(i));

    if(!m_velocityInterface->velocityMove(m_desiredJointValueDeg.data()))
    {
        yError() << "[RobotHelper::setVelocityReferences] Error while setting the desired position.";
        return false;
    }

    return true;
}

bool RobotHelper::close()
{
    m_rightWrenchPort.close();
    m_leftWrenchPort.close();
    switchToControlMode(VOCAB_CM_POSITION);
    m_controlMode = VOCAB_CM_POSITION;
    if(!m_robotDevice.close())
    {
        yError() << "[RobotHelper::close] Unable to close the device.";
        return false;
    }

    return true;
}

const iDynTree::VectorDynSize& RobotHelper::getJointPosition() const
{
    return m_positionFeedbackRad;
}
const iDynTree::VectorDynSize& RobotHelper::getJointVelocity() const
{
    return m_velocityFeedbackRad;
}

const iDynTree::Wrench& RobotHelper::getLeftWrench() const
{
    return m_leftWrench;
}

const iDynTree::Wrench& RobotHelper::getRightWrench() const
{
    return m_rightWrench;
}

const iDynTree::VectorDynSize& RobotHelper::getVelocityLimits() const
{
    return m_jointVelocitiesBounds;
}

const iDynTree::VectorDynSize& RobotHelper::getPositionUpperLimits() const
{
    return m_jointPositionsUpperBounds;
}

const iDynTree::VectorDynSize& RobotHelper::getPositionLowerLimits() const
{
    return m_jointPositionsLowerBounds;
}

const std::vector<std::string>& RobotHelper::getAxesList() const
{
    return m_axesList;
}

int RobotHelper::getActuatedDoFs()
{
    return m_actuatedDOFs;
}

WalkingPIDHandler& RobotHelper::getPIDHandler()
{
    return *m_PIDHandler;
}


const iDynTree::Transform& RobotHelper::getBaseTransform() const
{
    return m_robotBaseTransform;
}

const iDynTree::Twist& RobotHelper::getBaseTwist() const
{
    return m_robotBaseTwist;
}

const iDynTree::Transform& RobotHelper::getEstimatedBaseTransform() const
{
    return m_robotEstimatedBaseTransform;
}

const iDynTree::Twist& RobotHelper::getEstimatedBaseTwist() const
{
    return m_robotEstimatedBaseTwist;
}

const iDynTree::LinAcceleration& RobotHelper::getPelvisIMUAcceleration() const
{
    return m_pelvisimuAcceleration;
}

const iDynTree::AngVelocity& RobotHelper::getPelvisIMUAngularVelocity() const
{
    return m_pelvisimuAngularVelocity;
}

const iDynTree::Rotation& RobotHelper::getPelvisIMUOreintation() const
{
    return m_pelvisimuOrientation;
}

const iDynTree::Rotation& RobotHelper::getInitialPelvisIMUOreintation() const
{
    return m_initialPelvisIMUOrientation;
}


const iDynTree::LinAcceleration& RobotHelper::getHeadIMUAcceleration() const
{
    return m_headimuAcceleration;
}

const iDynTree::AngVelocity& RobotHelper::getHeadIMUAngularVelocity() const
{
    return m_headimuAngularVelocity;
}

const iDynTree::Rotation& RobotHelper::getHeadIMUOreintation() const
{
    return m_headimuOrientation;
}

const iDynTree::Rotation& RobotHelper::getInitialHeadIMUOreintation() const
{
    return m_initialHeadIMUOrientation;
}


void RobotHelper::setHeightOffset(const double& offset)
{
    m_heightOffset = offset;
}

bool RobotHelper::isExternalRobotBaseUsed()
{
    return m_useExternalRobotBase;
}

bool RobotHelper::isFloatingBaseEstimatorUsed()
{
    return m_useFloatingBaseEstimator;
}

bool RobotHelper::isPelvisIMUUsed()
{
    return m_usePelvisIMU;
}

bool RobotHelper::isHeadIMUUsed()
{
    return m_useHeadIMU;
}
