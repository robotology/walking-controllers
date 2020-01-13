#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPConversions.h>

#include <RobotHelper.hpp>
#include <Utils.hpp>
#include <iDynTree/Model/Model.h>

bool RobotHelper::getWorstError(const iDynTree::VectorDynSize& desiredJointPositionsRad,
                                std::pair<int, double>& worstError)
{
    if(!m_encodersInterface)
    {
        yError() << "[RobotHelper::getWorstError] The encoder I/F is not ready";
        return false;
    }

    // clear the std::pair
worstError.first = 0;
    worstError.second = 0.0;
    double currentJointPositionRad;
    double absoluteJointErrorRad;
    for(int i = 0; i < m_actuatedDOFs; i++)
    {
        currentJointPositionRad = iDynTree::deg2rad(m_positionFeedbackDeg[i]);
        absoluteJointErrorRad = std::fabs(iDynTreeHelper::shortestAngularDistance(currentJointPositionRad,
                                                                                  desiredJointPositionsRad(i)));
        if(m_currentModeofJoints.at(i)==yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF)
        {
            if(absoluteJointErrorRad > worstError.second)
                        {
                            worstError.first =i;
                            worstError.second = absoluteJointErrorRad;
                        }
        }
    }
    return true;
}



bool RobotHelper::getFeedbacksRaw(unsigned int maxAttempts, bool getBaseEst)
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
    bool okRFootIMU = true;
    bool okLFootIMU = true;
    bool okRFootIMUExperiment = true;
    bool okLFootIMUExperiment = true;

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

    if(m_useFeetIMUSimulation){
        okLFootIMU = !m_useFeetIMUSimulation;
        okRFootIMU = !m_useFeetIMUSimulation;
    }

    if(m_useFeetIMUExperiment){
        okLFootIMUExperiment = !m_useFeetIMUExperiment;
        okRFootIMUExperiment = !m_useFeetIMUExperiment;
    }

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
        if (m_useFeetIMUSimulation) {

            if(!okRFootIMU){
                yarp::sig::Vector *rFootIMU = NULL;
                rFootIMU=m_rightFootIMUPort.read(false);
                if (rFootIMU!=NULL) {

                    m_rightFootIMUOrientation=m_rightFootIMUOrientation.RPY(iDynTree::deg2rad((*rFootIMU)(0)),iDynTree::deg2rad((*rFootIMU)(1)),iDynTree::deg2rad((*rFootIMU)(2)));
                    m_rightFootIMUAcceleration(0)=(*rFootIMU)(3) ;
                    m_rightFootIMUAcceleration(1)=(*rFootIMU)(4) ;
                    m_rightFootIMUAcceleration(2)=(*rFootIMU)(5) ;

                    m_rightFootIMUAngularVelocity(0)=(*rFootIMU)(6) ;
                    m_rightFootIMUAngularVelocity(1)=(*rFootIMU)(7) ;
                    m_rightFootIMUAngularVelocity(2)=(*rFootIMU)(8) ;
                    okRFootIMU=true;
                }
            }

            if(!okLFootIMU){
                yarp::sig::Vector *lFootIMU = NULL;
                lFootIMU=m_leftFootIMUPort.read(false);
                if (lFootIMU!=NULL) {

                    m_leftFootIMUOrientation=m_leftFootIMUOrientation.RPY(iDynTree::deg2rad((*lFootIMU)(0)),iDynTree::deg2rad((*lFootIMU)(1)),iDynTree::deg2rad((*lFootIMU)(2)));
                    m_leftFootIMUAcceleration(0)=(*lFootIMU)(3) ;
                    m_leftFootIMUAcceleration(1)=(*lFootIMU)(4) ;
                    m_leftFootIMUAcceleration(2)=(*lFootIMU)(5) ;

                    m_leftFootIMUAngularVelocity(0)=(*lFootIMU)(6) ;
                    m_leftFootIMUAngularVelocity(1)=(*lFootIMU)(7) ;
                    m_leftFootIMUAngularVelocity(2)=(*lFootIMU)(8) ;
                    okLFootIMU=true;
                }
            }
        }


        if (m_useFeetIMUExperiment) {

            if(!okLFootIMUExperiment){
                double time=0;
                yarp::sig::Vector lFootGyro;
                lFootGyro.resize(3);
                lFootGyro.zero();
                yarp::sig::Vector lFootAccelerometer;
                lFootAccelerometer.resize(3);
                lFootAccelerometer.zero();
                yarp::sig::Vector lFootOrientationSensor;
                lFootOrientationSensor.resize(3);
                lFootOrientationSensor.zero();


                std::string nameOfSensor{" "};

                if(m_gyros->getNrOfThreeAxisGyroscopes()!=2)
                {
                    yError()<<"[RobotHelper::FootIMUExperiment]The number of feet gyroscopes is not two!";
                    return false;
                }





                m_gyros->getThreeAxisGyroscopeName(static_cast<size_t>(0),nameOfSensor);
                if (!(nameOfSensor=="l_foot_ft_gyro_3b13")) {
                    yError()<<"[RobotHelper::FootIMUExperiment]This index is not related to left foot IMU";
                    return false;
                }

//                if((m_accelerometers->getThreeAxisLinearAccelerometerStatus(0)==yarp::dev::MAS_OK)||
//                        (m_gyros->getThreeAxisGyroscopeStatus(0)==yarp::dev::MAS_OK)||
//                        (m_imu_orientation_sensors->getOrientationSensorStatus(0)==yarp::dev::MAS_OK)){
//                    yWarning()<<"[RobotHelper::FootIMUExperiment]unable to get right foot imu data";

//                }

                if (!m_gyros->getThreeAxisGyroscopeMeasure(0,lFootGyro,time) || !(m_gyros->getThreeAxisGyroscopeStatus(0)==yarp::dev::MAS_OK)) {
                    yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the left foot gyro data";
                     okLFootIMUExperiment=false;
                }
                else{
                 yInfo()<<nameOfSensor<<"Gyro"<<lFootGyro.toString();
                okLFootIMUExperiment=true;
                }


                if (!m_accelerometers->getThreeAxisLinearAccelerometerMeasure(0,lFootAccelerometer,time) || !(m_accelerometers->getThreeAxisLinearAccelerometerStatus(0)==yarp::dev::MAS_OK)) {
                    yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the left foot accelerometer data";
                    okLFootIMUExperiment=false;
                }
                else {
                      yInfo()<<nameOfSensor<<"Accel"<<lFootAccelerometer.toString();
                 okLFootIMUExperiment=true;
                }


                if (!m_imu_orientation_sensors->getOrientationSensorMeasureAsRollPitchYaw(0,lFootOrientationSensor,time) || !(m_imu_orientation_sensors->getOrientationSensorStatus(0)==yarp::dev::MAS_OK)) {
                   yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the left foot accelerometer data";
                   okLFootIMUExperiment=false;
                }
                else{
                    yInfo()<<nameOfSensor<<"Orientation"<<lFootOrientationSensor.toString();
                 okLFootIMUExperiment=true;
                }


                m_leftFootIMUOrientation=m_leftFootIMUOrientation.RPY(-1*iDynTree::deg2rad((lFootOrientationSensor)(1)),-1*iDynTree::deg2rad((lFootOrientationSensor)(2)),-1*iDynTree::deg2rad((lFootOrientationSensor)(0)));
                m_leftFootIMUAcceleration(0)=(lFootAccelerometer)(0) ;
                m_leftFootIMUAcceleration(1)=(lFootAccelerometer)(1) ;
                m_leftFootIMUAcceleration(2)=(lFootAccelerometer)(2) ;

                m_leftFootIMUAngularVelocity(0)=(lFootGyro)(0) ;
                m_leftFootIMUAngularVelocity(1)=(lFootGyro)(1) ;
                m_leftFootIMUAngularVelocity(2)=(lFootGyro)(2) ;
                okLFootIMU=okLFootIMUExperiment;
                }


            if(!okRFootIMUExperiment){
                double time=0;
                yarp::sig::Vector rFootGyro;
                rFootGyro.resize(3);
                rFootGyro.zero();
                yarp::sig::Vector rFootAccelerometer;
                rFootAccelerometer.resize(3);
                rFootAccelerometer.zero();
                yarp::sig::Vector rFootOrientationSensor;
                rFootOrientationSensor.resize(3);
                rFootOrientationSensor.zero();

                std::string nameOfSensor;
                m_gyros->getThreeAxisGyroscopeName(1,nameOfSensor);
                if (!(nameOfSensor=="r_foot_ft_gyro_3b14")) {
                    yError()<<"[RobotHelper::FootIMUExperiment]This index is not related to right foot IMU";
                    return false;
                }

//                if((m_accelerometers->getThreeAxisLinearAccelerometerStatus(1)==yarp::dev::MAS_OK)||
//                        (m_gyros->getThreeAxisGyroscopeStatus(1)==yarp::dev::MAS_OK)||
//                        (m_imu_orientation_sensors->getOrientationSensorStatus(1))==yarp::dev::MAS_OK){
//                    yWarning()<<"[RobotHelper::FootIMUExperiment]unable to get right foot imu data";
//                    okRFootIMUExperiment=false;
//                }
//                else {
//                    okRFootIMUExperiment=true;
//                }

                if (!m_gyros->getThreeAxisGyroscopeMeasure(1,rFootGyro,time) || !(m_gyros->getThreeAxisGyroscopeStatus(1)==yarp::dev::MAS_OK)) {
                    yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the right foot gyro data";
                    okRFootIMUExperiment=false;
                }
                else {
                    yInfo()<<nameOfSensor<<"Gyro"<<rFootGyro.toString();
                    okRFootIMUExperiment=true;
                }


                if (!m_accelerometers->getThreeAxisLinearAccelerometerMeasure(1,rFootAccelerometer,time) || !(m_accelerometers->getThreeAxisLinearAccelerometerStatus(1)==yarp::dev::MAS_OK)) {

                    yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the right foot accelerometer data";
                    okRFootIMUExperiment=false;
                }
                else {
                     yInfo()<<nameOfSensor<<"Accel"<<rFootAccelerometer.toString();
                    okRFootIMUExperiment=true;
                }

                if (!m_imu_orientation_sensors->getOrientationSensorMeasureAsRollPitchYaw(1,rFootOrientationSensor,time)  || !(m_imu_orientation_sensors->getOrientationSensorStatus(1)==yarp::dev::MAS_OK)) {

                    yWarning()<<"[RobotHelper::FootIMUExperiment]Unable to recieve the right foot accelerometer data";
                    okRFootIMUExperiment=false;
                }
                else {
                     yInfo()<<nameOfSensor<<"Orientation"<<rFootOrientationSensor.toString();
                    okRFootIMUExperiment=true;
                }


                m_rightFootIMUOrientation=m_rightFootIMUOrientation.RPY(iDynTree::deg2rad((rFootOrientationSensor)(0)),iDynTree::deg2rad((rFootOrientationSensor)(1)),iDynTree::deg2rad((rFootOrientationSensor)(2)));
                m_rightFootIMUAcceleration(0)=(rFootAccelerometer)(0) ;
                m_rightFootIMUAcceleration(1)=(rFootAccelerometer)(1) ;
                m_rightFootIMUAcceleration(2)=(rFootAccelerometer)(2) ;

                m_rightFootIMUAngularVelocity(0)=(rFootGyro)(0) ;
                m_rightFootIMUAngularVelocity(1)=(rFootGyro)(1) ;
                m_rightFootIMUAngularVelocity(2)=(rFootGyro)(2) ;
                okRFootIMU=okRFootIMUExperiment;
                }
            }



        if (m_usePelvisIMU) {
            if(!okPelvisIMU){
                yarp::sig::Vector *pelvisIMU = NULL;
                pelvisIMU=m_pelvisIMUPort.read(false);
                if (pelvisIMU!=NULL) {

                    m_pelvisimuOrientation=m_pelvisimuOrientation.RPY(iDynTree::deg2rad((*pelvisIMU)(0)),iDynTree::deg2rad((*pelvisIMU)(1)),iDynTree::deg2rad((*pelvisIMU)(2)));
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

                    m_headimuOrientation=m_headimuOrientation.RPY(iDynTree::deg2rad((*headIMU)(0)),iDynTree::deg2rad((*headIMU)(1)),iDynTree::deg2rad((*headIMU)(2)));

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

        if(okPosition && okVelocity && okLeftWrench && okRightWrench && okHeadIMU && okPelvisIMU && okLFootIMU  && okRFootIMU && okRFootIMU && (okBaseEstimation || okFloatingBaseEstimation))

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

    if(!okLFootIMU)
        yError() << "\t - Left Foot imu data";

    if(!okRFootIMU)
        yError() << "\t - Right Foot imu data";

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
    m_isJointModeStiffVector.resize(m_actuatedDOFs);
        m_JointModeStiffVectorDefult.resize(m_actuatedDOFs);
        m_jointModes.resize(m_actuatedDOFs);
        yInfo()<<m_actuatedDOFs;

        if(!YarpHelper::getVectorOfBooleanFromSearchable(config,"joint_is_stiff_mode",m_jointModes))
        {
            yError() << "[RobotInterface::configureRobot] Unable to find joint_is_stiff_mode into config file.";
            return false;
        }

        for (unsigned int i=0;i<m_actuatedDOFs;i++)
        {
            m_JointModeStiffVectorDefult.at(i)=yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
            if(m_jointModes[i])
            {
                m_isJointModeStiffVector.at(i)=yarp::dev::InteractionModeEnum::VOCAB_IM_STIFF;
            }
            else
            {
                m_isJointModeStiffVector.at(i)=yarp::dev::InteractionModeEnum::VOCAB_IM_COMPLIANT;
            }
        }
        m_currentModeofJoints=m_JointModeStiffVectorDefult;

         // open the device

    if(!m_robotDevice.open(options))
    {
        yError() << "[configureRobot] Could not open remotecontrolboardremapper object.";
        return false;
    }

    if(!m_robotDevice.view(m_InteractionInterface) || !m_InteractionInterface)
    {
             yError() << "[configureRobot] Cannot obtain IInteractionMode interface";
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



    m_useFeetIMUSimulation=config.check("m_use_feet_imu_simulation", yarp::os::Value("False")).asBool();
    if (m_useFeetIMUSimulation) {
        m_leftFootIMUPort.open("/" + name + "/leftFootIMU:i");
        // connect port

        std::string leftFootIMUPortName;
        if(!YarpHelper::getStringFromSearchable(config, "imu_lfoot_port_name", leftFootIMUPortName))
        {
            yError() << "[RobotHelper::leftFootIMUPort] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(leftFootIMUPortName,"/" + name + "/leftFootIMU:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/leftFootIMU::i";
            return false;
        }


        m_rightFootIMUPort.open("/" + name + "/rightFootIMU:i");
        // connect port

        std::string rightFootIMUPortName;
        if(!YarpHelper::getStringFromSearchable(config, "imu_rfoot_port_name", rightFootIMUPortName))
        {
            yError() << "[RobotHelper::rightFootIMUPort] Unable to get the string from searchable.";
            return false;
        }

        if(!yarp::os::Network::connect(rightFootIMUPortName,"/" + name + "/rightFootIMU:i"))
        {
            yError() << "Unable to connect to port " << "/" + name + "/rightFootIMU::i";
            return false;
        }
    }



    m_useFeetIMUExperiment=config.check("m_use_feet_imu_experiment", yarp::os::Value("False")).asBool();
    if (m_useFeetIMUExperiment) {

        yarp::os::Property left_mas_client_property;

        std::string leftFootIMUPortName;
        //  m_leftFootIMUPort.open("/" + name + "/left_leg/inertials");
        if(!YarpHelper::getStringFromSearchable(config, "imu_lfoot_port_name", leftFootIMUPortName))
        {
            yError() << "[RobotHelper::leftFootIMUPort] Unable to get the string from searchable(experiment)";
            return false;
        }

        left_mas_client_property.put("remote",leftFootIMUPortName);
        left_mas_client_property.put("local", "/" + name + "/left_leg/inertials");
        left_mas_client_property.put("device","multipleanalogsensorsclient");

        if(!left_leg_inertial_client.open(left_mas_client_property))
        {
            yError() << "[RobotHelper::leftFootIMUPort] Unable to open  left leg inertial client port ";
            return false;
        }

        yarp::os::Property right_mas_client_property;

        std::string rightFootIMUPortName;
        // m_leftFootIMUPort.open("/" + name + "/right_leg/inertials");
        if(!YarpHelper::getStringFromSearchable(config, "imu_rfoot_port_name", rightFootIMUPortName))
        {
            yError() << "[RobotHelper::FootIMUPorts] Unable to get the string from searchable(experiment)";
            return false;
        }

        right_mas_client_property.put("remote",rightFootIMUPortName);
        right_mas_client_property.put("local", "/" + name + "/right_leg/inertials");
        right_mas_client_property.put("device","multipleanalogsensorsclient");

        if(!right_leg_inertial_client.open(right_mas_client_property))
        {
            yError() << "[RobotHelper::FootIMUPorts] Unable to open  left leg inertial client port ";
            return false;
        }
        left_leg_inertials.key = "left_leg_inertials";
        left_leg_inertials.poly = &left_leg_inertial_client;

        right_leg_inertials.key = "right_leg_inertials";
        right_leg_inertials.poly = &right_leg_inertial_client;


        mas_clients_list.push(left_leg_inertials);
        mas_clients_list.push(right_leg_inertials);

        m_masRemapperProperty.put("device", "multipleanalogsensorsremapper");
        yarp::os::Bottle threeAxisGyroscopes;
        yarp::os::Bottle & gyrosList = threeAxisGyroscopes.addList();
        gyrosList.addString("l_foot_ft_gyro_3b13");
        gyrosList.addString("r_foot_ft_gyro_3b14");
        m_masRemapperProperty.put("ThreeAxisGyroscopesNames",threeAxisGyroscopes.get(0));


        yarp::os::Bottle threeAxisAccelerometer;
        yarp::os::Bottle & accelerometerList = threeAxisAccelerometer.addList();
        accelerometerList.addString("l_foot_ft_acc_3b13");
        accelerometerList.addString("r_foot_ft_acc_3b14");
        m_masRemapperProperty.put("ThreeAxisLinearAccelerometersNames",threeAxisAccelerometer.get(0));

        yarp::os::Bottle orientation;
        yarp::os::Bottle & orientationList = orientation.addList();
        orientationList.addString("l_foot_ft_eul_3b13");
        orientationList.addString("r_foot_ft_eul_3b14");
        m_masRemapperProperty.put("OrientationSensorsNames",orientation.get(0));

        if(!m_masRemapperFeetIMU.open(m_masRemapperProperty))
        {
            yError() << "[RobotHelper::FootIMUPorts] Unable to open  MAS remapper for feet IMU";
            return false;
        }

        // Attach the remapper to the clients
        if (!m_masRemapperFeetIMU.view(remapperWrapperInterface))
        {
            yError()<<"[RobotHelper::FootIMUPorts] wrapper View failed for the MAS interfaces";
            return false;
        }

        if (remapperWrapperInterface == nullptr)
        {
            yError()<<"[RobotHelper::FootIMUPorts] wrapper View failed for the MAS interfaces";
            return false;
        }

        if (!remapperWrapperInterface->attachAll(mas_clients_list))
	{
	    yError()<<"[RobotHelper::FootIMUPorts] could not attach to MAS clients";
            return false;
	}


        if (!(m_masRemapperFeetIMU.view(m_gyros)) ||!(m_masRemapperFeetIMU.view(m_accelerometers)) || !(m_masRemapperFeetIMU.view(m_imu_orientation_sensors))) {
            yError()<<"[RobotHelper::FootIMUPorts] View failed for the MAS interfaces";
            return false;
        }

        if (m_gyros==nullptr) {
            yError()<<"[RobotHelper::FootIMUPorts] Gyroscope interface pointer is null!";
            return false;
        }
        if (m_accelerometers==nullptr) {
            yError()<<"[RobotHelper::FootIMUPorts] Accelerometers sensor interface pointer is null!";
            return false;
        }

        if (m_imu_orientation_sensors==nullptr) {
            yError()<<"[RobotHelper::FootIMUPorts] Orientation sensor interface pointer is null!";
            return false;
        }

for (size_t idx = 0; idx < m_imu_orientation_sensors->getNrOfOrientationSensors(); idx++ )
{
	std::string temp{" "};
m_imu_orientation_sensors->getOrientationSensorName(idx, temp);
yInfo() << "asdsafdsfsd";
yInfo() << "---->" << temp;

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
    if(!getFeedbacksRaw(100, true))
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

bool RobotHelper::getFeedbacks( unsigned int maxAttempts)
{
    // by default we consider the presence of a base estimation block
    bool useBaseEst = true;

    if(!getFeedbacksRaw(maxAttempts, useBaseEst))
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

    if(m_InteractionInterface == nullptr)
        {
            yError() << "[RobotInterface::setPositionReferences] IInteractionMode interface is not ready.";
            return false;
        }

    m_desiredJointPositionRad = desiredJointPositionsRad;

    std::pair<int, double> worstError(0, 0.0);

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

    std::pair<int, double> worstError;
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
                 << m_axesList[worstError.first] << " has an error of " << worstError.second
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


    std::pair<int, double> worstError(0, 0.0);
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
    m_InteractionInterface->setInteractionModes(m_JointModeStiffVectorDefult.data());
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

bool RobotHelper::setInteractionMode()
{
    if(!m_InteractionInterface->setInteractionModes(m_isJointModeStiffVector.data()))
    {
        yError() << "[RobotInterface::setInteractionMode] Error while setting the interaction modes of the joints";
        return false;
    }

        m_currentModeofJoints = m_isJointModeStiffVector;

    return true;
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


const iDynTree::LinAcceleration& RobotHelper::getLeftFootIMUAcceleration() const
{
    return m_leftFootIMUAcceleration;
}

const iDynTree::AngVelocity& RobotHelper::getLeftFootIMUAngularVelocity() const
{
    return m_leftFootIMUAngularVelocity;
}

const iDynTree::Rotation& RobotHelper::getLeftFootIMUOreintation() const
{
    return m_leftFootIMUOrientation;
}

const iDynTree::LinAcceleration& RobotHelper::getRightFootIMUAcceleration() const
{
    return m_rightFootIMUAcceleration;
}

const iDynTree::AngVelocity& RobotHelper::getRightFootIMUAngularVelocity() const
{
    return m_rightFootIMUAngularVelocity;
}

const iDynTree::Rotation& RobotHelper::getRightFootIMUOreintation() const
{
    return m_rightFootIMUOrientation;
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

bool RobotHelper::isFeetIMUUsedSimulation()
{
    return m_useFeetIMUSimulation;
}

bool RobotHelper::isFeetIMUUsedExperiment()
{
    return m_useFeetIMUExperiment;
}
