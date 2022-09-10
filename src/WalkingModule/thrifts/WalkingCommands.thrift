/**
 * @file walkingCommands.thrift
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

struct YarpVector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

service WalkingCommands
{
    /**
     * Call this method to prepare the robot.
     * iCub will be moved in the home position and the positionDirect
     * interface will be set.
     * @return true/false in case of success/failure;
     */
    bool prepareRobot(1:bool onTheFly=0);

    /**
     * Run the entire walking controller.
     * @return true/false in case of success/failure;
     */
    bool startWalking();

    /**
     * Set the desired goal position to the planner.
     * @return true/false in case of success/failure;
     */
    bool setGoal(1:YarpVector plannerInput);

    /**
     * Pause the walking controller
     * @return true/false in case of success/failure;
     */
    bool pauseWalking();

    /**
     * Stop the walking controller
     * @return true/false in case of success/failure;
     */
    bool stopWalking();
}
