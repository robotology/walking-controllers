/**
 * @file walkingCommands.thrift
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

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
     * Set the desired goal position. It is important to notice that
     * x and y are expressed in the iCub main frame.
     * @return true/false in case of success/failure;
     */
    bool setGoal(1:double x, 2:double y);
}
