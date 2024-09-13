// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
