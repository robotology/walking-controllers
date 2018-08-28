/**
 * @file JoypadModule.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_JOYPAD_MODULE_HPP
#define WALKING_JOYPAD_MODULE_HPP

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/os/RpcClient.h>

/**
 * RFModule useful to handle the Joypad
 */
class JoypadModule : public yarp::os::RFModule
{
private:
    double m_dT; /**< RFModule period. */

    yarp::dev::PolyDriver m_joypad; /**< Joypad polydriver. */
    yarp::dev::IJoypadController* m_joypadController{nullptr}; /**< joypad interface. */

    double m_deadzone; /**< Value of the deadzone, if the output of the joypad is less than deadzone it will be set equal to zero. */
    double m_fullscale; /**< Max joypad output value. */

    double m_scaleX; /**< Scale of the joypad x-axis. */
    double m_scaleY; /**< Scale of the joypad y-axis. */

    std::string m_joypadOutputPortName; /**< Name of the joypad output port name. */
    std::string m_joypadInputPortName; /**< Name of the joypad input port name (This is the name of the port opened by the main module). */
    yarp::os::RpcClient m_rpcPort; /**< RPC port. */

    /**
     * Standard deadzone function.
     * @param input input of the deadzone
     * @return 0 if the abs(input) < abs(deadzone) otherwise return the input scaled.
     */
    double deadzone(const double &input);

public:

    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object.
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder &rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;
};

#endif
