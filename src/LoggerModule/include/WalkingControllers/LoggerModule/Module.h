/**
 * @file LoggerModule.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_LOGGER_MODULE_HPP
#define WALKING_LOGGER_MODULE_HPP

// std
#include <fstream>

// YARP
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>

namespace WalkingControllers
{

    /**
     * RFModule useful to collect data during an experiment.
     */
    class WalkingLoggerModule : public yarp::os::RFModule
    {
        double m_dT; /**< RFModule period. */
        std::ofstream m_stream; /**< std stream. */

        int m_numberOfValues; /**< Number of columns of the dataset. */
        double m_time0; /**< Initial time of a stream. */

        yarp::os::BufferedPort<yarp::sig::Vector> m_dataPort; /**< Data port. */
        yarp::os::RpcServer m_rpcPort; /**< RPC port. */

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
         * Respond to a message from the RPC port.
         * @param command is the received message.
         * The following message has to be a bottle with the following structure:
         * 1. ("record", <list of the names of the saved variables>);
         * 2. ("quit").
         * @param reply is the response of the server.
         * 1. 1 in case of success;
         * 2. 0 in case of failure.
         * @return true in case of success and false otherwise.
         */
        bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

        /**
         * Close the RFModule.
         * @return true in case of success and false otherwise.
         */
        bool close() override;
    };
};

#endif
