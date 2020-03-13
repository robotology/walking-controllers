/**
 * @file LoggerClient.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <WalkingControllers/YarpUtilities/Helper.h>

template <typename... Args>
void WalkingControllers::LoggerClient::sendData(const Args&... args)
{
    YarpUtilities::sendVariadicVector(m_dataPort, args...);
}
