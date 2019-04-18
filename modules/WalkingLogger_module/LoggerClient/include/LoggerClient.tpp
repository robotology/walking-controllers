/**
 * @file LoggerClient.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <Utils.hpp>

template <typename... Args>
void LoggerClient::sendData(const Args&... args)
{
    YarpHelper::sendVariadicVector(m_dataPort, args...);
}
