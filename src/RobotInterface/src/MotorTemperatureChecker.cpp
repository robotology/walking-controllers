// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <yarp/os/LogStream.h>

#include <WalkingControllers/RobotInterface/MotorTemperatureChecker.h>
#include <WalkingControllers/YarpUtilities/Helper.h>

using namespace WalkingControllers;

bool MotorsTemperatureChecker::configure(const yarp::os::Searchable& config, int dofs)
{
    m_limits.resize(dofs);
    if(!YarpUtilities::getVectorFromSearchable(config, "temperature_limits", m_limits))
    {
        yError() << "[MotorsTemperatureChecker::configure] Unable to get the parameter temperature limits";
        return false;
    }

    m_samplesAboveTheLimit = std::vector<unsigned int>(dofs, 0);

    if(!YarpUtilities::getNumberFromSearchable(config, "max_samples_above_the_limits", m_maxNumberOfSampleAboveTheLimits))
    {
        yError() << "[MotorsTemperatureChecker::configure] Unable to get the parameter 'max_samples_above_the_limits'";
        return false;
    }

    m_maxTemperature.resize(dofs);
    for (int i = 0; i < dofs; i++)
    {
        m_maxTemperature[i] = 0;
    }

    return true;
}

bool MotorsTemperatureChecker::isThereAMotorOverLimit() const
{
    if (m_maxNumberOfSampleAboveTheLimits < 0)
    {
        return false;
    }

    for (const auto sample : m_samplesAboveTheLimit)
    {
        if (sample > m_maxNumberOfSampleAboveTheLimits)
        {
            return true;
        }
    }
    return false;
}

std::vector<unsigned int> MotorsTemperatureChecker::getMotorsOverLimit() const
{
    std::vector<unsigned int>  indeces;

    if (m_maxNumberOfSampleAboveTheLimits < 0)
    {
        return indeces;
    }

    for (int i =0; i < m_samplesAboveTheLimit.size(); i++)
    {
        if (m_samplesAboveTheLimit[i] > m_maxNumberOfSampleAboveTheLimits)
        {
            indeces.push_back(i);
        }
    }
    return indeces;
}

bool MotorsTemperatureChecker::setMotorTemperatures(const iDynTree::VectorDynSize& temperature)
{
    if (temperature.size() != m_limits.size())
    {
        yError() << "[MotorsTemperatureChecker::setMotorTemperatures] Unexpected size of the temperature vector"
                 << "provided: " << temperature.size() << " expected: " << m_limits.size();
        return false;
    }

    if (m_maxNumberOfSampleAboveTheLimits < 0)
    {
        return true;
    }

    for (int i = 0; i < temperature.size(); i++)
    {
      
        if (m_limits(i) < 0)
	{
	    continue;
	}

	if (temperature(i) > m_limits(i))
        {
            m_samplesAboveTheLimit[i]++;
        }
        else
        {
            m_samplesAboveTheLimit[i] = 0;
        }

        if (temperature(i) > m_maxTemperature[i])
        {
            m_maxTemperature[i] = temperature(i);
        }
    }

    return true;
}

const std::vector<double>& MotorsTemperatureChecker::getMaxTemperature() const
{
    return m_maxTemperature;
}
