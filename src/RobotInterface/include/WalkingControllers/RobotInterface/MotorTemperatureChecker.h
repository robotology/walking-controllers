// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_MOTOR_TEMPERATURE_CHECKER
#define WALKING_CONTROLLERS_MOTOR_TEMPERATURE_CHECKER

// std
#include <vector>

#include <iDynTree/VectorDynSize.h>

#include <yarp/os/Searchable.h>

namespace WalkingControllers
{
    class MotorsTemperatureChecker
    {
    private:
        std::vector<double> m_maxTemperature;
        std::vector<unsigned int> m_samplesAboveTheLimit;
        iDynTree::VectorDynSize m_limits;
        int m_maxNumberOfSampleAboveTheLimits;

    public:

        const std::vector<double>& getMaxTemperature() const;
        std::vector<unsigned int> getMotorsOverLimit() const;
        bool isThereAMotorOverLimit() const;
        bool setMotorTemperatures(const iDynTree::VectorDynSize& temperature);
        bool configure(const yarp::os::Searchable& config, int dofs);
    };
};
#endif
