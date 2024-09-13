// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_STD_HELPER_H
#define WALKING_CONTROLLERS_STD_HELPER_H

// std
#include <vector>
#include <deque>

namespace WalkingControllers
{

/**
 * Helper for std library
 */
    namespace StdUtilities
    {
        /**
         * Allow you to append vector to a deque.
         * @param input input vector;
         * @param output output deque;
         * @param initPoint point where the vector will be append to the deque
         */
        template<typename T>
        bool appendVectorToDeque(const std::vector<T>& input, std::deque<T>& output, const size_t& initPoint);
    }
}
#include "Helper.tpp"

#endif
