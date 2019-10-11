/**
 * @file Helper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

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
    namespace StdHelper
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
