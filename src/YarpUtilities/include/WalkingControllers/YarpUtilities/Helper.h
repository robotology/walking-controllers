// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_YARP_HELPER_H
#define WALKING_CONTROLLERS_YARP_HELPER_H

// std
#include <vector>
#include <deque>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

namespace WalkingControllers
{

/**
 * Helper for YARP library.
 */
    namespace YarpUtilities
    {
        /**
         * Add a vector of string to a property of a given name.
         * @param prop yarp property;
         * @param key is the key;
         * @param list is the vector of strings that will be added into the property.
         * @return true/false in case of success/failure
         */
        bool addVectorOfStringToProperty(yarp::os::Property& prop, const std::string& key,
                                         const std::vector<std::string>& list);

        /**
         * Convert a yarp list into a vector of string
         * @param input is the pointer of a yarp value;
         * @param output is the vector of strings.
         * @return true/false in case of success/failure
         */
        bool yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output);

        /**
         * Convert a yarp list into a vector of bool
         * @param input is the pointer of a yarp value;
         * @param output is the vector of bool.
         * @return true/false in case of success/failure
         */
        bool yarpListToBoolVector(yarp::os::Value*& input, std::vector<bool>& output);

        /**
         * Extract a string from a searchable object.
         * @param config is the searchable object;
         * @param key the name to check for;
         * @param string is the string.
         * @return true/false in case of success/failure
         */
        bool getStringFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                     std::string& string);

        /**
         * Extract a double from a searchable object.
         * @param config is the searchable object;
         * @param key the name to check for;
         * @param number is the double.
         * @return true/false in case of success/failure
         */
        bool getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                     double& number);


        /**
         * Extract a double from a searchable object.
         * @param config is the searchable object;
         * @param key the name to check for;
         * @param number is the integer.
         * @return true/false in case of success/failure
         */
        bool getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                     int& number);


        /**
         * Convert a yarp value into a vector
         * @param input yarp value;
         * @param output vector
         * @return true/false in case of success/failure.
         */
        template <typename T>
            bool yarpListToVector(const yarp::os::Value& input, T& output);

        /**
         * Extract a vector from searchable
         * @param config is the searchable object;
         * @param key the name to check for;
         * @param vector a vector.
         * @return true/false in case of success/failure
         */
        template <typename T>
            bool getVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         T& vector);

        /**
         * Merge two vectors. vector = [vector, t]
         * @param vector the original vector. The new elements will be add at the end of this vector;
         * @param t vector containing the elements that will be merged with the original vector.
         */
        template <typename T>
            void mergeSigVector(yarp::sig::Vector& vector, const T& t);

        /**
         * Variadic fuction used to merge several vectors.
         * @param vector the original vector. The new elements will be add at the end of this vector;
         * @param t vector containing the elements that will be merged with the original vector.
         * @param args list containing all the vector that will be merged.
         */
        template <typename T, typename... Args>
            void mergeSigVector(yarp::sig::Vector& vector, const T& t, const Args&... args);

        /**
         * Send a variadic vector through a yarp buffered port
         * @param port is a Yarp buffered port
         * @param args list containing all the vector that will be send.
         */
        template <typename... Args>
            void sendVariadicVector(yarp::os::BufferedPort<yarp::sig::Vector>& port, const Args&... args);

        /**
         * Add strings to a bottle.
         * @param bottle this bottle will be filled.
         * @param strings list containing all the string.
         */
        void populateBottleWithStrings(yarp::os::Bottle& bottle, const std::initializer_list<std::string>& strings);

        /**
         * Get Vector of boolean from searchable.
         * @param config is the searchable object;
         * @param key the name to check for;
         * @param output a std vector.
         * @return true/false in case of success/failure.
         */
        bool getVectorOfBooleanFromSearchable(const yarp::os::Searchable& config, const std::string& key, std::vector<bool> & output);
    }
};

#include "Helper.tpp"

#endif
