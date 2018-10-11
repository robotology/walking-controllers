/**
 * @file Utils.hpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKING_UTILS_HPP
#define WALKING_UTILS_HPP

// std
#include <vector>
#include <deque>

// YARP
#include <yarp/os/Property.h>
#include <yarp/os/Value.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

// iDynTree
#include <iDynTree/Core/Triplets.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/SparseMatrix.h>
#include <iDynTree/Core/Rotation.h>

// eigen
#include <Eigen/Sparse>

typedef iDynTree::SparseMatrix<iDynTree::ColumnMajor> iDynSparseMatrix;

/**
 * Helper for iDynTree library.
 */
namespace iDynTreeHelper
{
    /**
     * Triplets namespace add some useful function to manage triplets
     */
    namespace Triplets
    {
        /**
         * Merge two set of triplets output = [output, input]
         * @param input is the input set of triplets;
         * @param output is the output set of triplets.
         */
        void pushTriplets(const iDynTree::Triplets& input, iDynTree::Triplets& output);

        /**
         * Merge two set of triplets. The output triplets object represents a matrix while
         * the input triplets represent a sub matrix.
         * @param startingRow is the row position of the sub matrix;
         * @param startingColumn is the column position of the sub matrix;
         * @param input is the input set of triplets;
         * @param output is the output set of triplets.
         */
        void pushTripletsAsSubMatrix(const unsigned& startingRow, const unsigned& startingColumn,
                                     const iDynTree::Triplets& input,
                                     iDynTree::Triplets& output);

        /**
         * Get triplets from yarp value list. It is useful if you want to convert a yarp list of triplets
         * into a iDynTree Triplets object.
         * @todo extend to non squared matrix.
         * @param input the yarp value list;
         * @param matrixDimension the dimension of the matrix. It used to check if the triplets are valid
         * (i.e. the row and the column of each triplet has to be compatible with the matrix dimension);
         * @param output is the idyntree triplets object.
         * @return true/false in case of success/failure
         */
        bool getTripletsFromValues(const yarp::os::Value& input,
                                   const int& matrixDimension,
                                   iDynTree::Triplets& output);
    }

    /**
     * SparseMatrix namespace add some useful function to manage iDynTree sparse matrices
     */
    namespace SparseMatrix
    {
        /**
         * Convert an eigen sparse matrix into an iDynTree sparse matrix
         * @param eigenSparseMatrix is a column Major eigen sparse matrix.
         * @return an iDynTree column major sparse matrix.
         */
        iDynSparseMatrix fromEigen(const Eigen::SparseMatrix<double>&  eigenSparseMatrix);
    }

    namespace Rotation
    {
        /**
         * Transform a 3x3 matrix into a skew-symmetric matrix.
         * @param input is a 3x3 matrix;
         * @return a 3x3 skew-symmetric matrix
         */
        iDynTree::Matrix3x3 skewSymmetric(const iDynTree::Matrix3x3& input);
    }

    /**
     * Given 2 angles, it returns the shortest angular difference.
     * The result would always be -pi <= result <= pi.
     *
     * This function is taken from ROS angles [api](http://docs.ros.org/lunar/api/angles/html/namespaceangles.html#a4436fe67ae0c9df020f6779101bbefab).
     * @param fromRad is the starting angle expressed in radians;
     * @param toRad is the final angle expressed in radians.
     * @return the shortest angular distance
     */
    double shortestAngularDistance(const double& fromRad, const double& toRad);
}

/**
 * Helper for YARP library.
 */
namespace YarpHelper
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
    bool getDoubleFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                 double& number);

    /**
     * Extract a vector from a searchable object.
     * @param config is the searchable object;
     * @param key the name to check for;
     * @param vector is the vector.
     * @return true/false in case of success/failure
     */
    bool getYarpVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                     yarp::sig::Vector& vector);

    /**
     * Convert a yarp value into an iDynTree::VectorFixSize<n>
     * @param input yarp value;
     * @param output iDynTree::VectorFixSize<n>.
     * @return true/false in case of success/failure.
     */
    template <unsigned int n>
    bool yarpListToiDynTreeVectorFixSize(const yarp::os::Value& input, iDynTree::VectorFixSize<n>& output);

    /**
     * Convert a yarp value into an iDynTree::VectorDynSize
     * @param input yarp value;
     * @param output iDynTree::VectorDynSize if the size of this vector is different from the size of the
     * YARP list it will be resized.
     * @return true/false in case of success/failure.
     */
    bool yarpListToiDynTreeVectorDynSize(const yarp::os::Value& input, iDynTree::VectorDynSize& output);

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
}

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
#include "Utils.tpp"

#endif
