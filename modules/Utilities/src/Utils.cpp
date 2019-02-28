/**
 * @file Utils.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// std
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>
#include <Utils.hpp>

iDynTree::Matrix3x3 iDynTreeHelper::Rotation::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output) = 0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

void iDynTreeHelper::Triplets::pushTriplets(const iDynTree::Triplets& input,
                                            iDynTree::Triplets& output)
{
    for(auto triplet: input)
        output.pushTriplet(triplet);

    return;
}

void iDynTreeHelper::Triplets::pushTripletsAsSubMatrix(const unsigned& startingRow,
                                                       const unsigned& startingColumn,
                                                       const iDynTree::Triplets& input,
                                                       iDynTree::Triplets& output)
{
    if(startingRow != 0 || startingColumn != 0)
    {
        iDynTree::Triplets inputShifted = input;

        for(auto triplet: inputShifted)
        {
            triplet.row += startingRow;
            triplet.column += startingColumn;
            output.pushTriplet(triplet);
        }
    }
    else
    {
        iDynTreeHelper::Triplets::pushTriplets(input, output);
    }
    return;
}

bool iDynTreeHelper::Triplets::getTripletsFromValues(const yarp::os::Value& input,
                                                     const int& matrixDimension,
                                                     iDynTree::Triplets& output)
{
    // clear the output
    output.clear();

    if (input.isNull())
    {
        yError() << "[getTripletsFromValues] Empty input values.";
        return false;
    }

    if (!input.isList() || !input.asList())
    {
        yError() << "[getSparseMatrixFromTriplets] Unable to read the input as a list.";
        return false;
    }

    yarp::os::Bottle *tripletsPtr = input.asList();

    // populate the triplets
    for (int i = 0; i < tripletsPtr->size(); ++i)
    {
        yarp::os::Bottle *tripletPtr = tripletsPtr->get(i).asList();

        if (tripletPtr->size() != 3)
        {
            yError() << "[getSparseMatrixFromTriplets] The triplet must have three elements.";
            return false;
        }

        int row = tripletPtr->get(0).asInt();
        int col = tripletPtr->get(1).asInt();

        if(col >= matrixDimension || row >= matrixDimension)
        {
            yError() << "[getSparseMatrixFromTriplets] element position exceeds the matrix dimension.";
            return false;
        }
        output.pushTriplet(iDynTree::Triplet(col, row, tripletPtr->get(2).asDouble()));
    }
    return true;
}

iDynSparseMatrix iDynTreeHelper::SparseMatrix::fromEigen(const Eigen::SparseMatrix<double>& eigenSparseMatrix)
{
    iDynTree::Triplets triplets;
    // populate the triplets
    for (int k=0; k < eigenSparseMatrix.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(eigenSparseMatrix, k); it; ++it)
            triplets.pushTriplet(iDynTree::Triplet(it.row(), it.col(), it.value()));

    // convert the triplets into a sparse matrix
    iDynSparseMatrix iDynTreeSparseMatrix(eigenSparseMatrix.rows(), eigenSparseMatrix.cols());
    iDynTreeSparseMatrix.setFromConstTriplets(triplets);

    return iDynTreeSparseMatrix;
}

bool YarpHelper::yarpListToiDynTreeVectorDynSize(const yarp::os::Value& input, iDynTree::VectorDynSize& output)
{
    if (input.isNull())
    {
        yError() << "[yarpListToiDynTreeVectorDynSize] Empty input value.";
        return false;
    }
    if (!input.isList() || !input.asList())
    {
        yError() << "[yarpListToiDynTreeVectorDynSize] Unable to read the input list.";
        return false;
    }
    yarp::os::Bottle *inputPtr = input.asList();

    if (inputPtr->size() != output.size())
    {
        yError() << "[yarpListToiDynTreeVectorDynSize] The size of the iDynTree vector and the size of "
                 << "the YARP list are not coherent.";
        return false;
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isDouble() && !inputPtr->get(i).isInt())
        {
            yError() << "[yarpListToiDynTreeVectorDynSize] The input is expected to be a double or a int";
            return false;
        }
        output(i) = inputPtr->get(i).asDouble();
    }
    return true;
}

bool YarpHelper::addVectorOfStringToProperty(yarp::os::Property& prop, const std::string& key,
                                             const std::vector<std::string>& list)
{
    // check if the key already exists
    if(prop.check(key))
    {
        yError() << "[addVectorOfStringToProperty] The property already exist.";
        return false;
    }

    prop.addGroup(key);
    yarp::os::Bottle& bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
        bot.addString(list[i].c_str());

    return true;
}

bool YarpHelper::yarpListToStringVector(yarp::os::Value*& input, std::vector<std::string>& output)
{
    // clear the std::vector
    output.clear();

    // check if the yarp value is a list
    if(!input->isList())
    {
        yError() << "[yarpListToStringVector] The input is not a list.";
        return false;
    }

    yarp::os::Bottle *bottle = input->asList();
    for(int i = 0; i < bottle->size(); i++)
    {
        // check if the elements of the bottle are strings
        if(!bottle->get(i).isString())
        {
            yError() << "[yarpListToStringVector] There is a field that is not a string.";
            return false;
        }
        output.push_back(bottle->get(i).asString());
    }
    return true;
}

bool YarpHelper::getStringFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         std::string& string)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getStringFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isString())
    {
        yError() << "[getStringFromSearchable] the value is not a string.";
        return false;
    }

    string = value->asString();
    return true;
}

bool YarpHelper::getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         double& number)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isDouble())
    {
        yError() << "[getNumberFromSearchable] the value is not a double.";
        return false;
    }

    number = value->asDouble();
    return true;
}

bool YarpHelper::getNumberFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                         int& number)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isInt())
    {
        yError() << "[getNumberFromSearchable] the value is not an integer.";
        return false;
    }

    number = value->asInt();
    return true;
}

bool YarpHelper::getYarpVectorFromSearchable(const yarp::os::Searchable& config, const std::string& key,
                                             yarp::sig::Vector& output)
{
    yarp::os::Value* value;
    if(!config.check(key, value))
    {
        yError() << "[getNumberFromSearchable] Missing field "<< key;
        return false;
    }

    if(!value->isList())
    {
        yError() << "[getNumberFromSearchable] the value is not a double.";
        return false;
    }

    yarp::os::Bottle *inputPtr = value->asList();

    if (inputPtr->size() != output.size())
    {
        yError() << "[getYarpVectorFromSearchable] The size of the YARP vector and the size of "
                 << "the YARP list are not coherent.";
        return false;
    }

    for (int i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isDouble() && !inputPtr->get(i).isInt())
        {
            yError() << "[getYarpVectorFromSearchable] The input is expected to be a double or a int";
            return false;
        }
        output(i) = inputPtr->get(i).asDouble();
    }

    return true;
}

void YarpHelper::populateBottleWithStrings(yarp::os::Bottle& bottle, const std::initializer_list<std::string>& strings)
{
    for(const auto& string : strings)
        bottle.addString(string);
}

double normalizeAnglePositive(const double& angle)
{
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double normalizeAngle(const double& angle)
{
    double a = normalizeAnglePositive(angle);
    if (a > M_PI)
        a -= 2.0 *M_PI;
    return a;
}

double iDynTreeHelper::shortestAngularDistance(const double& fromRad, const double& toRad)
{
    return normalizeAngle(toRad - fromRad);
}
