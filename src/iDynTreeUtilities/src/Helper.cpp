/**
 * @file Helper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2019 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2019
 */

// std
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>

// iDynTree
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Utils.h>

#include <WalkingControllers/iDynTreeUtilities/Helper.h>

using namespace WalkingControllers;

iDynTree::Matrix3x3 iDynTreeUtilities::Rotation::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output) = 0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

void iDynTreeUtilities::Triplets::pushTriplets(const iDynTree::Triplets& input,
                                            iDynTree::Triplets& output)
{
    for(auto triplet: input)
        output.pushTriplet(triplet);

    return;
}

void iDynTreeUtilities::Triplets::pushTripletsAsSubMatrix(const unsigned& startingRow,
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
        iDynTreeUtilities::Triplets::pushTriplets(input, output);
    }
    return;
}

bool iDynTreeUtilities::Triplets::getTripletsFromValues(const yarp::os::Value& input,
                                                     const int& matrixDimension,
                                                     iDynTree::Triplets& output)
{
    // clear the output
    output.clear();

    if (input.isNull())
    {
        iDynTree::reportError("iDynTreeUtilities", "getTripletsFromValues", "Empty input values");
        return false;
    }

    if (!input.isList() || !input.asList())
    {
        iDynTree::reportError("iDynTreeUtilities", "getTripletsFromValues", "Unable to read the input as a list");
        return false;
    }

    yarp::os::Bottle *tripletsPtr = input.asList();

    // populate the triplets
    for (int i = 0; i < tripletsPtr->size(); ++i)
    {
        yarp::os::Bottle *tripletPtr = tripletsPtr->get(i).asList();

        if (tripletPtr->size() != 3)
        {
            iDynTree::reportError("iDynTreeUtilities", "getTripletsFromValues", "The triplet must have three elements");
            return false;
        }

        int row = tripletPtr->get(0).asInt32();
        int col = tripletPtr->get(1).asInt32();

        if(col >= matrixDimension || row >= matrixDimension)
        {
            iDynTree::reportError("iDynTreeUtilities", "getTripletsFromValues", "Element position exceeds the matrix dimension");
            return false;
        }
        output.pushTriplet(iDynTree::Triplet(col, row, tripletPtr->get(2).asFloat64()));
    }
    return true;
}

iDynSparseMatrix iDynTreeUtilities::SparseMatrix::fromEigen(const Eigen::SparseMatrix<double>& eigenSparseMatrix)
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


double normalizeAnglePositive(const double& angle)
{
    return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double normalizeAngle(const double& angle)
{
    double a = normalizeAnglePositive(angle);
    if (a > M_PI)
        a -= 2.0 * M_PI;
    return a;
}

double iDynTreeUtilities::shortestAngularDistance(const double& fromRad, const double& toRad)
{
    return normalizeAngle(toRad - fromRad);
}
