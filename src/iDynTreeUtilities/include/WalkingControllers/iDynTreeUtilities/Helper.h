// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef WALKING_CONTROLLERS_IDYNTREE_HELPER_H
#define WALKING_CONTROLLERS_IDYNTREE_HELPER_H

// std
#include <vector>
#include <deque>

// iDynTree
#include <iDynTree/Triplets.h>
#include <iDynTree/VectorFixSize.h>
#include <iDynTree/SparseMatrix.h>
#include <iDynTree/Rotation.h>

// YARP
#include <yarp/os/Value.h>

// eigen
#include <Eigen/Sparse>

namespace WalkingControllers
{

    typedef iDynTree::SparseMatrix<iDynTree::ColumnMajor> iDynSparseMatrix;

    /**
     * Helper for iDynTree library.
     */
    namespace iDynTreeUtilities
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
    };
};

#endif
