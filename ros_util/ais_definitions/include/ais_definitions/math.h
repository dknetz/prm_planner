/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: math.h
 */

#ifndef H279B07D2_6971_4E6E_8EA1_2B570F4C6E77
#define H279B07D2_6971_4E6E_8EA1_2B570F4C6E77
#include <Eigen/Core>

//#define DEFINE_CONTROLLER_MATRICES(DIM) \
//	typedef Eigen::Matrix<double, DIM, 1> Vector##DIM##d;\
//	typedef Eigen::Matrix<double, DIM, 6> Matrix##DIM##x6;\
//	typedef Eigen::Matrix<double, 6, DIM> Matrix6x##DIM;\
//	typedef Eigen::Matrix<double, DIM, DIM> Matrix##DIM##x##DIM;\
//	typedef Eigen::Matrix<double, 3, DIM> Matrix3x##DIM;\
//	typedef Eigen::Matrix<double, DIM, 3> Matrix##DIM##x3;
//
//DEFINE_CONTROLLER_MATRICES(7);
//DEFINE_CONTROLLER_MATRICES(10);

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 5, 1> Vector5d;
//typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VectorNd;
//typedef Eigen::Matrix<double, Eigen::Dynamic, 6> MatrixNx6;
//typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixNx3;
//typedef Eigen::Matrix<double, Eigen::Dynamic, 5> MatrixNx5;
//typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3xN;
//typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6xN;
typedef Eigen::Matrix<double, 6, 6> Matrix6x6;
typedef Eigen::Matrix<double, 5, 5> Matrix5x5;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3;

#endif /* H279B07D2_6971_4E6E_8EA1_2B570F4C6E77 */
