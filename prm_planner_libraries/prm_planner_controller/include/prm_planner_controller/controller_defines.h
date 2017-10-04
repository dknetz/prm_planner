/*
 * controller_defines.h
 *
 *  Created on: Aug 22, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_CONTROLLER_DEFINES_H_
#define PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_CONTROLLER_DEFINES_H_
#include <ais_log/log.h>

namespace prm_planner
{

struct ControllerBubble
{
	Eigen::Vector3d position;
	Eigen::Vector3d obstacleToRobot;
	double obstacleDistance;
	double radius;
	int segment;
	std::vector<Eigen::Vector3d> octants;
};

struct ControllerParameters
{
	std::string type;
	int frequency;
	double maxVelocity;
	std::vector<double> k;
	double lambda;
	double leastSquareDampingLambdaMax;
	double leastSquareDampingEpsilon;
	bool debug;
	double thresholdGoalReachedPos;
	double thresholdGoalReachedAng;
	double maxTaskVelPos;
	double maxTaskVelAng;

	bool optimizeJointRangeInNullspace;
	double jointRangeNullspaceWeightStd;
	double jointRangeNullspaceWeightDefault;

	//collision avoidance not used currently
	bool collisionAvoidanceUse;
	double collisionAvoidanceRatioDoCollisionChecks; //don't do x% of collision checks
	double collisionAvoidanceVel0;
	double collisionAvoidanceDistance1;
	double collisionAvoidanceDistance2;
	double collisionDetectionStopDistance;
	double collisionAvoidanceMaxJointVel;
};

/**
 * Computes a damped least square pseudo inverse
 * of the robots Jacobian. It is based on the idea
 * of "Review of the Damped Least-Squares Inverse
 * Kinematics with Experiments on an Industrial
 * Robot Manipulator".
 *
 * By using this method the robot tries to avoid
 * singularities. The smallest singular value is
 * used to determine situations where a singularity
 * will happen in near future. In such a case the
 * corresponding direction will be damped to avoid
 * the singularity. If the robot is already in such
 * a singularity it is not possible to get a smooth
 * movement without large velocities by using this
 * method.
 *
 * @paper "Review of the Damped Least-Squares Inverse
 * 		Kinematics with Experiments on an Industrial
 * 		Robot Manipulator"
 */
template<int ROWS, int COLS>
inline void computeDampedLeastSquarePseudoInverse(const Eigen::Matrix<double, ROWS, COLS> jacobian,
		Eigen::Matrix<double, COLS, ROWS>& pinv,
		const ControllerParameters& parameters)
{
	double sigma;
	double lambdaSquared;
	double smallestSingularValue;
	double x;
	Eigen::Matrix<double, ROWS, 1> ui;

	Eigen::JacobiSVD<Eigen::Matrix<double, ROWS, COLS>> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);

	const Eigen::Matrix<double, ROWS, 1>& singularValues = svd.singularValues(); //6x1
	const Eigen::Matrix<double, ROWS, ROWS>& u = svd.matrixU(); //6x6
	const Eigen::Matrix<double, COLS, COLS>& v = svd.matrixV(); //7x7

	pinv.fill(0);

	//compute damping
	smallestSingularValue = singularValues(singularValues.rows() - 1, 0);
	if (smallestSingularValue >= parameters.leastSquareDampingEpsilon)
	{
		lambdaSquared = 0;
	}
	else
	{
		x = smallestSingularValue / parameters.leastSquareDampingEpsilon;
		lambdaSquared = (1.0 - x * x) * parameters.leastSquareDampingLambdaMax * parameters.leastSquareDampingLambdaMax;
	}

	for (int i = 0; i < singularValues.size(); ++i) //nonzeroSingularValues = rank
	{
		sigma = singularValues(i, 0);
		if (sigma > 0)
		{
			ui = u.col(i);
			pinv += (sigma / (sigma * sigma + lambdaSquared)) * v.col(i) * ui.transpose();
		}
	}
}
}

#endif /* PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_CONTROLLER_DEFINES_H_ */
