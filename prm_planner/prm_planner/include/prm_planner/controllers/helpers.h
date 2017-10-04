/*
 * helpers.h
 *
 *  Created on: Aug 19, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONTROLLERS_HELPERS_H_
#define PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONTROLLERS_HELPERS_H_

#include <kdl/jntarray.hpp>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_controller/controller_defines.h>
#include <vector>

namespace prm_planner
{
inline ControllerParameters getControllerParametersFromParameterServer(const std::string& controllerName)
{
	ControllerParameters params;

	parameters::ControllerConfig config = ParameterServer::controllerConfigs[controllerName];

	params.type = config.controllerType;
	params.debug = config.debug;
	params.frequency = config.frequency;
	params.k = config.k;
	params.lambda = config.lambda;
	params.leastSquareDampingEpsilon = config.leastSquareEpsilon;
	params.leastSquareDampingLambdaMax = config.leastSquareLambdaMax;
	params.maxTaskVelAng = config.maxTaskVelAng;
	params.maxTaskVelPos = config.maxTaskVelPos;
	params.maxVelocity = config.maxVelocity;
	params.thresholdGoalReachedAng = config.goalReachedDistAng;
	params.thresholdGoalReachedPos = config.goalReachedDistPos;

	//joint range optimization in nullspace
	params.optimizeJointRangeInNullspace = false;
	params.jointRangeNullspaceWeightDefault = config.jointRangeWeightDefault;
	params.jointRangeNullspaceWeightStd = config.jointRangeWeightStd;

	//collision avoidance
	params.collisionAvoidanceDistance1 = config.collisionAvoidanceDistance1;
	params.collisionAvoidanceDistance2 = config.collisionAvoidanceDistance2;
	params.collisionAvoidanceMaxJointVel = config.collisionAvoidanceMaxJointVel;
	params.collisionAvoidanceUse = config.collisionAvoidanceUse;
	params.collisionAvoidanceRatioDoCollisionChecks = config.collisionAvoidanceRatioDoCollisionChecks;
	params.collisionAvoidanceVel0 = config.collisionAvoidanceVel0;
	params.collisionDetectionStopDistance = 0.07;

	return params;
}

inline void convert(const KDL::JntArray& joints,
		std::vector<double>& jointsOut)
{
	jointsOut.resize(joints.rows());
	for (size_t i = 0; i < joints.rows(); ++i)
		jointsOut[i] = joints(i);
}

}

#endif /* PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_CONTROLLERS_HELPERS_H_ */
