/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: constraint.h
 */

#ifndef CONSTRAINT_H_
#define CONSTRAINT_H_
#include <ais_log/log.h>
#include <prm_planner_robot/trajectory.h>
#include <Eigen/Core>

namespace prm_planner
{

class Constraint
{
public:
	Constraint();
	virtual ~Constraint();

	virtual void computeError(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			Vector6d& result) = 0;
	virtual bool isGoalReached(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			const double thresholdGoalReachedPos,
			const double thresholdGoalReachedAng) const = 0;
	virtual void computeDifference(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			Vector6d& result);

	virtual void samplePose(Eigen::Affine3d& pose);

	virtual bool findValidPose(Eigen::Affine3d& pose);
	virtual void findValidPosition(Eigen::Affine3d& pose);
	virtual void findValidOrientation(Eigen::Affine3d& pose);

	virtual void findNearestValidPose(Eigen::Affine3d& pose);
	virtual void findNearestValidPosition(Eigen::Affine3d& pose);
	virtual void findNearestValidOrientation(Eigen::Affine3d& pose);

	virtual bool isValidPose(const Eigen::Affine3d& pose);
	virtual bool isValidPosition(const Eigen::Affine3d& pose);
	virtual bool isValidOrientation(const Eigen::Affine3d& pose);

	template<class VectorType>
	static void getDiff(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			VectorType& diff)
	{
		getLinearDiff(goal, start, diff);
		getAngularDiff(goal, start, diff);
	}

	template<class VectorType>
	static void getAngularDiff(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			VectorType& diff)
	{
		assert(diff.size() == 6);

		Eigen::Quaterniond diffQ = goal.quaternion * start.quaternion.inverse();
		KDL::Rotation rpy = KDL::Rotation::Quaternion(diffQ.x(), diffQ.y(), diffQ.z(), diffQ.w());
		rpy.GetRPY(diff(3, 0), diff(4, 0), diff(5, 0));
//		LOG_INFO(goal.quaternion.angularDistance(start.quaternion));

//		KDL::Rotation r1 = KDL::Rotation::RPY(p1(3, 0), p1(4, 0), p1(5, 0));
//		KDL::Rotation r2 = KDL::Rotation::RPY(p2(3, 0), p2(4, 0), p2(5, 0));
//
//		KDL::Vector angError = (r1.UnitX() * r2.UnitX() +
//				r1.UnitY() * r2.UnitY() +
//				r1.UnitZ() * r2.UnitZ());
//
//		diff(3, 0) = angError.x();
//		diff(4, 0) = angError.y();
//		diff(5, 0) = angError.z();
	}

	template<class VectorType>
	static void getLinearDiff(const Trajectory::Pose& goal,
			const Trajectory::Pose& start,
			VectorType& diff)
	{
		assert(diff.size() >= 3);

		diff(0, 0) = goal.x(0, 0) - start.x(0, 0);
		diff(1, 0) = goal.x(1, 0) - start.x(1, 0);
		diff(2, 0) = goal.x(2, 0) - start.x(2, 0);
	}
};

} /* namespace prm_planner */

#endif /* CONSTRAINT_H_ */
