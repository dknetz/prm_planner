/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 4, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: rrt.h
 */

#ifndef PRM_PLANNER_RRT_H_
#define PRM_PLANNER_RRT_H_

#include <prm_planner/planners/path_planner.h>
#include <prm_planner_robot/defines.h>

namespace prm_planner
{

FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(Path);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(RRTNode);
FORWARD_DECLARE(CollisionDetector);

class RRT: public PathPlanner
{
public:
	RRT(boost::shared_ptr<ProblemDefinition> pd,
			const double maxPlanningTime);
	virtual ~RRT();

	virtual bool plan(const KDL::JntArray& currentJointPoses,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path,
			bool directConnectionRequired = false);

	virtual bool planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const int startNodeId,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

private:
	RRTNode* findNearestNeighbor(const Eigen::Affine3d& pose,
			std::list<RRTNode*>& nodes);
	RRTNode* createNode(RRTNode* parent,
			const Eigen::Affine3d& pose,
			boost::shared_ptr<CollisionDetector>& cd);
	RRTNode* tryToReachGoal(const Eigen::Affine3d& goal,
			RRTNode* currentNode,
			boost::shared_ptr<CollisionDetector>& cd);
	boost::shared_ptr<Path> extractPath(RRTNode* lastRRTNode);

private:
	boost::shared_ptr<Constraint> m_constraint;
	const double m_maxPlanningTime;
	const double m_sampleGoal;
	const double m_maxExpansionDistance;
};

} /* namespace prm_planner */

#endif /* PRM_PLANNER_RRT_H_ */
