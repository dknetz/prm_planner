/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 18, 2017
 *      Author: kuhnerd
 * 	  Filename: database_cached_planner.cpp
 */

#include <octomap/OcTree.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/planners/database_cache/database_cached_planner.h>
#include <prm_planner/planners/database_cache/path_database.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner_robot/path.h>

namespace prm_planner
{

DatabaseCachedPlanner::DatabaseCachedPlanner(boost::shared_ptr<ProblemDefinition> pd) :
				PathPlanner(pd),
				c_config(pd->getConfig().databaseCacheConfig)
{
	m_database = PathDatabase::read(c_config.filename);
	std::vector<ais_point_cloud::SearchWrapperEigenVectorXWithId> points;
	m_database->getKdTreeData(points);
	m_kdtree.setDataset(points);
}

DatabaseCachedPlanner::~DatabaseCachedPlanner()
{
}

bool DatabaseCachedPlanner::plan(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path,
		bool directConnectionRequired)
{
	PathPlanner::plan(currentJointPose, currentTaskPose, goalTaskPose, cd, path, directConnectionRequired);

	Eigen::VectorXd query(6);
	PathDatabase::convertToSearchPoint(currentTaskPose, goalTaskPose, query);

	std::vector<int> indices;
	std::vector<double> dists;
	m_kdtree.knnSearch(ais_point_cloud::SearchWrapperEigenVectorXWithId(query, 0), indices, dists, 5);

	//brute force

	LOG_INFO("Found " << dists.size() << " nearest neighbors");

	for (auto& it: indices) {
		boost::shared_ptr<Path> p = m_database->getPath(it);
		LOG_INFO("Dist start:" << (currentTaskPose.translation() - p->front().getTaskPosition()).norm());
		LOG_INFO("Dist goal:" << (goalTaskPose.translation() - p->back().getTaskPosition()).norm());
	}

	return false;
}

bool DatabaseCachedPlanner::planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const int startNodeId,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	PathPlanner::planSingleStartMultipleGoal(currentJointPose, currentTaskPose, startNodeId, cd, path);
	return false;
}

void DatabaseCachedPlanner::publish()
{
	PLANNER_READ_LOCK();

	if (m_database.get() != NULL)
	{
		m_database->publish();
	}
}

}
/* namespace prm_planner */
