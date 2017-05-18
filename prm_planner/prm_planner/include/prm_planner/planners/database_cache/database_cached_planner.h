/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 18, 2017
 *      Author: kuhnerd
 * 	  Filename: database_cached_planner.h
 */

#ifndef HB9947FF7_7127_4704_8336_6E52A3BA5E1A
#define HB9947FF7_7127_4704_8336_6E52A3BA5E1A

#include <prm_planner/planners/path_planner.h>
#include <prm_planner/util/parameters.h>
#include <ais_point_cloud/kd_tree.h>

namespace prm_planner
{

FORWARD_DECLARE(PathDatabase);
FORWARD_DECLARE(CollisionDetector);

class DatabaseCachedPlanner: public PathPlanner
{
public:
	DatabaseCachedPlanner(boost::shared_ptr<ProblemDefinition> pd);
	virtual ~DatabaseCachedPlanner();

	/**
	 * @see PathPlanner
	 */
	virtual bool plan(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path,
			bool directConnectionRequired = false);

	/**
	 * @see PathPlanner
	 */
	virtual bool planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const int startNodeId,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

	/**
	 * @see PathPlanner
	 */
	virtual void publish();

private:
	//the database
	boost::shared_ptr<PathDatabase> m_database;

public:
	//config
	const parameters::DatabaseCacheConfig c_config;

	//kdtree to search for nearest neighbor paths
	ais_point_cloud::KdTree<ais_point_cloud::SearchWrapperEigenVectorXWithId> m_kdtree;
};

} /* namespace prm_planner */

#endif /* HB9947FF7_7127_4704_8336_6E52A3BA5E1A */
