/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 11, 2017
 *      Author: kuhnerd
 * 	  Filename: path_database.cpp
 */

#include <prm_planner/path_database/path_database.h>

namespace prm_planner
{

PathDatabase::PathDatabase()
{
}

PathDatabase::~PathDatabase()
{
}

boost::shared_ptr<Path> PathDatabase::findBestPlan(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose)
{
	KdTreeNode node(currentJointPose, goalTaskPose);
	std::vector<int> indices;
	std::vector<double> dists;
	if (m_kdTree.knnSearch(node, indices, dists, 1) == 0)
	{
		return boost::shared_ptr<Path>();
	}
	else
	{
		if (dists[0] > 0.1)
		{
//			LOG_INFO("Found path in database with distance (but don't use it because of distance)" << dists[0]);
			return boost::shared_ptr<Path>();
		}
		else
		{
//			LOG_INFO("Found path in database with distance " << dists[0]);
			boost::shared_ptr<Path> p(new Path(*m_kdTree.getDataPoint(indices[0]).m_path));
			p->setCachedPath(true);
			const size_t pathSize = p->size();

			//check if the new start pose can be connected to another then
			//the first waypoint to avoid some back-and-forth motions at the
			//beginning and end
			double min = std::numeric_limits<double>::max();
			double oldNorm = std::numeric_limits<double>::max();
			int startIndex = 0;
			for (size_t i = 0; i < pathSize; ++i)
			{
				double norm = (currentTaskPose.translation() - (*p)[i].pose.translation()).norm();
				if (i > 0 && oldNorm > norm)
					break;

				if (norm < min)
				{
					norm = min;
					startIndex = 0;
				}

				oldNorm = norm;
			}

			min = std::numeric_limits<double>::max();
			oldNorm = std::numeric_limits<double>::max();
			int goalIndex = pathSize - 1;
			for (size_t i = pathSize - 1; i >= 0; --i)
			{
				double norm = (goalTaskPose.translation() - (*p)[i].pose.translation()).norm();
				if (oldNorm > norm)
					break;

				if (i < pathSize - 1 && norm < min)
				{
					norm = min;
					goalIndex = 0;
				}

				oldNorm = norm;
			}

			//get sub path
			if (startIndex > 0 || goalIndex < pathSize - 1)
			{
				LOG_INFO("Using subpath to avoid ugly motion at the beginning or goal: start at "
						<< startIndex << ", goal at " <<goalIndex << ", path size: " << pathSize);
				p = p->getSubPath(startIndex, goalIndex);
				p->setCachedPath(true);
			}

			//add start and goal node
			p->prepend(Path::Waypoint(p->getMaxId(), currentTaskPose, currentJointPose), false);
			p->append(Path::Waypoint(p->getMaxId(), goalTaskPose), true);

			return p;
		}
	}
}

boost::shared_ptr<PathDatabase> PathDatabase::open(const std::string& filename)
{
	return boost::shared_ptr<PathDatabase>(new PathDatabase);
}

bool PathDatabase::addPath(boost::shared_ptr<Path> path)
{
	KdTreeNode node(path);
	m_kdTree.addPoint(node);
	LOG_INFO("added path");
	return true;
}

bool PathDatabase::prune()
{
	return true;
}

bool PathDatabase::write(const std::string& filename)
{
	return true;
}

} /* namespace prm_planner */
