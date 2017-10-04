/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 11, 2017
 *      Author: kuhnerd
 * 	  Filename: path_database.h
 */

#ifndef HDA0B2D49_31F2_4F3A_991F_B6D1A3D1E3A8
#define HDA0B2D49_31F2_4F3A_991F_B6D1A3D1E3A8
#include <ais_definitions/class.h>
#include <ais_point_cloud/kd_tree.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/trajectory.h>
#include <Eigen/Geometry>

namespace prm_planner
{

class PathDatabase
{
public:
	struct KdTreeNode
	{
		KdTreeNode()
		{
		}

		KdTreeNode(const KDL::JntArray& currentJointPose,
				const Eigen::Affine3d& goalTaskPose) :
						m_start(currentJointPose),
						m_goal(goalTaskPose)
		{
		}

		KdTreeNode(const boost::shared_ptr<Path> path) :
						m_start(path->front().jointPose),
						m_goal(path->back().pose),
						m_path(path)
		{
		}

		void setData(double* data) const
				{
			size_t i = 0;
			for (; i < m_start.rows(); ++i)
				data[i] = m_start(i);

			for (size_t j = 0; j < 6; ++i, ++j)
				data[i] = m_goal.x(j);
		}

		int getDim() const
		{
			return m_start.rows() + 6;
		}

		KDL::JntArray m_start;
		Trajectory::Pose m_goal;
		boost::shared_ptr<Path> m_path;
	};

public:
	PathDatabase();
	virtual ~PathDatabase();

	boost::shared_ptr<Path> findBestPlan(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose);

	bool addPath(boost::shared_ptr<Path> path);
	bool prune();

	static boost::shared_ptr<PathDatabase> open(const std::string& filename);
	bool write(const std::string& filename);

private:
	ais_point_cloud::KdTree<KdTreeNode> m_kdTree;

};

} /* namespace prm_planner */

#endif /* HDA0B2D49_31F2_4F3A_991F_B6D1A3D1E3A8 */
