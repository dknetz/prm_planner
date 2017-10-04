/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm.h
 */

#ifndef PRM_H_
#define PRM_H_

#include <ais_definitions/class.h>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/atomic.hpp>
#include <prm_planner_robot/defines.h>
#include <prm_planner/planners/prm/prm_defines.h>
#include <prm_planner/util/parameters.h>
#include <Eigen/Core>

FORWARD_DECLARE_N(octomap, OcTree)

namespace prm_planner
{
FORWARD_DECLARE(PRMNode);
FORWARD_DECLARE(PRMEdge);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(PRMView);

class PRM
{
public:
	/**
	 * Creates a probabilistic roadmap based on the number
	 * of nodes and the maximum visibility distance between
	 * two nodes.
	 * @param robot [in]
	 * @param config [in]: config
	 * @param pd [in]: the problem definition
	 * @loadFromFile [in]: Load it from file, if available
	 */
	PRM(const boost::shared_ptr<Robot>& robot,
			const parameters::PRMConfig& config,
			boost::shared_ptr<ProblemDefinition> pd,
			bool loadedFromFile = false);

	/**
	 * Creates a probabilistic roadmap based on a set of
	 * nodes and edges.
	 * @param robot [in]
	 * @param edgeVisibilityMaxRange [in]: max distance between
	 * 		two nodes to get connected
	 * @param pd [in]: the problem definition
	 * @nodes [in]: the nodes
	 * @edges [in]: the edges
	 */
	PRM(const boost::shared_ptr<Robot>& robot,
			const double edgeVisibilityMaxRange,
			boost::shared_ptr<ProblemDefinition> pd,
			PRMNodeMap nodes,
			PRMEdgeMap edges);

	virtual ~PRM();

	/**
	 * Updates the octomap and kdTree
	 */
	void update(const boost::shared_ptr<octomap::OcTree> octree);

	/**
	 * Used for visualization with ROS messages
	 */
	void publish();

	PRMView* getShallowDataCopy();

	void setConstraint(boost::shared_ptr<Constraint> constraint);

	static PRM* load(const parameters::PRMConfig& config,
			const boost::shared_ptr<Robot>& robot,
			const boost::shared_ptr<ProblemDefinition>& pd);
	void save(const std::string& filename);
	void computeCenterAndRadius(double& radius,
			Eigen::Vector3d& center) const;
	int nodeSize() const;
	int edgeSize() const;
	boost::shared_ptr<Robot> getRobot() const;

	/**
	 * Returns the start nodes, if there are
	 * nodes, which were tagged as a start node.
	 * Otherwise, the returned map is empty.
	 */
	const PRMNodeMap& getStartNodes() const;

	/**
	 * Returns the goal nodes, if there are
	 * nodes, which were tagged as a goal node.
	 * Otherwise, the returned map is empty.
	 */
	const PRMNodeMap& getGoalNodes() const;

	/**
	 * Sets new edges and nodes. Deletes the old edges
	 * and nodes.
	 */
	void resetNodesAndEdges(PRMNodeMap nodes,
			PRMEdgeMap edges);

private:
	void sampleNodes();
	void connectNodes();
	void cleanUpPRM();
	bool isVisible(const Eigen::Vector3d& pos1,
			const Eigen::Vector3d& pos2) const;
	bool isVisible(const Eigen::Vector3d& pos1,
			const Eigen::Vector3d& pos2,
			const double maxRange) const;
	void init();
	void updateVisibilities();

private:
	boost::shared_ptr<Robot> m_robot;
	PRMNodeMap m_nodes;
	PRMNodeMap m_startNodes;
	PRMNodeMap m_goalNodes;
	PRMEdgeMap m_edges;
	ros::Publisher m_pubMarker;
	bool m_initialized;
	mutable boost::shared_mutex m_mutex;
	boost::shared_ptr<octomap::OcTree> m_octomap;

	boost::shared_ptr<ProblemDefinition> m_pd;

	parameters::PRMConfig c_config;

	friend class PathPlanner;
	friend class PRMView;

public:
	boost::atomic_int m_nextIdNode, m_nextIdEdge;
	const std::string c_frame;
	const bool c_loadedFromFile;
	const bool c_needToDetermineStartsAndGoals;

	static const int PRM_VERSION = 2;
};

} /* namespace prm_planner */

#endif /* PRM_H_ */
