/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_view.h
 */

#ifndef H5A1812F8_CFCD_4A28_B729_B77CCA9BE8B2
#define H5A1812F8_CFCD_4A28_B729_B77CCA9BE8B2
#include <ais_point_cloud/easy_kd_tree.h>
#include <prm_planner/planners/prm/prm_defines.h>
#include <prm_planner_robot/defines.h>

FORWARD_DECLARE_N(fcl_collision_detection, FCLWrapper);

namespace prm_planner
{

FORWARD_DECLARE (SimulationRobotController);
FORWARD_DECLARE (PRMEdge);
FORWARD_DECLARE (PRMNode);
FORWARD_DECLARE (PRM);
FORWARD_DECLARE (CollisionDetector);

class PRMView
{
public:
	struct EdgeData
	{
		EdgeData();

		boost::shared_ptr<SimulationRobotController> m_controller;
		bool m_controllerSuccessful;
		bool m_isBlocked;
		PRMEdge* m_edge;
	};

	struct NodeData
	{
		NodeData();

		std::vector<PRMEdge*> m_edges;
		KDL::JntArray m_jointPose;
		PRMNode* m_node;
	};

	typedef std::unordered_map<int, EdgeData> EdgeDataMap;
	typedef std::unordered_map<int, NodeData> NodeDataMap;

	PRMView(const boost::atomic_int& nextIdNode,
			const boost::atomic_int& nextIdEdge);
	virtual ~PRMView();

	const PRMNode* addPose(const Eigen::Affine3d& pose,
			const KDL::JntArray& jointPose);
	const PRMNode* setStartPose(const int id,
			const KDL::JntArray& jointPose);
	const PRMNode* setStartPose(const Eigen::Affine3d& startPose,
			const KDL::JntArray& jointPose);
	const PRMNode* setGoalPose(const Eigen::Affine3d& goalPose);

	void removeNode(const PRMNode* node);
	void removeEdgeFromEdgeMap(const PRMEdge* edge);

	PRMNodeMap& getGoalNodes();
	PRMNodeMap& getStartNodes();

	bool connectNode(const PRMNode* node,
			int& activeConnections,
			const PRMNode*& samePoseNode);

	bool updateEdge(const PRMEdge* edge,
			const double maxWaitTime,
			KDL::JntArray& finalJointPose,
			const PRMNode* startNode,
			KDL::JntArray& startJointPose,
			boost::shared_ptr<CollisionDetector>& cd,
			const double dt = 0.01);

	void getJointPath(const PRMEdge* edge,
			ArmJointPath& path);

	bool isBlocked(const PRMEdge* edge);
	void setIsBlocked(const PRMEdge* edge,
			bool isBlocked);

	//nodes
	const std::vector<PRMEdge*>& getEdgesToNode(const PRMNode* node);
	void getJointPose(const PRMNode* node,
			KDL::JntArray& pose);
	void setJointPose(const PRMNode* node,
			const KDL::JntArray& jointPosition);

private:
	void updateStartAndGoal();

private:
	boost::atomic_int m_nextIdNode, m_nextIdEdge;

	EdgeDataMap m_edgeData;
	NodeDataMap m_nodeData;

	PRMBoolMap m_addedEdges;
	PRMBoolMap m_addedNodes;

	PRMNodeMap m_startNodes;
	PRMNodeMap m_goalNodes;

	boost::shared_ptr<ais_point_cloud::EasyKDTree> m_kdtree;

	PRM* m_prm;

	friend class PRM;
};

} /* namespace prm_planner */

#endif /* H5A1812F8_CFCD_4A28_B729_B77CCA9BE8B2 */
