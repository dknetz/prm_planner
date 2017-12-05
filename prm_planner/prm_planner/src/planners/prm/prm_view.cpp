/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Nov 23, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_view.cpp
 */

#include <ais_definitions/macros.h>
#include <ais_util/stop_watch.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/planners/prm/prm_view.h>
#include <prm_planner/util/parameter_server.h>

namespace prm_planner
{

PRMView::PRMView(const boost::atomic_int& nextIdNode,
		const boost::atomic_int& nextIdEdge) :
				m_nextIdEdge(nextIdEdge.load()),
				m_nextIdNode(nextIdNode.load()),
				m_prm(NULL)
{
}

PRMView::~PRMView()
{
	for (auto& it : m_addedNodes)
		DELETE_VAR(m_nodeData[it.first].m_node);

	for (auto& it : m_addedEdges)
		DELETE_VAR(m_edgeData[it.first].m_edge);
}

PRMView::EdgeData::EdgeData() :
				m_controllerSuccessful(true),
				m_isBlocked(false),
				m_edge(NULL)
{
}

PRMView::NodeData::NodeData() :
				m_node(NULL)
{
}

const PRMNode* PRMView::addPose(const Eigen::Affine3d& pose,
		const KDL::JntArray& jointPose)
{
	PRMNode* node = new PRMNode(pose, m_nextIdNode);
	int activeConnections;
	const PRMNode* sameNode;
	if (!connectNode(node, activeConnections, sameNode))
	{
		DELETE_VAR(node);
		return NULL;
	}
	else
	{
		if (activeConnections > 0)
		{
			LOG_INFO("Found connections to " << activeConnections << " nodes!");
			NodeData& data = m_nodeData[node->getId()];
			data.m_node = node;
			data.m_jointPose = jointPose;
			m_addedNodes[node->m_id] = true;
			return node;
		}
		else if (sameNode != NULL)
		{
			LOG_INFO("Same node => no new connections were added!");
			DELETE_VAR(node);
			setJointPose(sameNode, jointPose);
			return sameNode;
		}
		else
		{
			LOG_WARNING("No connections were found! Maybe the visibility distance is too small.");
			DELETE_VAR(node);
			return NULL;
		}
	}
}

const PRMNode* PRMView::setStartPose(const int id,
		const KDL::JntArray& jointPose)
{
	if (CHECK_MAP(m_nodeData, id))
	{
		NodeData& data = m_nodeData[id];
		data.m_jointPose = jointPose;
		return data.m_node;
	}
	else
	{
		LOG_ERROR("Unknown node " << id);
		return NULL;
	}
}

const PRMNode* PRMView::setStartPose(const Eigen::Affine3d& startPose,
		const KDL::JntArray& jointPose)
{
	LOG_INFO("Adding Start Pose...");
	return addPose(startPose, jointPose);
}

const PRMNode* PRMView::setGoalPose(const Eigen::Affine3d& goalPose)
{
	static const KDL::JntArray dummy;
	LOG_INFO("Adding Goal Pose...");
	return addPose(goalPose, dummy);
}

void PRMView::removeNode(const PRMNode* node)
{
	for (auto it = m_edgeData.begin(); it != m_edgeData.end();)
	{
		if (it->second.m_edge->getNode1()->getId() == node->getId()
				|| it->second.m_edge->getNode2()->getId() == node->getId())
		{
			const PRMNode* other = it->second.m_edge->getOtherNode(node->getId());
			NodeData& data = m_nodeData[other->getId()];

			//remove edge
			for (auto it2 = data.m_edges.begin(); it2 != data.m_edges.end(); ++it2)
			{
				if (*(*it2) == *it->second.m_edge)
				{
					data.m_edges.erase(it2);
					return;
				}
			}

			//Only delete if it was added by the view.
			//Otherwise, we need to keep it, because we
			//would delete the real edge
			IF_CHECK_MAP_VAR(m_addedEdges, it->first, delIt)
			{
				DELETE_VAR(it->second.m_edge);
				m_addedEdges.erase(delIt);
			}

			//remove the edge from the edges map anyway
			it = m_edgeData.erase(it);
		}
		else
		{
			++it;
		}
	}

	int id = node->getId();
	IF_CHECK_MAP_VAR(m_nodeData, id, it)
	{
		//only delete if added by the view
		IF_CHECK_MAP_VAR(m_addedNodes, id, addedIt)
		{
			DELETE_VAR(it->second.m_node);
			m_addedNodes.erase(addedIt);
		}

		m_nodeData.erase(it);

		//delete node from start nodes
		IF_CHECK_MAP_VAR(m_startNodes, id, it2)
			m_startNodes.erase(it2);

		//delete node from goal nodes
		IF_CHECK_MAP_VAR(m_goalNodes, id, it3)
			m_goalNodes.erase(it3);
	}
}

void PRMView::removeEdgeFromEdgeMap(const PRMEdge* edge)
{
	m_edgeData.erase(edge->m_id);
}

PRMNodeMap& PRMView::getGoalNodes()
{
	return m_goalNodes;
}

PRMNodeMap& PRMView::getStartNodes()
{
	return m_startNodes;
}

bool PRMView::connectNode(const PRMNode* node,
		int& activeConnections,
		const PRMNode*& samePoseNode)
{
	Eigen::Vector3d position1 = node->getPosition();
	activeConnections = 0;
	samePoseNode = NULL;

	//first check if node already exists (without adding edges)
	for (const auto& it : m_nodeData)
	{
		if (it.second.m_node->getPose().isApprox(node->getPose()))
		{
			activeConnections = -1;
			samePoseNode = it.second.m_node;
			return true;
		}
	}

	NodeData& newNodeData = m_nodeData[node->m_id];

	for (auto& it : m_nodeData)
	{
		if (it.first != node->m_id)
		{
			Eigen::Vector3d position2 = it.second.m_node->getPosition();
			if (m_prm->isVisible(position1, position2))
			{
				PRMEdge* edge = new PRMEdge(it.second.m_node, node, m_nextIdEdge);

				EdgeData& data = m_edgeData[edge->m_id];
				data.m_edge = edge;

				m_addedEdges[edge->m_id] = true;

				//add edges to nodes
				it.second.m_edges.push_back(edge);
				newNodeData.m_edges.push_back(edge);

				++activeConnections;
			}
		}
	}

	if (activeConnections == 0)
	{
		//remove the entry again, since we haven't found
		//any connection
		m_nodeData.erase(node->m_id);
		return false;
	}
	else
	{
		return true;
	}
}

bool PRMView::updateEdge(const PRMEdge* edge,
		const double maxWaitTime,
		KDL::JntArray& finalPose,
		const PRMNode* startNode,
		KDL::JntArray& startPose,
		boost::shared_ptr<CollisionDetector>& cd,
		const double dt)
{
	EdgeData& data = m_edgeData[edge->m_id];

	//lazy init
	if (data.m_controller.get() == NULL)
	{
		data.m_controller.reset(new SimulationRobotController(edge, m_prm->m_robot, m_prm->m_pd));
	}

//	LOG_INFO(data.m_edge);
//	LOG_INFO(data.m_edge->getId());
//	LOG_INFO(data.m_edge->getNode1()->getId());
//	LOG_INFO(data.m_edge->getNode2()->getId());
//	LOG_INFO(data.m_controller.get());

	if (!data.m_controller->updateFromEdge(startNode, startPose))
		return false;

	data.m_controller->setCollisionDetection(cd);
//	ais_util::StopWatch::getInstance()->start("update");
	data.m_controllerSuccessful = data.m_controller->canControl(maxWaitTime, dt);
//	ais_util::StopWatch::getInstance()->stopPrint("update");
	data.m_controller->getFinalJointState(finalPose);

	bool result = data.m_controllerSuccessful;

	return result;
}

void PRMView::getJointPath(const PRMEdge* edge,
		ArmJointPath& path)
{
	EdgeData& data = m_edgeData[edge->m_id];
	if (data.m_controller != NULL)
	{
		data.m_controller->getJointPath(path);
	}
}

bool PRMView::isBlocked(const PRMEdge* edge)
{
	EdgeData& dataEdge = m_edgeData[edge->m_id];
	NodeData& dataNode1 = m_nodeData[edge->m_node1->m_id];
	NodeData& dataNode2 = m_nodeData[edge->m_node2->m_id];
	return dataEdge.m_isBlocked;
}

void PRMView::setIsBlocked(const PRMEdge* edge,
		bool isBlocked)
{
	EdgeData& dataEdge = m_edgeData[edge->m_id];
	dataEdge.m_isBlocked = true;
}

const std::vector<PRMEdge*>& PRMView::getEdgesToNode(const PRMNode* node)
{
	return m_nodeData[node->m_id].m_edges;
}

void PRMView::getJointPose(const PRMNode* node,
		KDL::JntArray& jointPose)
{
	jointPose = m_nodeData[node->m_id].m_jointPose;
}

void PRMView::setJointPose(const PRMNode* node,
		const KDL::JntArray& jointPose)
{
	m_nodeData[node->m_id].m_jointPose = jointPose;
}

void PRMView::updateStartAndGoal()
{
	for (auto& it : m_nodeData)
	{
		if (it.second.m_node->getType() == PRMNode::StartNode)
			m_startNodes[it.first] = it.second.m_node;

		if (it.second.m_node->getType() == PRMNode::GoalNode)
			m_goalNodes[it.first] = it.second.m_node;
	}
//
//	LOG_INFO("Found " << m_startNodes.size() << " start nodes");
//	LOG_INFO("Found " << m_goalNodes.size() << " goal nodes");
}

} /* namespace prm_planner */

