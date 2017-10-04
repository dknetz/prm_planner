/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_node.cpp
 */

#include <ais_log/log.h>
#include <ais_util/thread.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>

#define PRM_NODE_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex)
#define PRM_NODE_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex)
#define PRM_NODE_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex)
#define PRM_NODE_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex)

namespace prm_planner
{

PRMNode::PRMNode(const Eigen::Affine3d& pose,
		boost::atomic_int& nodeCounter,
		PRMNode::Type type) :
				m_pose(pose),
				m_type(type)
{
	++nodeCounter;
	m_id = nodeCounter;
}

PRMNode::PRMNode(const int id,
		boost::atomic_int& nodeCounter,
		PRMNode::Type type) :
				m_id(id),
				m_type(type)
{
	++nodeCounter;
	ais_util::atomicMax(nodeCounter, id);
}

PRMNode::~PRMNode()
{
}

const Eigen::Affine3d PRMNode::getPose() const
{
	PRM_NODE_READ_LOCK();
	return m_pose;
}

void PRMNode::setPose(const Eigen::Affine3d& pose)
{
	PRM_NODE_WRITE_LOCK();
	m_pose = pose;
}

const Eigen::Vector3d PRMNode::getPosition() const
{
	PRM_NODE_READ_LOCK();
	return m_pose.translation();
}

void PRMNode::addEdge(PRMEdge* edge)
{
	PRM_NODE_WRITE_LOCK();
	m_edges.push_back(edge);
}

void PRMNode::removeEdge(PRMEdge* edge)
{
	PRM_NODE_WRITE_LOCK();
	for (auto it = m_edges.begin(); it != m_edges.end(); ++it)
	{
		if (*(*it) == *edge)
		{
			m_edges.erase(it);
			return;
		}
	}
}

bool PRMNode::operator ==(const PRMNode& b) const
		{
	return m_id == b.m_id;
}

const int PRMNode::getId() const
{
	return m_id;
}

PRMNode::Type PRMNode::getType() const
{
	return m_type;
}

} /* namespace prm_planner */


