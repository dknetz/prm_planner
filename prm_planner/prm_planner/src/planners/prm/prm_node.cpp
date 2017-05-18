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

namespace prm_planner
{

PRMNode::PRMNode(const Eigen::Affine3d& pose,
		boost::atomic_int& nodeCounter,
		PRMNode::Type type) :
				m_pose(pose),
				m_collisionFree(true),
				m_type(type)
{
	++nodeCounter;
	m_id = nodeCounter;
}

PRMNode::PRMNode(const int id,
		boost::atomic_int& nodeCounter,
		PRMNode::Type type) :
				m_id(id),
				m_collisionFree(true),
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
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_pose;
}

void PRMNode::setPose(const Eigen::Affine3d& pose)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_pose = pose;
}

const Eigen::Vector3d PRMNode::getPosition() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_pose.translation();
}

void PRMNode::addEdge(PRMEdge* edge)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_edges.push_back(edge);
}

void PRMNode::removeEdge(PRMEdge* edge)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
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

bool PRMNode::isCollisionFree() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_collisionFree;
}

void PRMNode::setCollisionFree(bool collisionFree)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_collisionFree = collisionFree;
}

PRMNode::Type PRMNode::getType() const
{
	return m_type;
}

} /* namespace prm_planner */


