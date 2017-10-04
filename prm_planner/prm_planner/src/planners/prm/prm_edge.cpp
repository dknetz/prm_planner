/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_edge.cpp
 */

#include <ais_log/log.h>
#include <ais_util/color.h>
#include <ais_util/thread.h>
#include <prm_planner/planners/prm/heuristic.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/util/defines.h>

namespace prm_planner
{

PRMEdge::PRMEdge(const PRMNode* node1,
		const PRMNode* node2,
		boost::atomic_int& edgeCounter) :
				m_node1(node1),
				m_node2(node2),
				m_isBlocked(false)
{
	ros::NodeHandle n;
	++edgeCounter;
	m_id = edgeCounter;
}

PRMEdge::PRMEdge(const PRMNode* node1,
		const PRMNode* node2,
		const int id,
		boost::atomic_int& edgeCounter) :
				m_node1(node1),
				m_node2(node2),
				m_id(id),
				m_isBlocked(false)
{
	ros::NodeHandle n;
	++edgeCounter;
	ais_util::atomicMax(edgeCounter, id);
}

PRMEdge::~PRMEdge()
{
}

const PRMNode* PRMEdge::getNode1() const
{
	return m_node1;
}

const PRMNode* PRMEdge::getNode2() const
{
	return m_node2;
}

const PRMNode* PRMEdge::getOtherNode(const int id) const
		{
	return id == m_node1->getId() ? m_node2 : m_node1;
}

double PRMEdge::getCosts() const
{
	return Heuristic::frameDistEndEffector(m_node1, m_node2);
}

bool PRMEdge::operator ==(const PRMEdge& b) const
		{
	return m_id == b.m_id;
}

int PRMEdge::getId() const
{
	return m_id;
}

bool PRMEdge::isBlocked() const
{
	return m_isBlocked;
}

void PRMEdge::setIsBlocked(bool isBlocked)
{
	m_isBlocked = isBlocked;
}

} /* namespace prm_planner */

