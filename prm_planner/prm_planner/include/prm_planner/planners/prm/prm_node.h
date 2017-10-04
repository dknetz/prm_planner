/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_node.h
 */

#ifndef PRM_NODE_H_
#define PRM_NODE_H_

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <prm_planner_robot/defines.h>
#include <Eigen/Geometry>

namespace prm_planner
{

class PRMEdge;

class PRMNode
{
public:
	enum Type
	{
		StartNode,
		GoalNode,
		Default
	};

	PRMNode(const Eigen::Affine3d& pose,
			boost::atomic_int& nodeCounter,
			Type type = Default);
	virtual ~PRMNode();

	/**
	 * Returns/Sets the pose and position of the
	 * node.
	 */
	const Eigen::Affine3d getPose() const;
	const Eigen::Vector3d getPosition() const;
	void setPose(const Eigen::Affine3d& pose);

	/**
	 * Add or remove an edge from the PRM
	 */
	void addEdge(PRMEdge* edge);
	void removeEdge(PRMEdge* edge);

	/**
	 * Checks, if the ids of two nodes
	 * are the same
	 */
	bool operator ==(const PRMNode &b) const;

	/**
	 * Returns the id
	 */
	const int getId() const;

	/**
	 * Returns the type. The PRM can have several
	 * multiple start and goal nodes.
	 */
	Type getType() const;

private:
	/**
	 * Internal constructor for loading the PRM from
	 * file. @see load() in PRM
	 */
	PRMNode(const int id,
			boost::atomic_int& nodeCounter,
			Type type = Default);

	std::vector<PRMEdge*> m_edges;
	Eigen::Affine3d m_pose;
	int m_id;
	mutable boost::shared_mutex m_mutex;

	Type m_type;

	friend class PRM;
	friend class PRMView;
};

} /* namespace prm_planner */

#endif /* PRM_NODE_H_ */
