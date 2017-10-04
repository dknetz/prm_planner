/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_edge.h
 */

#ifndef PRM_EDGE_H_
#define PRM_EDGE_H_

#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <prm_planner_robot/defines.h>

namespace prm_planner
{

class PRMNode;

class PRMEdge
{
public:
	PRMEdge(const PRMNode* node1,
			const PRMNode* node2,
			boost::atomic_int& edgeCounter);
	PRMEdge(const PRMNode* node1,
			const PRMNode* node2,
			const int id,
			boost::atomic_int& edgeCounter);
	virtual ~PRMEdge();

	const PRMNode* getNode1() const;
	const PRMNode* getNode2() const;

	const PRMNode* getOtherNode(const int id) const;
	double getCosts() const;

	bool operator ==(const PRMEdge &b) const;
	int getId() const;

	bool isBlocked() const;
	void setIsBlocked(bool isBlocked);

private:
	const PRMNode* m_node1;
	const PRMNode* m_node2;

	boost::atomic_bool m_isBlocked;
	int m_id;

	friend class PRM;
	friend class PRMView;
};

}
/* namespace prm_planner */

#endif /* PRM_EDGE_H_ */
