/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: heuristic.h
 */

#ifndef HEURISTIC_H_
#define HEURISTIC_H_
#include <prm_planner/planners/prm/prm_node.h>

namespace prm_planner
{

class Heuristic
{
public:
	Heuristic();
	virtual ~Heuristic();

	/**
	 * Euclidean distance between the positions of n1 and n2
	 * (no orientation in encountered)
	 */
	static double euclideanDistEndEffector(const PRMNode* n1,
			const PRMNode* n2);

	/**
	 * see: The kinematic roadmap: a motion planning based global approach
	 * for inverse kinematics of redundant robots
	 */
	static double frameDistEndEffector(const PRMNode* n1,
			const PRMNode* n2);
};

} /* namespace prm_planner */

#endif /* HEURISTIC_H_ */
