/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Nov 24, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_defines.h
 */

#ifndef H746C1CE9_90F8_4474_8BEB_FBA703164BB5
#define H746C1CE9_90F8_4474_8BEB_FBA703164BB5
#include <unordered_map>

namespace prm_planner
{
class PRMNode;
class PRMEdge;

typedef std::unordered_map<int, PRMNode*> PRMNodeMap;
typedef std::unordered_map<int, PRMEdge*> PRMEdgeMap;
typedef std::unordered_map<int, bool> PRMBoolMap;
}

#endif /* H746C1CE9_90F8_4474_8BEB_FBA703164BB5 */
