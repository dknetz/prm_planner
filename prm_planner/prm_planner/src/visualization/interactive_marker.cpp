/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: interactive_marker.cpp
 */

#include <prm_planner/prm_planner.h>
#include <prm_planner/visualization/interactive_marker.h>

namespace prm_planner
{

InteractiveMarker::InteractiveMarker(boost::shared_ptr<PRMPlanner> planner) :
				m_planner(planner)
{
}

InteractiveMarker::~InteractiveMarker()
{
}

} /* namespace prm_planner */

