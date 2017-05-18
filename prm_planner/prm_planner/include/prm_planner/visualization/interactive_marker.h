/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: interactive_marker.h
 */

#ifndef H33066137_D38E_422F_86B9_62643FA9553C
#define H33066137_D38E_422F_86B9_62643FA9553C

namespace prm_planner
{

class PRMPlanner;

class InteractiveMarker
{
public:
	InteractiveMarker(boost::shared_ptr<PRMPlanner> planner);
	virtual ~InteractiveMarker();

protected:
	boost::shared_ptr<PRMPlanner> m_planner;
};

} /* namespace prm_planner */

#endif /* H33066137_D38E_422F_86B9_62643FA9553C */
