/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 20, 2017
 *      Author: kuhnerd
 * 	  Filename: feasibility_checker.h
 */

#ifndef H51B03C5F_5E5E_4A61_A8B9_ED8CAB66411E
#define H51B03C5F_5E5E_4A61_A8B9_ED8CAB66411E
#include <ais_definitions/class.h>
#include <prm_planner_robot/defines.h>

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(RobotArm);
FORWARD_DECLARE(CollisionDetector);

class FeasibilityChecker
{
public:
	FeasibilityChecker(boost::shared_ptr<Robot> robot);
	virtual ~FeasibilityChecker();

	/**
	 * Checks for the given arm poses, if the joint
	 * limits were reached or not. Returns true, if
	 * none of the requested poses exceed the joint
	 * limits.
	 * @poses [in]: the poses to check
	 * @robot [in]: the robot
	 * @cd [in]: the collision detector
	 */
	bool check(const Eigen::Affine3d& pose,
			boost::shared_ptr<CollisionDetector> cd,
			bool printCollisions = false);

private:
	boost::shared_ptr<Robot> m_robot;
};

} /* namespace prm_planner */

#endif /* H51B03C5F_5E5E_4A61_A8B9_ED8CAB66411E */
