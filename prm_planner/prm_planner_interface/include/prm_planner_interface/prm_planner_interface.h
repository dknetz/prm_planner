/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jun 9, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: roadmap_planner.h
 */

#ifdef BUILD_NEUROBOTS_INTERFACE

#ifndef PRM_PLANNER_INTERFACE_H_
#define PRM_PLANNER_INTERFACE_H_

#include <prm_planner_msgs/GoalAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_interface_definition/robot_interface.h>
#include <ros/ros.h>

namespace prm_planner
{

class PRMPlannerInterface: public robot_interface_definition::RobotInterface
{
public:
	PRMPlannerInterface(const std::string& plannerNamespace);
	virtual ~PRMPlannerInterface();

	virtual bool plan(const Eigen::Affine3d& goal);
	virtual bool planRel(const Eigen::Affine3d& goal);
	virtual bool plan(const std::vector<std::string>& param1);

	virtual bool execute();

	virtual bool grasp(const std::string& object);
	virtual bool drop(const std::string& object,
			const Eigen::Affine3d& pose);

	virtual bool activate();
	virtual bool deactivate();

private:
	const std::string c_namespace;
	actionlib::SimpleActionClient<prm_planner_msgs::GoalAction>* m_actionClient;
	ros::ServiceClient m_setState;
};

} /* namespace prm_planner */

#endif /* PRM_PLANNER_INTERFACE_H_ */

#endif
