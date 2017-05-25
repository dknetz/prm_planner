/*
 * ros_action_server.cpp
 *
 *  Created on: May 25, 2017
 *      Author: kuhnerd
 */

#include <actionlib/client/simple_action_client.h>
#include <ais_log/log.h>
#include <eigen_conversions/eigen_msg.h>
#include <prm_planner_msgs/GoalAction.h>
#include <Eigen/Geometry>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "prm_planner_action_server_interface");
	ros::NodeHandle n;

	//create action client and wait until server is available
	actionlib::SimpleActionClient<prm_planner_msgs::GoalAction> actionClient("prm_planner/goals", true);
	actionClient.waitForServer();

	//define goals: We use the end effector pose relative to the planning frame as a goal
	Eigen::Affine3d goal1;
	Eigen::Affine3d goal2;

	goal1.matrix() << 0, 0, 1, 0.85,
			0, -1, 0, -0.40,
			1, 0, 0, 0.7,
			0, 0, 0, 1;

	goal2.matrix() << 0, 0, 1, 0.85,
			0, -1, 0, 0.40,
			1, 0, 0, 0.75,
			0, 0, 0, 1;

	prm_planner_msgs::GoalGoal goalMsg1;
	tf::poseEigenToMsg(goal1, goalMsg1.goal);
	goalMsg1.action = prm_planner_msgs::GoalGoal::ACTION_MOVE;

	prm_planner_msgs::GoalGoal goalMsg2;
	tf::poseEigenToMsg(goal2, goalMsg2.goal);
	goalMsg2.action = prm_planner_msgs::GoalGoal::ACTION_MOVE;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	while (ros::ok())
	{
		actionClient.sendGoal(goalMsg1);
		actionClient.waitForResult();
		if (!actionClient.getResult()->success)
		{
			LOG_ERROR("Cannot find a trajectory to goal 1");
		}

		sleep(1);

		if (!ros::ok())
			break;

		actionClient.sendGoal(goalMsg2);
		actionClient.waitForResult();
		if (!actionClient.getResult()->success)
		{
			LOG_ERROR("Cannot find a trajectory to goal 2");
		}

		sleep(1);
	}

	return 0;
}

