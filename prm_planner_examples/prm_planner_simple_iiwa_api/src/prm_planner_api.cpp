/*
 * ros_action_server.cpp
 *
 *  Created on: May 25, 2017
 *      Author: kuhnerd
 */

#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <eigen_conversions/eigen_msg.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_msgs/GoalAction.h>
#include <Eigen/Geometry>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "prm_planner_action_server_interface");
	ros::NodeHandle n;

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

	try
	{
		//load parameters before creating an instance of the planner!
		if (!prm_planner::ParameterServer::loadParameters())
		{
			ros::shutdown();
			LOG_ERROR("You have not specified all parameters in the namespace " << n.getNamespace())
			return -100;
		}

		//create the planner. Important: create it as a shared pointer, otherwise you get runtime
		//errors! You don't have to create a ROS spinner -> it is started in the planner.
		boost::shared_ptr<prm_planner::PRMPlanner> planner(new prm_planner::PRMPlanner);

		//initialize the planner
		planner->init();

		while (ros::ok())
		{
			if (!planner->planAndExecuteSync(goal1))
			{
				LOG_ERROR("Cannot find a trajectory to goal 1");
			}

			sleep(1);

			if (!ros::ok())
				break;

			if (!planner->planAndExecuteSync(goal2))
			{
				LOG_ERROR("Cannot find a trajectory to goal 2");
			}

			sleep(1);
		}
	}
	catch (ais_definitions::Exception& ex)
	{
		LOG_ERROR("Thrown ais_definitions::Exception: " << ex.what());
	}
	catch (std::exception& ex)
	{
		LOG_ERROR("Thrown ais_definitions::Exception: " << ex.what());
	}
	catch (...)
	{
		LOG_ERROR("Thrown unknown exception, use debugger for further information");
	}
	return 0;
}

