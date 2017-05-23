#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <ais_util/stop_watch.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameter_server.h>
#include <ros/ros.h>
#include <exception>

using namespace prm_planner;

int main(int argc,
		char** argv)
{
	bool active = true;
	if (argc > 1)
	{
		active = std::string(argv[1]) != "false";
	}

	try
	{
		ros::init(argc, argv, "prm_planner");
		ros::NodeHandle n;

		if (!ParameterServer::loadParameters())
		{
			ros::shutdown();
			LOG_ERROR("You have not specified all parameters in the namespace " << n.getNamespace())
			return -100;
		}

		PRMPlanner::PlannerParameters params;
		params.active = active;

		//shared ptr necessary because of enable_shared_from_this (there must be at least one
		//shared ptr that contains the object)
		boost::shared_ptr<PRMPlanner> planner(new PRMPlanner(params));
		planner->init();

		ros::waitForShutdown();
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
