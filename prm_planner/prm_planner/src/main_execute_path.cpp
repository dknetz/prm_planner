#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <ais_util/stop_watch.h>
#include <prm_planner/execution/executer.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_robot/path.h>
#include <ros/ros.h>
#include <exception>

using namespace prm_planner;

int main(int argc,
		char** argv)
{
	std::string pathFile = "";
	if (argc > 1)
	{
		pathFile = argv[1];
	}
	else
	{
		LOG_ERROR("Please provide the filename of the file which contains the path!");
		exit(123);
	}

	try
	{
		ros::init(argc, argv, "prm_planner");
		ros::NodeHandle n;

		if (!ParameterServer::loadParameters())
		{
			return -100;
		}

		PRMPlanner::PlannerParameters params;
		params.active = true;

		//shared ptr necessary because of enable_shared_from_this (there must be at least one
		//shared ptr that contains the object)
		boost::shared_ptr<PRMPlanner> planner(new PRMPlanner(params));
		planner->init();

		boost::shared_ptr<Executer> executer = planner->getExecuter();

		boost::shared_ptr<Path> path;
		Eigen::Affine3d tcp;
		Executer::readPath(pathFile, path, tcp);

		boost::shared_ptr<Robot> robot = planner->getRobot();

		LOG_INFO(tcp.matrix());
		robot->setToolCenterPointTransformation(tcp);
		executer->reset();

		LOG_INFO("Read path with " << path->size() << " waypoints");

		executer->executePreprocessedPathMap(path);

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
