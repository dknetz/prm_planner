/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 18, 2017
 *      Author: kuhnerd
 * 	  Filename: main_sample_database.cpp
 */
#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <prm_planner/planners/database_cache/path_database.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <ros/ros.h>

using namespace prm_planner;

boost::shared_ptr<PathDatabase> db;

int main(int argc,
		char** argv)
{
	try
	{
		if (argc != 2)
		{
			LOG_ERROR("Call with arguments: database_path");
			exit(4582);
		}

		std::string filename = argv[1];

		ros::init(argc, argv, "prm_planner");
		ros::NodeHandle n;

		if (!ParameterServer::loadParameters())
			return 100;

		//we don't need the goal action server
		ParameterServer::startSubscribers = false;

		db = PathDatabase::read(filename);
		LOG_INFO("Loaded database with "<<db->getSize()<<" paths");

		ros::Rate r(1);
		while (ros::ok())
		{
			db->publish();
			ros::spinOnce();
			r.sleep();
		}
	}
	catch (ais_definitions::Exception& ex)
	{
		LOG_ERROR("Thrown ais_definitions::Exception: " << ex.what());
	}
	catch (std::exception& ex)
	{
		LOG_ERROR("Thrown std::exception: " << ex.what());
	}
	catch (...)
	{
		LOG_ERROR("Thrown unknown exception, use debugger for further information");
	}
	return 0;
}

