/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 23, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: main_new_prm.cpp
 */

#include <ais_point_cloud/rgbd_image.h>
#include <boost/filesystem.hpp>
#include <omp.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameter_server.h>
#include <ros/ros.h>

using namespace prm_planner;

void printHelp()
{
	LOG_INFO("new_roadmap filename size max_distance");
}

int main(int argc,
		char** argv)
{
//	ros::init(argc, argv, "prm_planner");
//	ros::NodeHandle n("/prm_planner");
//
//	if (argc < 5)
//	{
//		printHelp();
//		return -1;
//	}
//
//	std::string filename = argv[1];
//	int size = std::stoi(argv[2]);
//	double maxDistance = std::stod(argv[3]);
//	std::string problem = argv[4];
//
//	//parameters
//	ParameterServer::verbose = false;
//	ParameterServer::loadParameters();
//	ParameterServer::prmLoadFromFile = false;
//	ParameterServer::prmSaveToFile = true;
//	ParameterServer::prmFileName = filename;
//	ParameterServer::prmSize = size;
//	ParameterServer::prmVisibilityMaxDistance = maxDistance;
//
//	ProblemDefinitionManager::getInstance()->init(ParameterServer::problem, boost::shared_ptr<PRMPlanner>());
//	boost::shared_ptr<Robot> robot = ProblemDefinitionManager::getInstance()->getRobot();
//
//	PRM* prm = new PRM(robot,
//			ParameterServer::prmSize /*size*/,
//			ParameterServer::prmVisibilityMaxDistance /*edgeVisibilityMaxRange*/,
//			ProblemDefinitionManager::getInstance()->getProblemDefinition()->getFrame());
//	prm->save(filename);
//
//	DELETE_VAR(prm);

	LOG_ERROR("You need to adapt this program to work with the new prm parameters");

	return 0;
}

