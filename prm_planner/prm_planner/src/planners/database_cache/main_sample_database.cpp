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
#include <ais_util/progress_bar.h>
#include <prm_planner/planners/database_cache/path_database.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <ros/ros.h>
#include <omp.h>

using namespace prm_planner;

void samplePaths(const std::string& database,
		boost::shared_ptr<PRMPlanner>& planner,
		int samples)
{
	boost::shared_ptr<Robot> robot = planner->getRobot();

	boost::shared_ptr<PathDatabase> db;

	//if the file exists, load it and continue
	if (boost::filesystem::exists(database))
	{
		db = PathDatabase::read(database);
		LOG_INFO("File already exists and has " << db->getSize() << " paths, continuing...")
	}
	else
	{
		db.reset(new PathDatabase);
	}

	//activates the passive mode, i.e., does not talk to the robot
	robot->setPassiveMode(true);

	KDL::JntArray startSample, goalSample;
	Eigen::Affine3d start, goal;

	int notFound = 0;
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

	PlanningParameters p;
	p.mode = Default;

	ais_util::ProgressBar progress("Sampling", samples);
	progress.set(db->getSize());

	omp_set_num_threads(8);

#	pragma omp parallel for shared(notFound)
	for (size_t i = db->getSize(); i < samples; ++i)
	{
		int found = 10;
		while (found > 0)
		{
			robot->sampleValidChainJointState(startSample, true);
			robot->sampleValidChainJointState(goalSample, true);

			robot->getFK(startSample, start);
			robot->getFK(goalSample, goal);

			boost::shared_ptr<Path> path;
			if (pd->plan(startSample, start, goal, path, p))
			{
				db->add(path);
				progress.increment();
				break;
			}
			else
			{
				++notFound;
				--found;
			}
		}

		if (i % 100 == 0)
		{
			db->write(database);
		}

		if (!ros::ok())
			i = samples; //since openmp doesn't allow break
	}
	progress.finish();

	LOG_INFO("Could not find a plan for " << notFound << " queries");

	db->write(database);
}

int main(int argc,
		char** argv)
{
	try
	{
		if (argc != 3)
		{
			LOG_ERROR("Call with arguments: database_path samples");
			exit(4582);
		}

		std::string filename = argv[1];
		int samples = std::stoi(argv[2]);

		ros::init(argc, argv, "prm_planner");
		ros::NodeHandle n;

		if (!ParameterServer::loadParameters())
			return 100;

		//we don't need the goal action server
		ParameterServer::startSubscribers = false;
		ParameterServer::executionMode = parameters::NoExecution;
		ParameterServer::visualize = false;

		PRMPlanner::PlannerParameters params;
		params.active = true;

		//shared ptr necessary because of enable_shared_from_this (there must be at least one
		//shared ptr that contains the object)
		boost::shared_ptr<PRMPlanner> planner(new PRMPlanner(params));
		planner->init();

		samplePaths(filename, planner, samples);
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

