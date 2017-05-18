/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 19, 2017
 *      Author: kuhnerd
 * 	  Filename: path_database.cpp
 */

#include <ais_log/log.h>
#include <ais_util/serialization_std.h>
#include <ais_definitions/macros.h>
#include <prm_planner/planners/database_cache/path_database.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/trajectory.h>
#include <fstream>
#include <iostream>

//Read and write lock which locks m_mutex
#define PATH_DATABASE_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex)
#define PATH_DATABASE_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex)
#define PATH_DATABASE_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex)
#define PATH_DATABASE_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex)

namespace prm_planner
{

PathDatabase::PathDatabase() :
				m_counter(0)
{
	ros::NodeHandle n;

	m_pub = n.advertise<visualization_msgs::MarkerArray>("database", 0);
}

PathDatabase::~PathDatabase()
{
}

bool PathDatabase::add(boost::shared_ptr<Path>& path)
{
	PATH_DATABASE_WRITE_LOCK();
	m_paths[m_counter++] = path;
	return true;
}

bool PathDatabase::write(const std::string& filename)
{
	PATH_DATABASE_WRITE_LOCK();

	std::ofstream ofs;
	ofs.open(filename, std::ofstream::out | std::ofstream::binary);
	if (!ofs.is_open())
	{
		LOG_ERROR("Cannot open file: " << filename);
		return false;
	}

	boost::archive::binary_oarchive oa(ofs);
	oa << m_paths << m_counter;

	return true;
}

void PathDatabase::publish()
{
	if (m_pub.getNumSubscribers() > 0)
	{
		PATH_DATABASE_READ_LOCK();
		visualization_msgs::MarkerArray array;
		int id = 0;

		for (auto& it : m_paths)
		{
			it.second->getROSVisualizationMessage(array, id, false, true);
		}

		m_pub.publish(array);
	}
}

void PathDatabase::getKdTreeData(std::vector<ais_point_cloud::SearchWrapperEigenVectorXWithId>& points)
{
	PATH_DATABASE_READ_LOCK();

	points.clear();
	points.reserve(m_paths.size());

	for (auto& it : m_paths)
	{
		auto& path = it.second;
		Eigen::VectorXd p(6);
		convertToSearchPoint(path->front().pose, path->back().pose, p);
		points.push_back(ais_point_cloud::SearchWrapperEigenVectorXWithId(p, it.first));
	}
}

boost::shared_ptr<PathDatabase> PathDatabase::read(const std::string& filename)
{
	boost::shared_ptr<PathDatabase> db(new PathDatabase);

	std::ifstream ifs;
	ifs.open(filename, std::ifstream::in | std::ifstream::binary);
	if (ifs.is_open())
	{
		boost::archive::binary_iarchive ia(ifs);
		ia >> db->m_paths >> db->m_counter;
	}
	else
	{
		LOG_ERROR("The file " << filename << " doesn't exists.");
	}

	return db;
}

boost::shared_ptr<Path> PathDatabase::getPath(const int id)
{
	PATH_DATABASE_READ_LOCK();

	IF_CHECK_MAP_VAR(m_paths, id, it)
	{
		return it->second;
	}
	else
	{
		return boost::shared_ptr<Path>();
	}
}

unsigned int PathDatabase::getSize() const
{
	PATH_DATABASE_READ_LOCK();

	return m_paths.size();
}

void PathDatabase::convertToSearchPoint(const Eigen::Affine3d& start,
		const Eigen::Affine3d& goal,
		Eigen::VectorXd& point)
{
//	//get pose to convert it into 6d position
//	Trajectory::Pose pStart(start);
//	Trajectory::Pose pGoal(goal);
//
//	point.resize(12); //6d start + 6d goal
//	point.head(6) = pStart.x;
//	point.tail(6) = pGoal.x;

	point.resize(6);
	point.head(3) = start.translation();
	point.tail(3) = goal.translation();
}

} /* namespace prm_planner */


