/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: executer.cpp
 */

#include <ais_log/log.h>
#include <pluginlib/class_loader.h>
#include <prm_planner/execution/executer.h>
#include <prm_planner_robot/path.h>
#include <ais_point_cloud/easy_kd_tree.h>
#include <ais_ros/ros_base_interface.h>
#include <boost/archive/binary_oarchive.hpp>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_controller/controller.h>
#include <prm_planner_robot/robot_arm.h>

namespace prm_planner
{

Executer::Executer() :
				m_currentPathSegment(0),
				m_savePath(false)
{
	ProblemDefinitionManager* pdm = ProblemDefinitionManager::getInstance();
	m_robot = pdm->getRobot();
}

Executer::~Executer()
{
}

void Executer::splitPath(const boost::shared_ptr<Path>& path,
		SubPath& splittedPath)
{
	boost::shared_ptr<Path> subpath(new Path(path->c_frame));
//
//	if (path->empty())
//		return;
//
//	double oldVel = path->begin()->maxAngularVel;
	for (auto& it : *path)
	{
		//add all robot waypoints as long as there is no special
		//waypoint as grasping or droping
//		double vel = it.maxAngularVel;
		if (it.type == Path::Waypoint::RobotWaypoint) // && vel == oldVel
		{
			subpath->append(it);
		}
		//we have anything else
		else
		{
//			if (it.type == Path::Waypoint::RobotWaypoint && vel != oldVel)
//			{
//				LOG_INFO("Split due to velocity change from " << oldVel << " to " << vel);
//			}

			//store the sub path
			LOG_INFO("Subpath size: " << subpath->size());
			splittedPath.push_back(subpath);

			//create a new sub path
			subpath.reset(new Path(path->c_frame));
			subpath->append(it);

//			//we only have one waypoint for specials
//			if (it.type != Path::Waypoint::RobotWaypoint)
//			{
			splittedPath.push_back(subpath);

			//new sub path for the next waypoints
			subpath.reset(new Path(path->c_frame));
//			}

//			oldVel = vel;
		}
	}

	//it's a normal path without special commands,
	//so push it to the path vector or there is a
	//trajectory at the end of the paths vector
	if (!subpath->empty())
	{
		LOG_INFO("Subpath size: " << subpath->size());
		splittedPath.push_back(subpath);
	}
}

bool Executer::executePath(const boost::shared_ptr<Path> path)
{
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

	if (path.get() == NULL)
		return false;

	LOG_DEBUG("path size: " << path->size());

	//make dense path (important for e.g., constraints)
//	path->makeDense(0.03);

	path->publish();

	//transform the taks poses to arm frame
	std::string robotFrame = m_robot->getRootFrame();
	boost::shared_ptr<Path> pathArmFrame(new Path(robotFrame));
	Eigen::Affine3d tPlanningToArm = ais_ros::RosBaseInterface::getRosTransformation(pd->getFrame(), robotFrame);

	//compute paths for each arm
	for (auto& it : *path)
	{
		Path::Waypoint wp(it);
		wp.pose = tPlanningToArm * it.pose;
		pathArmFrame->append(wp);
	}

	//clean all paths
	pathArmFrame->cleanPath();

	//splits the paths at special waypoints (grasping,...)
	EXECUTER_LOCK();
	splitPath(pathArmFrame, m_pathSegments);
	m_currentPathSegment = 0;

	//serializes the path map, which can be used for
	//repetitive motions without planning
	if (m_savePath)
		savePath(m_pathFileName, pathArmFrame);

	return true;
}

bool Executer::executePreprocessedPathMap(const boost::shared_ptr<Path>& path)
{
	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

	//splits the paths at special waypoints (grasping,...)
	EXECUTER_LOCK();
	splitPath(path, m_pathSegments);
	m_currentPathSegment = 0;

	return true;
}

void Executer::sendPath(SubPath& path,
		const int currentIndex,
		Controller* controller)
{
	controller->updateFromPath(path[currentIndex]);
}

bool Executer::savePath(const std::string& filename,
		const boost::shared_ptr<Path>& path)
{
	std::ofstream ofs;
	ofs.open(filename, std::ofstream::out | std::ofstream::binary);

	Eigen::Affine3d tcp = Eigen::Affine3d::Identity();
	if (m_robot->isUseTcp())
		tcp = m_robot->getTcp();

	if (!ofs.is_open())
	{
		LOG_ERROR("Cannot open file: " << filename);
		return false;
	}

	boost::archive::binary_oarchive oa(ofs);
	oa << path << tcp;

	return true;
}

bool Executer::readPath(const std::string& filename,
		boost::shared_ptr<Path>& path,
		Eigen::Affine3d& tcp)
{
	std::ifstream ifs;
	ifs.open(filename, std::ifstream::in | std::ifstream::binary);
	if (ifs.is_open())
	{
		boost::archive::binary_iarchive ia(ifs);
		ia >> path >> tcp;
	}
	else
	{
		LOG_ERROR("The file " << filename << " doesn't exists.");
		return false;
	}

	return true;
}

void Executer::handleSpecialCommand(SubPath& pathSegments,
		int currentIndex)
{
	auto& path = pathSegments[m_currentPathSegment];
	Path::Waypoint& wp = *path->begin();
	Path::Waypoint::Type type = path->getFirstWaypointType();

	if (type == Path::Waypoint::OpenGripper)
	{
		auto& gripper = m_robot->getGripper();
		gripper->open();

		//remove object
		gripper->setCurrentObject("");
		gripper->setTGripperToObject(Eigen::Affine3d::Identity());
	}
	else if (type == Path::Waypoint::CloseGripper)
	{
		//get the object name which can be stored in the
		//helper variable of the waypoint
		std::string objectName = path->getFirstWaypointHelper();

		auto& gripper = m_robot->getGripper();
		gripper->close();

		//check if we grasped an object. If yes, we
		//assume that the object was really grasped
		//and set the corresponding information in
		//the gripper
		if (!objectName.empty())
		{
			//get object
			boost::shared_ptr<GraspableObject> obj = ObjectManager::getInstance()->getObject(objectName);
			std::string planningFrame = ProblemDefinitionManager::getInstance()->getProblemDefinition()->getConfig().planningFrame;

			//update gripper
			Eigen::Affine3d transformation;
			if (!obj->getTransformationFromFrame(planningFrame, transformation))
			{
				//fallback: if we cannot get a transformation between robot
				//and object, we use the old information that is stored in the
				//waypoint to compute an estimate of the transformation
				Eigen::Affine3d robotPose;
				m_robot->getCurrentFK(robotPose);
				transformation = robotPose.inverse() * wp.pose;
			}

			gripper->setCurrentObject(objectName);
			gripper->setTGripperToObject(transformation);
		}
	}
	else
	{
		LOG_ERROR("Unknown Command Type: " << type);
	}
}

std::string Executer::getPathFileName() const
{
	EXECUTER_LOCK();
	return m_pathFileName;
}

void Executer::setPathFileName(const std::string& pathFileName)
{
	EXECUTER_LOCK();
	m_pathFileName = pathFileName;
}

bool Executer::isSavePath() const
{
	EXECUTER_LOCK();
	return m_savePath;
}

void Executer::setSavePath(bool savePath)
{
	EXECUTER_LOCK();
	m_savePath = savePath;
}

bool Executer::writeTrajectory(const std::string& filename)
{
	std::ofstream file(filename);

	if (!file.is_open())
	{
		LOG_ERROR("cannot write to " << filename);
		return false;
	}

	for (auto& it : m_executedTrajectory)
	{
		for (int i = 0; i < it.rows(); ++i)
		{
			file << it.data(i);
			if (i < it.data.rows() - 1)
				file << ", ";
		}
		file << "\n";
	}

	file.flush();
	return true;
}

}

/* namespace prm_planner */

