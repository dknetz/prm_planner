/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jun 7, 2017
 *      Author: kuhnerd
 * 	  Filename: ik.cpp
 */

#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner_robot/kinematics.h>
#include <prm_planner_robot/robot_arm.h>

namespace prm_planner
{

pluginlib::ClassLoader<Kinematics>* Kinematics::s_loader = NULL;

Kinematics::Kinematics()

{
}

Kinematics::~Kinematics()
{
}

bool Kinematics::getFK(const KDL::JntArray& jointPosition,
		Eigen::Affine3d& pose,
		int segmentNR)
{
	KDL::Frame x;
	if (getFK(jointPosition, x, segmentNR))
	{
		tf::transformKDLToEigen(x, pose);
		return true;
	}
	else
	{
		return false;
	}
}

boost::shared_ptr<Kinematics> Kinematics::load(const std::string& package,
		const std::string& library)
{
	//create instance if not already available
	if (s_loader == NULL)
		s_loader = new pluginlib::ClassLoader<Kinematics>("prm_planner", "prm_planner::Kinematics");

	boost::shared_ptr<Kinematics> interface;

	try
	{
		interface = s_loader->createInstance(library);
	}
	catch (pluginlib::PluginlibException& ex)
	{
		LOG_FATAL("The plugin failed to load for some reason. Error: " << ex.what());
		exit(3);
	}

	return interface;
}

} /* namespace prm_planner */

