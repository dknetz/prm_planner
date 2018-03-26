/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: gripper_sdh.cpp
 */

#include <prm_schunk_sdh2/schunk_sdh2_gripper_interface.h>
#include <ais_log/log.h>
#include <pluginlib/class_list_macros.h>

#ifdef FOUND_SDH
#include <sdh2_hand/SDHAction.h>
#endif

namespace prm_schunk_sdh2
{

SchunkSDH2GripperInterface::SchunkSDH2GripperInterface() :
				prm_planner::GripperInterface()
{
}

SchunkSDH2GripperInterface::~SchunkSDH2GripperInterface()
{
}

bool SchunkSDH2GripperInterface::open()
{
#ifdef FOUND_SDH
	sdh2_hand::SDHAction srv;
	srv.request.ratio = 0.3;
	srv.request.velocity = 0.7;
	srv.request.type = sdh2_hand::SDHActionRequest::PARALLEL;
	srv.request.gripType = sdh2_hand::SDHActionRequest::NOSTOP;

	return m_service.call(srv) && srv.response.result;
#else
	LOG_ERROR("You need to compile the SDH2 Gripper Interface with available sdh2_hand package");
	return false;
#endif
}

void SchunkSDH2GripperInterface::init(GripperInterfaceParameters& parameters)
{
#ifdef FOUND_SDH
	GripperInterface::init(parameters);
	ros::NodeHandle n;
	m_service = n.serviceClient<sdh2_hand::SDHAction>(c_parameters.topic);
#else
	LOG_ERROR("You need to compile the SDH2 Gripper Interface with available sdh2_hand package");
#endif
}

bool SchunkSDH2GripperInterface::close()
{
#ifdef FOUND_SDH
	sdh2_hand::SDHAction srv;
	srv.request.ratio = 0.9;
	srv.request.velocity = 0.3;
	srv.request.type = sdh2_hand::SDHActionRequest::PARALLEL;
	srv.request.gripType = sdh2_hand::SDHActionRequest::STOPFINGER;

	return m_service.call(srv) && srv.response.result;
#else
	LOG_ERROR("You need to compile the SDH2 Gripper Interface with available sdh2_hand package");
	return false;
#endif
}

} /* namespace prm_schunk_sdh2 */

PLUGINLIB_EXPORT_CLASS(prm_schunk_sdh2::SchunkSDH2GripperInterface, prm_planner::GripperInterface)
