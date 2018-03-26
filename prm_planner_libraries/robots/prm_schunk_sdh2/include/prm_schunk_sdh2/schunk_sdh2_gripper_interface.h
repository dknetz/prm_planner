/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: gripper_sdh.h
 */

#ifndef HCF1D5BA6_E5C7_482B_B196_1DDC6389C2C3
#define HCF1D5BA6_E5C7_482B_B196_1DDC6389C2C3

#include <prm_planner_robot/gripper_interface.h>
#include <ros/ros.h>

namespace prm_schunk_sdh2
{

class SchunkSDH2GripperInterface: public prm_planner::GripperInterface
{
public:
	SchunkSDH2GripperInterface();
	virtual ~SchunkSDH2GripperInterface();

	virtual void init(GripperInterfaceParameters& parameters);

	virtual bool open();
	virtual bool close();

private:
	ros::ServiceClient m_service;
};

} /* namespace prm_schunk_sdh2 */

#endif /* HCF1D5BA6_E5C7_482B_B196_1DDC6389C2C3 */
