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

#include <actionlib/client/simple_action_client.h>
#include <prm_planner_robot/gripper_interface.h>

#ifdef FOUND_WSG
#include <wsg_gripper/GripperCommandAction.h>
#endif

namespace prm_schunk_wsg
{

class SchunkWSGGripperInterface: public prm_planner::GripperInterface
{
public:
	SchunkWSGGripperInterface();
	virtual ~SchunkWSGGripperInterface();

	virtual void init(GripperInterfaceParameters& parameters);

	virtual bool open();
		virtual bool close();

private:
#ifdef FOUND_WSG
		actionlib::SimpleActionClient<wsg_gripper::GripperCommandAction>* m_actionClient;
#endif
};

} /* namespace prm_schunk_wsg */

#endif /* HCF1D5BA6_E5C7_482B_B196_1DDC6389C2C3 */
