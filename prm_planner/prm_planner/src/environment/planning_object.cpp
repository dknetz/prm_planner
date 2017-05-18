/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: planning_object.cpp
 */

#include <ais_util/color.h>
#include <eigen_conversions/eigen_msg.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <prm_planner/environment/planning_object.h>

namespace prm_planner
{

PlanningObject::PlanningObject(const prm_planner_msgs::CollisionObject& msg) :
				m_type((Type) msg.type),
				m_color(msg.color.r, msg.color.g, msg.color.b, msg.color.a),
				m_size(msg.size),
				m_name(msg.name),
				m_frame(msg.parent_frame),
				m_collisionMatrix(msg.name, msg.allowedCollisions)
{
	tf::transformMsgToEigen(msg.transformation.transform, m_transformation);
}

PlanningObject::~PlanningObject()
{
}

} /* namespace prm_planner */

std::ostream& operator <<(std::ostream& stream,
		const prm_planner::PlanningObject& o)
{
	stream << "\n\tPlanningObject:\n" <<
			"\t\tname: " << o.m_name << "\n"
			"\t\ttype: " << o.m_type;
	return stream;
}
