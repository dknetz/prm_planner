/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: planning_object.h
 */

#ifndef H7FC96360_FADD_40B6_8B9C_52390673250A
#define H7FC96360_FADD_40B6_8B9C_52390673250A

#include <ais_definitions/class.h>
#include <ais_util/color.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <prm_planner_msgs/CollisionObject.h>
#include <Eigen/Geometry>

FORWARD_DECLARE_N(ais_util, Color);

namespace prm_planner
{

class PlanningObject
{
public:
	enum Type {
		BOX,
		SPHERE,
		PLANE
	};

public:
	PlanningObject(const prm_planner_msgs::CollisionObject& msg);
	virtual ~PlanningObject();

public:
	Type m_type;
	ais_util::Color m_color;
	std::vector<double> m_size;
	Eigen::Affine3d m_transformation;
	std::string m_name;
	std::string m_frame;
	fcl_collision_detection::CollisionMatrix m_collisionMatrix;
};

} /* namespace prm_planner */

std::ostream& operator<<(std::ostream& stream, const prm_planner::PlanningObject& o);

#endif /* H7FC96360_FADD_40B6_8B9C_52390673250A */
