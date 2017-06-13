/*
 * geometry.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#include <fcl_wrapper/robot_model/geometry.h>

namespace fcl_robot_model
{

Geometry::Geometry(const std::string& name) :
				c_name(name)
{
}

Geometry::~Geometry()
{
}

} /* namespace fcl_robot_model */
