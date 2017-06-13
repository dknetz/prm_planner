/*
 * geometry.h
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#ifndef FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_GEOMETRY_H_
#define FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_GEOMETRY_H_
#include <boost/shared_ptr.hpp>
#include <fcl/collision.h>
#include <fcl/math/transform.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>

namespace fcl_robot_model {

class Geometry {
public:
	Geometry(const std::string& name);
	virtual ~Geometry();

	virtual void getFCLModel(const fcl::Transform3f& transform,
			FCL_POINTER<fcl::CollisionObject>& fclCollisionModel) = 0;

	std::string c_name;
};

} /* namespace fcl_robot_model */

#endif /* FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_GEOMETRY_H_ */
