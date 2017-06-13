/*
 * sphere.h
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#ifndef FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_SPHERE_H_
#define FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_SPHERE_H_

#include <fcl_wrapper/robot_model/geometry.h>

namespace fcl_robot_model
{

class Sphere: public Geometry
{
public:
	Sphere(const std::string& name,
			double radius);
	virtual ~Sphere();

	virtual void getFCLModel(const fcl::Transform3f& transform,
			FCL_POINTER<fcl::CollisionObject>& fclCollisionModel);

		private:
			double m_radius;
		};

	}
	/* namespace fcl_robot_model */

#endif /* FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_COLLISION_DETECTION_SPHERE_H_ */
