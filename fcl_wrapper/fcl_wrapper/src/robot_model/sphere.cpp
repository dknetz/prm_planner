/*
 * sphere.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl_wrapper/robot_model/sphere.h>

namespace fcl_robot_model
{

Sphere::Sphere(const std::string& name,
		double radius) :
				Geometry(name),
				m_radius(radius)
{
}

Sphere::~Sphere()
{
}

void Sphere::getFCLModel(const fcl::Transform3f& transform,
		FCL_POINTER<fcl::CollisionObject>& fclCollisionModel)
		{
			FCL_POINTER<fcl::Sphere> model(new fcl::Sphere(m_radius));
			model->setUserData(&c_name);
			fclCollisionModel.reset(new fcl::CollisionObject(model, transform));
		}

	}
	/* namespace fcl_robot_model */
