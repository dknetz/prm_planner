/*
 * cylinder.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#include <fcl/collision.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl_wrapper/robot_model/cylinder.h>

namespace fcl_robot_model
{

Cylinder::Cylinder(const std::string& name,
		double radius,
		double height) :
				Geometry(name),
				m_radius(radius),
				m_height(height)
{

}

Cylinder::~Cylinder()
{
}

void Cylinder::getFCLModel(const fcl::Transform3f& transform,
		FCL_POINTER<fcl::CollisionObject>& fclCollisionModel)
		{
			FCL_POINTER<fcl::Cylinder> model(new fcl::Cylinder(m_radius, m_height));
			model->setUserData(&c_name);
			fclCollisionModel.reset(new fcl::CollisionObject(model, transform));
		}

	}
	/* namespace fcl_robot_model */
