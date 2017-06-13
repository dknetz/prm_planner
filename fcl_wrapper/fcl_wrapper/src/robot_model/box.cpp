/*
 * box.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#include <boost/shared_ptr.hpp>
#include <fcl/collision.h>
#include <fcl_wrapper/robot_model/box.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl_wrapper/collision_detection/common.h>

using fcl::CollisionObject;

namespace fcl_robot_model
{

Box::Box(const std::string& name,
		double sizeX,
		double sizeY,
		double sizeZ) :
				Geometry(name),
				m_sizeX(sizeX),
				m_sizeY(sizeY),
				m_sizeZ(sizeZ)
{
}

Box::~Box()
{
}

void Box::getFCLModel(const fcl::Transform3f& transform,
		FCL_POINTER<fcl::CollisionObject>& fclCollisionModel)
		{
			FCL_POINTER<fcl::Box> model(new fcl::Box(m_sizeX, m_sizeY, m_sizeZ));
			model->setUserData(&c_name);
			fclCollisionModel.reset(new fcl::CollisionObject(model, transform));
		}

	}
	/* namespace fcl_robot_model */
