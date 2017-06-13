/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: sphere.cpp
 */

#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/sphere.h>

namespace fcl_collision_detection
{

Sphere::Sphere(double radius,
		const std::string& name,
		const std::string& frame,
		const std::string& worldFrame) :
				PhysicalObject(name, frame, worldFrame),
				m_radius(radius)
{
}

Sphere::~Sphere()
{
}

void Sphere::initFCLModel()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	FCL_POINTER<fcl::Sphere> model(new fcl::Sphere(m_radius));
	model->setUserData(&m_name);
	m_fclCollisionObject.reset(new fcl::CollisionObject(model, m_tf));
}

} /* namespace fcl_collision_detection */
