/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: box.cpp
 */

#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl_wrapper/collision_detection/box.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>

namespace fcl_collision_detection
{

Box::Box(double sizeX,
		double sizeY,
		double sizeZ,
		const std::string& name,
		const std::string& frame,
		const std::string& worldFrame) :
				PhysicalObject(name, frame, worldFrame),
				m_sizeX(sizeX),
				m_sizeY(sizeY),
				m_sizeZ(sizeZ)
{
}

Box::~Box()
{
}

void Box::initFCLModel()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	FCL_POINTER<fcl::Box> model(new fcl::Box(m_sizeX, m_sizeY, m_sizeZ));
	model->setUserData(&m_name);
	m_fclCollisionObject.reset(new fcl::CollisionObject(model, m_tf));
}

}
/* namespace fcl_collision_detection */
