/*
 * box.h
 *
 *  Created on: Aug 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 */

#ifndef FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_BOX_H_
#define FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_BOX_H_

#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/robot_model/geometry.h>

namespace fcl_robot_model
{

class Box: public Geometry
{
public:
	Box(const std::string& name,
			double sizeX,
			double sizeY,
			double sizeZ);
	virtual ~Box();

	virtual void getFCLModel(const fcl::Transform3f& transform,
			FCL_POINTER<fcl::CollisionObject>& fclCollisionModel);

		private:
			double m_sizeX;
			double m_sizeY;
			double m_sizeZ;
		};

	}
	/* namespace fcl_robot_model */

#endif /* FCL_WRAPPER_FCL_WRAPPER_INCLUDE_FCL_WRAPPER_ROBOT_MODEL_BOX_H_ */
