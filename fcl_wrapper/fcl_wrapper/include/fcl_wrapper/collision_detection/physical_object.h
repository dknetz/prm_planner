/*
 * This file (world_object.h) is part of the Scene Analyzer of Daniel Kuhner.
 *
 * It is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
 *
 * created:		Feb 9, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef FEBKGkozBo0KQ0o5F67o
#define FEBKGkozBo0KQ0o5F67o

#include <boost/thread/recursive_mutex.hpp>
#include <fcl/collision.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <Eigen/Geometry>

namespace fcl_collision_detection
{

class PhysicalObject
{
public:
	typedef FCL_POINTER<fcl::CollisionObject> CollisionObjectPtr;

	PhysicalObject();
	PhysicalObject(const std::string& name,
			const std::string& frame,
			const std::string& worldFrame);
	virtual ~PhysicalObject();

	virtual void initFCLModel() = 0;
	virtual CollisionObjectPtr getFCLModel();
	virtual void setUserData();
	virtual bool hasCollision() const;
	virtual void setHasCollision(const bool& hasCollision);
	virtual void setNoCollision();
	const std::string& getName() const;
	void setName(const std::string& name);

	/**
	 * Set transformation to world by hand
	 */
	void setTransform(const Eigen::Affine3d& tf);
	void setTransform(const fcl::Transform3f& tf);

	const std::string& getFrame() const;
	const std::string& getWorldFrame() const;

	/**
	 * Set transformation to world via
	 */
	void updateTransformRosTF();

protected:
	mutable boost::recursive_mutex m_mutex;
	CollisionObjectPtr m_fclCollisionObject;
	bool m_hasCollision;
	fcl::Transform3f m_tf;

	std::string m_name;
	std::string m_frame;
	std::string m_worldFrame;
};

} /* namespace fcl_collision_detection */

#endif /* FEBKGkozBo0KQ0o5F67o */
