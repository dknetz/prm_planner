/*
 * This file (physical_object.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
#include <ais_log/log.h>
#include <ais_ros/ros_base_interface.h>
#include <fcl_wrapper/collision_detection/common.h>
#include <fcl_wrapper/collision_detection/physical_object.h>

namespace fcl_collision_detection
{

PhysicalObject::PhysicalObject() :
				m_hasCollision(false)
{
}

PhysicalObject::PhysicalObject(const std::string& name,
		const std::string& frame,
		const std::string& worldFrame) :
				m_hasCollision(false),
				m_name(name),
				m_frame(frame),
				m_worldFrame(worldFrame)
{
//	LOG_INFO("init object " << name << " with " << m_frame << " " << m_worldFrame);
}

PhysicalObject::~PhysicalObject()
{
}

void PhysicalObject::setUserData()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (m_fclCollisionObject.get() != NULL)
	{
		m_fclCollisionObject->setUserData(this);
	}
}

PhysicalObject::CollisionObjectPtr PhysicalObject::getFCLModel()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	//update tf if frames are available
	if (!m_worldFrame.empty() && !m_frame.empty())
	{
		updateTransformRosTF();
	}

	m_fclCollisionObject->setTransform(m_tf);
	m_fclCollisionObject->computeAABB();
	return m_fclCollisionObject;
}

bool PhysicalObject::hasCollision() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_hasCollision;
}

void PhysicalObject::setHasCollision(const bool& hasCollision)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_hasCollision = hasCollision;
}

void PhysicalObject::setNoCollision()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_hasCollision = false;
}

const std::string& PhysicalObject::getName() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_name;
}

void PhysicalObject::setName(const std::string& name)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_name = name;
}

void PhysicalObject::setTransform(const Eigen::Affine3d& tf)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_tf = transform2fcl(tf);
}

void PhysicalObject::setTransform(const fcl::Transform3f& tf)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_tf = tf;
}

const std::string& PhysicalObject::getFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_frame;
}

const std::string& PhysicalObject::getWorldFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_worldFrame;
}

void PhysicalObject::updateTransformRosTF()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_tf = transform2fcl(ais_ros::RosBaseInterface::getRosTransformation(m_frame, m_worldFrame));
}

} /* namespace world */

