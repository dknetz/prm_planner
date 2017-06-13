/*
 * This file (robot_joint.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 4, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */

#include <ais_log/log.h>
#include <fcl_wrapper/robot_model/robot_joint.h>
#include <fcl_wrapper/robot_model/robot_link.h>

namespace fcl_robot_model
{

RobotJoint::RobotJoint(const std::string& name,
		const Eigen::Vector3d& axis,
		const Type& type,
		const double lowerLimit,
		const double upperLimit,
		const double speedLimit,
		const Eigen::Affine3d& transformationToParentLink) :
				m_name(name),
				m_axis(axis),
				c_transformationToParentLink(transformationToParentLink),
				m_transformationJoint(Eigen::Affine3d::Identity()),
				m_transformationToParentLink(Eigen::Affine3d::Identity()),
				m_jointValue(0),
				m_lowerLimit(lowerLimit),
				m_upperLimit(upperLimit),
				m_velocitiyLimit(speedLimit),
				m_type(type)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	setJointValue(m_jointValue);
}

RobotJoint::~RobotJoint()
{
}

void RobotJoint::setParentLink(FCL_POINTER<RobotLink>& parent)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_parentLink.first = parent->getName();
	m_parentLink.second = parent;
}

void RobotJoint::setChildLink(FCL_POINTER<RobotLink>& child)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_childLink.first = child->getName();
	m_childLink.second = child;
}

RobotJoint::LinkPair RobotJoint::getParentLink()
{
	return std::make_pair(m_parentLink.first, FCL_POINTER<RobotLink>(m_parentLink.second));
}

RobotJoint::LinkPair RobotJoint::getChildLink()
{
	return std::make_pair(m_childLink.first, FCL_POINTER<RobotLink>(m_childLink.second));
}

Eigen::Affine3d RobotJoint::getStaticTransformationToParent() const
{
	return c_transformationToParentLink;
}

std::string RobotJoint::getName() const
{
	return m_name;
}

Eigen::Affine3d RobotJoint::getTransformationToParent() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_transformationToParentLink;
}

Eigen::Affine3d RobotJoint::getTransformationToParent(const double angle) const
		{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Eigen::Affine3d joint;
	joint.setIdentity();
	if (m_type == Fixed)
	{
		//do nothing
	}
	else if (m_type == Prismatic)
	{
		joint.translation() = m_axis * angle;
	}
	else
	{
		joint.linear() = Eigen::AngleAxisd(angle, m_axis).toRotationMatrix();
	}
	return c_transformationToParentLink * joint;
}

void RobotJoint::setJointValue(const double jointValue)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (m_type == Fixed)
	{
		//do nothing
	}
	else if (m_type == Prismatic)
	{
		m_transformationJoint.translation() = m_axis * jointValue;
	}
	else
	{
		m_transformationJoint.linear() = Eigen::AngleAxisd(jointValue, m_axis).toRotationMatrix();
	}

	m_transformationToParentLink = c_transformationToParentLink * m_transformationJoint;
	m_jointValue = jointValue;
}

double RobotJoint::getJointValue() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_jointValue;
}

double RobotJoint::getUpperJointLimit() const
{
	return m_upperLimit;
}

double RobotJoint::getLowerJointLimit() const
{
	return m_lowerLimit;
}

double RobotJoint::getVelocityLimit() const
{
	return m_velocitiyLimit;
}

bool RobotJoint::isInJointLimit(const double angle) const
		{
	return angle <= m_upperLimit && angle >= m_lowerLimit;
}

void RobotJoint::print(int depth) const
		{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	auto ptrParent = m_parentLink.second.lock();
	auto ptrChild = m_childLink.second.lock();
	std::string tabs(depth, '\t');
	LOG_INFO(tabs<<"-JOINT-----------------------------");
	LOG_INFO(tabs<<"name: " << m_name);
	LOG_INFO(tabs<<"type: " << m_type);
	LOG_INFO(tabs<<"jointValue: " << m_jointValue);
	LOG_INFO(tabs<<"translation: " << Eigen::Vector3d(c_transformationToParentLink.translation()).transpose());
	Eigen::Vector3d angles = c_transformationToParentLink.rotation().eulerAngles(0, 1, 2);
	LOG_INFO(tabs<<"rotation: " << angles.transpose());
	LOG_INFO(tabs<<"parent link: " << ptrParent->getName());
	LOG_INFO(tabs<<"child link: " << ptrChild->getName());
	ptrChild->print(depth + 1);
	LOG_INFO(tabs<<"-----------------------------------");
}

} /* namespace robot_model */

