/*
 * This file (robot_joint.h) is part of the Scene Analyzer of Daniel Kuhner.
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
#ifndef zrR713FN6TiHNgEaVGQh
#define zrR713FN6TiHNgEaVGQh

#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <Eigen/Geometry>
#include <unordered_map>

namespace fcl_robot_model
{

class RobotLink;

class RobotJoint
{
public:
	typedef std::pair<std::string, FCL_WEAK_POINTER<RobotLink>> LinkWeakPair;
	typedef std::pair<std::string, FCL_POINTER<RobotLink>> LinkPair;

public:
	enum Type
	{
		Revolute,
		Prismatic,
		Fixed
	};

	RobotJoint(const std::string& name,
			const Eigen::Vector3d& axis,
			const Type& type,
			const double lowerLimit,
			const double upperLimit,
			const double speedLimit,
			const Eigen::Affine3d& transformationToParentLink);
	virtual ~RobotJoint();

	void setParentLink(FCL_POINTER<RobotLink>& parent);
	void setChildLink(FCL_POINTER<RobotLink>& child);
	LinkPair getParentLink();
	LinkPair getChildLink();
	Eigen::Affine3d getStaticTransformationToParent() const;
	Eigen::Affine3d getTransformationToParent() const;
	Eigen::Affine3d getTransformationToParent(const double angle) const;

	std::string getName() const;

	void setJointValue(const double jointValue);
	double getJointValue() const;

	double getUpperJointLimit() const;
	double getLowerJointLimit() const;
	double getVelocityLimit() const;
	bool isInJointLimit(const double angle) const;
	void print(int depth = 0) const;

private:
	mutable boost::recursive_mutex m_mutex;
	const std::string m_name;
	LinkWeakPair m_parentLink;
	LinkWeakPair m_childLink;
	Eigen::Vector3d m_axis;
	const Eigen::Affine3d c_transformationToParentLink;
	Eigen::Affine3d m_transformationJoint;
	Eigen::Affine3d m_transformationToParentLink;
	double m_jointValue;
	double m_upperLimit;
	double m_lowerLimit;
	double m_velocitiyLimit;
	Type m_type;

public:
	friend class RobotModel;
};

}
/* namespace robot_model */

#endif /* zrR713FN6TiHNgEaVGQh */
