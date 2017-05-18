/*
 * This file (common.h) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 6, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef qM4TUJzoQjpU0QFXtvtB
#define qM4TUJzoQjpU0QFXtvtB
#include <fcl/math/transform.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <Eigen/Geometry>
#include <unordered_map>

namespace fcl_collision_detection
{
typedef std::unordered_map<std::string, FCL_POINTER<PhysicalObject>> ObjectMap;

inline void transform2fcl(const Eigen::Affine3d &b,
		fcl::Transform3f &f)
{
	Eigen::Quaterniond q(b.rotation());
	f.setTranslation(fcl::Vec3f(b.translation().x(), b.translation().y(), b.translation().z()));
	f.setQuatRotation(fcl::Quaternion3f(q.w(), q.x(), q.y(), q.z()));
}

inline fcl::Transform3f transform2fcl(const Eigen::Affine3d &b)
{
	fcl::Transform3f t;
	transform2fcl(b, t);
	return t;
}
}

#endif /* qM4TUJzoQjpU0QFXtvtB */
