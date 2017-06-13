/*
 * This file (octomap.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 11, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */

#include <ais_log/log.h>
#include <fcl/collision.h>
#include <fcl_wrapper/collision_detection/octomap.h>
#include <octomap/math/Quaternion.h>

namespace fcl_collision_detection
{
Octomap::Octomap(boost::shared_ptr<octomap::OcTree> octomap,
		const std::string& name,
		const std::string& frame,
		const std::string& worldFrame) :
				PhysicalObject(name, frame, worldFrame),
				m_octree(octomap),
				m_fclOctree(new fcl::OcTree(CONVERT_TO_STD_POINTER(m_octree)))
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_fclCollisionObject.reset(new fcl::CollisionObject(m_fclOctree, fcl::Transform3f()));
}

Octomap::~Octomap()
{
}

void Octomap::updateOctomap(boost::shared_ptr<octomap::OcTree> octomap)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_octree = octomap;

	FCL_POINTER<fcl::OcTree> fclOctree(new fcl::OcTree(CONVERT_TO_STD_POINTER(m_octree)));
	fclOctree->setUserData(&m_name);
	CollisionObjectPtr collisionObject(new fcl::CollisionObject(fclOctree, fcl::Transform3f()));

	m_fclOctree = fclOctree;
	m_fclCollisionObject = collisionObject;
}

void Octomap::initFCLModel()
{
}

}

