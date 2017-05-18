/*
 * This file (world.h) is part of the Scene Analyzer of Daniel Kuhner.
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
#ifndef AlXM0V7pXEqpklU2Okoc
#define AlXM0V7pXEqpklU2Okoc

#include <boost/thread/recursive_mutex.hpp>
#include <fcl_wrapper/collision_detection/collection.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <string>
#include <unordered_map>

namespace fcl_collision_detection
{

class FCLWrapper
{
public:
	typedef std::vector<std::pair<PhysicalObject*, PhysicalObject*>> CollisionsVector;

	FCLWrapper();
	virtual ~FCLWrapper();

	/**
	 * Checks, if an object is already available.
	 *
	 * @return: true, if available
	 */
	bool hasObject(const std::string& name);

	/**
	 * Adds an object. If checkAllCollisions is true,
	 * the corresponding entries in the collision matrix are made
	 */
	void addObject(boost::shared_ptr<PhysicalObject> object,
			bool checkAllCollisions = false);
	void addObject(boost::shared_ptr<Collection> collection);

	void updateObject(boost::shared_ptr<PhysicalObject> object);

	bool checkCollisions(bool findAllCollisions = false);

	void getCollisions(CollisionsVector& collisions);

	bool isUseCollisionMatrix() const;
	void setUseCollisionMatrix(bool useCollisionMatrix);

	void lock();
	void unlock();

protected:
	mutable boost::recursive_mutex m_mutex;
	std::unordered_map<std::string, FCL_POINTER<PhysicalObject>> m_worldObjects;
	CollisionMatrix::Ptr m_collisionMatrix;
	CollisionsVector m_collisions;
	bool m_useCollisionMatrix;
};

} /* namespace fcl_collision_detection */

#endif /* AlXM0V7pXEqpklU2Okoc */
