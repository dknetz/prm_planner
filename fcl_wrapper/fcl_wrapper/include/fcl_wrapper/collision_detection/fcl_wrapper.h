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
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/collision_data.h>
#include <fcl_wrapper/collision_detection/collection.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <string>
#include <unordered_map>

namespace fcl_collision_detection
{

class FCLWrapper
{
public:
	typedef std::vector<std::pair<std::string, std::string>> CollisionsVector;
	typedef std::vector<std::pair<PhysicalObject*, PhysicalObject*>> CollisionsVectorObjects;

	struct CollisionData
	{
		CollisionData()
		{
			done = false;
		}

		/// @brief Collision request
		fcl::CollisionRequest request;

		/// @brief Collision result
		fcl::CollisionResult result;

		// collision matrix
		CollisionMatrix::Ptr cm;

		/// @brief Whether the collision iteration can stop
		bool done;
	};

	FCLWrapper(const std::string& worldFrame);
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
	bool checkPairwiseCollisions(bool findAllCollisions = false);

	void getCollisions(CollisionsVector& collisions);

	/**
	 * Only call this method, if you called checkPairwiseCollisions before!
	 */
	void getPairwiseCollisions(CollisionsVectorObjects& collisions);

	bool isUseCollisionMatrix() const;
	void setUseCollisionMatrix(bool useCollisionMatrix);

	void lock();
	void unlock();

	void publish(fcl::DynamicAABBTreeCollisionManager* collisionManager);

private:
	void updateCollisionsVector(std::vector<fcl::Contact>& contacts,
			const std::string& object1,
			const std::string& object2);

protected:
	mutable boost::recursive_mutex m_mutex;
	std::unordered_map<std::string, FCL_POINTER<PhysicalObject>> m_worldObjects;
	std::unordered_map<std::string, FCL_POINTER<Collection>> m_objectHierarchies;
	CollisionsVector m_collisions;
	CollisionsVectorObjects m_collisionsObjects;
	bool m_useCollisionMatrix;
	const std::string c_worldFrame;
	static ros::NodeHandle* m_nodeHandle;
	static ros::Publisher m_pubAABB;
};

}
/* namespace fcl_collision_detection */

#endif /* AlXM0V7pXEqpklU2Okoc */
