/*
 * This file (FCLCollisionDetectionWrapper.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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

#include <ais_definitions/macros.h>
#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <fcl/collision.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>

using namespace fcl;

namespace fcl_collision_detection
{

bool defaultCollisionFunction(fcl::CollisionObject* o1,
		fcl::CollisionObject* o2,
		void* cdata_)
{
	auto* cdata = static_cast<FCLWrapper::CollisionData*>(cdata_);
	const auto& request = cdata->request;
	auto& result = cdata->result;

	PhysicalObject* po1 = (PhysicalObject*) o1->getUserData();
	PhysicalObject* po2 = (PhysicalObject*) o2->getUserData();

	if (cdata->done)
		return true;

	if (cdata->cm.get() != NULL)
	{
		if (!cdata->cm->collide(po1->getName(), po2->getName()))
		{
			return true;
		}
	}

	collide(o1, o2, request, result);

	if (!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
		cdata->done = true;

	return cdata->done;
}

FCLWrapper::FCLWrapper() :
				m_collisionMatrix(new CollisionMatrix),
				m_useCollisionMatrix(true)
{
}

FCLWrapper::~FCLWrapper()
{
}

bool FCLWrapper::hasObject(const std::string& name)
{
	return CHECK_MAP(m_worldObjects, name);
}

void FCLWrapper::addObject(boost::shared_ptr<PhysicalObject> object,
		bool checkAllCollisions)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	object->initFCLModel();
	object->setUserData();

	std::string name = object->getName();

	if (CHECK_MAP(m_worldObjects, name))
	{
		throw ais_definitions::Exception(std::string("The object already exists in the FCLWrapper"));
	}

	//collision checks against all links
	if (checkAllCollisions)
	{
		m_collisionMatrix->setDoAllCollisionChecks(name, true);
	}

	m_worldObjects[name] = CONVERT_TO_STD_POINTER(object);
}

void FCLWrapper::updateObject(boost::shared_ptr<PhysicalObject> object)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (!CHECK_MAP(m_worldObjects, object->getName()))
	{
		throw ais_definitions::Exception(std::string("You cannot update an object, which was not added so far!"));
	}

	object->initFCLModel();
	object->setUserData();
}

void FCLWrapper::addObject(boost::shared_ptr<Collection> collection)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	const CollisionMatrix::Ptr& collisionMatrix = collection->getCollisionMatrix();
	const ObjectMap& objects = collection->getObjects();

	for (auto& it : objects)
	{
		it.second->initFCLModel();
		it.second->setUserData();
	}

	m_objectHierarchies[collection->getName()] = CONVERT_TO_STD_POINTER(collection);

//	for (auto& it : objects)
//	{
//		std::string name = collection->getName() + "/" + it.second->getName();
//		if (CHECK_MAP(m_worldObjects, name))
//		{
//			throw ais_definitions::Exception(std::string("The object ") + name + " already exists!");
//		}
//		it.second->initFCLModel();
//		it.second->setUserData();
//		m_worldObjects[name] = it.second;
//	}

	if (collisionMatrix.get() != NULL)
	{
		m_collisionMatrix->addOtherCollisionMatrix(collisionMatrix);
	}
}

bool FCLWrapper::checkCollisions(bool findAllCollisions)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	for (auto it : m_worldObjects)
	{
		it.second->setNoCollision();
	}

	m_collisions.clear();

	//check collisions:
	//1. Collisions between world objects (single objects) and hierarchies
	//2. Collisions between hierarchies
	//3. Collisions of objects in hierarchies

	for (auto it = m_objectHierarchies.begin(); it != m_objectHierarchies.end(); ++it)
	{
		fcl::DynamicAABBTreeCollisionManager* hierarchy = new fcl::DynamicAABBTreeCollisionManager();
		std::vector<fcl::CollisionObject*> objects;
		objects.reserve(it->second->getObjects().size());
		const ObjectMap& om = it->second->getObjects();
		for (auto& object : om)
		{
			objects.push_back(object.second->getFCLModel().get());
		}
		hierarchy->registerObjects(objects);
		hierarchy->setup();

		//check collision between hierarchy and object
		for (auto it2 = m_worldObjects.begin(); it2 != m_worldObjects.end(); ++it2)
		{
			PhysicalObject::CollisionObjectPtr object = it2->second->getFCLModel();

			CollisionData data;
			data.request.num_max_contacts = 1;

			if (object.get() == NULL)
				continue;

//			LOG_INFO("Coll Check " << it->first << " " << it2->first);
//
//			LOG_INFO("h " << hierarchy->getTree().getRoot()->bv.min_ << " " << hierarchy->getTree().getRoot()->bv.max_);
//			LOG_INFO("o " << object->getAABB().min_ << " " << object->getAABB().max_);

			hierarchy->collide(object.get(), &data, defaultCollisionFunction);

			if (data.result.isCollision())
			{
				std::vector<fcl::Contact> contacts;
				data.result.getContacts(contacts);
				updateCollisionsVector(contacts, it->second->getName(), it2->second->getName());

				if (!findAllCollisions)
				{
					delete hierarchy;
					return true;
				}
			}
		}

		//check collision between hierarchy and hierarchy
		auto it2 = it;
		++it2;
		for (; it2 != m_objectHierarchies.end(); ++it2)
		{
			//TODO: Not optimal, some of the hierarchies are generated multiple times
			fcl::BroadPhaseCollisionManager* hierarchy2 = new fcl::DynamicAABBTreeCollisionManager();
			for (auto& object : it2->second->getObjects())
			{
				hierarchy2->registerObject(object.second->getFCLModel().get());
			}
			hierarchy2->setup();

			CollisionData data;
			data.request.num_max_contacts = 1;

			hierarchy->collide(hierarchy2, &data, defaultCollisionFunction);

			if (data.result.isCollision())
			{
				std::vector<fcl::Contact> contacts;
				data.result.getContacts(contacts);
				updateCollisionsVector(contacts, it->second->getName(), it2->second->getName());

				if (!findAllCollisions)
				{
					delete hierarchy;
					delete hierarchy2;
					return true;
				}
			}

			delete hierarchy2;
		}

		//check self collisions
		CollisionData data;
		data.request.num_max_contacts = 1;
		if (m_useCollisionMatrix)
			data.cm = m_collisionMatrix;
		hierarchy->collide(&data, defaultCollisionFunction);

		if (data.result.isCollision())
		{
			std::vector<fcl::Contact> contacts;
			data.result.getContacts(contacts);
			updateCollisionsVector(contacts, it->second->getName(), it->second->getName());

			if (!findAllCollisions)
			{
				delete hierarchy;
				return true;
			}
		}

		delete hierarchy;
	}

	return m_collisions.size() != 0;
}

bool FCLWrapper::checkPairwiseCollisions(bool findAllCollisions)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	for (auto it : m_worldObjects)
	{
		it.second->setNoCollision();
	}

	m_collisions.clear();

	std::unordered_map<std::string, FCL_POINTER<PhysicalObject>> worldObjects = m_worldObjects;

	for (auto it = m_objectHierarchies.begin(); it != m_objectHierarchies.end(); ++it)
	{
		const ObjectMap& objects = it->second->getObjects();

		for (auto& it2 : objects)
		{
			std::string name = it->second->getName() + "/" + it2.second->getName();
			if (CHECK_MAP(m_worldObjects, name))
			{
				throw ais_definitions::Exception(std::string("The object ") + name + " already exists!");
			}
			worldObjects[name] = it2.second;
		}
	}

	for (auto it = worldObjects.begin(); it != worldObjects.end(); ++it)
	{
		auto it2 = it;
		++it2;
		for (; it2 != worldObjects.end(); ++it2)
		{
			if (!m_useCollisionMatrix || m_collisionMatrix->collide(it->first, it2->first))
			{
				PhysicalObject::CollisionObjectPtr c1 = it->second->getFCLModel();
				PhysicalObject::CollisionObjectPtr c2 = it2->second->getFCLModel();

//				LOG_INFO("checking collision between " << it->second->getName() << " and " << it2->second->getName());

				if (c1.get() == NULL || c2.get() == NULL)
				{
					continue;
				}

				CollisionRequest request;
				CollisionResult result;

				collide(c1.get(), c2.get(), request, result);

				if (result.isCollision())
				{
//					LOG_INFO("collision between " << it->second->getName() << " and " << it2->second->getName());

					it->second->setHasCollision(true);
					it2->second->setHasCollision(true);

					m_collisionsObjects.push_back(std::make_pair(it->second.get(), it2->second.get()));

					if (!findAllCollisions)
					{
						return m_collisions.size() != 0;;
					}
				}
			}
		}
	}

	return m_collisions.size() != 0;
}

bool FCLWrapper::isUseCollisionMatrix() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_useCollisionMatrix;
}

void FCLWrapper::setUseCollisionMatrix(bool useCollisionMatrix)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_useCollisionMatrix = useCollisionMatrix;
}

void FCLWrapper::getCollisions(FCLWrapper::CollisionsVector& collisions)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	collisions = m_collisions;
}

void FCLWrapper::getPairwiseCollisions(FCLWrapper::CollisionsVectorObjects& collisions)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	collisions = m_collisionsObjects;
}

void FCLWrapper::lock()
{
	m_mutex.lock();
}

void FCLWrapper::unlock()
{
	m_mutex.unlock();
}

void FCLWrapper::updateCollisionsVector(std::vector<fcl::Contact>& contacts,
		const std::string& object1,
		const std::string& object2)
{
//	for (auto& contact : contacts)
//	{
//		PhysicalObject* o1 = (PhysicalObject*) contact.o1->getUserData();
//		PhysicalObject* o2 = (PhysicalObject*) contact.o2->getUserData();
//
//		if (o1 != NULL && o2 != NULL)
//		{
//			LOG_INFO("collision between " << o1->getName() << " and " << o2->getName());
//			o1->setHasCollision(true);
//			o2->setHasCollision(true);
//			m_collisions.push_back(std::make_pair(o1->getName(), o2->getName()));
//		}
//		else if (o1 != NULL)
//		{
//			LOG_INFO("collision of " << o1->getName());
//			o1->setHasCollision(true);
//			m_collisions.push_back(std::make_pair(o1->getName(), "unknown"));
//		}
//		else if (o2 != NULL)
//		{
//			LOG_INFO("collision of between " << object << " and " << o2->getName());
//			o2->setHasCollision(true);
//			m_collisions.push_back(std::make_pair(object, o2->getName()));
//		}
//		else
//		{
//			LOG_INFO("found a collision between " << object1 << " and " << object2);
			m_collisions.push_back(std::make_pair(object1, object2));
//		}
//	}
}

} /* namespace FCLCollisionDetectionWrapper */

