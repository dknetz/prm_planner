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
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>

using namespace fcl;

namespace fcl_collision_detection
{

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

	std::string name = object->getName();
}

void FCLWrapper::addObject(boost::shared_ptr<Collection> collection)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	const CollisionMatrix::Ptr& collisionMatrix = collection->getCollisionMatrix();
	const ObjectMap& objects = collection->getObjects();

	for (auto& it : objects)
	{
		std::string name = collection->getName() + "/" + it.second->getName();
		if (CHECK_MAP(m_worldObjects, name))
		{
			throw ais_definitions::Exception(std::string("The object ") + name + " already exists!");
		}
		it.second->initFCLModel();
		it.second->setUserData();
		m_worldObjects[name] = it.second;
	}

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

//	m_collisionMatrix->print();

	for (auto it = m_worldObjects.begin(); it != m_worldObjects.end(); ++it)
	{
		auto it2 = it;
		++it2;
		for (; it2 != m_worldObjects.end(); ++it2)
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

					m_collisions.push_back(std::make_pair(it->second.get(), it2->second.get()));

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

void FCLWrapper::lock()
{
	m_mutex.lock();
}

void FCLWrapper::unlock()
{
	m_mutex.unlock();
}

} /* namespace FCLCollisionDetectionWrapper */

