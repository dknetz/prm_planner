/*
 * object_manager.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: kuhnerd
 */

#include <ais_definitions/macros.h>
#include <ais_ros/ros_base_interface.h>
#include <prm_planner/objects/object_manager.h>

namespace prm_planner
{

SINGLETON_SOURCE(ObjectManager)

ObjectManager::ObjectManager()
{
}

ObjectManager::~ObjectManager()
{
}

void ObjectManager::loadObjects(std::unordered_map<std::string, parameters::ObjectConfig>& params,
		const parameters::DroppingConfig& droppingConfig,
		double octomapResolution,
		const std::string& frame)
{
	boost::mutex::scoped_lock lock(m_mutex);
	for (auto& it : params)
	{
		GraspableObject::ObjectParameters op;
		op.frame = frame;
		op.objectFrameName = it.second.frameName;
		op.meshFilename = it.second.file;
		op.name = it.second.name;
		op.octomapResolution = octomapResolution;

		boost::shared_ptr<GraspableObject> object(new GraspableObject(op, droppingConfig));
		m_objects[it.first] = object;
//		m_remove[it.first] = false;
	}
}

void ObjectManager::addObject(boost::shared_ptr<GraspableObject> object)
{
	boost::mutex::scoped_lock lock(m_mutex);
	m_objects[object->c_params.name] = object;
//	m_remove[object->c_params.name] = false;
}

void ObjectManager::publish()
{
	boost::mutex::scoped_lock lock(m_mutex);
	for (auto& it : m_objects)
	{
		it.second->publish();
	}
}

boost::shared_ptr<GraspableObject> ObjectManager::getObject(const std::string& name)
{
	boost::mutex::scoped_lock lock(m_mutex);
	if (CHECK_MAP(m_objects, name))
	{
		return m_objects[name];
	}
	else
	{
		return boost::shared_ptr<GraspableObject>();
	}
}

std::unordered_map<std::string, boost::shared_ptr<GraspableObject> >& ObjectManager::getObjects()
{
	return m_objects;
}

void ObjectManager::updatePosesFromTF()
{
	boost::mutex::scoped_lock lock(m_mutex);
	for (auto& it : m_objects)
	{
		if (it.second->isDoUpdate())
		{
			it.second->updatePoseFromTF();
			it.second->updateBoundingBox();
		}
	}
}

void ObjectManager::setNeglectObject(const std::vector<std::string>& neglectedObjects)
{
	boost::mutex::scoped_lock lock(m_mutex);
	LOG_INFO("Setting object updates:");
	for (auto& it : m_objects)
	{
		bool update = std::find(neglectedObjects.begin(), neglectedObjects.end(), it.first) == neglectedObjects.end();
		it.second->setDoUpdate(update);
		LOG_INFO("  - " << it.first << ": " << (update ? "true" : "false"));
	}
}

//bool ObjectManager::setRemoveObject(const std::string& name,
//		bool remove)
//{
//	if (CHECK_MAP(m_remove, name))
//	{
//		bool& value = m_remove[name];
//		bool old = value;
//		value = remove;
//		return old != value;
//	}
//
//	return false;
//}

//void ObjectManager::toggleRemoveObject(const std::string& name)
//{
//	if (CHECK_MAP(m_remove, name))
//		m_remove[name] = !m_remove[name];
//}

//void ObjectManager::updateOctomapWithObjects(boost::shared_ptr<octomap::OcTree>& octomap)
//{
//	for (auto& it : m_objects)
//	{
//		it.second->updateBoundingBox();
//		if ((CHECK_MAP(m_remove, it.first) && m_remove[it.first]) || !it.second->isActive())
//			it.second->removeObjectFromOctomap(octomap);
//		else
//			it.second->addObjectToOctomap(octomap);
//	}
//}

} /* namespace prm_planner */

