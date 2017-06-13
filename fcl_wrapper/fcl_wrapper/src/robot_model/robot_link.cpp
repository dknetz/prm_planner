/*
 * This file (robot_link.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 2, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */

#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <ais_definitions/exception.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <fcl_wrapper/collision_detection/common.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <fcl_wrapper/robot_model/mesh.h>
#include <fcl_wrapper/robot_model/robot_joint.h>
#include <fcl_wrapper/robot_model/robot_link.h>
#include <fcl_wrapper/robot_model/robot_state.h>

namespace fcl_robot_model
{

RobotLink::RobotLink(const std::string& name,
		const std::string& rootFrame,
		const std::string& worldFrame,
		const FCL_POINTER<RobotLinkModel>& collisionModel,
const FCL_POINTER<RobotLinkModel>& visualModel) :
PhysicalObject(name, rootFrame, worldFrame),
m_collisionModel(collisionModel),
m_visualModel(visualModel),
m_transformationToParentJoint(Eigen::Affine3d::Identity()),
m_transformationToParentLink(Eigen::Affine3d::Identity())
{
	loadMesh(m_collisionModel);
	loadMesh(m_visualModel);
}

RobotLink::~RobotLink()
{
}

RobotLink::RobotLinkModel::RobotLinkModel() :
				scale(Eigen::Vector3d::Ones()),
				origin(Eigen::Affine3d::Identity()),
				hasMesh(false)
{
}

RobotLink::RobotLinkModel::~RobotLinkModel()
{
}

void RobotLink::loadMesh(FCL_POINTER<RobotLinkModel>& model)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Assimp::Importer importer;

	if (model->meshFileName.empty())
	{
		return;
	}

	//scene gets destroyed by Importer, which keeps
	//the ownership
	const aiScene* scene = importer.ReadFile(model->meshFileName.c_str(), aiProcess_CalcTangentSpace |
	aiProcess_Triangulate |
	aiProcess_JoinIdenticalVertices |
	aiProcess_SortByPType);

	if (!scene)
	{
		throw ais_definitions::Exception(std::string("Cannot read mesh file '")
		+ model->meshFileName + "'. Error: " + importer.GetErrorString());

		return;
	}

	FCL_POINTER<Meshes> meshes(new Meshes(m_name));
	meshes->m_meshes.resize(scene->mNumMeshes);
	FOR_COUNTER(i, scene->mNumMeshes)
	{
		meshes->m_meshes[i].reset(new Mesh(scene->mMeshes[i], model->scale));
	}

	model->geometry = meshes;
}

void RobotLink::setParentJoint(FCL_POINTER<RobotJoint>& parent)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_parentJoint = parent;

	m_transformationToParentJoint = m_collisionModel->origin;

	Eigen::Affine3d jointMatrix = Eigen::Affine3d::Identity();
	if (parent.get() != NULL)
	jointMatrix = parent->getStaticTransformationToParent();

	m_transformationToParentLink = jointMatrix * m_transformationToParentJoint;
}

void RobotLink::addChildJoint(FCL_POINTER<RobotJoint>& child)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_childJoints[child->getName()] = child;
}

FCL_POINTER<RobotJoint> RobotLink::getParentJoint()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return FCL_POINTER<RobotJoint>(m_parentJoint);
}

RobotLink::JointMap RobotLink::getChildJoints()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	JointMap map;
	for (auto& it : m_childJoints)
		map[it.first] = it.second.lock();
	return map;
}

Eigen::Affine3d RobotLink::getTransformationToParentLink() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_transformationToParentLink;
}

Eigen::Affine3d RobotLink::getTransformationToParentLink(const double angle) const
		{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	auto ptr = m_parentJoint.lock();
	if (ptr.get() == NULL)
	{
		LOG_WARNING("Parent joint doesn't exist! It's NULL");
		return Eigen::Affine3d::Identity();
	}

	return ptr->getTransformationToParent(angle);
}

Eigen::Affine3d RobotLink::getTransformationToParentJoint() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_transformationToParentJoint;
}

void RobotLink::setTransformationToParentLink(const Eigen::Affine3d& t)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_transformationToParentLink = t;
}

void RobotLink::updateTransformationToParentLink()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	auto ptr = m_parentJoint.lock();
	if (ptr.get() != NULL)
	{
		m_transformationToParentLink = ptr->getTransformationToParent();
	}
	else
	{
		m_transformationToParentLink.setIdentity();
	}
}

void RobotLink::initFCLModel()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	fcl::Transform3f t = fcl_collision_detection::transform2fcl(m_transformationToBase * m_collisionModel->origin);
	m_collisionModel->geometry->getFCLModel(t, m_fclCollisionObject);

//	LOG_DEBUG("fcl model of the link " << m_name << " successfully initialized!");
}

fcl_collision_detection::PhysicalObject::CollisionObjectPtr RobotLink::getFCLModel()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (m_fclCollisionObject.get() != NULL)
	{
		//update tf if frames are available
		if (!m_worldFrame.empty() && !m_frame.empty())
		{
			updateTransformRosTF();
		}

//		LOG_INFO(m_name << ": " << m_tf.getTranslation()[0] << " " << m_tf.getTranslation()[1] << " " <<m_tf.getTranslation()[2]);

		fcl::Transform3f t = m_tf * fcl_collision_detection::transform2fcl(m_transformationToBase * m_collisionModel->origin);
		m_fclCollisionObject->setTransform(t);
		m_fclCollisionObject->computeAABB();
	}

	return m_fclCollisionObject;
}

fcl_collision_detection::PhysicalObject::CollisionObjectPtr RobotLink::getFCLModel(const RobotState& state)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (m_fclCollisionObject.get() != NULL)
	{
		fcl::Transform3f t = fcl_collision_detection::transform2fcl(m_transformationToBase * m_collisionModel->origin);
		m_fclCollisionObject->setTransform(t);
		m_fclCollisionObject->computeAABB();
	}

	return m_fclCollisionObject;
}

Eigen::Affine3d RobotLink::getFCLTransformation() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	return m_transformationToBase * m_collisionModel->origin;
}

void RobotLink::updateTransformations(const Eigen::Affine3d& transformationToBaseOfParent,
		const Eigen::Affine3d& transformationFromBaseOfParent)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	updateTransformationToParentLink();

	Eigen::Affine3d tOriginToBase = transformationToBaseOfParent * m_transformationToParentLink;
	Eigen::Affine3d tOriginFromBase = m_transformationToParentLink * transformationFromBaseOfParent;
	for (auto& it : m_childJoints)
	{
		auto ptr = it.second.lock();
		ptr->getChildLink().second->updateTransformations(tOriginToBase, tOriginFromBase);
	}
	m_transformationToBase = tOriginToBase; // * m_collisionModel->origin;
	m_transformationFromBase = tOriginFromBase;
}

void RobotLink::print(int depth) const
		{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	std::string tabs(depth, '\t');
	auto ptr = m_parentJoint.lock();
	LOG_INFO(tabs << "-LINK------------------------------");
	LOG_INFO(tabs << "name: " << m_name);
	LOG_INFO(tabs << "parent joint: " << (ptr.get() == 0 ? "no parent" : ptr->getName()));
	LOG_INFO(tabs << "child joints:");
	for (auto& it : m_childJoints)
	{
		auto ptr = it.second.lock();
		LOG_INFO(tabs << "\t- " << ptr->getName());
		ptr->print(depth + 1);
	}
	LOG_INFO(tabs << "-----------------------------------");
}

Eigen::Affine3d RobotLink::getOrigin() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_collisionModel->origin;
}

Eigen::Vector3d RobotLink::getPositionInBaseFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return Eigen::Vector3d(m_transformationToBase.translation());
}

Eigen::Affine3d RobotLink::getTransformationInBaseFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_transformationToBase;
}

Eigen::Affine3d RobotLink::getTransformationFromBaseFrame() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_transformationFromBase;
}

//int RobotLink::getCollisionMeshSize() const
//{
//	int number = 0;
//	for (const auto& it : m_collisionModel->meshes)
//	{
//		number += it->m_vertices.size();
//	}
//	return number;
//}

//int RobotLink::getVisualMeshSize() const
//{
//	int number = 0;
//	for (const auto& it : m_visualModel->meshes)
//	{
//		number += it->m_vertices.size();
//	}
//	return number;
//}

FCL_POINTER<RobotLink> RobotLink::getParentLink()
{
	auto ptr = m_parentJoint.lock();
	if (ptr.get() != NULL)
	{
		return ptr->getParentLink().second;
	}
	else
	{
		return FCL_POINTER<RobotLink>();
	}
}

}
/* namespace urdf_model */

