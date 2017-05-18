/*
 * This file (robot_model.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Jan 30, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */

#include <ais_definitions/class.h>
#include <ais_definitions/exception.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <fcl_wrapper/collision_detection/common.h>
#include <fcl_wrapper/robot_model/box.h>
#include <fcl_wrapper/robot_model/cylinder.h>
#include <fcl_wrapper/robot_model/mesh.h>
#include <fcl_wrapper/robot_model/robot_joint.h>
#include <fcl_wrapper/robot_model/robot_link.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <fcl_wrapper/robot_model/sphere.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <memory>

namespace fcl_robot_model
{

RobotModel::RobotModel(const std::string& robotDescriptionParamName,
		const std::string& name,
		const std::string& tfPrefix,
		const std::string& worldFrame,
		const std::string& collisionMatrixFile) :
				Collection(name, "", worldFrame),
				m_robotModel(new urdf::Model),
				m_tfPrefix(tfPrefix)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (!m_robotModel->initParam(robotDescriptionParamName))
	{
		throw ais_definitions::Exception("Cannot initialize robot model using the ROS parameter server");
	}

	if (!collisionMatrixFile.empty())
	{
		fcl_collision_detection::CollisionMatrix::load(collisionMatrixFile, m_collisionMatrix);
		m_collisionMatrix->setPrefix(name, { "environment" });
	}

	load();
}

RobotModel::~RobotModel()
{
	DELETE_VAR(m_robotModel);
}

void RobotModel::load()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_frame = (m_tfPrefix.empty() ? "" : (m_tfPrefix + "/")) + m_robotModel->getRoot()->name;
	for (auto& it : m_robotModel->links_)
	{
		FCL_POINTER<RobotLink::RobotLinkModel> collisionModel(new RobotLink::RobotLinkModel);
		FCL_POINTER<RobotLink::RobotLinkModel> visualModel(new RobotLink::RobotLinkModel);

		//has collision info
		if (it.second->collision.get() != NULL)
		{
			//has a geometry associated with the collision model
			if (it.second->collision->geometry.get() != NULL)
			{
				if (it.second->collision->geometry->type == urdf::Geometry::MESH)
				{
					FCL_POINTER<urdf::Mesh> mesh = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Mesh>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
					parseURI(mesh->filename, collisionModel->meshFileName);
					collisionModel->scale = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
				}
				else if (it.second->collision->geometry->type == urdf::Geometry::BOX)
				{
					FCL_POINTER<urdf::Box> box = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Box>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
					collisionModel->meshFileName.clear();
					collisionModel->scale = Eigen::Vector3d::Ones();
					collisionModel->geometry.reset(new Box(box->dim.x, box->dim.y, box->dim.z));
				}
				else if (it.second->collision->geometry->type == urdf::Geometry::CYLINDER)
				{
					FCL_POINTER<urdf::Cylinder> cylinder = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Cylinder>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
					collisionModel->meshFileName.clear();
					collisionModel->scale = Eigen::Vector3d::Ones();
					collisionModel->geometry.reset(new Cylinder(cylinder->radius, cylinder->length));
				}
				else if (it.second->collision->geometry->type == urdf::Geometry::SPHERE)
				{
					FCL_POINTER<urdf::Sphere> sphere = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Sphere>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
					collisionModel->meshFileName.clear();
					collisionModel->scale = Eigen::Vector3d::Ones();
					collisionModel->geometry.reset(new Sphere(sphere->radius));
				}
				else
				{
					LOG_ERROR("I do not support type " << it.second->collision->geometry->type << " at the moment!");
					continue;
				}

				collisionModel->hasMesh = true;
				toEigenTransform(it.second->collision->origin, collisionModel->origin);
			}
			else
			{
				toEigenTransform(it.second->collision->origin, collisionModel->origin);
			}
		}

		//has visual info
//		if (it.second->visual.get() != NULL)
//		{
//			//has a geometry associated with the visual model
//			if (it.second->visual->geometry.get() != NULL)
//			{
////				LOG_INFO(it.first << " " << it.second.);
//				if (it.second->visual->geometry->type == urdf::Geometry::MESH)
//				{
//					FCL_POINTER<urdf::Mesh> mesh = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Mesh>(CONVERT_TO_BOOST_POINTER(it.second->visual->geometry));
//					parseURI(mesh->filename, visualModel->meshFileName);
//					visualModel->scale = Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z);
//				}
//				else if (it.second->collision->geometry->type == urdf::Geometry::BOX)
//				{
//					FCL_POINTER<urdf::Box> box = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Box>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
//					collisionModel->meshFileName.clear();
//					collisionModel->scale = Eigen::Vector3d::Ones();
//					collisionModel->geometry.reset(new Box(box->dim.x, box->dim.y, box->dim.z));
//				}
//				else if (it.second->collision->geometry->type == urdf::Geometry::CYLINDER)
//				{
//					FCL_POINTER<urdf::Cylinder> cylinder = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Cylinder>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
//					collisionModel->meshFileName.clear();
//					collisionModel->scale = Eigen::Vector3d::Ones();
//					collisionModel->geometry.reset(new Cylinder(cylinder->radius, cylinder->length));
//				}
//				else if (it.second->collision->geometry->type == urdf::Geometry::SPHERE)
//				{
//					FCL_POINTER<urdf::Sphere> sphere = FCL_POINTER_NAMESPACE::dynamic_pointer_cast<urdf::Sphere>(CONVERT_TO_BOOST_POINTER(it.second->collision->geometry));
//					collisionModel->meshFileName.clear();
//					collisionModel->scale = Eigen::Vector3d::Ones();
//					collisionModel->geometry.reset(new Sphere(sphere->radius));
//				}
//				else
//				{
//					LOG_ERROR("I do not support type " << it.second->collision->geometry->type << " at the moment!");
//					continue;
//				}
//
//				visualModel->hasMesh = true;
//				toEigenTransform(it.second->collision->origin, visualModel->origin);
//			}
//			else
//			{
//				toEigenTransform(it.second->visual->origin, visualModel->origin);
//			}
//		}

		m_links[it.first].reset(new RobotLink(it.first, m_frame, m_worldFrame, collisionModel, visualModel));

		if (it.second->collision.get() != NULL)
		{
			m_linksWorldObjects[it.first] = m_links[it.first];
		}
	}

	for (auto& it : m_robotModel->joints_)
	{
		Eigen::Affine3d transformation;
		toEigenTransform(it.second->parent_to_joint_origin_transform, transformation);
		Eigen::Vector3d axis(it.second->axis.x, it.second->axis.y, it.second->axis.z);

		double low = 0, up = 0, speed = 0;
		if (it.second->limits.get() != NULL)
		{
			low = it.second->limits->lower;
			up = it.second->limits->upper;
			speed = it.second->limits->velocity;
		}

		m_joints[it.first].reset(new RobotJoint(it.first, axis, low, up, speed, transformation));

		if (CHECK_MAP(m_links, it.second->parent_link_name))
		{
			m_joints[it.first]->setParentLink(m_links[it.second->parent_link_name]);
		}

		if (CHECK_MAP(m_links, it.second->child_link_name))
		{
			m_joints[it.first]->setChildLink(m_links[it.second->child_link_name]);
		}
	}

	m_root = m_links[m_robotModel->getRoot()->name];

	loadHierarchy(m_robotModel->getRoot(), m_root);

	//update transformations
	if (m_root != NULL)
	{
		m_root->updateTransformations(Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity());
	}
}

void RobotModel::loadHierarchy(const boost::shared_ptr<const urdf::Link>& parent,
		FCL_POINTER<RobotLink>& myParent)
{
	for (auto& it : parent->child_joints)
	{
		myParent->addChildJoint(m_joints[it->name]);
	}

	for (auto& it : parent->child_links)
	{
		m_links[it->name]->setParentJoint(m_joints[it->parent_joint->name]);
//		m_transformations[std::make_pair(it->name, parent->name)] = m_links[it->name]->getTransformationToParentLink();
		loadHierarchy(it, m_links[it->name]);
	}
}

fcl_collision_detection::CollisionMatrix::Ptr RobotModel::getCollisionMatrix()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_collisionMatrix;
}

fcl_collision_detection::ObjectMap RobotModel::getObjects()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_linksWorldObjects;
}

double RobotModel::getLowerJointLimit(const std::string& jointName)
{
	return m_joints[jointName]->getLowerJointLimit();
}

double RobotModel::getUpperJointLimit(const std::string& jointName)
{
	return m_joints[jointName]->getUpperJointLimit();
}

std::unordered_map<std::string, double> RobotModel::getAllVelocityLimits()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	std::unordered_map<std::string, double> limits;
	for (auto& it : m_joints)
	{
		limits[it.second->getName()] = it.second->getVelocityLimit();
	}
	return limits;
}

Eigen::Affine3d RobotModel::getTransformationToRoot(const std::string& frame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return getTransformationToSomeParent(frame, m_root->getName());
}

std::string RobotModel::getEndEffectorFrameName() const
{
	return m_frameEndEffector;
}

void RobotModel::setEndEffectorFrameName(const std::string& name)
{
	m_frameEndEffector = name;
}

std::string RobotModel::getManipulatorBaseFrameName() const
{
	return m_frameManipulatorBase;
}

void RobotModel::setManipulatorBaseFrameName(const std::string& name)
{
	m_frameManipulatorBase = name;
	m_transformationManipulatorBaseToRoot = getTransformationOfFrameFromBase(m_frameManipulatorBase);
}

Eigen::Affine3d RobotModel::getManipulatorBaseFrame() const
{
	return m_transformationManipulatorBaseToRoot;
}

void RobotModel::print() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	LOG_INFO("ROBOT_MODEL");
	LOG_INFO(m_links.size() << " links");
	LOG_INFO(m_joints.size() << " joints");
	m_root->print();
}

void RobotModel::toEigenTransform(const urdf::Pose& pose,
		Eigen::Affine3d& transform)
{
	Eigen::Translation3d t(pose.position.x, pose.position.y, pose.position.z);
	Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
	transform = t * q;
}

void RobotModel::parseURI(const std::string& path,
		std::string& absPath)
{
	//ros package
	if (boost::starts_with(path, "package://"))
	{
		std::vector<std::string> strs;
		boost::algorithm::split_regex(strs, path, boost::regex("//"));

		if (strs.size() != 2)
		{
			throw ais_definitions::Exception("Error in path parsing! Only has "
					+ std::to_string(strs.size()) + " elements in path " + path);
			return;
		}

		boost::split(strs, strs[1], boost::is_any_of("/"));

		if (strs.empty())
		{
			throw ais_definitions::Exception("Path seems to be empty!");
			return;
		}

		std::string rosPath = ros::package::getPath(strs[0]);

		if (rosPath.empty())
		{
			throw ais_definitions::Exception("Cannot find ros package: !" + strs[0]);
			return;
		}

		absPath = rosPath;
		for (size_t i = 1; i < strs.size(); ++i)
		{
			absPath += "/" + strs[i];
		}
	}
	else
	{
		absPath = path;
	}
}

void RobotModel::getJointNames(std::vector<std::string>& names)
{
	for (auto& it : m_joints)
	{
		names.push_back(it.second->getName());
	}
}

void RobotModel::setRobotState(const RobotState& state)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	const std::string search = m_name + "/";
	for (auto& it : state)
	{
		std::string nameWithoutPrefix = it.first;
		boost::replace_first(nameWithoutPrefix, search, "");
		m_joints[nameWithoutPrefix]->setAngle(it.second);
	}

	if (m_root != NULL)
	{
		m_root->updateTransformations(Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity());
	}
}

void RobotModel::setRobotState(const std::unordered_map<std::string, double>& state)
{
	RobotState robotState(state);
	setRobotState(robotState);
}

void RobotModel::getRobotState(RobotState& state)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	for (auto& it : m_joints)
	{
		state.setJoint(m_name + "/" + it.first, it.second->getAngle());
	}
}

void RobotModel::getChildLinks(const std::string name,
		std::vector<FCL_POINTER<RobotLink> >& childs)
{
	for (auto& it : m_links[name]->getChildJoints())
	{
		childs.push_back(it.second->getChildLink().second);
	}
}

void RobotModel::getParentLink(const std::string name,
		FCL_POINTER<RobotLink>& parent)
{
	if (m_links[name]->getParentJoint().get() == NULL)
	{
		parent.reset();
	}
	else
	{
		parent = m_links[name]->getParentJoint()->getParentLink().second;
	}
}

double RobotModel::getDistance(const std::string& frame1,
		const std::string& frame2)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Eigen::Vector3d p1 = m_links[frame1]->getPositionInBaseFrame();
	Eigen::Vector3d p2 = m_links[frame2]->getPositionInBaseFrame();
	return (p1 - p2).norm();
}

Eigen::Affine3d RobotModel::getTransformationToSomeParent(const std::string& child,
		const std::string& someParent)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Eigen::Affine3d t;
	t.setIdentity();

	getTransformationToSomeParentRec(child, someParent, t);

	return t;
}

bool RobotModel::getTransformationToSomeParentRec(const std::string& child,
		const std::string& someParent,
		Eigen::Affine3d& t)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (child == someParent)
	{
		return true;
	}

	FCL_POINTER<RobotLink> parent = m_links[child]->getParentLink();

	if (parent.get() != NULL)
	{
		t = m_links[child]->getTransformationToParentLink() * t;
		return getTransformationToSomeParentRec(parent->getName(), someParent, t);
	}
	else
	{
		return false;
	}
}

Eigen::Affine3d RobotModel::getFrameOfLinkInSomeParentFrame(const std::string& frame,
		const std::string& someParent)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Eigen::Affine3d t;
	t.setIdentity();

	getFrameOfLinkInSomeParentFrameRec(frame, someParent, t);

	return t;
}

void RobotModel::setTransform(const Eigen::Affine3d& tf)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	fcl::Transform3f fclTf = fcl_collision_detection::transform2fcl(tf);
	for (auto& it : m_links)
	{
		it.second->setTransform(fclTf);
	}
}

bool RobotModel::getFrameOfLinkInSomeParentFrameRec(const std::string& child,
		const std::string& someParent,
		Eigen::Affine3d& t)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	if (child == someParent)
	{
		return true;
	}

	FCL_POINTER<RobotLink> parent = m_links[child]->getParentLink();

	if (parent.get() != NULL)
	{
		t = parent->getTransformationToParentLink().inverse() * t;
		return getTransformationToSomeParentRec(parent->getName(), someParent, t);
	}
	else
	{
		return false;
	}
}

Eigen::Vector3d RobotModel::getPositionOfFrame(const std::string& frame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_links[frame]->getPositionInBaseFrame();
}

Eigen::Affine3d RobotModel::getTransformationOfFrameToBase(const std::string& frame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_links[frame]->getTransformationInBaseFrame();
}

Eigen::Affine3d RobotModel::getTransformationOfFrameFromBase(const std::string& frame)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_links[frame]->getTransformationFromBaseFrame();
}

FCL_POINTER<RobotLink> RobotModel::getLink(const std::string& name)
{
	return m_links[name];
}

FCL_POINTER<RobotJoint> RobotModel::getJoint(const std::string& name)
{
	return m_joints[name];
}

void RobotModel::sampleValidJoints(RobotState& state)
{
	std::random_device gen;

	for (auto& it : m_joints)
	{
		std::uniform_real_distribution<double> dist(it.second->m_lowerLimit, it.second->m_upperLimit);
		double angle = dist(gen);
		state[it.second->getName()] = angle;
	}
}

}

/* namespace robot_model */

