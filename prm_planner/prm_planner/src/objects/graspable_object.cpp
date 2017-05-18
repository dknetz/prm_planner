/*
 * graspable_object.cpp
 *
 *  Created on: Sep 10, 2016
 *      Author: kuhnerd
 */

#include <ais_definitions/exception.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <ais_ros/ros_base_interface.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <octomap_msgs/conversions.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner_robot/gripper_interface.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/kdtree/kdtree_flann.h>

namespace prm_planner
{

GraspableObject::GraspableObject(const GraspableObject::ObjectParameters& parameters,
		const parameters::DroppingConfig& droppingParameters) :
				c_params(parameters),
				c_droppingConfig(droppingParameters),
				m_pose(Eigen::Affine3d::Identity()),
				m_radius(0),
				m_height(0),
				m_graspType(Cylinder),
				m_graspingPosesComputed(false),
				m_dropingPosesComputed(false),
				m_doUpdate(true),
				m_activeObject(false)
{
	loadModel();
	ros::NodeHandle n;
	m_pubGraspPoses = n.advertise<visualization_msgs::MarkerArray>("objects/grasping_poses_" + c_params.name, 1);
	m_pubDropPoses = n.advertise<visualization_msgs::MarkerArray>("objects/droping_poses_" + c_params.name, 1);
}

GraspableObject::~GraspableObject()
{
}

void GraspableObject::removeObjectFromOctomap(boost::shared_ptr<octomap::OcTree>& octomap) const
		{
	octomap::point3d offset(c_params.octomapResolution, c_params.octomapResolution, c_params.octomapResolution);
	offset *= 2;

	m_mutex.lock();
	octomap::point3d bb1 = octomap::point3d(m_bbMin.x(), m_bbMin.y(), m_bbMin.z()) - offset;
	octomap::point3d bb2 = octomap::point3d(m_bbMax.x(), m_bbMax.y(), m_bbMax.z()) + offset;
	m_mutex.unlock();

	//iterate over all voxels in bounding box
	for (octomap::OcTree::leaf_bbx_iterator it = octomap->begin_leafs_bbx(bb1, bb2),
			end = octomap->end_leafs_bbx(); it != end; ++it)
	{
		octomap->deleteNode(it.getCoordinate());
	}
}

void GraspableObject::addObjectToOctomap(boost::shared_ptr<octomap::OcTree>& octomap) const
		{
	double resolution = octomap->getResolution();
	double occupied = octomap->getClampingThresMaxLog();
//	int xBoxes = 0, yBoxes = 0, zBoxes = 0;
//	for (double x = std::min(min.x(), max.x()), xEnd = std::max(min.x(), max.x()); x <= xEnd; x += resolution)
//		xBoxes++;
//	for (double y = std::min(min.y(), max.y()), yEnd = std::max(min.y(), max.y()); y <= yEnd; y += resolution)
//			yBoxes++;
//	for (double z = std::min(min.z(), max.z()), zEnd = std::max(min.z(), max.z()); z <= zEnd; z += resolution)
//			zBoxes++;

//	LOG_INFO(m_bbMin.transpose());
//	LOG_INFO(m_bbMax.transpose());
//	LOG_INFO(min.transpose());
//	LOG_INFO(max.transpose());

	m_mutex.lock();
	Eigen::Vector3d bbMin = m_bbMin;
	Eigen::Vector3d bbMax = m_bbMax;
	m_mutex.unlock();

	for (double x = std::min(bbMin.x(), bbMax.x()), xEnd = std::max(bbMin.x(), bbMax.x()); x <= xEnd; x += resolution)
		for (double y = std::min(bbMin.y(), bbMax.y()), yEnd = std::max(bbMin.y(), bbMax.y()); y <= yEnd; y += resolution)
			for (double z = std::min(bbMin.z(), bbMax.z()), zEnd = std::max(bbMin.z(), bbMax.z()); z <= zEnd; z += resolution)
			{
				octomap->setNodeValue(octomap::point3d(x, y, z), occupied, true);
			}

	octomap->updateInnerOccupancy();
}

void GraspableObject::publish()
{
	static const std_msgs::ColorRGBA green = ais_util::Color::green().toROSMsg();
	static const std_msgs::ColorRGBA red = ais_util::Color::red().toROSMsg();
	static const std_msgs::ColorRGBA blue = ais_util::Color::blue().toROSMsg();

	//prm
	if (m_pubGraspPoses.getNumSubscribers() > 0 && m_graspingPosesComputed)
	{
		visualization_msgs::MarkerArray markers;

		int id = 0;

		//nodes
		visualization_msgs::Marker nodeXMarker;
		nodeXMarker.header.frame_id = c_params.frame;
		nodeXMarker.header.stamp = ros::Time();
		nodeXMarker.ns = "nodes_x";
		nodeXMarker.id = id++;
		nodeXMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeXMarker.action = visualization_msgs::Marker::ADD;
		nodeXMarker.scale.x = 0.005;
		nodeXMarker.color = red;

		visualization_msgs::Marker nodeYMarker;
		nodeYMarker.header.frame_id = c_params.frame;
		nodeYMarker.header.stamp = ros::Time();
		nodeYMarker.ns = "nodes_y";
		nodeYMarker.id = id++;
		nodeYMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeYMarker.action = visualization_msgs::Marker::ADD;
		nodeYMarker.scale.x = 0.005;
		nodeYMarker.color = green;

		visualization_msgs::Marker nodeZMarker;
		nodeZMarker.header.frame_id = c_params.frame;
		nodeZMarker.header.stamp = ros::Time();
		nodeZMarker.ns = "nodes_z";
		nodeZMarker.id = id++;
		nodeZMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeZMarker.action = visualization_msgs::Marker::ADD;
		nodeZMarker.scale.x = 0.005;
		nodeZMarker.color = blue;

		boost::recursive_mutex::scoped_lock lock(m_mutex);

		//pre pose
		{
			const Eigen::Vector3d t = m_graspingPrePose.translation();
			const Eigen::Matrix3d rotation = m_graspingPrePose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		//pose
		{
			const Eigen::Vector3d t = m_graspingPose.translation();
			const Eigen::Matrix3d rotation = m_graspingPose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		//post pose
		{
			const Eigen::Vector3d t = m_graspingPostPose.translation();
			const Eigen::Matrix3d rotation = m_graspingPostPose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		markers.markers.push_back(nodeXMarker);
		markers.markers.push_back(nodeYMarker);
		markers.markers.push_back(nodeZMarker);

		m_pubGraspPoses.publish(markers);
	}

	if (m_pubDropPoses.getNumSubscribers() > 0 && m_dropingPosesComputed)
	{
		visualization_msgs::MarkerArray markers;

		int id = 0;

		//nodes
		visualization_msgs::Marker nodeXMarker;
		nodeXMarker.header.frame_id = c_params.frame;
		nodeXMarker.header.stamp = ros::Time();
		nodeXMarker.ns = "nodes_x";
		nodeXMarker.id = id++;
		nodeXMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeXMarker.action = visualization_msgs::Marker::ADD;
		nodeXMarker.scale.x = 0.005;
		nodeXMarker.color = red;

		visualization_msgs::Marker nodeYMarker;
		nodeYMarker.header.frame_id = c_params.frame;
		nodeYMarker.header.stamp = ros::Time();
		nodeYMarker.ns = "nodes_y";
		nodeYMarker.id = id++;
		nodeYMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeYMarker.action = visualization_msgs::Marker::ADD;
		nodeYMarker.scale.x = 0.005;
		nodeYMarker.color = green;

		visualization_msgs::Marker nodeZMarker;
		nodeZMarker.header.frame_id = c_params.frame;
		nodeZMarker.header.stamp = ros::Time();
		nodeZMarker.ns = "nodes_z";
		nodeZMarker.id = id++;
		nodeZMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeZMarker.action = visualization_msgs::Marker::ADD;
		nodeZMarker.scale.x = 0.005;
		nodeZMarker.color = blue;

		boost::recursive_mutex::scoped_lock lock(m_mutex);

		//pre pose
		{
			const Eigen::Vector3d t = m_dropingPrePose.translation();
			const Eigen::Matrix3d rotation = m_dropingPrePose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		//pose
		{
			const Eigen::Vector3d t = m_dropingPose.translation();
			const Eigen::Matrix3d rotation = m_dropingPose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		//post pose
		{
			const Eigen::Vector3d t = m_dropingPostPose.translation();
			const Eigen::Matrix3d rotation = m_dropingPostPose.rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}

		markers.markers.push_back(nodeXMarker);
		markers.markers.push_back(nodeYMarker);
		markers.markers.push_back(nodeZMarker);

		m_pubDropPoses.publish(markers);
	}
}

const Eigen::Affine3d GraspableObject::getPose() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_pose;
}

GraspableObject::GraspType GraspableObject::getGraspType() const
{
	return m_graspType;
}

double GraspableObject::getHeight() const
{
	return m_height;
}

double GraspableObject::getRadius() const
{
	return m_radius;
}

double GraspableObject::getHeightAboveCenter() const
{
	return m_heightAboveCenter;
}

double GraspableObject::getHeightBelowCenter() const
{
	return m_heightBelowCenter;
}

bool GraspableObject::isDoUpdate() const
{
	return m_doUpdate;
}

void GraspableObject::setDoUpdate(bool doUpdate)
{
	m_doUpdate = doUpdate;
}

bool GraspableObject::isActive() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_activeObject;
}

bool GraspableObject::sampleGraspPose(Eigen::Affine3d& prePose,
		Eigen::Affine3d& pose,
		Eigen::Affine3d& postPose,
		const parameters::HandConfig& handConfig,
		const boost::shared_ptr<GripperInterface> gripper)
{
	static std::uniform_real_distribution<double> dist(-M_PI, M_PI);
//	static std::uniform_int_distribution<int> dist2(1, 1);
	static std::mt19937 gen(time(0));

	const GripperInterface::GripperInterfaceParameters& gripperParams = gripper->c_parameters;

	if (m_graspType == Cylinder)
	{
		boost::recursive_mutex::scoped_lock lock(m_mutex);

		//position
		double minHeight = -m_heightBelowCenter + handConfig.minHeight;
		double centerHeight = -m_heightBelowCenter + m_height / 2.0;
		double height = std::max(minHeight, centerHeight);

		Eigen::Vector3d posPre(gripperParams.graspPreDistance, 0, height);
		Eigen::Vector3d pos(m_radius - gripperParams.graspRadius, 0, height);
		Eigen::Vector3d posPost(m_radius - gripperParams.graspRadius, 0, height + gripperParams.graspPostHeight);

		Eigen::Vector3d z(-posPre.x(), -posPre.y(), 0);
		z.normalize();
		Eigen::Vector3d y(0, 0, -1); //use +1 if you want to use 180° rotated gripper
		Eigen::Vector3d x = y.cross(z);

		Eigen::Matrix3d rot;
		rot.col(0) = x;
		rot.col(1) = y;
		rot.col(2) = z;

		Eigen::AngleAxisd randomRotation(dist(gen), Eigen::Vector3d::UnitZ());

		prePose.setIdentity();
		prePose.translation() = posPre;
		prePose.linear() = rot;
		prePose = randomRotation * prePose;
		prePose.translation() += m_pose.translation();

		pose.setIdentity();
		pose.translation() = pos;
		pose.linear() = rot;
		pose = randomRotation * pose;
		pose.translation() += m_pose.translation();

		postPose.setIdentity();
		postPose.translation() = posPost;
		postPose.linear() = rot;
		postPose = randomRotation * postPose;
		postPose.translation() += m_pose.translation();

		m_graspingPrePose = prePose;
		m_graspingPose = pose;
		m_graspingPostPose = postPose;
		m_graspingPosesComputed = true;

		return true;
	}
	else
	{
		LOG_FATAL("Unknown grasp type");
		exit(-1);
	}
}

bool GraspableObject::sampleDropPose(Eigen::Affine3d& prePose,
		Eigen::Affine3d& pose,
		Eigen::Affine3d& postPose,
		const Eigen::Vector3d goalPos,
		const boost::shared_ptr<GripperInterface> gripper)
{
	static std::uniform_real_distribution<double> dist(-M_PI, M_PI);
	static std::mt19937 gen(time(0));

	const GripperInterface::GripperInterfaceParameters& gripperParams = gripper->c_parameters;

	Eigen::Affine3d poseObject = gripper->getTGripperToObject();

	if (m_graspType == Cylinder)
	{
//		boost::recursive_mutex::scoped_lock lock(m_mutex);

		Eigen::Vector3d centerPos = (goalPos + m_heightBelowCenter * Eigen::Vector3d::UnitZ());

		Eigen::Matrix3d orientationObject;
		if (m_upAxis.isApprox(Eigen::Vector3d::UnitX()) || m_upAxis.isApprox(Eigen::Vector3d::UnitY()))
		{
			Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
//			Eigen::Vector3d ortho = worldUp.cross(m_upAxis).normalized(); //take this - if you want to grasp with 180° rotated gripper
			Eigen::Vector3d ortho = m_upAxis.cross(worldUp).normalized();
			orientationObject.col(0) = ortho;
			orientationObject.col(1) = worldUp;
			orientationObject.col(2) = -m_upAxis; //remove - if you want to grasp with 180° rotated gripper
		}
		else
		{
			orientationObject.setIdentity();
		}

		Eigen::AngleAxisd rot(dist(gen), m_upAxis);
		orientationObject = orientationObject * rot;

		Eigen::Affine3d poseObjectDesired;
		poseObjectDesired.translation() = centerPos;
		poseObjectDesired.linear() = orientationObject;

		pose = poseObjectDesired * poseObject.inverse();
		Eigen::Matrix3d poseOrientation = pose.linear();

		prePose = pose;
		prePose.translation() += Eigen::Vector3d(0, 0, gripperParams.dropPreDistance);

		postPose = pose;
		postPose.translation() -= poseOrientation.col(2) * gripperParams.graspPreDistance;

//		m_dropingPrePose = prePose;
//		m_dropingPose = pose;
//		m_dropingPostPose = postPose;
//		m_dropingPosesComputed = true;

		return true;
	}
	else
	{
		LOG_FATAL("Unknown grasp type");
		exit(-1);
	}
}

void GraspableObject::loadModel()
{
	Assimp::Importer importer;

	const aiScene* scene = importer.ReadFile(c_params.meshFilename.c_str(),
			aiProcess_CalcTangentSpace |
					aiProcess_Triangulate |
					aiProcess_JoinIdenticalVertices |
					aiProcess_SortByPType);

	if (!scene)
	{
		throw ais_definitions::Exception(std::string("Cannot read object mesh file '")
				+ c_params.meshFilename + "'. Error: " + importer.GetErrorString());

		return;
	}

	boost::recursive_mutex::scoped_lock lock(m_mutex);

	FOR_COUNTER(i, scene->mNumMeshes)
		{
		const aiMesh* mesh = scene->mMeshes[i];
		size_t size = mesh->mNumVertices;
		m_vertices.reserve(m_vertices.size() + size);
		FOR_COUNTER(j, size)
				{
			m_vertices.push_back(Eigen::Vector3d(mesh->mVertices[j].x,
					mesh->mVertices[j].y,
					mesh->mVertices[j].z));
		}
	}

	//compute center
	m_center = Eigen::Vector3d::Zero();
	for (auto& it : m_vertices)
	{
		m_center += m_pose * it;
	}
	m_center /= m_vertices.size();

	computeBoundingBox(m_vertices, m_center, m_bbMin, m_bbMax);
	Eigen::Vector3d toMin = m_center - m_bbMin;
	Eigen::Vector3d toMax = m_center - m_bbMax;
	Eigen::Vector3d maxToMin = m_bbMax - m_bbMin;

	//compute height and radius, assuming that z is pointing upwards
//	m_radius = 0;
//	m_height = 0;
//	double r, hLow = 0, hTop = 0;
	if (m_graspType == Cylinder)
	{
		if (fabs(maxToMin.x()) > fabs(maxToMin.y()))
		{
			if (fabs(maxToMin.x()) > fabs(maxToMin.z()))
			{
				m_height = fabs(maxToMin.x());
				m_heightBelowCenter = fabs(m_bbMin.x());
				m_heightAboveCenter = fabs(m_bbMax.x());
				m_radius = std::max(std::max(m_bbMin.y(), m_bbMax.y()), std::max(m_bbMin.z(), m_bbMax.z()));
				m_upAxis = Eigen::Vector3d::UnitX();
			}
			else
			{
				m_height = fabs(maxToMin.z());
				m_heightBelowCenter = fabs(m_bbMin.z());
				m_heightAboveCenter = fabs(m_bbMax.z());
				m_radius = std::max(std::max(m_bbMin.y(), m_bbMax.y()), std::max(m_bbMin.x(), m_bbMax.x()));
				m_upAxis = Eigen::Vector3d::UnitZ();
			}
		}
		else
		{
			if (fabs(maxToMin.y()) > fabs(maxToMin.z()))
			{
				m_height = fabs(maxToMin.y());
				m_heightBelowCenter = fabs(m_bbMin.y());
				m_heightAboveCenter = fabs(m_bbMax.y());
				m_radius = std::max(std::max(m_bbMin.x(), m_bbMax.x()), std::max(m_bbMin.z(), m_bbMax.z()));
				m_upAxis = Eigen::Vector3d::UnitY();
			}
			else
			{
				m_height = fabs(maxToMin.z());
				m_heightBelowCenter = fabs(m_bbMin.z());
				m_heightAboveCenter = fabs(m_bbMax.z());
				m_radius = std::max(std::max(m_bbMin.y(), m_bbMax.y()), std::max(m_bbMin.x(), m_bbMax.x()));
				m_upAxis = Eigen::Vector3d::UnitZ();
			}
		}

//		for (auto& it : m_vertices)
//		{
//			r = it.head(2).norm();
//			if (r > m_radius)
//				m_radius = r;
//
//			if (it.z() > hTop)
//				hTop = it.z();
//			else if (it.z() < hLow)
//				hLow = it.z();
//		}
	}
//	m_heightBelowCenter = hLow;
//	m_height = hTop - hLow;

//TODO: make it better (see assumption)
//	if (m_height < m_radius)
//	{
//		double h = m_height;
//		m_height = m_radius;
//		m_radius = h * 0.5;
//	}
}

void GraspableObject::updatePoseFromTF()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (!m_doUpdate)
		return;

	m_activeObject = ais_ros::RosBaseInterface::getRosTransformationWithResult(c_params.objectFrameName, c_params.frame, m_pose);
}

void GraspableObject::print()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	LOG_INFO("Object:");
	LOG_INFO("	Name: "<<c_params.name);
	LOG_INFO("	Frame: "<<c_params.frame);
	LOG_INFO("	Mesh: "<<c_params.meshFilename);
	LOG_INFO("	Object Frame Name: "<<c_params.objectFrameName);
	LOG_INFO("	Octomap Resolution: "<<c_params.octomapResolution);
	LOG_INFO("	Radius: "<<m_radius);
	LOG_INFO("	Height: "<<m_height);
	LOG_INFO("	Grasp Type: "<<m_graspType);
	LOG_INFO(
			"  Bounding Box Size (x, y, z): " << fabs(m_bbMax.x() - m_bbMin.x()) << " "<< fabs(m_bbMax.y() - m_bbMin.y())<<" "<< fabs(m_bbMax.z() - m_bbMin.z()));

}

void GraspableObject::updateBoundingBox()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (!m_doUpdate)
		return;

	std::vector<Eigen::Vector3d> pos;
	pos.reserve(m_vertices.size());
	Eigen::Vector3d p;

	//compute center
	for (auto& it : m_vertices)
	{
		p = m_pose * it;
		pos.push_back(p);
	}
	Eigen::Vector3d center = m_pose * m_center;

	computeBoundingBox(pos, center, m_bbMin, m_bbMax);
}

bool GraspableObject::getTransformationFromFrame(const std::string& frame,
		Eigen::Affine3d& t)
{
	return ais_ros::RosBaseInterface::getRosTransformationWithResult(c_params.objectFrameName, frame, t);
}

bool GraspableObject::isInGripper(const boost::shared_ptr<GripperInterface>& gripper)
{
	static const double MAX_DISTANCE_TO_BE_IN_GRIPPER = 0.2;

	Eigen::Affine3d t;
	if (!ais_ros::RosBaseInterface::getRosTransformationWithResult(c_params.objectFrameName, gripper->c_parameters.graspFrame, t))
		return false;

	return t.translation().norm() < MAX_DISTANCE_TO_BE_IN_GRIPPER;
}

void GraspableObject::computeBoundingBox(const std::vector<Eigen::Vector3d>& points,
		const Eigen::Vector3d& center,
		Eigen::Vector3d& minBBPoint,
		Eigen::Vector3d& maxBBPoint)
{
	minBBPoint = maxBBPoint = center;

	for (auto& it : points)
	{
		if (it.x() < minBBPoint.x())
			minBBPoint.x() = it.x();
		if (it.x() > maxBBPoint.x())
			maxBBPoint.x() = it.x();

		if (it.y() < minBBPoint.y())
			minBBPoint.y() = it.y();
		if (it.y() > maxBBPoint.y())
			maxBBPoint.y() = it.y();

		if (it.z() < minBBPoint.z())
			minBBPoint.z() = it.z();
		if (it.z() > maxBBPoint.z())
			maxBBPoint.z() = it.z();
	}

}

void GraspableObject::lock()
{
	m_mutex.lock();
}

void GraspableObject::unlock()
{
	m_mutex.unlock();
}

} /* namespace prm_planner */

