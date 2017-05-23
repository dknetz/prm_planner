/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 28, 2017
 *      Author: kuhnerd
 * 	  Filename: environment.cpp
 */

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner/environment/planning_object.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/util/parameter_server.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <eigen_conversions/eigen_msg.h>

namespace prm_planner
{

PlanningScene::PlanningScene(boost::shared_ptr<ProblemDefinition> pd) :
				m_workspaceRadius(0),
				m_problemDefinition(pd)
{
	init();
}

PlanningScene::~PlanningScene()
{
}

void PlanningScene::init()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	initOctomap();
	initROS();

	m_workspaceRadius = m_problemDefinition->getWorkspaceRadius();
	m_workspaceCenter = m_problemDefinition->getWorkspaceCenter();
}

void PlanningScene::initROS()
{
	ros::NodeHandle n;

	if (ParameterServer::startPublishers)
	{
		m_pubPointCloud = n.advertise<ais_point_cloud::MyPointCloud>("point_cloud", 1);
		m_pubOctomap = n.advertise<octomap_msgs::Octomap>("octomap", 1);
		m_pubWorkspace = n.advertise<visualization_msgs::MarkerArray>("workspace", 1);
		m_pubObjects = n.advertise<visualization_msgs::MarkerArray>("planning_scene", 1);
	}
}

void PlanningScene::initOctomap()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	octomap.reset(new octomap::OcTree(ParameterServer::octomapResolution));
	octomap->setProbHit(ParameterServer::octomapProbHit);
	octomap->setProbMiss(ParameterServer::octomapProbMiss);
	octomap->setClampingThresMin(ParameterServer::octomapClampingThresholdMin);
	octomap->setClampingThresMax(ParameterServer::octomapClampingThresholdMax);
}

bool PlanningScene::initFromPointCloud()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (!rgbd)
		return false;

	ais_point_cloud::MyPointCloudP downsampledCloud = rgbd->getNanFilteredDownsampledCloud(3, false);

	//filter cloud
	ais_point_cloud::MyPointCloudP cloudFiltered(new ais_point_cloud::MyPointCloud);
	pcl::StatisticalOutlierRemoval<ais_point_cloud::MyPointType> sor;
	sor.setInputCloud(downsampledCloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(2.0);
	sor.setKeepOrganized(true);
	sor.setUserFilterValue(std::numeric_limits<double>::quiet_NaN());
	sor.filter(*cloudFiltered);
	downsampledCloud = cloudFiltered;

	//filter cloud to remove clutter
	ais_point_cloud::MyPointCloudP cloud;
	filterPointCloud(downsampledCloud, cloud, m_workspaceRadius, m_workspaceCenter);

	//sometimes it can happen, that we get a very small/empty cloud, e.g,
	//after activating the planner. In those cases we skip further
	//processing.
	if (cloud->size() == 0)
	{
		return false;
	}
	else
	{
		//update the octomap based on the filtered cloud
		updateOctomap(cloud);
		processedCloud = cloud;
		return true;
	}
}

void PlanningScene::updateOctomap(const ais_point_cloud::MyPointCloudP cloud)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	initOctomap();

	Eigen::Vector3d originPos =
			ais_ros::RosBaseInterface::getRosTransformation(ParameterServer::topicCameraPrefix + "_depth_optical_frame",
					m_problemDefinition->getFrame()).translation();
	octomap::point3d origin(originPos.x(), originPos.y(), originPos.z());

	octomap::Pointcloud omCloud;
	omCloud.reserve(cloud->size());
	for (auto& it : cloud->points)
	{
		omCloud.push_back(it.x, it.y, it.z);
	}

	octomap->insertPointCloud(omCloud, origin);

	//	//TODO: Improve the following code to have memory of the scene
	//	octomap::KeySet freeCells, occupiedCells;
	//
	//	//we don't need to filter max ranges, because the cloud is already filtered
	//	for (auto& it : cloud->points)
	//	{
	//		//filter nan's
	//		if (std::isnan(it.x) || std::isnan(it.y) || std::isnan(it.z))
	//			continue;
	//
	//		octomap::point3d pos(it.x, it.y, it.z);
	//		octomap::KeyRay keyRay;
	//
	//		if (m_octomap->computeRayKeys(origin, pos, keyRay))
	//			freeCells.insert(keyRay.begin(), keyRay.end());
	//
	//		octomap::OcTreeKey key;
	//		if (m_octomap->coordToKeyChecked(pos, key))
	//			occupiedCells.insert(key);
	//	}
	//
	//	// mark free cells only if not seen occupied in this cloud
	//	for (octomap::KeySet::iterator it = freeCells.begin(), end = freeCells.end(); it != end; ++it)
	//		if (occupiedCells.find(*it) == occupiedCells.end())
	//			m_octomap->updateNode(*it, false, true);
	//
	//	// now mark all occupied cells:
	//	for (octomap::KeySet::iterator it = occupiedCells.begin(), end = occupiedCells.end(); it != end; it++)
	//		m_octomap->updateNode(*it, true, true);

	//	m_objectManager->updateOctomapWithObjects(m_octomap);

	//because of lazy updates
	octomap->updateInnerOccupancy();
	octomap->prune();

	m_mutexVisualization.lock();
	octomap_msgs::fullMapToMsg(*octomap, m_octomapMsg);
	m_octomapMsg.header.frame_id = m_problemDefinition->getFrame();
	m_pubOctomap.publish(m_octomapMsg);
	m_mutexVisualization.unlock();
}

void PlanningScene::publish()
{
	static const std_msgs::ColorRGBA blue = ais_util::Color::blue().toROSMsg();

	if (m_pubWorkspace.getNumSubscribers() > 0)
	{
		boost::mutex::scoped_lock lock(m_mutexVisualization);
		visualization_msgs::MarkerArray markers;
		double diameter = 2 * m_workspaceRadius;

		visualization_msgs::Marker marker;
		marker.header.frame_id = m_problemDefinition->getFrame();
		marker.header.stamp = ros::Time();
		marker.ns = "workspace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.x = diameter;
		marker.scale.y = diameter;
		marker.scale.z = diameter;
		marker.color = blue;
		marker.color.a = 0.3;
		marker.pose.position.x = m_workspaceCenter.x();
		marker.pose.position.y = m_workspaceCenter.y();
		marker.pose.position.z = m_workspaceCenter.z();
		markers.markers.push_back(marker);

		m_pubWorkspace.publish(markers);
	}

	if (m_pubOctomap.getNumSubscribers() > 0)
	{
		boost::mutex::scoped_lock lock(m_mutexVisualization);
		if (octomap.get() != NULL && m_octomapMsg.data.size() > 0)
			m_pubOctomap.publish(m_octomapMsg);
	}

	if (m_pubPointCloud.getNumSubscribers() > 0)
	{
		boost::mutex::scoped_lock lock(m_mutexVisualization);
		if (rgbd.get() != NULL)
			m_pubPointCloud.publish(rgbd->getCloud());
	}

	if (m_pubObjects.getNumSubscribers() > 0)
	{
		boost::mutex::scoped_lock lock(m_mutexVisualization);
		visualization_msgs::MarkerArray markers;

		for (auto& o : objects)
		{
			visualization_msgs::Marker marker;
			marker.header.frame_id = o.second->m_frame;
			marker.header.stamp = ros::Time();
			marker.ns = o.second->m_name;
			marker.id = 0;
			marker.action = visualization_msgs::Marker::ADD;

			switch (o.second->m_type)
			{
				case PlanningObject::BOX:
					marker.type = visualization_msgs::Marker::CUBE;
					marker.scale.x = o.second->m_size[0];
					marker.scale.y = o.second->m_size[1];
					marker.scale.z = o.second->m_size[2];
					break;
				case PlanningObject::SPHERE:
					marker.type = visualization_msgs::Marker::SPHERE;
					marker.scale.x = o.second->m_size[0];
					break;
				case PlanningObject::PLANE:
					marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
					LOG_ERROR("To be done...");
					break;
				default:
					break;
			}

			marker.color = o.second->m_color.toROSMsg();
			tf::poseEigenToMsg(o.second->m_transformation, marker.pose);
			markers.markers.push_back(marker);
		}

		m_pubObjects.publish(markers);
	}
}

bool PlanningScene::modifyPlanningScene(prm_planner_msgs::ModifyPlanningScene::Request& req,
		prm_planner_msgs::ModifyPlanningScene::Response& res)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	//add
	if (req.mode == prm_planner_msgs::ModifyPlanningScene::Request::ADD_OBJECT)
	{
		for (auto o : req.objects)
		{
			IF_CHECK_MAP_VAR(objects, o.name, object)
			{
				object->second->m_color.setColor(o.color.r, o.color.g, o.color.b, o.color.a);
				object->second->m_frame = o.parent_frame;
				object->second->m_size = o.size;
				object->second->m_type = (PlanningObject::Type) o.type;
				tf::transformMsgToEigen(o.transformation.transform, object->second->m_transformation);
				object->second->m_collisionMatrix = fcl_collision_detection::CollisionMatrix(o.name, o.allowedCollisions);
				LOG_INFO("Modified object! " << *object->second);
			}
			else
			{
				objects[o.name].reset(new PlanningObject(o));
				LOG_INFO("Added object! " << *objects[o.name]);
			}
		}
	}
	//remove
	else if (req.mode == prm_planner_msgs::ModifyPlanningScene::Request::REMOVE_OBJECT)
	{
		for (auto o : req.objects)
			objects.erase(o.name);
	}
	//clear
	else
	{
		objects.clear();
	}

	res.result = true;

	return true;
}

void PlanningScene::filterPointCloud(const ais_point_cloud::MyPointCloudP& cloudIn,
		ais_point_cloud::MyPointCloudP& cloudOut,
		double prmRadius,
		const Eigen::Vector3d& prmCenter)
{
	cloudOut.reset(new ais_point_cloud::MyPointCloud);
	cloudOut->header = cloudIn->header;
	for (auto& it : cloudIn->points)
	{
		Eigen::Vector3d p(it.x, it.y, it.z);
		//m_prmRadius is 0 if the prm is empty (not build yet)
		if (prmRadius == 0 || (p - prmCenter).norm() < prmRadius)
		{
			cloudOut->push_back(it);
		}
	}
}

void PlanningScene::lock()
{
	m_mutex.lock();
}

void PlanningScene::unlock()
{
	m_mutex.unlock();
}

} /* namespace prm_planner */

