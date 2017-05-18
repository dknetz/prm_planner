/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: object_interactive_marker.cpp
 */

#include <eigen_conversions/eigen_msg.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/visualization/object_interactive_marker.h>
#include <ais_point_cloud/point_cloud_helpers.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>

namespace prm_planner
{

ObjectInteractiveMarker::ObjectInteractiveMarker(boost::shared_ptr<PRMPlanner> planner,
		const std::unordered_map<std::string, boost::shared_ptr<GraspableObject>>& objects,
		const std::string& frame) :
				InteractiveMarker(planner),
				c_frame(frame),
				m_server(new interactive_markers::InteractiveMarkerServer(m_nodeHandle.getNamespace() + "/object_goals")),
				m_objects(objects),
				m_sendTable(true) //use corresponding button in menu to activate later
{
	initMarkerServer();
	m_timer = m_nodeHandle.createTimer(ros::Duration(1), &ObjectInteractiveMarker::update, this);
}

ObjectInteractiveMarker::~ObjectInteractiveMarker()
{
}

void ObjectInteractiveMarker::update(const ros::TimerEvent& e)
{
	using namespace ais_point_cloud;

	boost::mutex::scoped_lock lock(m_mutex);
	if (!m_sendTable)
	{
		ais_point_cloud::MyPointCloudP cloud = m_planner->getCloud();
		bool foundPlane = true;
		if (cloud.get() != NULL && !cloud->empty())
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

			while (true)
			{
				MyPointCloudP cloudOutliers(new MyPointCloud);

				bool bigEnough; //we dont use it here
				inliers->indices.clear();

				if (!point_cloud_helpers::getPlaneWithOrientation(cloud, inliers, coefficients, bigEnough,
						0.01, 100, ProblemDefinitionManager::getInstance()->getProblemDefinition()->getConfig().upDir, 0.2))
				{
					if (!point_cloud_helpers::extract(cloud, inliers, cloudOutliers, true))
					{
						foundPlane = false;
						break;
					}

					cloud = cloudOutliers;
					continue;
				}

				break;
			}

			if (foundPlane)
			{
				double xMin, yMin;
				double xMax, yMax;

				xMin = yMin = std::numeric_limits<double>::max();
				xMax = yMax = std::numeric_limits<double>::lowest();

				for (auto& it : inliers->indices)
				{
					const ais_point_cloud::MyPointType& p = cloud->points[it];
					if (p.x < xMin)
						xMin = p.x;
					else if (p.x > xMax)
						xMax = p.x;

					if (p.y < yMin)
						yMin = p.y;
					else if (p.y > yMax)
						yMax = p.y;
				}

				Eigen::Vector3d p1(xMin, yMin,
						-(coefficients->values[0] * xMin + coefficients->values[1] * yMin + coefficients->values[3]) / coefficients->values[2]);
				Eigen::Vector3d p2(xMin, yMax,
						-(coefficients->values[0] * xMin + coefficients->values[1] * yMax + coefficients->values[3]) / coefficients->values[2]);
				Eigen::Vector3d p3(xMax, yMin,
						-(coefficients->values[0] * xMax + coefficients->values[1] * yMin + coefficients->values[3]) / coefficients->values[2]);
				Eigen::Vector3d p4(xMax, yMax,
						-(coefficients->values[0] * xMax + coefficients->values[1] * yMax + coefficients->values[3]) / coefficients->values[2]);

				//create marker
				visualization_msgs::InteractiveMarker interactiveMarker;
				interactiveMarker.header.frame_id = c_frame; //we use the name, which is equal to the frame
				interactiveMarker.header.stamp = ros::Time::now();
				interactiveMarker.name = "table";
				interactiveMarker.description = "Table";
				interactiveMarker.scale = 1.0;
				tf::poseEigenToMsg(Eigen::Affine3d::Identity(), interactiveMarker.pose); //it.second->getPose()

				visualization_msgs::Marker tableMarker;
				tableMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;
				tableMarker.color = ais_util::Color::green().toROSMsg();
				tableMarker.scale.x = tableMarker.scale.y = tableMarker.scale.z = 1.0;

				geometry_msgs::Point triangleP1;
				triangleP1.x = p1.x();
				triangleP1.y = p1.y();
				triangleP1.z = p1.z();

				geometry_msgs::Point triangleP2;
				triangleP2.x = p2.x();
				triangleP2.y = p2.y();
				triangleP2.z = p2.z();

				geometry_msgs::Point triangleP3;
				triangleP3.x = p3.x();
				triangleP3.y = p3.y();
				triangleP3.z = p3.z();

				geometry_msgs::Point triangleP4;
				triangleP4.x = p4.x();
				triangleP4.y = p4.y();
				triangleP4.z = p4.z();

				tableMarker.points.push_back(triangleP3);
				tableMarker.points.push_back(triangleP2);
				tableMarker.points.push_back(triangleP1);

				tableMarker.points.push_back(triangleP4);
				tableMarker.points.push_back(triangleP2);
				tableMarker.points.push_back(triangleP3);

				visualization_msgs::InteractiveMarkerControl control;
				control.always_visible = true;
				control.name = "table_control";
				control.orientation.w = 1.0;
				control.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
				control.markers.push_back(tableMarker);
				interactiveMarker.controls.push_back(control);

				//insert marker
				m_server->insert(interactiveMarker);

				//sphere
				m_tableCenter = (p1 + p4) / 2.0;
				geometry_msgs::Point planeCenter;
				planeCenter.x = m_tableCenter.x();
				planeCenter.y = m_tableCenter.y();
				planeCenter.z = m_tableCenter.z();

				//MOVE_PLANE movement in y-z plane
				Eigen::Matrix3d orientation;
				orientation.col(1) = (p2 - p1).normalized();
				orientation.col(2) = (p3 - p1).normalized();
				orientation.col(0) = orientation.col(1).cross(orientation.col(2));
				Eigen::Quaterniond planeOrientation(orientation);

				visualization_msgs::InteractiveMarker interactiveMarkerSphere;
				interactiveMarkerSphere.header.frame_id = c_frame; //we use the name, which is equal to the frame
				interactiveMarkerSphere.header.stamp = ros::Time::now();
				interactiveMarkerSphere.name = "table_sphere";
				interactiveMarkerSphere.description = "Drop Location";
				interactiveMarkerSphere.scale = 1.0;
				tf::poseEigenToMsg(Eigen::Affine3d::Identity(), interactiveMarkerSphere.pose); //it.second->getPose()

				visualization_msgs::Marker sphereMarker;
				sphereMarker.pose.position = planeCenter;
				sphereMarker.type = visualization_msgs::Marker::SPHERE;
				sphereMarker.color = ais_util::Color::yellow().toROSMsg();
				sphereMarker.scale.x = sphereMarker.scale.y = sphereMarker.scale.z = 0.05;

				visualization_msgs::InteractiveMarkerControl controlSphere;
				controlSphere.always_visible = true;
				controlSphere.name = "sphere_control";
				controlSphere.orientation.x = planeOrientation.x();
				controlSphere.orientation.y = planeOrientation.y();
				controlSphere.orientation.z = planeOrientation.z();
				controlSphere.orientation.w = planeOrientation.w();
				controlSphere.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
				controlSphere.markers.push_back(sphereMarker);
				interactiveMarkerSphere.controls.push_back(controlSphere);

				m_server->insert(interactiveMarkerSphere);

				m_menuHandlerDrop.apply(*m_server, interactiveMarkerSphere.name);

				m_server->applyChanges();

				m_sendTable = true;
			}
		}
	}

	visualization_msgs::InteractiveMarker dummy;
	for (auto& it : ObjectManager::getInstance()->getObjects())
	{
		if (it.second->isActive())
		{
			visualization_msgs::InteractiveMarker& m = m_objectMarkers[it.first];

			if (!m_server->get(m.name, dummy))
			{
				m_server->insert(m);
				m_menuHandlerObject.apply(*m_server, m.name);
			}
			m_server->applyChanges();
		}
		else
		{
			m_server->erase(m_objectMarkers[it.first].name);
			m_server->applyChanges();
		}
	}
}

void ObjectInteractiveMarker::processFeedbackObjects(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	boost::mutex::scoped_lock lock(m_mutex);

	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			if (feedback->menu_entry_id == m_menuEntryGrasp)
			{
				m_planner->graspObject(feedback->marker_name);
			}
//			else if (feedback->menu_entry_id == m_menuEntryToggleOctomap)
//			{
//				ObjectManager::getInstance()->toggleRemoveObject(feedback->marker_name);
//			}
			else if (feedback->menu_entry_id == m_menuEntryDropSample)
			{
				m_planner->dropObject(feedback->marker_name, Eigen::Vector3d::Zero());
			}
			else if (feedback->menu_entry_id == m_menuEntryUpdateTable1)
			{
				m_sendTable = false;
			}
			else if (feedback->menu_entry_id == m_menuEntryPrintInfo)
			{
				ObjectManager::getInstance()->getObject(feedback->marker_name)->print();
			}
			break;
	}
}

void ObjectInteractiveMarker::processFeedbackTable(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	boost::mutex::scoped_lock lock(m_mutex);

	//compute position
	Eigen::Affine3d pose;
	tf::poseMsgToEigen(feedback->pose, pose);

	//why do we get the answer in another frame than the marker?
	Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, c_frame);
	pose = t * pose;

	Eigen::Vector3d pos = pose.translation();
	pos += m_tableCenter;

	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			if (feedback->menu_entry_id == m_menuEntryDrop)
			{
				m_planner->dropObject(feedback->marker_name, pos);
			}
			else if (feedback->menu_entry_id == m_menuEntryDropPrint)
			{
				LOG_INFO("Drop Location: [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]");
			}
			else if (feedback->menu_entry_id == m_menuEntryUpdateTable2)
			{
				m_sendTable = false;
			}
			break;
	}
}

void ObjectInteractiveMarker::initMarkerServer()
{
	boost::mutex::scoped_lock lock(m_mutex);

	for (auto& it : ObjectManager::getInstance()->getObjects())
	{
		// create an interactive marker for our server
		visualization_msgs::InteractiveMarker interactiveMarker;
		interactiveMarker.header.frame_id = it.second->c_params.objectFrameName; //we use the name, which is equal to the frame
		interactiveMarker.header.stamp = ros::Time(0);
		interactiveMarker.name = it.first;
		interactiveMarker.description = "Object " + it.first;
		interactiveMarker.scale = 1.0;
		tf::poseEigenToMsg(Eigen::Affine3d::Identity(), interactiveMarker.pose); //it.second->getPose()

		// create a red sphere marker
		visualization_msgs::Marker objectMarker;
		objectMarker.id = 0;
		objectMarker.ns = "marker_" + it.first;
		objectMarker.pose.orientation.w = 1.0;
		objectMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
		objectMarker.mesh_resource = "file://" + it.second->c_params.meshFilename;
		objectMarker.color = ais_util::Color::red().toROSMsg();
		objectMarker.scale.x = objectMarker.scale.y = objectMarker.scale.z = 1.0;

		visualization_msgs::InteractiveMarkerControl control;
		control.always_visible = true;
		control.name = it.first;
		control.orientation.w = 1.0;
		control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
		control.markers.push_back(objectMarker);
		interactiveMarker.controls.push_back(control);

		m_objectMarkers[it.first] = interactiveMarker;

		// add the interactive marker to our collection &
		// tell the server to call processFeedback() when feedback arrives for it
		m_server->insert(interactiveMarker, boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));
	}

	initMenu();

	// 'commit' changes and send to all clients
	m_server->applyChanges();
}

void ObjectInteractiveMarker::initMenu()
{
	m_menuEntryGrasp = m_menuHandlerObject.insert("Grasp", boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));
//	m_menuEntryToggleOctomap = m_menuHandlerObject.insert("Remove from/Show in octomap",
//			boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));
	m_menuEntryDropSample = m_menuHandlerObject.insert("Drop (Sample Position Automatically)",
			boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));
	m_menuEntryPrintInfo = m_menuHandlerObject.insert("Print Information", boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));
	m_menuEntryUpdateTable1 = m_menuHandlerObject.insert("Update Table", boost::bind(&ObjectInteractiveMarker::processFeedbackObjects, this, _1));

	m_menuEntryDrop = m_menuHandlerDrop.insert("Drop", boost::bind(&ObjectInteractiveMarker::processFeedbackTable, this, _1));
	m_menuEntryDropPrint = m_menuHandlerDrop.insert("Print Pose", boost::bind(&ObjectInteractiveMarker::processFeedbackTable, this, _1));
	m_menuEntryUpdateTable2 = m_menuHandlerObject.insert("Update Table", boost::bind(&ObjectInteractiveMarker::processFeedbackTable, this, _1));

	for (auto& it : m_objectMarkers)
	{
		m_menuHandlerObject.apply(*m_server, it.second.name);
	}
}

} /* namespace prm_planner */
