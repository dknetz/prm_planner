/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 28, 2017
 *      Author: kuhnerd
 * 	  Filename: environment.h
 */

#ifndef HEDB72EE7_3379_4CDE_8445_4AB16AD32A2E
#define HEDB72EE7_3379_4CDE_8445_4AB16AD32A2E

#include <ais_definitions/class.h>
#include <ais_point_cloud/point_cloud.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <octomap_msgs/Octomap.h>
#include <ros/publisher.h>
#include <ros/service_server.h>
#include <prm_planner_msgs/ModifyPlanningScene.h>
#include <unordered_map>

FORWARD_DECLARE_N(octomap, OcTree);
FORWARD_DECLARE_N(ais_point_cloud, RGBDImage);

namespace prm_planner
{

FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(PlanningObject);

class PlanningScene
{
public:
	PlanningScene(boost::shared_ptr<ProblemDefinition> pd);
	virtual ~PlanningScene();

	/**
	 * Initializes the ROS publishers for visualization in rviz
	 * and the octomap
	 */
	void init();
	void initROS();
	void initOctomap();

	bool initFromPointCloud();

	void publish();

	void lock();
	void unlock();

	bool modifyPlanningScene(prm_planner_msgs::ModifyPlanningScene::Request& req,
			prm_planner_msgs::ModifyPlanningScene::Response& res);

public:
	boost::shared_ptr<octomap::OcTree> octomap;
	boost::shared_ptr<ais_point_cloud::RGBDImage> rgbd;
	ais_point_cloud::MyPointCloudP processedCloud;
	std::unordered_map<std::string, boost::shared_ptr<PlanningObject>> objects;

private:
	void updateOctomap(const ais_point_cloud::MyPointCloudP cloud);

	static void filterPointCloud(const ais_point_cloud::MyPointCloudP& cloudIn,
			ais_point_cloud::MyPointCloudP& cloudOut,
			double prmRadius,
			const Eigen::Vector3d& prmCenter);

private:
	boost::recursive_mutex m_mutex;

	//publishers
	ros::Publisher m_pubPointCloud;
	ros::Publisher m_pubOctomap;
	ros::Publisher m_pubWorkspace;
	ros::Publisher m_pubObjects;

	boost::shared_ptr<ProblemDefinition> m_problemDefinition;

	//visualization
	octomap_msgs::Octomap m_octomapMsg;
	boost::mutex m_mutexVisualization;

	//workspace
	double m_workspaceRadius;
	Eigen::Vector3d m_workspaceCenter;
};

} /* namespace prm_planner */

#endif /* HEDB72EE7_3379_4CDE_8445_4AB16AD32A2E */
