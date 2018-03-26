/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Mar 29, 2017
 *      Author: kuhnerd
 * 	  Filename: main.cpp
 */

#include <ais_log/log.h>
#include <prm_planner_msgs/ModifyPlanningScene.h>
#include <ros/ros.h>

int main(int argc,
		char** argv)
{
	ros::init(argc, argv, "omnirob_two_boxes");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<prm_planner_msgs::ModifyPlanningScene>("/prm_planner/modify_planning_scene");
	client.waitForExistence();

	prm_planner_msgs::ModifyPlanningScene srv;

	prm_planner_msgs::CollisionObject o1;
	o1.color.r = 1;
	o1.color.g = 0;
	o1.color.b = 0;
	o1.color.a = 1;
	o1.name = "box1";
	o1.parent_frame = "/iiwa/iiwa_0_link";
	o1.size =
	{	0.5, 0.5, 1.0};
	o1.transformation.transform.translation.x = 1.0;
	o1.transformation.transform.translation.y = 0.5;
	o1.transformation.transform.translation.z = 0.5;
	o1.transformation.transform.rotation.w = 1.0;
	o1.transformation.transform.rotation.x = 0.0;
	o1.transformation.transform.rotation.y = 0.0;
	o1.transformation.transform.rotation.z = 0.0;
	o1.type = prm_planner_msgs::CollisionObject::TYPE_BOX;

	srv.request.mode = prm_planner_msgs::ModifyPlanningSceneRequest::ADD_OBJECT;
	srv.request.objects.push_back(o1);

	o1.color.r = 0;
	o1.color.g = 1;
	o1.color.b = 0;
	o1.name = "box2";
	o1.size =
	{	0.5, 0.5, 0.5};
	o1.transformation.transform.translation.x = 1.0;
	o1.transformation.transform.translation.y = -0.5;
	o1.transformation.transform.translation.z = 0.25;
	o1.transformation.transform.rotation.w = 1.0;
	o1.transformation.transform.rotation.x = 0.0;
	o1.transformation.transform.rotation.y = 0.0;
	o1.transformation.transform.rotation.z = 0.0;
	srv.request.objects.push_back(o1);

	if (client.call(srv) && srv.response.result)
	{
		LOG_INFO("Added o1 and o2 to planning scene");
	}

	ros::spin();

	return 0;
}

