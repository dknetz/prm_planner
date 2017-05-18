/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Sep 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: object_interactive_marker.h
 */

#ifndef H1B4A75A2_F4E2_4DA0_B6CC_1A6BE9E01C76
#define H1B4A75A2_F4E2_4DA0_B6CC_1A6BE9E01C76

#include <boost/thread/pthread/mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <ros/ros.h>

namespace prm_planner
{

class GraspableObject;

class ObjectInteractiveMarker: public InteractiveMarker
{
public:
	ObjectInteractiveMarker(boost::shared_ptr<PRMPlanner> planner,
			const std::unordered_map<std::string, boost::shared_ptr<GraspableObject>>& objects,
			const std::string& frame);
	virtual ~ObjectInteractiveMarker();

	virtual void update(const ros::TimerEvent& e);

protected:
	virtual void processFeedbackObjects(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	virtual void processFeedbackTable(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	virtual void initMarkerServer();
	virtual void initMenu();

protected:
	mutable boost::mutex m_mutex;
	ros::NodeHandle m_nodeHandle;
	ros::Timer m_timer;
	interactive_markers::InteractiveMarkerServer* m_server;
	interactive_markers::MenuHandler m_menuHandlerObject;
	interactive_markers::MenuHandler m_menuHandlerDrop;
	std::unordered_map<std::string, boost::shared_ptr<GraspableObject>> m_objects;
	std::unordered_map<std::string, visualization_msgs::InteractiveMarker> m_objectMarkers;
	const std::string c_frame;
	boost::atomic_bool m_sendTable;
	Eigen::Vector3d m_tableCenter;

	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryGrasp;
//	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryToggleOctomap;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryPrintInfo;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryUpdateTable1, m_menuEntryUpdateTable2;

	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryDrop;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryDropSample;
	visualization_msgs::InteractiveMarkerFeedback::_menu_entry_id_type m_menuEntryDropPrint;
};

} /* namespace prm_planner */

#endif /* H1B4A75A2_F4E2_4DA0_B6CC_1A6BE9E01C76 */
