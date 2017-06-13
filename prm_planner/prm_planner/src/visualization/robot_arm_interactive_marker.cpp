/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 15, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: interactive_marker.cpp
 */

#include <interactive_markers/interactive_marker_server.h>
#include <ais_definitions/class.h>
#include <ais_log/log.h>
#include <ais_point_cloud/point_cloud.h>
#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <boost/algorithm/string.hpp>
#include <prm_planner_constraints/constraints.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/robot/robot_state_publisher.h>
#include <prm_planner/visualization/robot_trajectory_visualizer.h>
#include <prm_planner/visualization/robot_arm_interactive_marker.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner/visualization/robot_pose_visualizer.h>
#include <fstream>

namespace prm_planner
{

RobotArmInteractiveMarker::RobotArmInteractiveMarker(boost::shared_ptr<Robot> robot,
		boost::shared_ptr<PRMPlanner> planner,
		boost::shared_ptr<Constraint> constraint,
		const std::string& name) :
				InteractiveMarker(planner),
				m_robot(robot),
				m_server(new interactive_markers::InteractiveMarkerServer(m_nodeHandle.getNamespace() + "/" + name)),
				m_fk(m_robot->getChain()),
				m_ikVel(m_robot->getChain()),
				m_constraint(constraint),
				m_firstValid(false),
				c_name(name)
{
	std::vector<urdf::Joint> joints;
	m_robot->getChainJoints(joints);

	KDL::JntArray upper(joints.size()), lower(joints.size());
	for (size_t i = 0; i < joints.size(); ++i)
	{
		if (joints[i].type == urdf::Joint::CONTINUOUS)
		{
			upper(i, 0) = std::numeric_limits<double>::max();
			lower(i, 0) = std::numeric_limits<double>::lowest();
		}
		else
		{
			upper(i, 0) = joints[i].limits->upper - 0.05;
			lower(i, 0) = joints[i].limits->lower + 0.05;
		}
	}

	m_ik = new KDL::ChainIkSolverPos_NR_JL(m_robot->getChain(), lower, upper, m_fk, m_ikVel, 100, 1e-2);

	initMarkerServer();

	//visualize the ik solutions
//	m_poseVisualizer = new RobotPoseVisualizer(m_robot, "ik");
//	if (ParameterServer::visualize)
//		m_poseVisualizer->start();
}

RobotArmInteractiveMarker::~RobotArmInteractiveMarker()
{
	DELETE_VAR(m_server);
//	DELETE_VAR(m_poseVisualizer);
	DELETE_VAR(m_ik);
}

void RobotArmInteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
	switch (feedback->event_type)
	{
		case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
			{
			if (feedback->menu_entry_id == m_menuEntryPlanAndExecute)
			{
				Eigen::Affine3d pose;
				tf::poseMsgToEigen(feedback->pose, pose);

				//why do we get the answer in another frame than the marker?
				Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, m_interactiveMarker.header.frame_id);
				pose = t * pose;

				m_planner->planAndExecute(pose, m_robot->getRootFrame());
			}
			else if (feedback->menu_entry_id == m_menuEntryReset)
			{
				Eigen::Affine3d pose;
				geometry_msgs::Pose tfPose;

				m_robot->getCurrentFK(pose);
				tf::poseEigenToMsg(pose, tfPose);
				m_server->setPose(m_interactiveMarker.name, tfPose);
				m_menuHandler.reApply(*m_server);
				m_server->applyChanges();
				m_firstValid = false;
			}
			else if (feedback->menu_entry_id == m_menuEntryRandomValid)
			{
				Eigen::Affine3d pose;
				geometry_msgs::Pose tfPose;

				m_robot->sampleValidChainEEFPose(pose, m_joints, 0.1);
				tf::poseEigenToMsg(pose, tfPose);
				m_server->setPose(m_interactiveMarker.name, tfPose);
				m_menuHandler.reApply(*m_server);
				m_server->applyChanges();
			}
			else if (feedback->menu_entry_id == m_menuEntrySavePose)
			{
				std::ofstream file;
				file.open("/tmp/joint_poses.txt", std::iostream::app);
				if (!file.is_open())
				{
					LOG_ERROR("Cannot open /tmp/joint_poses.txt");
					return;
				}
				file << std::setprecision(12);
				for (size_t i = 0; i < m_joints.rows(); ++i)
				{
					if (i != 0)
					{
						file << " ";
					}
					file << m_joints.data(i);
				}
				file << std::endl;
				file.close();
			}
			else if (feedback->menu_entry_id == m_menuEntryPrintPose)
			{
				Eigen::Affine3d pose;
				tf::poseMsgToEigen(feedback->pose, pose);

				Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, m_interactiveMarker.header.frame_id);
				pose = t * pose;

				LOG_INFO("Pose: \n" << pose.matrix());
			}
			break;
		}
		case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
			{
			KDL::Frame x;
			Eigen::Affine3d pose;
			tf::poseMsgToEigen(feedback->pose, pose);

			//why do we get the answer in another frame than the marker?
			Eigen::Affine3d t = m_planner->getTransformation(feedback->header.frame_id, m_interactiveMarker.header.frame_id);
			pose = t * pose;

//			if (!m_firstValid && !m_constraint->isValidPose(pose))
//			{
//				m_constraint->findValidPose(pose);
//			}

			tf::transformEigenToKDL(pose, x);

			KDL::JntArray solution;
			bool valid = m_constraint->isValidPose(pose);

			if (valid)
			{
				if (!m_robot->getIKWithInit(m_joints, pose, solution))
				//				if (m_ik->CartToJnt(m_joints, x, solution) < 0)
				{
//					LOG_INFO("No solution found");
					m_server->setPose(m_interactiveMarker.name, m_oldPose);
					m_menuHandler.reApply(*m_server);
					m_server->applyChanges();
				}
				else
				{
//					LOG_INFO(std::setprecision(12) << solution.data.transpose());

					geometry_msgs::Pose p;
					tf::poseEigenToMsg(pose, p);

					if (!m_firstValid)
					{
						m_server->setPose(m_interactiveMarker.name, p);
						m_menuHandler.reApply(*m_server);
						m_server->applyChanges();
					}
					m_firstValid = true;
					m_oldPose = p;
					m_joints = solution;

//					if (ParameterServer::visualize)
//						m_poseVisualizer->setPose(m_joints);
				}
			}
			else if (!valid && m_firstValid)
			{
				m_server->setPose(m_interactiveMarker.name, m_oldPose);
				m_menuHandler.reApply(*m_server);
				m_server->applyChanges();
			}
			break;
		}
		default:
			return;
	}
}

void RobotArmInteractiveMarker::initMarkerServer()
{
	Eigen::Affine3d eef;
	m_robot->getCurrentFK(eef);
//	m_constraint->findValidPose(eef);

	m_joints = m_robot->getKDLChainJointState();

	// create an interactive marker for our server
	m_interactiveMarker.header.frame_id = m_robot->getRootFrame();
	m_interactiveMarker.header.stamp = ros::Time::now();
	m_interactiveMarker.name = c_name + "_pose_marker";
	m_interactiveMarker.scale = 0.3;
	m_interactiveMarker.description = "6-DOF Control";
	tf::poseEigenToMsg(eef, m_interactiveMarker.pose);

	// create a red sphere marker
	visualization_msgs::Marker sphereMarker;
	sphereMarker.type = visualization_msgs::Marker::SPHERE;
	sphereMarker.scale.x = 0.05;
	sphereMarker.scale.y = 0.05;
	sphereMarker.scale.z = 0.05;
	sphereMarker.color = ais_util::Color::red().toROSMsg();

	visualization_msgs::InteractiveMarkerControl sphereControl;
	sphereControl.always_visible = true;
	sphereControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
	sphereControl.markers.push_back(sphereMarker);

	m_interactiveMarker.controls.push_back(sphereControl);

	visualization_msgs::InteractiveMarkerControl control;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "rotate_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_x";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "rotate_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_z";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "rotate_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
	m_interactiveMarker.controls.push_back(control);
	control.name = "move_y";
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	m_interactiveMarker.controls.push_back(control);

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	m_server->insert(m_interactiveMarker, boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));

	initMenu();

	// 'commit' changes and send to all clients
	m_server->applyChanges();
}

void RobotArmInteractiveMarker::update(const ros::TimerEvent& e)
{
	ros::Time now = ros::Time::now();
	std::unordered_map<std::string, double> joints;

	getJoints(joints);

	//set gripper joints
	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();
	if (gripper.get() != NULL)
	{
		const std::unordered_map<std::string, double> gripperJoints = m_robot->getGripper()->getJoints();
		for (const auto& it : gripperJoints)
			joints[it.first] = it.second;
	}

	//set transformations of robot
//	m_statePublisher->publishTransforms(joints, now, m_robot->getName(), "_interactive_marker_" + c_name);
//	m_statePublisher->publishFixedTransforms(m_robot->getName(), "_interactive_marker_" + c_name);

	//send a static transformation to connect this robot with the real robot
	m_transformBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1)), now,
			m_robot->getRootFrame(), m_robot->getRootFrame() + "_interactive_marker_" + c_name));
}

void RobotArmInteractiveMarker::getJoints(std::unordered_map<std::string, double>& joints)
{
	int i = 0;
	for (auto& it : m_robot->getChainJointNames())
	{
		joints[it] = m_joints.data(i++, 0);
	}
}

void RobotArmInteractiveMarker::initMenu()
{
	m_menuEntryPlanAndExecute = m_menuHandler.insert("Plan and execute", boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));
	m_menuEntryReset = m_menuHandler.insert("Set to current robot pose", boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));
	m_menuEntryRandomValid = m_menuHandler.insert("Set to random valid", boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));
	m_menuEntrySavePose = m_menuHandler.insert("Save joint pose (/tmp folder)", boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));
	m_menuEntryPrintPose = m_menuHandler.insert("Print pose", boost::bind(&RobotArmInteractiveMarker::processFeedback, this, _1));
	m_menuHandler.apply(*m_server, m_interactiveMarker.name);
}

} /* namespace prm_planner */
