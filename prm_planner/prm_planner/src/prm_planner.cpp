/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 16, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_planner.cpp
 */

#include <ais_log/log.h>
#include <ais_util/progress_bar.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <boost/filesystem.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <kdl/segment.hpp>
#include <octomap_msgs/conversions.h>
#include <prm_planner/collision_detection/collision_detector.h>

#include <prm_planner/controllers/helpers.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_constraints/constraints.h>
#include <prm_planner/execution/follow_joint_trajectory_executer.h>
#include <prm_planner/execution/robot_executer.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner_robot/path.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/robot/trajectory_validator.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner/visualization/robot_trajectory_visualizer.h>
#include <prm_planner/visualization/interactive_marker.h>
#include <prm_planner/visualization/robot_arm_interactive_marker.h>

#define SCOPE_LOCK_ALL() boost::recursive_mutex::scoped_lock lock1(m_mutexPlan);\
	boost::recursive_mutex::scoped_lock lock2(m_mutexPublish);\
	boost::recursive_mutex::scoped_lock lock3(m_mutexUpdate);\
	boost::recursive_mutex::scoped_lock lock4(m_mutex);\
	boost::recursive_mutex::scoped_lock lock5(m_mutexOctomap);\

namespace prm_planner
{

PRMPlanner::PlannerParameters::PlannerParameters() :
				active(true)
{
}

PRMPlanner::PRMPlanner(PlannerParameters params) :
				m_executer(NULL),
				m_counter(0),
				m_envInitialized(false),
				m_runUpdate(false),
				m_runPlan(false),
				m_planningSuccess(true),
				m_problemDefinitionManager(NULL),
				m_objectManager(NULL),
				m_goalActionServer(NULL),
				m_active(params.active),
				m_debugIsCollisionDetectionActive(false),
				m_useNextPlanningObjectPoses(false)
{
}

PRMPlanner::~PRMPlanner()
{
	stopThreads(true);
	DELETE_VAR(m_goalActionServer);

	m_robot.reset();
	m_path.reset();
	m_executer.reset();
	m_planningScene.reset();
	m_validator.reset();
	m_robotTrajectoryVisualizer.reset();
}

void PRMPlanner::reset(bool resetPD)
{
	SCOPE_LOCK_ALL();

	m_executer.reset();
	m_validator.reset();
	m_path.reset();

	if (resetPD)
	{
		m_problemDefinition->reset();
	}

	//create empty planning scene
	m_planningScene.reset(new PlanningScene(m_problemDefinition));

	resetExecuter();

	m_counter = 0;
	m_planningSuccess = true;
	m_envInitialized = false;
}

void PRMPlanner::init()
{
	//	boost::recursive_mutex::scoped_lock lock(m_mutex);

	//start base only if in active mode
	LOG_DEBUG("Init Base");
	if (m_active)
		initBase();
	startRosBaseInterface(ParameterServer::startRosSpinner);

	LOG_DEBUG("Init objects");
	initObjectManager();

	LOG_DEBUG("Init problem");
	initProblemDefinition(); //initialize after ros and base

	LOG_DEBUG("Init robot");
	initRobot();

	LOG_DEBUG("Init visualization");
	initVisualization();

	//don't reset problem definition, because it is already reseted
	reset(false);

	//Start threads only in active mode.
	//Additionally, getting the initial object state of an
	//potential object in the gripper only works in active
	//state
	if (m_active)
	{
		LOG_INFO("Starting Threads");
		startThreads();
		initObjectStates();
	}
	else
	{
		LOG_INFO("Planner was started in non-active mode, i.e., you first need to call the "
				"service to activate the planner!");
	}

	LOG_DEBUG("Init ROS");
	initROS();
	initDynamicReconfigure();

	LOG_DEBUG("Planner initialized successfully!");
}

void PRMPlanner::initBase()
{
	//get planning frame from parameter server
	IF_CHECK_MAP_VAR(ParameterServer::problemConfigs, ParameterServer::problem, problem)
	{
		//vrep sends mirrored images?!
		setMirrorImages(ParameterServer::connectedToVRep);
		useDepthImage(ParameterServer::topicCameraDepth, ParameterServer::topicCameraDepthInfo);
		setFrameNamesOfCamera(ParameterServer::topicCameraPrefix + "_depth_optical_frame",
				problem->second.planningFrame);

		initTransformListener();

		//wait some time until tfs are available
//		usleep(500000);

		m_nodeHandle = ros::NodeHandle();
		LOG_INFO("Starting Planner in namespace " << m_nodeHandle.getNamespace() <<" in active mode!");
	}
	else
	{
		LOG_FATAL("Unknown Problem Configuration: "<<ParameterServer::problem);
		exit(2310);
	}
}

void PRMPlanner::initRobot()
{
	m_robot = m_problemDefinitionManager->getRobot();

	//setup hand joint state callbacks and joint publisher
	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();
	if (gripper.get() != NULL)
	{
		//subscriber which also passes the arm and hand name to the callback
		m_subsHandJointStates.push_back(m_nodeHandle.subscribe<sensor_msgs::JointState>(gripper->c_parameters.jointStateTopic, 50,
				boost::bind(&PRMPlanner::callbackHandJointState, this, _1, gripper->c_parameters.name)));
	}

//		m_pubJointStates[arm.first] = m_nodeHandle.advertise<sensor_msgs::JointState>("/" + arm.first + "/joint_states", 1);

	//check if joint states were received
	bool gotJoints = false;
	int counter = 0;
	while (!gotJoints && ros::ok())
	{
		gotJoints = m_robot->waitForData();
		++counter;

		if (counter > 100)
		{
			LOG_WARNING("Still don't receive any joint data of the robot");
		}
	}
}

void PRMPlanner::initProblemDefinition()
{
	ProblemDefinitionManager::getInstance()->init(ParameterServer::problem, shared_from_this());
	m_problemDefinitionManager = ProblemDefinitionManager::getInstance();
	m_problemDefinition = m_problemDefinitionManager->getProblemDefinition();
}

void PRMPlanner::initObjectManager()
{
	m_objectManager = ObjectManager::getInstance();
}

void PRMPlanner::initObjectStates()
{
	auto& objects = m_objectManager->getObjects();
	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();

	if (gripper.get() == NULL)
		return;

	//gripper already has an object
	if (!gripper->getCurrentObject().empty())
		return;

	for (auto& object : objects)
	{
		if (object.second->isActive() && object.second->isInGripper(gripper))
		{
			gripper->setCurrentObject(object.first);

			Eigen::Affine3d transformation;
			if (!object.second->getTransformationFromFrame(gripper->c_parameters.graspFrame, transformation))
				continue;

			gripper->setTGripperToObject(transformation);

			LOG_INFO("Object " << object.first << " was found in gripper " << gripper->c_parameters.name);
		}
	}
}

void PRMPlanner::initVisualization()
{
	m_robotTrajectoryVisualizer.reset(new RobotTrajectoryVisualizer(m_robot, "trajectory"));
	m_robotTrajectoryVisualizer->start();
}

void PRMPlanner::initROS()
{
	if (ParameterServer::startPublishers)
	{
		m_pubTrajectory = m_nodeHandle.advertise<nav_msgs::Path>("path", 1);
		ais_util::ProgressBar::activateRosPublisher("/prm_planner/progress");
	}

	if (ParameterServer::startSubscribers)
	{
		m_goalActionServer = new actionlib::SimpleActionServer<prm_planner_msgs::GoalAction>(
				m_nodeHandle, "goals", boost::bind(&PRMPlanner::callbackGoal, this, _1), false);
		m_goalActionServer->start();

		m_serviceServerSetState = m_nodeHandle.advertiseService("set_state",
				&PRMPlanner::callbackSetState, this);
		m_serviceServerGetImage = m_nodeHandle.advertiseService("get_image",
				&PRMPlanner::callbackGetImage, this);
		m_serviceServerSetObjectPoseType = m_nodeHandle.advertiseService("set_object_pose_type",
				&PRMPlanner::callbackSetObjectPoseType, this);
		m_serviceModifyPlanningScene = m_nodeHandle.advertiseService("modify_planning_scene",
				&PRMPlanner::callbackModifyPlanningScene, this);
	}
}

void PRMPlanner::initDynamicReconfigure()
{
	m_dynReconfigureCallback = boost::bind(&PRMPlanner::callbackDynamicReconfigure, this, _1, _2);
	m_dynReconfigureServer.setCallback(m_dynReconfigureCallback);
}

void PRMPlanner::startThreads()
{
	m_threadUpdate = boost::thread(&PRMPlanner::threadedUpdate, this);

	//Starts the visualization thread
	if (ParameterServer::startPublishers && ParameterServer::visualize)
		m_threadVisPublish = boost::thread(&PRMPlanner::threadedVisualizationPublisher, this);

	//Initializes the markers. We don't need to store the results, they
	//will be stored in the problem definition and update themself.
	m_problemDefinition->getInteractiveMarkers();
}

void PRMPlanner::stopThreads(bool stopSpinner)
{
	if (stopSpinner && spinner != NULL)
	{
		spinner->stop();
	}

	//stop threads
	m_threadPlan.interrupt();
	m_threadUpdate.interrupt();
	m_threadVisPublish.interrupt();

	if (m_executer.get() != NULL)
		m_executer->interrupt();

	m_threadPlan.join();
	m_threadUpdate.join();
	m_threadVisPublish.join();

	m_runPlan = false;
	m_envInitialized = false;
}

void PRMPlanner::callbackRGBDImage(ais_point_cloud::RGBDImage::Ptr& image)
{
	if (!m_active)
		return;

	boost::recursive_mutex::scoped_lock lock(m_mutex);

	//wait until the problem definition was initialized
	if (!m_problemDefinition)
		return;

	//TODO: we wait a bit until self filter got joint state
	if (m_counter < 5)
	{
		++m_counter;
		return;
	}

	//if we do not need an image abort it

	if (m_counter >= 5 && !m_runUpdate)
	{
		if (!ParameterServer::receiveImage())
			return;

		m_planningScene->rgbd = image;
		m_runUpdate = true;
	}

	++m_counter;
}

void PRMPlanner::callbackHandJointState(const ros::MessageEvent<sensor_msgs::JointState> event,
		const std::string& hand)
{
	if (!m_active)
		return;

	std::unordered_map<std::string, double> state;

	sensor_msgs::JointStateConstPtr js = event.getConstMessage();
	std::vector<std::string> joints = m_robot->getChainJointNames();

	if (js->name.empty())
		return;

	//other joint values
	bool foundArmJoints = false;
	for (auto& it : js->name)
	{
		if (std::find(joints.begin(), joints.end(), it) != joints.end())
		{
			foundArmJoints = true;
		}
	}

	if (!foundArmJoints)
	{
		for (size_t i = 0; i < js->name.size(); ++i)
		{
			state[js->name[i]] = js->position[i];
		}

		m_robot->getGripper()->setJoints(state);
	}
}

void PRMPlanner::callbackGoal(const prm_planner_msgs::GoalGoalConstPtr& goal)
{
	if (!m_active)
	{
		m_goalASResult.success = false;
		m_goalASResult.final_state = "Cannot run plan because planner is inactive!";
		m_goalActionServer->setSucceeded(m_goalASResult);
		return;
	}

	bool result = true;

	Eigen::Affine3d goalPose;
	tf::poseMsgToEigen(goal->goal, goalPose);

	//interrupt current planning
	if (m_runPlan)
		m_threadPlan.interrupt();

	if (goal->action == prm_planner_msgs::GoalGoal::ACTION_MOVE)
	{
		LOG_INFO("Action Interface: Move");
		result = planAndExecuteSync(goalPose);
	}
	else if (goal->action == prm_planner_msgs::GoalGoal::ACTION_MOVE_REL)
	{
		LOG_INFO("Action Interface: Move relative");
		result = planAndExecuteRelSync(goalPose);
	}
	else if (goal->action == prm_planner_msgs::GoalGoal::ACTION_GRASP)
	{
		std::string objectName = goal->object_name;
		LOG_INFO("Action Interface: Grasp object " << objectName);

		result = graspObjectSync(objectName);
	}
	else if (goal->action == prm_planner_msgs::GoalGoal::ACTION_DROP)
	{
		std::string objectName = goal->object_name;
		LOG_INFO("Action Interface: Drop object " << objectName);

		result = dropObjectSync(objectName, goalPose.translation());
	}
	else if (goal->action == prm_planner_msgs::GoalGoal::ACTION_CUSTOM)
	{
		LOG_INFO("Action Interface: Custom");
		m_mutex.lock();
		m_planningInputStrings = goal->str;
		m_mutex.unlock();
		result = planAndExecuteSync(goalPose);
	}
	else
	{
		LOG_ERROR("Unknown action!");
		m_goalASResult.success = false;
		m_goalASResult.final_state = "Unknown action: " + goal->action;
		m_goalActionServer->setAborted(m_goalASResult);
		return;
	}

	m_goalASFeedback.progress = 0.1;
	m_goalASFeedback.text = "Planning";
	m_goalActionServer->publishFeedback(m_goalASFeedback);

	ros::Rate r(1000);
	while (ros::ok() && m_runPlan)
	{
		if (m_goalActionServer->isPreemptRequested())
		{
			m_threadPlan.interrupt();
			m_threadPlan.join();

			m_goalASFeedback.progress = 1.0;
			m_goalASFeedback.text = "Preempt requested. Stopping planning thread...";
			m_goalActionServer->publishFeedback(m_goalASFeedback);
			m_goalASResult.success = result;
			m_goalActionServer->setAborted(m_goalASResult);
			return;
		}

		r.sleep();
	}

	result &= isPlanningSuccess();
	m_goalASResult.success = result;
	if (result)
		m_goalActionServer->setSucceeded(m_goalASResult);
	else
		m_goalActionServer->setAborted(m_goalASResult);
}

bool PRMPlanner::callbackSetState(prm_planner_msgs::SetState::Request& req,
		prm_planner_msgs::SetState::Response& res)
{
	if (!req.active && m_active)
	{
		LOG_DEBUG("Stopping callbacks and threads...");

		stopThreads(false);
		m_executer.reset();
		m_active = false;
		m_runUpdate = false;
		deactivateImageCallbacks();

		LOG_DEBUG("Done!");
	}
	else if (req.active && !m_active)
	{
		{
			LOG_DEBUG("Starting callbacks and threads...");

			m_active = true;
			m_runUpdate = false;
			m_envInitialized = false;
			initBase();
			startThreads();
			initObjectStates();
			resetExecuter();

			LOG_DEBUG("Done!");
		}

		//wait until environment was updated
		ros::Rate r(1000);
		while (!m_envInitialized)
		{
			r.sleep();
		}
	}

	res.new_running_state = m_active;

	return true;
}

bool PRMPlanner::callbackGetImage(prm_planner_msgs::GetImage::Request& req,
		prm_planner_msgs::GetImage::Response& res)
{
	ParameterServer::setReceiveImage();
	ros::Rate r(1000);
	while (!ParameterServer::hasImageReceived())
		r.sleep();

	res.received = true;
	return true;
}

bool PRMPlanner::callbackSetObjectPoseType(prm_planner_msgs::SetObjectPoseType::Request& req,
		prm_planner_msgs::SetObjectPoseType::Response& res)
{

	switch (req.mode)
	{
		case prm_planner_msgs::SetObjectPoseType::Request::UPDATE_OBJECT_POSES:
			LOG_INFO("Update object poses")
			;

			m_neglectObjectPoseUpdates = req.neglectedObjects;
			m_objectManager->setNeglectObject(req.neglectedObjects);
			m_useNextPlanningObjectPoses = false;
			break;
		case prm_planner_msgs::SetObjectPoseType::Request::USE_CURRENT_POSE_WITHOUT_UPDATE:
			{
			LOG_INFO("Use current object pose (no update starting now)");

			for (auto& it : req.neglectedObjects)
			{
				LOG_INFO("  - " << it);
			}

			//wait until all objects are active
			ros::Time n = ros::Time::now();
			ros::Rate r(10);

			bool available = true;
			while ((ros::Time::now() - n).toSec() < 60 && ros::ok())
			{
				available = true;
				for (auto& it : req.neglectedObjects)
				{
					boost::shared_ptr<GraspableObject> o = m_objectManager->getObject(it);
					if (o.get() == NULL)
					{
						res.success = false;
						return true;
					}

					o->lock();
					o->updatePoseFromTF();
					o->updateBoundingBox();
					available &= o->isActive();
					if (available)
					{
						o->setDoUpdate(false);
						LOG_INFO(m_nodeHandle.getNamespace() << " "<<o->c_params.name<<" "<<o->getPose().matrix());
					}
					o->unlock();
				}

				if (available)
					break;

				r.sleep();
			}

			if (available)
			{
				m_neglectObjectPoseUpdates = req.neglectedObjects;
				m_objectManager->setNeglectObject(req.neglectedObjects);
				m_useNextPlanningObjectPoses = false;

				res.success = true;
				return true;
			}
			else
			{
				LOG_INFO("not available");
				res.success = false;
				return true;
			}

			break;
		}
		case prm_planner_msgs::SetObjectPoseType::Request::USE_OBJECT_POSES_OF_NEXT_PLANNING_WITHOUT_UPDATE:
			m_neglectObjectPoseUpdates = req.neglectedObjects;
			m_useNextPlanningObjectPoses = true;

			LOG_INFO("Use current object pose (no update after next plan())")
			;

			for (auto& it : m_neglectObjectPoseUpdates)
			{
				LOG_INFO("  - " << it);
			}
			break;
	}
	res.success = true;
	return true;
}

void PRMPlanner::callbackDynamicReconfigure(PRMPlannerConfig& config,
		uint32_t level)
{
	boost::recursive_mutex::scoped_lock lock(m_mutexUpdate);

	//activate
	if (config.activate_debug_collision_checks && !m_debugIsCollisionDetectionActive)
	{
		LOG_INFO("ACTIVATING DEBUG FEATURE: COLLISION DETECTION!");

		//create new collision detection
		m_debugCollisionDetection.reset(new CollisionDetector(m_robot, m_planningScene));
		m_debugIsCollisionDetectionActive = true;
		m_debugCollisionDetectionInteractiveMarker.reset(new RobotArmInteractiveMarker(
				m_robot, shared_from_this(), m_problemDefinition->getConstraint(), "collision"));
		m_threadDebugCollisionChecks = boost::thread(&PRMPlanner::threadDebugCollisionChecks, this);
	}
	else if (!config.activate_debug_collision_checks && m_debugIsCollisionDetectionActive)
	{
		LOG_INFO("DEACTIVATING DEBUG FEATURE: COLLISION DETECTION!");

		//removing collision detection
		m_debugCollisionDetection.reset();
		m_debugIsCollisionDetectionActive = false;
		m_threadDebugCollisionChecks.interrupt();
		m_threadDebugCollisionChecks.join();
		m_debugCollisionDetectionInteractiveMarker.reset();
	}

	//set save path option
	if (m_executer.get() != NULL)
	{
		m_executer->setPathFileName(config.save_path_to_disk_filename);
		m_executer->setSavePath(config.save_path_to_disk);
	}
}

bool PRMPlanner::callbackModifyPlanningScene(prm_planner_msgs::ModifyPlanningScene::Request& req,
		prm_planner_msgs::ModifyPlanningScene::Response& res)
{
	bool result = m_planningScene->modifyPlanningScene(req, res);

	if (result)
	{
		m_problemDefinition->update(m_planningScene);
		return true;
	}

	return false;
}

void PRMPlanner::setRGBDImage(ais_point_cloud::RGBDImage::Ptr& image)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (m_runUpdate)
	{
		LOG_ERROR("update is currently running");
		return;
	}

	m_planningScene->rgbd = image;
	m_runUpdate = true;
}

void PRMPlanner::threadedUpdate()
{
	ros::Rate r(30);
	int i = 0;
	while (ros::ok() && !boost::this_thread::interruption_requested())
	{
		{
			boost::recursive_mutex::scoped_lock lock(m_mutexUpdate);

			m_mutex.lock();
			if (m_runUpdate && m_planningScene.get() != NULL && m_planningScene->rgbd.get() != NULL)
			{
				m_mutex.unlock();

				if (!m_planningScene->initFromPointCloud())
				{
					m_mutex.lock(); //unlock after if
					m_runUpdate = false;
				}
				else
				{
					//update octomap and kdtree
					m_problemDefinition->update(m_planningScene);

					m_mutex.lock(); //unlock after if
					m_envInitialized = true;
					m_runUpdate = false;

					//set corresponding variable on the parameter server
					//to disable continous image data
					ParameterServer::setImageReceived();
				}
			}
			m_mutex.unlock();

			++i;

			//update tfs once a second
			if (i % 30 == 0 && m_objectManager != NULL)
			{
				m_objectManager->updatePosesFromTF();
				initObjectStates(); //check if object is in gripper
				i = 0;
			}
		}

		r.sleep();
	}
}

void PRMPlanner::concurrentPlan(const bool executeTrajectory)
{
	boost::shared_ptr<Path> path;

	m_planningSuccess = true;

	try
	{
		PlanningParameters p;
		p.input = m_planningInputStrings;

		p.mode = m_planningMode;
		p.objectName = m_planningObject;
		p.useCollisionDetection = ParameterServer::useCollisionDetection;
//		p.directConnectionRequired
		if (m_problemDefinition->plan(m_goalPose, path, p))
		{
			LOG_INFO("Found plan");
			{
				boost::recursive_mutex::scoped_lock lock(m_mutexPlan);
				m_path = path;
			}

			LOG_INFO("Planning finished");

			//deactivate update of object poses if required
			if (m_useNextPlanningObjectPoses)
			{
				m_useNextPlanningObjectPoses = false;
				m_objectManager->setNeglectObject(m_neglectObjectPoseUpdates);
			}

			if (executeTrajectory)
				execute(path);
		}
		else
		{
			LOG_INFO("Found no plan");
			boost::recursive_mutex::scoped_lock lock(m_mutexPlan);
			m_path.reset();
			m_planningSuccess = false;
		}
	}
	//catch if there was an interruption
	catch (const boost::thread_interrupted&)
	{
	}

	m_runPlan = false;
}

void PRMPlanner::threadedVisualizationPublisher()
{
	ros::Rate r(25);

	while (ros::ok() && !boost::this_thread::interruption_requested())
	{
		//lock/unlock to avoid too long locks
		{
			boost::recursive_mutex::scoped_lock lock(m_mutexPublish);

			if (m_executer.get() != NULL)
			{
				m_executer->publish();
			}

			m_mutex.lock();
			//paths
			if (m_path)
			{
				nav_msgs::PathConstPtr path = m_path->getRosPath();
				if (path.get() != NULL)
				{
					m_pubTrajectory.publish(path);
				}
			}
			m_mutex.unlock();

			m_mutex.lock();
			m_objectManager->publish();
			m_mutex.unlock();

			m_mutex.lock();
			m_problemDefinition->publish();
			m_mutex.unlock();

			m_mutex.lock();
			m_planningScene->publish();
			m_mutex.unlock();
		}

		r.sleep();
	}
}

bool PRMPlanner::plan(const Eigen::Affine3d& goalPose,
		boost::shared_ptr<Path>& path,
		const std::string frame)
{
	if (!m_active)
		return false;

	LOG_INFO("start planer");

	Eigen::Affine3d transformedGoal = goalPose;
	if (!frame.empty())
	{
		Eigen::Affine3d t;
		if (!getRosTransformationWithResult(frame, m_problemDefinition->getFrame(), t))
		{
			t.setIdentity();
		}

		transformedGoal = t * transformedGoal;
	}

	{
		boost::recursive_mutex::scoped_lock lock(m_mutex);

		//interrupt current planning
		m_threadPlan.interrupt();
		m_threadPlan.join();

		m_goalPose = transformedGoal;
		m_runPlan = true;
		m_planningMode = Default;
		m_threadPlan = boost::thread(&PRMPlanner::concurrentPlan, this, true);
	}

	ros::Rate r(1000);
	while (m_runPlan)
	{
		r.sleep();
	}

	boost::recursive_mutex::scoped_lock lock(m_mutex);
	path = m_path;

	return isPlanningSuccess();
}

void PRMPlanner::execute(const boost::shared_ptr<Path>& path)
{
	//nothing to execute, e.g., start and goal pose are similar
	if (path.get() == NULL)
	{
		return;
	}

	//do nothing in inactive mode
	if (!m_active)
	{
		return;
	}

	//send path to trajectory visualization in rviz
	if (ParameterServer::visualize)
	{
		m_robotTrajectoryVisualizer->setTrajectory(m_path);
	}

	//don't use path after execution if you want the original version,
	//because setNewPath will add additional waypoints and deletes the
	//trajectories
	if (m_executer.get() != NULL)
	{
		LOG_DEBUG("execute");
		m_executer->executePath(path);
	}
}

bool PRMPlanner::planAndExecute(const Eigen::Affine3d& goalPose,
		const std::string frame)
{
	if (!m_active)
		return false;

	Eigen::Affine3d transformedGoal = goalPose;
	if (!frame.empty())
	{
		Eigen::Affine3d t;
		if (!getRosTransformationWithResult(frame, m_problemDefinition->getFrame(), t))
		{
			t.setIdentity();
		}

		transformedGoal = t * transformedGoal;
	}

	boost::recursive_mutex::scoped_lock lock(m_mutex);

	//interrupt current planning
	m_threadPlan.interrupt();
	m_threadPlan.join();

	m_goalPose = transformedGoal;
	m_runPlan = true;
	m_planningMode = Default;
	m_threadPlan = boost::thread(&PRMPlanner::concurrentPlan, this, true);

	return isPlanningSuccess();
}

bool PRMPlanner::planAndExecuteRel(const Eigen::Affine3d& goalPose)
{
	if (!m_active)
		return false;

	Eigen::Affine3d currentPose;
	m_robot->getCurrentFK(currentPose);

	Eigen::Affine3d goalInPlanningFrame = currentPose * goalPose;

	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_threadPlan.interrupt();
	m_threadPlan.join();

	m_goalPose = goalInPlanningFrame;
	m_runPlan = true;
	m_planningMode = Default;
	m_threadPlan = boost::thread(&PRMPlanner::concurrentPlan, this, true);

	return isPlanningSuccess();
}

bool PRMPlanner::planAndExecuteSync(const Eigen::Affine3d& goalPose,
		const std::string frame)
{
	if (!m_active)
		return false;

	planAndExecute(goalPose, frame);

	ros::Rate r(1000);
	while (ros::ok() && m_runPlan)
	{
		r.sleep();
	}

	if (!isPlanningSuccess())
		return false;

	if (m_executer.get() != NULL)
	{
		while (ros::ok() && !m_executer->isGoalReached() && !m_executer->hasErrors())
		{
			if (m_goalActionServer->isPreemptRequested())
			{
				LOG_INFO("Aborting current goal");
				m_executer->stopMotion();
			}

			r.sleep();
		}

		return m_executer->isGoalReached() && !m_executer->hasErrors();
	}

	return true;
}

bool PRMPlanner::planAndExecuteRelSync(const Eigen::Affine3d& goalPose)
{
	if (!m_active)
		return false;

	planAndExecuteRel(goalPose);

	ros::Rate r(1000);
	while (ros::ok() && m_runPlan)
	{
		r.sleep();
	}

	if (!isPlanningSuccess())
		return false;

	if (m_executer.get() != NULL)
	{
		while (ros::ok() && !m_executer->isGoalReached() && !m_executer->hasErrors())
		{
			if (m_goalActionServer->isPreemptRequested())
				m_executer->stopMotion();

			r.sleep();
		}

		return m_executer->isGoalReached() && !m_executer->hasErrors();
	}

	return true;
}

bool PRMPlanner::planAndExecutePD()
{
	LOG_ERROR("DEPRACTED!");
	return false;
}

bool PRMPlanner::graspObject(const std::string& name)
{
	if (!m_active)
		return false;

	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_threadPlan.interrupt();
	m_threadPlan.join();

	m_runPlan = true;
	m_planningMode = GraspObject;
	m_planningObject = name;
	m_threadPlan = boost::thread(&PRMPlanner::concurrentPlan, this, true);

	return true;
}

bool PRMPlanner::graspObjectSync(const std::string& name)
{
	if (!graspObject(name))
		return false;

	ros::Rate r(1000);
	while (ros::ok() && m_runPlan)
	{
		r.sleep();
	}

	if (!isPlanningSuccess())
		return false;

	if (m_executer.get() != NULL)
	{
		while (ros::ok() && !m_executer->isGoalReached() && !m_executer->hasErrors())
		{
			if (m_goalActionServer->isPreemptRequested())
				m_executer->stopMotion();

			r.sleep();
		}

		return m_executer->isGoalReached() && !m_executer->hasErrors();
	}

	return true;
}

bool PRMPlanner::dropObject(const std::string& name,
		const Eigen::Vector3d& pos)
{
	if (!m_active)
		return false;

	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_threadPlan.interrupt();
	m_threadPlan.join();

	m_runPlan = true;
	m_planningMode = DropObject;
	m_goalPose.translation() = pos;
	m_planningObject = name;
	m_threadPlan = boost::thread(&PRMPlanner::concurrentPlan, this, true);

	return true;
}

bool PRMPlanner::dropObjectSync(const std::string& name,
		const Eigen::Vector3d& pos)
{
	if (!dropObject(name, pos))
		return false;

	ros::Rate r(1000);
	while (ros::ok() && m_runPlan)
	{
		r.sleep();
	}

	if (!isPlanningSuccess())
		return false;

	if (m_executer.get() != NULL)
	{
		while (ros::ok() && !m_executer->isGoalReached() && !m_executer->hasErrors())
		{
			if (m_goalActionServer->isPreemptRequested())
				m_executer->stopMotion();

			r.sleep();
		}

		return m_executer->isGoalReached() && !m_executer->hasErrors();
	}

	return true;
}

bool PRMPlanner::isExecutingMotion() const
{
	if (!m_active)
		return false;

	if (m_executer.get() != NULL)
		return false;

	boost::recursive_mutex::scoped_lock lock(m_mutex);

	return !m_executer->hasErrors() && !m_executer->isGoalReached();
}

void PRMPlanner::stopMotion()
{
	m_threadPlan.interrupt();
	m_threadPlan.join();
	m_executer->stopMotion();
}

boost::shared_ptr<Robot> PRMPlanner::getRobot() const
{
	return m_robot;
}

bool PRMPlanner::isReady() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_envInitialized;
}

//void PRMPlanner::buildOctomap(const boost::shared_ptr<octomap::OcTree>& octomap,
//		ais_point_cloud::MyPointCloudP& cloud)
//{
//	cloud.reset(new ais_point_cloud::MyPointCloud);
//	std::vector<Eigen::Vector3d> points;
//	octomap::OcTreeNode* node;
//	for (auto it = octomap->begin_leafs(), end = octomap->end_leafs(); it != end; ++it)
//	{
//		if ((node = octomap->search(it.getKey())) == NULL)
//		{
//			continue;
//		}
//
//		if (!octomap->isNodeOccupied(node))
//		{
//			continue;
//		}
//
//		ais_point_cloud::MyPointType p;
//		octomap::point3d octopoint = it.getCoordinate();
//		p.x = octopoint.x();
//		p.y = octopoint.y();
//		p.z = octopoint.z();
//		cloud->push_back(p);
//		points.push_back(Eigen::Vector3d(p.x, p.y, p.z));
//	}
////	kdtree.reset(new ais_point_cloud::EasyKDTree(50));
////	kdtree->setData(points);
//}

bool PRMPlanner::isPlanningSuccess() const
{
	if (!m_active)
		return false;

	return m_planningSuccess;
}

bool PRMPlanner::executeSync(const boost::shared_ptr<Path>& path)
{
	if (!m_active)
		return false;

	execute(path);

	ros::Rate r(1000);
	while (isExecutingMotion())
	{
		r.sleep();
	}

	return true;
}

const ais_point_cloud::RGBDImage::Ptr PRMPlanner::getRGBDImage() const
{
	return m_planningScene->rgbd;
}

void PRMPlanner::resetExecuter()
{
	if (ParameterServer::executionMode == parameters::HardwareInterface)
		m_executer.reset(new RobotExecuter);
	else if (ParameterServer::executionMode == parameters::FollowJointTrajectoryPublisher)
		m_executer.reset(new FollowJointTrajectoryExecuter);

	if (m_executer.get() != NULL)
		m_executer->init();

	//setup trajectory
	m_path.reset(new Path(m_problemDefinition->getConfig().planningFrame));
}

bool PRMPlanner::activateToolFrameFromPD()
{
	if (m_problemDefinition->updateToolFrames())
	{
		if (m_executer.get() != NULL)
			m_executer->reset();

		return true;
	}
	else
		return false;
}

void PRMPlanner::activateToolFrame(const Eigen::Affine3d tcp)
{
	m_robot->setToolCenterPointTransformation(tcp);

	if (m_executer.get() != NULL)
		m_executer->reset();
}

const ais_point_cloud::MyPointCloudP& PRMPlanner::getCloud() const
{
	return m_planningScene->processedCloud;
}

const boost::shared_ptr<Path> PRMPlanner::getPath() const
{
	return m_path;
}

void PRMPlanner::threadDebugCollisionChecks()
{
	ros::Rate r(1);
	while (ros::ok() && !boost::this_thread::interruption_requested())
	{
		//update joint poses
		KDL::JntArray currentJointPose = m_robot->getKDLChainJointState();

		fcl_robot_model::RobotState state;
		m_debugCollisionDetectionInteractiveMarker->getJoints(state.m_joints);
		m_debugCollisionDetection->robot->setRobotState(state);

		if (m_debugCollisionDetection->fcl->checkCollisions(true))
		{
			fcl_collision_detection::FCLWrapper::CollisionsVector collisions;
			m_debugCollisionDetection->fcl->getCollisions(collisions);

			LOG_INFO("Collisions: ")
			for (auto& it : collisions)
			{
				LOG_INFO("-- " << it.first << " <-> " << it.second);
			}
		}

		r.sleep();
	}
}

boost::shared_ptr<Executer> PRMPlanner::getExecuter() const
{
	return m_executer;
}

} /* namespace prm_planner */

