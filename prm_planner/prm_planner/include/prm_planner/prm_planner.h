/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 16, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm_planner.h
 */

#ifndef PRM_PLANNER_H_
#define PRM_PLANNER_H_

#include <ais_ros/ros_base_interface.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <octomap_msgs/Octomap.h>
#include <prm_planner/util/defines.h>
#include <prm_planner_robot/defines.h>
#include <prm_planner_msgs/GoalAction.h>
#include <prm_planner_msgs/SetState.h>
#include <prm_planner_msgs/GetImage.h>
#include <prm_planner_msgs/SetObjectPoseType.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <prm_planner/PRMPlannerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <prm_planner_msgs/ModifyPlanningScene.h>

namespace prm_planner
{

FORWARD_DECLARE(Path);
FORWARD_DECLARE(InteractiveMarker);
FORWARD_DECLARE(RobotArmInteractiveMarker);
FORWARD_DECLARE(TrajectoryValidator);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(RobotTrajectoryVisualizer);
FORWARD_DECLARE(ProblemDefinitionManager);
FORWARD_DECLARE(ObjectManager);
FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Executer);
FORWARD_DECLARE(CollisionDetector);
FORWARD_DECLARE(PlanningScene);

class PRMPlanner: public boost::enable_shared_from_this<PRMPlanner>, public ais_ros::RosBaseInterface
{
public:
	struct PlannerParameters
	{
		PlannerParameters();

		bool active; 				//true if the planner is started with threads an callbacks active, default: true
	};

	// =======================================================================
	// ========================= Init ========================================
	// =======================================================================
public:
	PRMPlanner(PlannerParameters params = PlannerParameters());
	virtual ~PRMPlanner();

	void init();
	void initObjectStates();

protected:
	void initBase();
	void initRobot();
	void initProblemDefinition();
	void initObjectManager();
	void initVisualization();
	void initROS();
	void initDynamicReconfigure();

	void startThreads();

	// =======================================================================
	// ========================= Reset =======================================
	// =======================================================================
public:
	/*
	 * Resets the whole planner
	 *
	 * @reset PD: if, true the problem definition gets reseted
	 */
	void reset(bool resetPD = true);

protected:
	void resetExecuter();

	// =======================================================================
	// ========================= Plan & Execute ==============================
	// =======================================================================
public:
	/**
	 * The method only computes a plan to the desired
	 * goal using problem definitions plan method. The
	 * method waits for the result of the planner and
	 * returns the path, which then can be executed with
	 * execute(path).
	 *
	 * @goalPose [in]: The goal pose needs to be provided in the
	 * 		planning frame by default. However you can specify
	 * 		another frame using the second parameter 'frame'
	 * @path [out]: if the return value is true, the path will
	 * 		be stored in this reference.
	 * @frame [in]: the frame, in which the goalPose is given. By
	 * 		default the planning frame is assumed.
	 */
	virtual bool plan(const Eigen::Affine3d& goalPose,
			boost::shared_ptr<Path>& path,
			const std::string frame = "");

	/**
	 * Executes a given path. The method returns
	 * Immediately without waiting for a result.
	 *
	 * @path [in]
	 */
	virtual void execute(const boost::shared_ptr<Path>& path);

	/**
	 * Synchronous version of execute
	 */
	bool executeSync(const boost::shared_ptr<Path>& path);

	/**
	 * Plans and moves the robots end effector to the
	 * desired goal pose.
	 * @goalPose: The goal pose needs to be provided in the
	 * 		planning frame by default. However you can specify
	 * 		another frame using the second parameter 'frame'
	 * @frame: the frame, in which the goalPose is given. By
	 * 		default the planning frame is assumed.
	 */
	virtual bool planAndExecute(const Eigen::Affine3d& goalPose,
			const std::string frame = "");

	/**
	 * Plans and moves the robots end effector to the
	 * desired goal pose, which needs to be given relatively
	 * to the current end effector pose.
	 * @goalPose: The goal pose needs to be provided in the
	 * 		end effector frame
	 */
	virtual bool planAndExecuteRel(const Eigen::Affine3d& goalPose);

	/**
	 * Calls planAndExecute and waits until the motion has
	 * been completed
	 */
	virtual bool planAndExecuteSync(const Eigen::Affine3d& goalPose,
			const std::string frame = "");

	/**
	 * Calls planAndExecuteRel and waits until the motion has
	 * been completed
	 */
	virtual bool planAndExecuteRelSync(const Eigen::Affine3d& goalPose);

	/**
	 * This method calls the custom planning method in
	 * the problem definition
	 */
	virtual bool planAndExecutePD();

	/**
	 * Calls plan() of problem definition in
	 * corresponding mode.
	 * @name [in]: name of the object
	 */
	virtual bool graspObject(const std::string& name);
	virtual bool graspObjectSync(const std::string& name);

	/**
	 * Calls plan() of problem definition in
	 * corresponding mode.
	 * @name [in]: name of the object
	 * @pos [in]: the position were the object
	 * 		should be placed
	 */
	virtual bool dropObject(const std::string& name,
			const Eigen::Vector3d& pos);
	virtual bool dropObjectSync(const std::string& name,
			const Eigen::Vector3d& pos);

	/**
	 * Returns the executer
	 */
	boost::shared_ptr<Executer> getExecuter() const;

	/**
	 * Stops any motion and planning
	 */
	void stopMotion();

protected:
	/**
	 * Internal method, which is called within a
	 * thread to be able to react to new goals.
	 * If there is a new goal, call interrupt on
	 * the corresponding thread to stop the planning
	 * (and execution).
	 *
	 * @executeTrajectorz [in]: if true, the execution
	 * 		will be started after successful planning
	 */
	void concurrentPlan(const bool executeTrajectory);

	// =======================================================================
	// ========================= Getter ======================================
	// =======================================================================
public:
	const ais_point_cloud::MyPointCloudP& getCloud() const;
	const ais_point_cloud::RGBDImage::Ptr getRGBDImage() const;
	boost::shared_ptr<Robot> getRobot() const;
	const boost::shared_ptr<Path> getPath() const;

	bool isExecutingMotion() const;
	bool isPlanningSuccess() const;
	bool isReady() const;

	// =======================================================================
	// ========================= Setter ======================================
	// =======================================================================
public:
	void setRGBDImage(ais_point_cloud::RGBDImage::Ptr& image);

	// =======================================================================
	// ========================= Debug =======================================
	// =======================================================================
protected:
	void threadDebugCollisionChecks();

	// =======================================================================
	// ========================= ROS =========================================
	// =======================================================================
protected:
	//callbacks
	virtual void callbackRGBDImage(ais_point_cloud::RGBDImage::Ptr& image);
	void callbackHandJointState(const ros::MessageEvent<sensor_msgs::JointState> event,
			const std::string& hand);
	void callbackGoal(const prm_planner_msgs::GoalGoalConstPtr& goal);
	bool callbackSetState(prm_planner_msgs::SetState::Request& req,
			prm_planner_msgs::SetState::Response& res);
	bool callbackGetImage(prm_planner_msgs::GetImage::Request& req,
			prm_planner_msgs::GetImage::Response& res);
	bool callbackSetObjectPoseType(prm_planner_msgs::SetObjectPoseType::Request& req,
			prm_planner_msgs::SetObjectPoseType::Response& res);
	void callbackDynamicReconfigure(PRMPlannerConfig &config,
			uint32_t level);
	bool callbackModifyPlanningScene(prm_planner_msgs::ModifyPlanningScene::Request& req,
			prm_planner_msgs::ModifyPlanningScene::Response& res);

	// =======================================================================
	// ========================= Other =======================================
	// =======================================================================
public:
	/**
	 * The first method uses updateToolFrame from the problem definition.
	 * If armName is empty the first available arm is assumed.
	 */
	bool activateToolFrameFromPD();
	void activateToolFrame(const Eigen::Affine3d tcp);

protected:
	void threadedUpdate();
	void threadedVisualizationPublisher();

	void stopThreads(bool stopSpinner);

private:
	// =======================================================================
	// ========================= ROS =========================================
	// =======================================================================
	//publishers
	ros::Publisher m_pubTrajectory;

	//dynamic reconfigure
	dynamic_reconfigure::Server<PRMPlannerConfig> m_dynReconfigureServer;
	dynamic_reconfigure::Server<PRMPlannerConfig>::CallbackType m_dynReconfigureCallback;

	//subscriber
	std::vector<ros::Subscriber> m_subsHandJointStates;

	//action server
	actionlib::SimpleActionServer<prm_planner_msgs::GoalAction>* m_goalActionServer;
	prm_planner_msgs::GoalResult m_goalASResult;
	prm_planner_msgs::GoalFeedback m_goalASFeedback;

	//services
	ros::ServiceServer m_serviceServerSetState;
	ros::ServiceServer m_serviceServerGetImage;
	ros::ServiceServer m_serviceServerSetObjectPoseType;
	ros::ServiceServer m_serviceModifyPlanningScene;

	//other
	ros::NodeHandle m_nodeHandle;

	// =======================================================================
	// ========================= Planner =====================================
	// =======================================================================
	//planners
	boost::atomic<PlanningMode> m_planningMode;

	//paths & goal
	boost::shared_ptr<Path> m_path;
	Eigen::Affine3d m_goalPose;
	std::vector<std::string> m_planningInputStrings;

	//problems
	ProblemDefinitionManager* m_problemDefinitionManager;
	boost::shared_ptr<ProblemDefinition> m_problemDefinition;

	//other
	boost::shared_ptr<TrajectoryValidator> m_validator;
	std::string m_planningObject;

	// =======================================================================
	// ========================= Debugging ===================================
	// =======================================================================
	//collision detection
	boost::atomic_bool m_debugIsCollisionDetectionActive;
	boost::shared_ptr<CollisionDetector> m_debugCollisionDetection;
	boost::thread m_threadDebugCollisionChecks;
	boost::shared_ptr<RobotArmInteractiveMarker> m_debugCollisionDetectionInteractiveMarker;

	// =======================================================================
	// ========================= ROBOTs ======================================
	// =======================================================================
	//hardware
	boost::shared_ptr<Robot> m_robot;

	//controller
	boost::shared_ptr<Executer> m_executer;

	// =======================================================================
	// ========================= Environment =================================
	// =======================================================================
	ObjectManager* m_objectManager;
	boost::shared_ptr<PlanningScene> m_planningScene;
	boost::atomic_bool m_envInitialized;
	std::vector<std::string> m_neglectObjectPoseUpdates;
	boost::atomic_bool m_useNextPlanningObjectPoses;

	// =======================================================================
	// ========================= Visualization ===============================
	// =======================================================================
//	std::vector<boost::shared_ptr<InteractiveMarker>> m_interactiveMarkers;
	boost::shared_ptr<RobotTrajectoryVisualizer> m_robotTrajectoryVisualizer;

	// =======================================================================
	// ========================= General =====================================
	// =======================================================================
	int m_counter;
	boost::atomic_bool m_active;

	//mutexes
	mutable boost::recursive_mutex m_mutex;
	mutable boost::recursive_mutex m_mutexOctomap;
	mutable boost::recursive_mutex m_mutexVisualization;
	mutable boost::recursive_mutex m_mutexUpdate;
	mutable boost::recursive_mutex m_mutexPlan;
	mutable boost::recursive_mutex m_mutexPublish;
	mutable boost::recursive_mutex m_mutexJointStatePublish;

	//update thread
	boost::thread m_threadUpdate;
	boost::atomic<bool> m_runUpdate; //update required?

	//plan thread
	boost::thread m_threadPlan;
	boost::atomic<bool> m_runPlan; //plan required?
	boost::atomic<bool> m_planningSuccess;

	//publish thread
	boost::thread m_threadVisPublish;
};

}
/* namespace prm_planner */

#endif /* PRM_PLANNER_H_ */
