/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_arm.h
 */

#ifndef H217D3218_1523_44FE_9F6D_8DB79923DCF0
#define H217D3218_1523_44FE_9F6D_8DB79923DCF0
#include <actionlib/client/simple_action_client.h>
#include <prm_planner_robot/defines.h>
#include <boost/thread/recursive_mutex.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <prm_planner_robot/robot_interface.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <Eigen/Geometry>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <prm_planner_robot/gripper_interface.h>
#include <prm_planner_robot/kinematics.h>

namespace prm_planner
{

class RobotArm
{
public:
	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryAC;

public:
	RobotArm(const RobotArmConfig& armConfig,
			bool initRobotConnection);
	virtual ~RobotArm();

	/**
	 * Initializes the hardware interface.
	 * Uses either a ros plugin or a joint state topic & a follow joint
	 * trajectory action client to execute the motion depending
	 * on the parameter useHWInterface
	 */
	void initHardwareInterface();

	/**
	 * Returns the joint state of the chain as a KDL JntArray
	 */
	KDL::JntArray getKDLChainJointState() const;

	/**
	 * Returns the joint velocity of the chain as a KDL JntArray
	 */
	KDL::JntArray getKDLChainVelocities() const;

	/**
	 * Returns all joint states
	 */
	std::unordered_map<std::string, double> getAllJointStates() const;

	/**
	 * Receives the robot data from the hardware interface.
	 * Call this method frequently to get new data.
	 * If you use the 'follow joint trajectory' interface
	 * you don't need to call this method, because there is
	 * a joint state callback, which will be used to get the
	 * current robot data
	 */
	void receiveData(const ros::Time& time,
			const ros::Duration& period);

	/**
	 * Callback method for joint states if you use the
	 * 'follow joint trajectory' interface
	 */
	void receiveJointState(const sensor_msgs::JointStateConstPtr& jointState);

	/**
	 * Sends the velocity commands to the robot using
	 * the hardware interface. If you use the 'follow
	 * joint trajectory' interface call sendTrajectory()
	 * instead.
	 */
	void sendVelocity(const ros::Time& time,
			const ros::Duration& period,
			const KDL::JntArray& cmd);

	/**
	 * Sends the torque commands to the robot using
	 * the hardware interface. If you use the 'follow
	 * joint trajectory' interface call sendTrajectory()
	 * instead.
	 */
	void sendTorque(const ros::Time& time,
			const ros::Duration& period,
			const KDL::JntArray& cmd);

	/**
	 * Sends the joint position command to the robot using
	 * the hardware interface. If you use the 'follow
	 * joint trajectory' interface call sendTrajectory()
	 * instead.
	 */
	void sendChainJointPosition(const KDL::JntArray& joints);

	/**
	 * Use this method if you use the 'follow joint trajectory'
	 * interface. You need to provide a complete trajectory using
	 * a control_msgs::FollowJointTrajectoryGoal message of ROS.
	 * If you use a hardware interface this method will return
	 * Immediately without sending anything
	 */
	void sendTrajectory(const control_msgs::FollowJointTrajectoryGoal& trajectory);

	/**
	 * Stops the the motion of the arm
	 */
	void stopMotion();

	/**
	 * Returns false, if the current execution (sendTrajectory only)
	 * is still running, otherwise true
	 */
	bool isTrajectoryExecutionFinished();

	/**
	 * Waits 100 ms to get the first data. If you use
	 * this method it will automatically call receiveData()
	 * in the case of the hardware interface. If you use the
	 * 'follow joint trajectory' the method just waits to receive
	 * data from via ROS callback.
	 */
	bool waitForData();

	/**
	 * Activates/Deactivates the passive mode
	 *
	 * @passive: true, if passive mode should be active
	 */
	void setPassiveMode(const bool passive);

	/**
	 * Sets the joint state. The new joint state is only
	 * applied, if the robot arm is in passive mode. Otherwise,
	 * the joint states would be overwritten by receive()
	 *
	 * @joints: the joint state
	 */
	void setChainJointState(const KDL::JntArray& joints);

	/**
	 * Returns the chain
	 */
	const KDL::Chain& getChain() const;

	/**
	 * Returns the joint limits of a given joint
	 */
	boost::shared_ptr<urdf::JointLimits> getJointLimits(const std::string& name);

	/**
	 * Returns all joints of the chain
	 */
	void getChainJoints(std::vector<urdf::Joint>& out);

	/**
	 * Returns a vector of joint names belonging to the current chain
	 */
	const std::vector<std::string>& getChainJointNames() const;

	/**
	 * Returns the upper joint limits of the joints of the current chain
	 */
	const KDL::JntArray& getChainLimitMax() const;

	/**
	 * Returns the lower joint limits of the joints of the current chain
	 */
	const KDL::JntArray& getChainLimitMin() const;

	/**
	 * Samples a end effector pose for the chain
	 */
	void sampleValidChainEEFPose(Eigen::Affine3d& pose,
			KDL::JntArray& joints,
			const double borderAvoidance = 0.05);

	/**
	 * Samples a joint state for the chain
	 */
	void sampleValidChainJointState(KDL::JntArray& jointState,
			const double borderAvoidance = 0.05);

	bool getIK(const Eigen::Affine3d& pose,
			KDL::JntArray& ik);
	bool getIKWithInitCurrent(const Eigen::Affine3d& pose,
			KDL::JntArray& ik);
	bool getIKWithInit(const KDL::JntArray& init,
			const Eigen::Affine3d& pose,
			KDL::JntArray& ik);
	bool getFK(const KDL::JntArray& joints,
			Eigen::Affine3d& pose);
	bool getCurrentFK(Eigen::Affine3d& pose);

	const KDL::Tree& getRobot() const;

	const std::string& getRobotDescription() const;
	const std::string& getRootLink() const;
	const std::string& getRootFrame() const;
	const std::string& getTipLink() const;
	const std::string& getOriginalTipLink() const;
	const std::string& getName() const;
	const std::string& getCollisionMatrixFile() const;
	const boost::shared_ptr<GripperInterface> getGripper() const;
	void setGripper(const boost::shared_ptr<GripperInterface> gripper);
	const std::string& getOriginalTipLinkFrame() const;
	const RobotArmConfig& getParameters();

	//relative to the tip link
	void setToolCenterPointTransformation(const Eigen::Affine3d& tcp);
	Eigen::Affine3d getTcp() const;
	bool isUseTcp() const;

	const urdf::Model& getUrdf() const;
	const boost::shared_ptr<Kinematics>& getKinematics() const;

protected:
	mutable boost::shared_mutex m_mutex;

	//hardware interfaces
	boost::shared_ptr<prm_planner::RobotInterface> m_interface;

	//follow joint trajectory interface
	FollowJointTrajectoryAC* m_actionClient;
	ros::Subscriber m_subJointState;

	boost::atomic_bool m_receivedState;

	std::string m_rootTfLink;
	std::string m_tipLink;
	std::string m_originalTipLinkFrame;

	RobotArmConfig c_armConfig;

	//if the robot arm is in passive mode, no send and receive commands
	//will be called. You can set the joint states by yourself using setJointState()
	boost::atomic_bool m_passiveMode;

	KDL::Tree m_robot;
	KDL::Chain m_chain;
	urdf::Model m_urdf;
	boost::shared_ptr<Kinematics> m_kinematics;

	KDL::JntArray m_chainLimitMin, m_chainLimitMax;
	KDL::JntArray m_chainPositions;
	KDL::JntArray m_chainVelocities;
	KDL::JntArray m_chainCommands;
	std::vector<std::string> m_chainJointNames;

	std::unordered_map<std::string, double> m_allPositions;

	boost::shared_ptr<GripperInterface> m_gripper;

	Eigen::Affine3d m_tcp;
	bool m_useTCP;

	const int c_id;
	static int s_idCounter;
};

} /* namespace prm_planner */

#endif /* H217D3218_1523_44FE_9F6D_8DB79923DCF0 */
