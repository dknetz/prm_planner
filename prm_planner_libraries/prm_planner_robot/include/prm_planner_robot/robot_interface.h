/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_interface.h
 */

#ifndef HC679E788_35DC_4446_A0BA_7DBD7C561733
#define HC679E788_35DC_4446_A0BA_7DBD7C561733
#include <kdl/jntarray.hpp>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <unordered_map>

namespace prm_planner
{

class RobotInterface
{
public:
	struct JointData
	{
		enum Mode
		{
			Position,
			Velocity,
			Torque,
			Unknown
		};

		JointData();

		//command to the robot: position
		double cmdPos;

		//command to the robot: velocity
		double cmdVel;

		//command to the robot: torque
		double cmdTorque;

		double desiredPos;

		//the current position of the joint
		double pos;

		//the current velocity of the joint
		double vel;

		//the current torque of the joint
		double torque;

		//internal field: last position to compute velocity
		double lastPos;

		//internal field: last time to compute velocity
		ros::Time lastTime;

		//distinguishes real and virtual joints (virtual
		//joints are used to move the robot on the floor
		//(adding three additional joints))
		bool isRealJoint;

		//the mode: @see Mode
		Mode mode;
	};

public:
	typedef std::unordered_map<std::string, JointData> Data;

public:
	RobotInterface();
	virtual ~RobotInterface();

	/**
	 * Implement this method. It is called
	 * in the initialization step.
	 */
	virtual bool start() = 0;

	/**
	 * Implement this method. It is called for
	 * getting data from the robot (e.g., the joint
	 * state). You need to write the data into
	 * the corresponding fields of m_data
	 */
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0)) = 0;

	/**
	 * Implement this method. It is called for
	 * sending data to the robot (e.g., the joint
	 * state). It is read from the corresponding
	 * fields in m_data.
	 */
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0)) = 0;

	/**
	 * Implement this method.
	 */
	virtual bool stop() = 0;

	/**
	 * Returns the joint state as a KDL JntArray. You need to
	 * provide the order of the joints. To get new data you need
	 * to call read() first!
	 * @[out] 	jointState: 	the joint state based on the current data
	 * @[in] 	names: 			the order of joint names
	 */
	virtual void getJointState(KDL::JntArray& jointState,
			const std::vector<std::string>& names);

	/**
	 * Sets the cmdPos attribute of the JointData map using
	 * the joint state and name order provided by the user.
	 * Additionally, the corresponding mode is set to Position
	 * Call this method before write().
	 * @[in]	jointState: 	the joint state
	 * @[in]	names: 			the order of joint names
	 */
	virtual void setJointPositionCommand(const KDL::JntArray& jointState,
			const std::vector<std::string>& names);

	/**
	 * Sets the cmdVel attribute of the JointData map using
	 * the velocities and name order provided by the user.
	 * Additionally, the corresponding mode is set to Velocity
	 * Call this method before write().
	 * @[in]	velocities: 	the velocities vector
	 * @[in]	names: 			the order of joint names
	 */
	virtual void setJointVelocityCommand(const KDL::JntArray& velocities,
			const std::vector<std::string>& names);

	/**
	 * Sets the cmdTorque attribute of the JointData map using
	 * the torques and name order provided by the user.
	 * Additionally, the corresponding mode is set to Torque
	 * Call this method before write().
	 * @[in]	torques: 		the torques vector
	 * @[in]	names: 			the order of joint names
	 */
	virtual void setJointTorqueCommand(const KDL::JntArray& torques,
			const std::vector<std::string>& names);

	/**
	 * Loads an robot interface from a ROS plugin.
	 * @package: the ROS package name
	 * @library: namespace::class_name
	 */
	static boost::shared_ptr<RobotInterface> load(const std::string& package,
			const std::string& library);

public:
	Data m_data;

private:
	/**
	 * The object needs to live the complete lifetime
	 * of the program. Otherwise we get an annoying warning
	 */
	static pluginlib::ClassLoader<RobotInterface>* s_loader;
};

} /* namespace prm_planner */

#endif /* HC679E788_35DC_4446_A0BA_7DBD7C561733 */
