/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jun 7, 2017
 *      Author: kuhnerd
 * 	  Filename: ik.h
 */

#ifndef H3D9A1E9E_51F3_43DF_ADC7_DFBEFD0F68C0
#define H3D9A1E9E_51F3_43DF_ADC7_DFBEFD0F68C0

#include <ais_definitions/class.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_loader.h>
#include <Eigen/Geometry>

namespace prm_planner
{

FORWARD_DECLARE(RobotArm);

class Kinematics
{
public:
	Kinematics();
	virtual ~Kinematics();

	virtual void init(const RobotArm* robot) = 0;

	virtual boost::shared_ptr<Kinematics> getCopy(const RobotArm* robot) = 0;

	/**
	 * Takes the joint position and computes the end effector pose. If no
	 * solution can be found, false is returned.
	 */
	virtual bool getJacobian(const KDL::JntArray& jointPosition,
			KDL::Jacobian& jacobian,
			int segmentNR = -1) = 0;

	/**
	 * Takes the joint position and computes the end effector pose. If no
	 * solution can be found, false is returned.
	 */
	virtual bool getFK(const KDL::JntArray& jointPosition,
			Eigen::Affine3d& pose,
			int segmentNR = -1);
	virtual bool getFK(const KDL::JntArray& jointPosition,
			KDL::Frame& pose,
			int segmentNR = -1) = 0;

	/**
	 * Takes a pose and a initial joint position
	 * and computes the ik. If no
	 * solution can be found, false is returned.
	 */
	virtual bool getIK(const Eigen::Affine3d& pose,
			const KDL::JntArray& init,
			KDL::JntArray& jointPosition) = 0;

	/**
	 * Loads an robot interface from a ROS plugin.
	 * @package: the ROS package name
	 * @library: namespace::class_name
	 */
	static boost::shared_ptr<Kinematics> load(const std::string& package,
			const std::string& library);

protected:
	boost::shared_ptr<RobotArm> m_robot;

private:
	/**
	 * The object needs to live the complete lifetime
	 * of the program. Otherwise we get an annoying warning
	 */
	static pluginlib::ClassLoader<Kinematics>* s_loader;
};

} /* namespace prm_planner */

#endif /* H3D9A1E9E_51F3_43DF_ADC7_DFBEFD0F68C0 */
