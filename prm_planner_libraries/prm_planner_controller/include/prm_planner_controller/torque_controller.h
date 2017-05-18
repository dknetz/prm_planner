/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 28, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: torque_controller.h
 */

#ifndef H7FB6A22D_82BF_416A_A5B6_4D4C9AB76FBD
#define H7FB6A22D_82BF_416A_A5B6_4D4C9AB76FBD
#include <ais_point_cloud/easy_kd_tree.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <kdl/jntarray.hpp>
#include <prm_planner_robot/path.h>
#include <ros/ros.h>
#include <Eigen/Geometry>

#include <prm_planner_controller/controller.h>
#include <prm_planner_controller/controller_defines.h>

namespace prm_planner
{

class Constraint;

class TorqueController: public Controller, public ControllerEigenDefines<7, double>
{
	CONTROLLER_USE_TYPEDEFS_PARAM(7, double)

public:
	TorqueController(const ControllerParameters& parameters,
			const boost::shared_ptr<Constraint> constraint,
			boost::shared_ptr<RobotArm> robotArm,
			const std::string& planningFrame,
			const double octomapResolution);
	virtual ~TorqueController();

	virtual bool updateFromPath(const boost::shared_ptr<Path> path);
	virtual void initRobotControl();

	virtual bool isGoalReached() const;
	virtual bool isSuccess() const;
	virtual bool isTrajectoryAvailable() const;

	virtual void reset();

	virtual double getPathLength();
	virtual double getExecutionPathLength();

	virtual double getPredictedExecutionTime() const;
	virtual void setPredictedExecutionTime(double predictedExecutionTime);

	virtual bool update(const ros::Time& now,
			const ros::Duration& dt);

private:
	bool computeJacobian(Matrix6xN& jacobian);
	void computePseudoInverse();
	void computeTargetForce();
	void computeKineticEnergyMatrix();
	void computeCoriolisAndCentrifugalTerm();
	void computeGravityTerm();
};

} /* namespace prm_planner */

#endif /* H7FB6A22D_82BF_416A_A5B6_4D4C9AB76FBD */
