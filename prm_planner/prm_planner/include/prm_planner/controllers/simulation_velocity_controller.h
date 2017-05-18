/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 13, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: elastic_roadmap_edge_controller.h
 */

#ifndef SIMULATION_VELOCITY_CONTROLLER_7_DOF_H_
#define SIMULATION_VELOCITY_CONTROLLER_7_DOF_H_

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <prm_planner_robot/defines.h>
#include <ais_definitions/math.h>
#include <prm_planner/util/defines.h>
#include <prm_planner_controller/velocity_controller.h>

namespace prm_planner
{

class Path;
class ProblemDefinition;

template<int Dim, class Type = double>
class SimulationVelocityControllerN: public VelocityController<Dim, Type>
{
public:
	/**
	 * This constructor can be used if you specify the
	 * goals in a later step.  Only the most important
	 * things are initialized. You
	 */
	SimulationVelocityControllerN(const ControllerParameters& parameters,
			boost::shared_ptr<RobotArm> robotArm,
			boost::shared_ptr<ProblemDefinition> pd);

	//for edge simulation
	SimulationVelocityControllerN(const Eigen::Affine3d& start,
			const Eigen::Affine3d& goal,
			const KDL::JntArray& qStart,
			const ControllerParameters& parameters,
			boost::shared_ptr<RobotArm> robotArm,
			boost::shared_ptr<ProblemDefinition> pd);

	//for validation
	SimulationVelocityControllerN(
			const boost::shared_ptr<SimulationVelocityControllerN<Dim, Type>> controller,
			const ControllerParameters& parameters,
			boost::shared_ptr<RobotArm> robotArm,
			boost::shared_ptr<ProblemDefinition> pd);

	virtual ~SimulationVelocityControllerN();

	virtual bool update(const ros::Time& now,
			const ros::Duration& dt);

	/**
	 * Updates the controller with new start and
	 * goal pose.
	 */
	virtual bool updateFromValues(const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goalPose,
			const KDL::JntArray& startJoints);

	/**
	 * Update using a path
	 */
	virtual bool updateFromPath(const boost::shared_ptr<Path> path);

	virtual const ArmJointPath& getJointPositions() const;

	/**
	 * Override it to avoid calling a mutexed method of RobotArm
	 */
	virtual KDL::JntArray getJointStateFromRobot();

protected:
	Eigen::Affine3d m_startPose, m_goalPose;
	ArmJointPath m_jointPositions;
};

} /* namespace prm_planner */

#endif /* SIMULATION_VELOCITY_CONTROLLER_7_DOF_H_ */
