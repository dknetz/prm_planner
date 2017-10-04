/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: simulation_robot_controller.h
 */

#ifndef H7F2B36FC_477E_41D0_AC70_1FDF2255897E
#define H7F2B36FC_477E_41D0_AC70_1FDF2255897E

#include <kdl/jntarray.hpp>
#include <prm_planner/util/defines.h>
#include <prm_planner_controller/controller.h>
#include <prm_planner_controller/controller_defines.h>
#include <prm_planner_robot/defines.h>
#include <Eigen/Geometry>

namespace prm_planner
{

FORWARD_DECLARE(PRMEdge);
FORWARD_DECLARE(PRMNode);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(CollisionDetector);

class SimulationRobotController
{
public:
	/**
	 * Constructor which takes an PRM edge on which it should
	 * operate. Call updateFromEdge() to provide the direction
	 * by setting the start node
	 */
	SimulationRobotController(const PRMEdge* edge,
			boost::shared_ptr<Robot> robot,
			boost::shared_ptr<ProblemDefinition> pd);

	/**
	 * You can freely set the start and end pose of the
	 * motion. Additionally you need to provide the start joint
	 * pose of the robot
	 */
	SimulationRobotController(const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goalPose,
			const KDL::JntArray& startJointState,
			const ControllerParameters& params,
			boost::shared_ptr<Robot> robot,
			boost::shared_ptr<ProblemDefinition> pd);

	/**
	 * Creates the controllers, but doesn't initialize them.
	 * Can be used in combination with either updateFromValues
	 * or updateFromPath
	 */
	SimulationRobotController(boost::shared_ptr<Robot> robot,
			boost::shared_ptr<ProblemDefinition> pd);

	virtual ~SimulationRobotController();

	/**
	 * Sets the collision detection instance and the robot
	 * description for FCL. If you don't want to use it, just
	 * don't call it.
	 */
	void setCollisionDetection(boost::shared_ptr<CollisionDetector>& cd);

	/**
	 * Simulates movement. If execution time is larger than maxWaitTime
	 * the simulation loop is stopped and false returned. Otherwise the
	 * result depends on the collision/nan/joint-range checks. If
	 * checkGoalIsReached is true the this methods tests, if the desired
	 * goal was reached. If so, the result is true, otherwise false.
	 * If checkGoalIsReached is false, the method returns true, if the
	 * simulation loop has been finished.
	 * dt is only used, if it is > 0
	 */
	virtual bool canControl(const double maxWaitTime,
			const double dt);

	/**
	 * Use this method together with the edge constructor. Call
	 * this method before you call canControl() to use the new
	 * data from the edge/node.
	 */
	virtual bool updateFromEdge(const PRMNode* startNode,
			KDL::JntArray& startJoint);

	/**
	 * You can directly provide the start and goal pose and the
	 * start joint state.
	 */
	virtual bool updateFromValues(const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goalPose,
			KDL::JntArray& startJoint);

	/**
	 * Given a path the controller generates a motion which
	 * follows the path given the parameters of the controller
	 */
//	virtual bool updateFromTaskPath(const boost::shared_ptr<Path> path);
	/**
	 * Computes a trajectory for one or more robot arms.
	 *
	 * @path [in]:
	 * 		The executer splits the path into pieces, such
	 * 		that there is no other than the same waypoint
	 * 		type in a single path, i.e., the grasp and drop
	 * 		types are filtered out.
	 * @currentIndex [in]: the index which will be used to
	 * 		get the data from the vectors of each robot arm
	 * @adjustRuntimes [in]: if true, the runtimes of each
	 * 		relevant controller will be estimated. All
	 * 		controllers will be set to the maximum of all
	 * 		runtimes. Set this to true if you want multiple
	 * 		robots to follow a task trajectory.
	 */
	virtual bool updateFromRobotPath(const std::vector<boost::shared_ptr<Path>> path,
			const int currentIndex,
			const bool adjustRuntimes);

	/**
	 * Returns the final joint state of the goal pose
	 * after running the controller.
	 */
	void getFinalJointState(KDL::JntArray& finalJointState);

	/**
	 * Returns the dense path after running the controller
	 */
	void getJointPath(ArmJointPath& jointPath) const;

//	virtual void setKdTree(boost::shared_ptr<ais_point_cloud::EasyKDTree> kdtree);

	void reset();

private:
	Controller* m_controller;
	boost::shared_ptr<Robot> m_robot;

	const PRMEdge* m_edge;

	//collsion detection
	boost::shared_ptr<CollisionDetector> m_cd;

	//problem defintion
	boost::shared_ptr<ProblemDefinition> m_pd;
};

} /* namespace prm_planner */

#endif /* H7F2B36FC_477E_41D0_AC70_1FDF2255897E */
