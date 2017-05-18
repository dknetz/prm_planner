/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 4, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: rrt.cpp
 */

#include <ais_definitions/class.h>
#include <ais_definitions/exception.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/controllers/helpers.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/planners/rrt/rrt.h>
#include <prm_planner/planners/rrt/rrt_node.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner_constraints/constraint.h>

namespace prm_planner
{

RRT::RRT(boost::shared_ptr<ProblemDefinition> pd,
		const double maxPlanningTime) :
				PathPlanner(pd),
				m_constraint(pd->getConstraint()),
				m_maxPlanningTime(maxPlanningTime),
				m_sampleGoal(pd->getConfig().rrtConfig.sampleGoal),
				m_maxExpansionDistance(pd->getConfig().rrtConfig.maxExpansionDistance)
{
}

RRT::~RRT()
{
}

bool RRT::plan(const KDL::JntArray& currentJointPoses,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path,
		bool directConnectionRequired)
{
	PathPlanner::plan(currentJointPoses, currentTaskPose, goalTaskPose, cd, path, directConnectionRequired);

	//check if the first kd tree was set
	{
		PLANNER_READ_LOCK();

		if (!m_planningScene)
		{
			LOG_INFO("no planning scene");
			return false;
		}
	}

	RRTNode* nearestNeighbor;
	RRTNode* newNode;
	RRTNode* currentNode;
	RRTNode* finalNode;

	bool testGoal = true;

	Eigen::Affine3d pose;

	//init tree
	std::list<RRTNode*> nodes;
	RRTNode* tree = new RRTNode(currentJointPoses, currentTaskPose);
	nodes.push_back(tree);

	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);

	currentNode = tree;

	ros::Time start = ros::Time::now();
	for (int i = 0; (ros::Time::now() - start).toSec() < m_maxPlanningTime; ++i)
	{
		//check if goal can be reached
		if (testGoal)
		{
			finalNode = tryToReachGoal(goalTaskPose, currentNode, cd );
		}

		if (finalNode != NULL)
		{
			//found path
			path = extractPath(finalNode);

			DELETE_VECTOR(nodes);
			tree = NULL;//deleted with m_nodes

			return true;
		}

		double p = distribution(generator);
		if (p < m_sampleGoal)
		{
			pose = goalTaskPose;
		}
		else
		{
			m_constraint->samplePose(pose);
		}

		nearestNeighbor = findNearestNeighbor(pose, nodes);
		newNode = createNode(nearestNeighbor, pose, cd);

		if (newNode == NULL)
		{
			testGoal = false;
			delete newNode;
			continue;
		}

		currentNode = newNode;
		testGoal = true;

		nodes.push_back(newNode);

		//check if there was an interruption
		if (++i % 5 == 0)
		{
			boost::this_thread::interruption_point();
		}
	}

	path.reset();

	DELETE_VECTOR(nodes);
	tree = NULL; //deleted with m_nodes

	return false;
}

RRTNode* RRT::findNearestNeighbor(const Eigen::Affine3d& pose,
		std::list<RRTNode*>& nodes)
{
	double nearestDist = std::numeric_limits<double>::max();
	RRTNode* nearestNode = NULL;

	Eigen::Vector3d pos = pose.translation();
	double dist;

	for (auto& it : nodes)
	{
		dist = (it->m_pose.translation() - pos).norm();
		if (dist < nearestDist)
		{
			nearestNode = it;
		}
	}

	return nearestNode;
}

RRTNode* RRT::createNode(RRTNode* parent,
		const Eigen::Affine3d& pose,
		boost::shared_ptr<CollisionDetector>& cd)
{
	//move point
	Eigen::Vector3d pos = pose.translation();
	Eigen::Vector3d parentPos = parent->m_pose.translation();
	double norm = (pos - parentPos).norm();

	if (norm > m_maxExpansionDistance)
	{
		pos = parentPos + (pos - parentPos).normalized() * m_maxExpansionDistance;
	}

	RRTNode* node = new RRTNode(pose);
	node->m_pose = pose;
	node->m_pose.matrix().block<3, 1>(0, 3) = pos;
	node->m_parent = parent;

	//check controller
	node->m_controller = new SimulationRobotController(parent->m_pose, node->m_pose, parent->m_joints, m_robot, m_pd);
	node->m_controller->setCollisionDetection(cd);
	node->m_controllerSuccess = node->m_controller->canControl(3500, 0.1);

	if (node->m_controllerSuccess)
	{
		node->m_controller->getFinalJointState(node->m_joints);
		return node;
	}
	else
	{
		delete node; //cannot be reached, so remove it
		return NULL;
	}

	return NULL;
}

RRTNode* RRT::tryToReachGoal(const Eigen::Affine3d& goal,
		RRTNode* currentNode,
		boost::shared_ptr<CollisionDetector>& cd)
{
	RRTNode* node = new RRTNode(goal);
	node->m_pose = goal;
	node->m_parent = currentNode;

	node->m_controller = new SimulationRobotController(currentNode->m_pose, node->m_pose, currentNode->m_joints, m_robot, m_pd);
	node->m_controller->setCollisionDetection(cd);
	node->m_controllerSuccess = node->m_controller->canControl(3500, 0.01);

	if (node->m_controllerSuccess)
	{
		node->m_controller->getFinalJointState(node->m_joints);
		return node;
	}
	else
	{
		delete node; //cannot be reached, so remove it
		return NULL;
	}
}

bool RRT::planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const int startNodeId,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	PathPlanner::planSingleStartMultipleGoal(currentJointPose, currentTaskPose, startNodeId, cd, path);

	throw ais_definitions::NotImplementedException();
	return false;
}

boost::shared_ptr<Path> RRT::extractPath(RRTNode* lastRRTNode)
{
	int id = 0;
	boost::shared_ptr<Path> path(new Path(m_pd->getFrame()));
	std::vector<Path::Waypoint> waypoints;
	Path::Waypoint wp;

	RRTNode* node = lastRRTNode;
	while (node != NULL)
	{
		wp.id = id++;
		wp.pose = node->m_pose;
		wp.maxTranslationalVel = -1;
		wp.maxAngularVel = -1;

		if (node->m_parent != NULL)
		{
			node->m_controller->getJointPath(wp.trajectory);
			wp.jointPose = node->m_joints;
		}

		waypoints.push_back(wp);
		node = node->m_parent;
	}

	std::reverse(waypoints.begin(), waypoints.end());
	path->setWaypoints(waypoints);

	return path;
}

} /* namespace prm_planner */

