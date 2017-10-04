/*
 * a_star_planner.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#include <ais_definitions/class.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <prm_planner/planners/prm/prma_star.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_view.h>
#include <prm_planner/planners/prm/heuristic.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_robot/trajectory.h>
#include <prm_planner_robot/path.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <chrono>
#include <boost/filesystem.hpp>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <prm_planner/environment/planning_scene.h>

namespace prm_planner
{

PRMAStar::PRMAStar(boost::shared_ptr<ProblemDefinition> pd,
		const double timeout) :
				PathPlanner(pd),
				m_prm(NULL),
				m_timeout(timeout)
{
	//load prm
	parameters::ProblemConfig pc = pd->getConfig();
	m_prm = loadPRM(pc.prmConfig);
}

PRMAStar::PRMAStar(boost::shared_ptr<ProblemDefinition> pd,
		PRM* prm,
		const double timeout) :
				PathPlanner(pd),
				m_prm(prm),
				m_timeout(timeout)
{
}

PRMAStar::~PRMAStar()
{
	DELETE_VAR(m_prm);
}

bool PRMAStar::plan(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path,
		bool directConnectionRequired)
{
	if (!PathPlanner::plan(currentJointPose, currentTaskPose, goalTaskPose, cd, path, directConnectionRequired))
		return false;

	PRMView* view;
	{
		PLANNER_READ_LOCK();

		if (m_prm == NULL)
		{
			LOG_ERROR("PRM is NULL!");
			return false;
		}

		view = m_prm->getShallowDataCopy();
	}

	//get and add current pose
	const PRMNode* startNode = view->setStartPose(currentTaskPose, currentJointPose);
	if (startNode == NULL)
	{
		delete view;
		LOG_ERROR_COND(VERB, "Cannot connect start state with the PRM. Maybe there is a collision!");
		return false;
	}

	//add goal pose
	const PRMNode* goalNode = view->setGoalPose(goalTaskPose);
	if (goalNode == NULL)
	{
		delete view;
		LOG_ERROR_COND(VERB, "Cannot connect goal state with the PRM.");
		return false;
	}

	//no motion required
	if (startNode->getId() == goalNode->getId())
	{
		LOG_INFO_COND(VERB, "No motion required, returning empty path!");
		delete view;
		return false;
	}

	//stop here if necessary
	if (m_stopAllThreads)
	{
		delete view;
		return false;
	}

	//if requested first check if there is a chance without running
	//A* and directly connect both poses with a controller
	KDL::JntArray startJoints = currentJointPose;	//because of const
	if (ParameterServer::useShortcuts && testDirectConnection(startNode, goalNode, startJoints, view, cd, path))
	{
		delete view;
		return true;
	}

	//we found no direct connection, if one is required return
	if (directConnectionRequired)
	{
//		LOG_INFO("Direct connection required, but no plan was found");
		delete view;
		return false;
	}

	//stop here if necessary
	if (m_stopAllThreads)
	{
		delete view;
		return false;
	}

	bool result = plan(currentJointPose, currentTaskPose, goalTaskPose, startNode, goalNode, view, cd, path);

	delete view;

	return result;
}

bool PRMAStar::planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const int startNodeId,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	if (!PathPlanner::planSingleStartMultipleGoal(currentJointPose, currentTaskPose, startNodeId, cd, path))
		return false;

	PRMView* view;
	{
		PLANNER_READ_LOCK();

		if (m_prm == NULL)
		{
			LOG_ERROR("PRM is NULL!");
			return false;
		}

		view = m_prm->getShallowDataCopy();
	}

	//get start pose
	const PRMNode* startNode = view->setStartPose(startNodeId, currentJointPose);

	if (startNode == NULL)
	{
		delete view;
		LOG_ERROR_COND(VERB, "Cannot get start node. Maybe id is out of range");
		return false;
	}

	bool result = plan(currentJointPose, currentTaskPose, Eigen::Affine3d::Identity(), startNode, NULL, view, cd, path);

	delete view;

	return result;
}

void PRMAStar::expandNode(AStarNode* goal,
		AStarNode* node,
		KDL::JntArray& goalJoints,
		AStarNodeMap& closedList,
		AStarNodeMap& openListElements,
		Heap& openList,
		PRMView* view,
		CounterMap& nodeCounter)
{
	//expand node
	for (auto it : view->getEdgesToNode(node->node))
	{
		const PRMNode* nextNode = it->getOtherNode(node->node->getId());
//		LOG_INFO("Check node " << nextNode->getId());
		int nextId = nextNode->getId();
		if (CHECK_MAP(closedList, nextId))
		{
			continue;
		}

		if (view->isBlocked(it))
		{
			continue;
		}

		double tentativeG = node->g + it->getCosts();
		bool inOpenList = CHECK_MAP(openListElements, nextId);
		if (inOpenList && tentativeG >= openListElements[nextId]->g)
		{
			continue;
		}

		if (inOpenList)
		{
			AStarNode* a = openListElements[nextId];
			a->g = tentativeG;
			a->h = heuristic(a, goal);
			a->predecessor = node;
			a->edgeToThisNode = it;
			a->startJoints = goalJoints;
			a->updateF();
			openList.decrease(a->handle);
		}
		else
		{
			AStarNode* a = new AStarNode(nextNode, it, nextId, node, tentativeG);
			a->h = heuristic(a, goal);
			a->updateF();
			a->handle = openList.push(a);
			a->startJoints = goalJoints;
			openListElements[a->id] = a;
			nodeCounter[a->id] = 0;
		}
	}
}

bool PRMAStar::extractPath(AStarNode* goalNode,
		AStarNode* startNode,
		PRMView* view,
		Path& path,
		EdgeList& blockedEdges)
{
	AStarNode* current = goalNode;
	AStarNode* last = NULL;

	std::vector<Path::Waypoint> waypoints;
	std::list<AStarNode*> nodes;

	while (current != NULL)
	{
		Path::Waypoint w;
		w.pose = current->node->getPose();
		w.id = current->node->getId();
		w.maxTranslationalVel = -1;
		w.maxAngularVel = -1;
		w.jointPose = current->startJoints;

		//store computed trajectory
//		if (current->predecessor != NULL)
//		{
			if (last != NULL)
				view->getJointPath(last->edgeToThisNode, w.trajectory);
//		}

		waypoints.push_back(w);
		nodes.push_front(current);
		last = current;
		current = current->predecessor;
	}

	//test it again with smaller timesteps
	KDL::JntArray finalPose;
	KDL::JntArray startPose;
//	LOG_INFO(m_robot->getKDLPositions().data.transpose());
	boost::shared_ptr<CollisionDetector> dummy;
	for (const auto& it : nodes)
	{
		if (it->edgeToThisNode != NULL)
		{
//			LOG_INFO(startPose.data.transpose());
			const PRMNode* startPRMNode = it->edgeToThisNode->getOtherNode(it->node->getId());
//			LOG_INFO(startPRMNode->getId() << " -> " << it->node->getId());

			//don't run collision checks here
			if (!view->updateEdge(it->edgeToThisNode, 2100, finalPose, startPRMNode, startPose,
					dummy, 0.025))
			{
				view->setIsBlocked(it->edgeToThisNode, true);
				blockedEdges.push_back(it->edgeToThisNode);
				return false;
			}
		}
		else
		{
			finalPose = it->startJoints;
		}

		startPose = finalPose;
	}

//	LOG_INFO("successful");

	std::reverse(waypoints.begin(), waypoints.end());
	path.setWaypoints(waypoints);

	return true;
}

PRMAStar::AStarNode::AStarNode(const PRMNode* node,
		const PRMEdge* edgeToThisNode,
		const uint64_t id,
		AStarNode* pre,
		const double g) :
				g(g),
				f(0),
				h(0),
				node(node),
				predecessor(pre),
				id(id),
				edgeToThisNode(edgeToThisNode)
{
}

double PRMAStar::getTimeout() const
{
	PLANNER_READ_LOCK();

	return m_timeout;
}

void PRMAStar::setTimeout(double timeout)
{
	PLANNER_WRITE_LOCK();

	m_timeout = timeout;
}

void PRMAStar::AStarNode::updateF()
{
	f = g + h;
}

double PRMAStar::heuristic(AStarNode* n1,
		AStarNode* n2)
{
	//for unknown goal nodes
	//it's not a real A* anymore
	if (n1->node == NULL || n2->node == NULL)
	{
		return 0.0;
	}

//	return Heuristic::euclideanDistEndEffector(n1, n2);
	return Heuristic::frameDistEndEffector(n1->node, n2->node);
}

bool PRMAStar::plan(const KDL::JntArray& currentJointPose,
		const Eigen::Affine3d& currentTaskPose,
		const Eigen::Affine3d& goalTaskPose,
		const PRMNode* startNode,
		const PRMNode* goalNode,
		PRMView* view,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	bool verbose = VERB;
	bool runOuterLoop = true;
	AStarNode* endAStarNode = NULL;
	AStarNode* startAStarNode = NULL;
	bool controllable = false;
	struct timespec startTime, nowTime;

	EdgeList blockedEdges;
	Heap openList;
	AStarNodeMap openListElements;
	AStarNodeMap closedList;
	CounterMap nodeCounter;

	//stop here if necessary
	if (m_stopAllThreads)
	{
		return false;
	}

	//loop is used to restart planning, if there was an edge which was
	//not executable with a smaller dt.
	clock_gettime( CLOCK_THREAD_CPUTIME_ID, &startTime);//we use CLOCK_THREAD_CPUTIME_ID to get thread time
	while (runOuterLoop)
	{
		if (m_timeout > 0)
		{
			clock_gettime( CLOCK_THREAD_CPUTIME_ID, &nowTime);
			double diff = ( nowTime.tv_sec - startTime.tv_sec ) + ( nowTime.tv_nsec - startTime.tv_nsec ) / 1000000000.0;
			if (diff > m_timeout)
			break;
		}

		//stop here if necessary
		if (m_stopAllThreads)
		return false;

		KDL::JntArray predictedJointPose;
		KDL::Frame nodePose;

		const int startId = startNode->getId();

		endAStarNode = new AStarNode(goalNode, NULL, goalNode == NULL ? -1 : goalNode->getId(), NULL, 0);
		startAStarNode = new AStarNode(startNode, NULL, startId, NULL, 0);
		startAStarNode->h = heuristic(startAStarNode, endAStarNode);
		startAStarNode->updateF();
		startAStarNode->handle = openList.push(startAStarNode);
		view->getJointPose(startNode, startAStarNode->startJoints);
		openListElements[startId] = startAStarNode;
		nodeCounter[startId] = 0;
		path.reset(new Path(m_pd->getFrame()));
		controllable = false;
		int counter = 0;
		boost::shared_ptr<Path> directPath;

		while (!openList.empty())
		{
			if (m_timeout > 0)
			{
				clock_gettime( CLOCK_THREAD_CPUTIME_ID, &nowTime);
				double diff = ( nowTime.tv_sec - startTime.tv_sec ) + ( nowTime.tv_nsec - startTime.tv_nsec ) / 1000000000.0;
				if (diff > m_timeout)
				{
					break;
				}
			}

			if (m_stopAllThreads)
			return false;

			AStarNode* node = openList.top();

			openList.pop();

//			LOG_INFO("pop " << node->node->getId() << " " << node->f);

			openListElements.erase(node->id);

			//update edges between the parent and the current node
			if (node->edgeToThisNode != NULL)
			{
				if (view->isBlocked(node->edgeToThisNode))
				{
					DELETE_VAR(node);
					continue;
				}

				//Check if we can directly reach the goal. It is not optimal,
				//because we can miss some better solutions. Turn this of, if
				//require an optimal solution
//				if (goalNode != NULL && node->node->getId() != goalNode->getId() && testDirectConnection(node->node, goalNode, node->startJoints,view, directPath))
//				{
//					LOG_INFO("Found direction connection to goal between " << node->edgeToThisNode->getNode1()->getId()
//							<< " and " << node->edgeToThisNode->getNode2()->getId());
//					if (extractPath(node, startAStarNode, view, *path, blockedEdges))
//					{
//						for (auto& it: *directPath)
//						{
//							path->addWaypoint(it);
//						}
//
//						//unblock edges
//						for (auto& it: blockedEdges)
//						{
//							view->setIsBlocked(it, false);
//						}
//						blockedEdges.clear();
//
//						runOuterLoop = false;
//						break;
//					}
//				}

				//update edge controller based on final predicted joint pose
				//of the parent node. The start node should correspond to the
				//current robot pose
				const PRMNode* startPRMNode = node->edgeToThisNode->getOtherNode(node->node->getId());

				controllable = view->updateEdge(node->edgeToThisNode, 1000, predictedJointPose, startPRMNode, node->startJoints, cd, 0.01);//0.08 0.01s = 70 iterations notebook

				if (!controllable)
				{
//					LOG_INFO("Found no connection between " << startPRMNode->getId() << " and " << node->node->getId());
					int& counter = nodeCounter[node->id];
					++counter;

					//we block the edge if were not able to find a path
					//for n times
					if (counter >= 10)
					{
						closedList[node->id] = node;
						continue;
					}
					else
					{
						//here we need to delete the pointer directly
						//otherwise the pointer is deleted implicitly via m_closedList
						DELETE_VAR(node);

						continue;
					}
				}
			}
			else //start node
			{
				predictedJointPose = node->startJoints;
			}

			closedList[node->id] = node;

			if ((goalNode == NULL && node->node->getType() == PRMNode::GoalNode)
					|| (goalNode != NULL && *node->node == *goalNode))
			{
				if (!extractPath(node, startAStarNode, view, *path, blockedEdges))
				{
					DELETE_MAP(closedList);
					DELETE_MAP(openListElements);
					DELETE_VAR(endAStarNode);

					closedList.clear();
					openListElements.clear();
					openList.clear();
//					m_nodeCounter.clear() //we dont clear the counter list here
					break;//start planning again (extractPath blocks the corresponding edge)
				}
				//we found a valid path
				else
				{
					//unblock edges
					for (auto& it: blockedEdges)
					{
						view->setIsBlocked(it, false);
					}
					blockedEdges.clear();

					runOuterLoop = false;
					break;
				}
			}

			expandNode(endAStarNode, node, predictedJointPose, closedList, openListElements, openList, view, nodeCounter);

			//check if there was an interruption
			if (++counter % 5 == 0)
			{
				boost::this_thread::interruption_point();
			}
		}

		if (openList.empty())
		{
			break;
		}
	}

	bool result = false;
	if (path->empty())
	{
//		LOG_INFO_COND(verbose, "No plan was found");
		result = false;
	}
	else
	{
//		LOG_INFO_COND(verbose, "Plan found with " << path->size() << " waypoints");
		result = true;
	}

	DELETE_MAP(closedList);
	DELETE_MAP(openListElements);
	DELETE_VAR(endAStarNode);

	//than clear the maps
	closedList.clear();
	openListElements.clear();
	openList.clear();
	nodeCounter.clear();

	return result;
}

void PRMAStar::update(const boost::shared_ptr<PlanningScene> planningScene)
{
	PathPlanner::update(planningScene);

	PLANNER_WRITE_LOCK();

	if (m_prm != NULL)
		m_prm->update(planningScene->octomap);
}

void PRMAStar::publish()
{
	PLANNER_READ_LOCK();

	if (m_prm != NULL)
	m_prm->publish();
}

bool PRMAStar::testDirectConnection(const PRMNode* startNode,
		const PRMNode* goalNode,
		KDL::JntArray& startJoints,
		PRMView* view,
		boost::shared_ptr<CollisionDetector>& cd,
		boost::shared_ptr<Path>& path)
{
	PRMEdge* edge = new PRMEdge(startNode, goalNode, m_prm->m_nextIdEdge); //m_prm->addEdge(startNode, goalNode);
	KDL::JntArray finalJoints;
	bool edgeOK = view->updateEdge(edge, 1750, finalJoints, startNode, startJoints, cd, 0.01); //0.25s = 1750 iterations notebook

	if (edgeOK)
	{
		path.reset(new Path(m_pd->getFrame()));

		std::vector<Path::Waypoint> waypoints;
		Path::Waypoint w1(startNode->getId(), startNode->getPose());
		Path::Waypoint w2(goalNode->getId(), goalNode->getPose());
		view->getJointPath(edge, w1.trajectory);
		w1.jointPose = startJoints;
		w2.jointPose = finalJoints;
		waypoints.push_back(w1);
		waypoints.push_back(w2);
		path->setWaypoints(waypoints);

		view->removeEdgeFromEdgeMap(edge);
		DELETE_VAR(edge);
		return true;
	}
	else
	{
		view->removeEdgeFromEdgeMap(edge);
		DELETE_VAR(edge);

		return false;
	}
}

void PRMAStar::setPRM(PRM* prm)
{
	PLANNER_WRITE_LOCK();
	DELETE_VAR(m_prm);
	m_prm = prm;
}

PRM* PRMAStar::loadPRM(const parameters::PRMConfig& config)
{
	if (boost::filesystem::exists(config.filename))
	{
		return PRM::load(config, m_robot, m_pd);
	}
	else
	{
		m_prm = new PRM(m_robot,
				config,
				m_pd);

		if (config.save)
			m_prm->save(config.filename);

		return m_prm;
	}
}

void PRMAStar::initCollisionAvoidance()
{
	//do nothing here, because the planner will
	//generate and delete the instances by itself
	//to be threadsafe
//	PathPlanner::initCollisionAvoidance();
}

}

/* namespace prm_planner */
