/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: problem_definition.cpp
 */

#include <ais_point_cloud/point_cloud.h>
#include <ais_ros/ros_base_interface.h>
#include <ais_point_cloud/point_cloud_helpers.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ais_util/progress_bar.h>
#include <boost/filesystem.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <eigen_conversions/eigen_kdl.h>
#include <omp.h>
#include <pcl/filters/project_inliers.h>
#include <pluginlib/class_loader.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner/prm_planner.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/util/parameter_server.h>
#include <prm_planner_constraints/constraint_factory.h>
#include <prm_planner_robot/path.h>
#include <prm_planner/planners/prm/prma_star.h>
#include <prm_planner/planners/rrt/rrt.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/objects/dropping_region.h>
#include <prm_planner/path_database/path_database.h>
#include <prm_planner/problem_definitions/approaching_problem_definition.h>
#include <prm_planner/robot/feasibility_checker.h>
#include <prm_planner/robot/trajectory_optimizer.h>
#include <prm_planner/visualization/object_interactive_marker.h>
#include <prm_planner/visualization/robot_arm_interactive_marker.h>

namespace prm_planner
{

pluginlib::ClassLoader<ProblemDefinition>* ProblemDefinition::s_loader = NULL;

ProblemDefinition::ProblemDefinition() :
				m_objectManager(NULL),
				m_transformationPlanningToArm(Eigen::Affine3d::Identity())
{
}

ProblemDefinition::~ProblemDefinition()
{
}

void ProblemDefinition::init(const boost::shared_ptr<Robot> robot,
		const boost::shared_ptr<Constraint> constraint,
		const boost::shared_ptr<PRMPlanner> plannerInterface,
		const parameters::ProblemConfig& config)
{
	PD_WRITE_LOCK();

	m_robot = robot;
	m_constraint = constraint;
	m_plannerInterface = plannerInterface;
	m_pathDatabase = PathDatabase::open("...");
	c_config = config;
	m_objectManager = ObjectManager::getInstance();

	//initialize object manager
	ObjectManager::getInstance()->loadObjects(c_config.objects,
			c_config.droppingConfig, ParameterServer::octomapResolution, c_config.planningFrame);

	initPlanner();
}

void ProblemDefinition::reset()
{
	PD_WRITE_LOCK();

	initPlanner();
}

void ProblemDefinition::update(const boost::shared_ptr<PlanningScene>& planningScene)
{
	PD_WRITE_LOCK();

	m_planningScene = planningScene;
	m_planner->update(m_planningScene);
}

void ProblemDefinition::initPlanner()
{
	if (c_config.plannerType == "prm_a_star")
	{
		m_planner.reset(new PRMAStar(shared_from_this(), ParameterServer::maxPlanningTime));
	}
	else if (c_config.plannerType == "rrt")
	{
		m_planner.reset(new RRT(shared_from_this(), ParameterServer::maxPlanningTime));
	}
	else
	{
		LOG_FATAL("Unknown planner type: " << c_config.plannerType << ". Known planners are:"
				"prm_a_star, rrt");
		exit(3452);
	}
}

const std::string& ProblemDefinition::getFrame() const
{
	return c_config.planningFrame;
}

const std::string& ProblemDefinition::getRootFrame() const
{
	return c_config.rootFrame;
}

void ProblemDefinition::publish()
{
	PLANNER_READ_LOCK();
	if (m_planner)
	{
		m_planner->publish();
	}

	boost::recursive_mutex::scoped_lock lock2(m_droppingRegionMutex);
	if (m_droppingRegion.get() != NULL)
	{
		m_droppingRegion->publish();
	}
}

const parameters::ProblemConfig& ProblemDefinition::getConfig() const
{
	return c_config;
}

parameters::ProblemConfig ProblemDefinition::getConfig()
{
	return c_config;
}

const boost::shared_ptr<Constraint>& ProblemDefinition::getConstraint() const
{
	return m_constraint;
}

const boost::shared_ptr<Robot>& ProblemDefinition::getRobot() const
{
	return m_robot;
}

bool ProblemDefinition::updateToolFrames()
{
	//do nothing, since we don't use tool frames in the base problem definition
	return true;
}

bool ProblemDefinition::plan(const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path,
		const PlanningParameters& parameters)
{
	//some planners (RRT) needs to be reseted before
	//running the planner. Other planners can ignore
	//the reset method. If you don't implement the
	//reset method nothing happens.
	m_planner->reset();

	bool success = false;

	//update tfs between object and gripper
	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();
	if (gripper.get() != NULL && !gripper->getCurrentObject().empty())
	{
		boost::shared_ptr<GraspableObject> object = m_objectManager->getObject(gripper->getCurrentObject());
		Eigen::Affine3d transformation;
		if (object->getTransformationFromFrame(gripper->c_parameters.graspFrame, transformation))
			gripper->setTGripperToObject(transformation);
	}

	switch (parameters.mode)
	{
		case GraspObject:
			//using the first arm for grasping
			return planGrasping(parameters.objectName, path);
		case DropObject:
			//using the first arm for droping
			return planDropping(parameters.objectName, goal, path);
		default:
			//uses empty start poses => using current
			return planDefault(KDL::JntArray(), Eigen::Affine3d::Identity(), goal, path, parameters);
	}
}

bool ProblemDefinition::plan(const KDL::JntArray& startJoint,
		const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path,
		const PlanningParameters& parameters)
{
	//some planners (RRT) needs to be reseted before
	//running the planner. Other planners can ignore
	//the reset method. If you don't implement the
	//reset method nothing happens.
	m_planner->reset();

	bool success = false;

	switch (parameters.mode)
	{
		case GraspObject:
			//using the first arm for grasping
			return planGrasping(parameters.objectName, path);
		case DropObject:
			//using the first arm for droping
			return planDropping(parameters.objectName, goal, path);
		default:
			return planDefault(startJoint, startPose, goal, path, parameters);
	}
}

boost::shared_ptr<ProblemDefinition> ProblemDefinition::load(const std::string& package,
		const std::string& library)
{
	//create instance if not already available
	if (s_loader == NULL)
		s_loader = new pluginlib::ClassLoader<ProblemDefinition>("prm_planner", "prm_planner::ProblemDefinition");

	boost::shared_ptr<ProblemDefinition> interface;

	try
	{
		interface = s_loader->createInstance(library);
	}
	catch (pluginlib::PluginlibException& ex)
	{
		LOG_FATAL("The plugin failed to load for some reason. Error: " << ex.what());
		exit(2);
	}

	return interface;
}

double ProblemDefinition::getWorkspaceRadius()
{
	return 1.3;
}

Eigen::Vector3d ProblemDefinition::getWorkspaceCenter()
{
	return Eigen::Vector3d::Zero();
}

boost::shared_ptr<PathPlanner> ProblemDefinition::getPathPlanner() const
{
	return m_planner;
}

bool ProblemDefinition::planDefault(const KDL::JntArray& startJoint,
		const Eigen::Affine3d& startPose,
		const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path,
		const PlanningParameters& params)
{
	Eigen::Affine3d currentTaskPose = startPose;
	KDL::JntArray currentJointPose = startJoint;

	if (startJoint.rows() == 0)
	{
		currentTaskPose = getCurrentTaskPose();
		currentJointPose = m_robot->getKDLChainJointState();
	}

	//goal already reached
	if (currentTaskPose.isApprox(goal, 1e-5))
	{
		LOG_INFO("Start and goal pose are the same");
		return true;
	}

	boost::shared_ptr<CollisionDetector> cd;
	if (params.useCollisionDetection)
	{
		m_mutex.lock_shared();
		cd.reset(new CollisionDetector(m_robot, m_planningScene));
		m_mutex.unlock_shared();
	}

	FeasibilityChecker feasibilityChecker(m_robot);
	if (!feasibilityChecker.check(goal, cd, true))
	{
		LOG_WARNING_COND(VERB, "Goal state invalid!");
		return false;
	}

	//check database if a plan fits
	if (ParameterServer::usePathDatabase)
	{
		path = m_pathDatabase->findBestPlan(currentJointPose, currentTaskPose, goal);
		if (path.get() != NULL)
		{
			LOG_ERROR("Check return value of optimizer (i.e. check if optimizable in optimize()");
			boost::shared_ptr<Path> optimizedPath(new Path(getFrame()));

			TrajectoryOptimizer opt(path, cd, m_robot, shared_from_this());
			if (opt.optimize(optimizedPath, true))
			{
				path = optimizedPath;
				return true;
			}
		}
	}
	bool result = m_planner->plan(currentJointPose, currentTaskPose, goal, cd, path, params.directConnectionRequired);

	//optimize trajectory
	if (result)
	{
		if (ParameterServer::usePathDatabase)
		{
			//add the path to the database
			m_pathDatabase->addPath(path);

			//run prune from time to time
		}

		//optimization
		if (ParameterServer::useTrajectoryOptimization)
		{
			TrajectoryOptimizer opt(path, cd, m_robot, shared_from_this());

			boost::shared_ptr<Path> optimizedPath(new Path(getFrame()));
			if (!opt.optimize(optimizedPath, false))
			{
				return false;
			}

			path = optimizedPath;
		}
		return true;
	}
	else
	{
		return false;
	}
}

bool ProblemDefinition::planGrasping(const std::string& object,
		boost::shared_ptr<Path>& path)
{
	const int steps = 100;

	//get object pose
	boost::shared_ptr<GraspableObject> obj = m_objectManager->getObject(object);
	if (obj.get() == NULL)
	{
		LOG_INFO("Object unknown: " << object);
		return false;
	}

	ros::Rate r(10);
	int counter = 0;
	while (!obj->isActive())
	{
		LOG_INFO_COND(counter % 100 == 0, "No transformation of object " << object << " available!");
		r.sleep();
		if (counter++ == 1000)
			return false;
	}

	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();

	const parameters::ArmConfig& ac = ParameterServer::robotConfigs[m_robot->c_robotName].arms[m_robot->getName()];
	const parameters::HandConfig& hc = ac.hand;

	if (gripper.get() == NULL)
	{
		LOG_ERROR("The robot has no gripper");
		return false;
	}

	if (!gripper->getCurrentObject().empty())
	{
		if (!obj->isInGripper(gripper))
		{
			gripper->setCurrentObject("");
			LOG_INFO("Removed object from gripper, because it could not be found!");
		}
		else
		{
			LOG_ERROR("This object is already in the gripper: " << gripper->getCurrentObject());
			return false;
		}
	}

	Eigen::Affine3d pose = obj->getPose();

	//get robot state
	const Eigen::Affine3d currentTaskPose = getCurrentTaskPose();
	KDL::JntArray currentPose = m_robot->getKDLChainJointState();

	//setup threads
	boost::atomic_bool found(false);
	FeasibilityChecker feasibilityChecker(m_robot);

	PD_READ_LOCK();
	CollisionDetector::OpenMPCollisionDetectorStruct cds(m_robot, m_planningScene, { obj });

	ais_util::ProgressBar progress("Planning", steps);

#pragma omp parallel for shared(found) schedule(dynamic) firstprivate(cds)
	for (int i = 0; i < steps; ++i)
	{
		progress.increment();

		if (found)
			continue;

		boost::shared_ptr<Path> pathToObject, pathToPost, pathToCurrent;
		boost::shared_ptr<Path> tmpPath;

		int threadNum = omp_get_thread_num();

		//compute grasp pose
		Eigen::Affine3d prePose, pose, postPose;
		obj->sampleGraspPose(prePose, pose, postPose, hc, gripper);

		//check feasibility
		if (!feasibilityChecker.check(prePose, cds.cdWithObject)
				|| !feasibilityChecker.check(pose, cds.cdWithoutObject)
				|| !feasibilityChecker.check(postPose, cds.cdWithoutObject))
			continue;

		if (found)
			continue;

		//plan to pre pose
		if (!m_planner->plan(currentPose, currentTaskPose, prePose, cds.cdWithObject, tmpPath))
			continue;

		if (found)
			continue;

		//compute pre -> pose
		if (!m_planner->plan(tmpPath->back().jointPose, tmpPath->back().pose, pose, cds.cdWithoutObject, pathToObject, true))
			continue;

		if (found)
			continue;

		//compute pose -> post
		if (!m_planner->plan(pathToObject->back().jointPose, pathToObject->back().pose, postPose, cds.cdWithoutObject, pathToPost, true))
			continue;

		if (found)
			continue;

		//compute post -> current
		if (!m_planner->plan(pathToPost->back().jointPose, pathToPost->back().pose, currentTaskPose, cds.cdWithoutObject, pathToCurrent, true))
			continue;

		if (found)
			continue;

		//add waypoint for opening the gripper
		Path::Waypoint wpOpen = Path::Waypoint::getOpenGripperWaypoint();
		tmpPath->append(wpOpen);

		//move to the object, i.e., appending path 2
		tmpPath->append(*pathToObject);

		//add waypoint for closing the gripper
		Path::Waypoint wpClose = Path::Waypoint::getCloseGripperWaypoint();
		wpClose.helper = object; //set the name for the executer
		wpClose.pose = pose;
		tmpPath->append(wpClose);

		tmpPath->append(*pathToPost);
		tmpPath->append(*pathToCurrent);

		tmpPath->cleanPath();

#pragma omp critical
		{
			path = tmpPath;
			found = true;

			//stop all planning threads
			m_planner->stopAllThreads();
		}
	}
	progress.finish();

	//allow planning again
	m_planner->resetStopAllThreads();

	return found;
}

bool ProblemDefinition::planDropping(const std::string& objectName,
		const Eigen::Affine3d& goal,
		boost::shared_ptr<Path>& path)
{
	const int steps = c_config.droppingConfig.tries;
	const int ompThread = c_config.droppingConfig.numberOfCores;

	boost::shared_ptr<GripperInterface> gripper = m_robot->getGripper();

	if (gripper.get() == NULL)
	{
		LOG_ERROR("The robot has no hand");
		return false;
	}

	const std::string name = gripper->getCurrentObject();
	if (name.empty())
	{
		LOG_ERROR("There is no object in the gripper");
		return false;
	}

	boost::shared_ptr<GraspableObject> object = m_objectManager->getObject(name);
	if (object.get() == NULL)
	{
		LOG_ERROR("There is an unknown object in the gripper!");
		return false;
	}

	//get robot state
	Eigen::Affine3d currentTaskPose = getCurrentTaskPose();
	KDL::JntArray currentPose = m_robot->getKDLChainJointState();

	Eigen::Vector3d goalPos = goal.translation();

	//not optimal yet. so: TODO: implement a flag to find a pose
	//automatically.
	const bool automaticSearch = goalPos.isZero();

	boost::shared_ptr<Path> bestPath;
	double bestLength = std::numeric_limits<double>::max();

	//get automatic dropping region
	if (automaticSearch)
	{
		{
			boost::recursive_mutex::scoped_lock lock2(m_droppingRegionMutex);
			m_droppingRegion.reset();
		}

		Eigen::Affine3d goalPose;
		if (!findDropPosition(object, goalPose))
		{
			LOG_ERROR("Cannot find a dropping pose automatically. Maybe there is"
					" no large enough horizontal plane");

			return false;
		}

		goalPos = goalPose.translation();
	}

	boost::atomic_int found(0);

	//setup threads
//	omp_set_num_threads(ompThread);

	//create feasibility checker
	FeasibilityChecker feasibilityChecker(m_robot);

	//create collision detectors
	PD_READ_LOCK();
	CollisionDetector::OpenMPCollisionDetectorStruct cd(m_robot, m_planningScene);

	ais_util::ProgressBar progress("Planning", steps);

#pragma omp parallel for shared(found, bestLength, bestPath) schedule(dynamic) firstprivate(cd)
	for (int i = 0; i < steps; ++i) // && !boost::this_thread::interruption_requested()
	{
		//get only 5 best results
		if (found < 5)
		{
			int threadNum = omp_get_thread_num();
			boost::shared_ptr<Path> tmpPath;
			boost::shared_ptr<Path> pathToObject, pathToPost;

			progress.increment(bestLength > 100000 ? "No solution found!" : "Current path length: " + std::to_string(bestLength));
			//compute grasp pose
			Eigen::Affine3d prePose, pose, postPose;

			//if automatic search is need, try to find a position automatically
			if (automaticSearch)
			{
				Eigen::Affine3d goalPose;
				if (!m_droppingRegion->sample(goalPose))
				{
					LOG_ERROR("Cannot find a dropping pose automatically. Maybe there is"
							" no large enough horizontal plane");

					found = -1;
				}

				goalPos = goalPose.translation();
			}

			if (found >= 0)
			{
				object->sampleDropPose(prePose, pose, postPose, goalPos, m_robot->getGripper());

				//check feasibility
				if (!feasibilityChecker.check(prePose, cd.cdWithoutObject)
						|| !feasibilityChecker.check(pose, cd.cdWithoutObject)
						|| !feasibilityChecker.check(postPose, cd.cdWithoutObject))
					continue;

				//plan to pre pose
				if (!m_planner->plan(currentPose, currentTaskPose, prePose, cd.cdWithoutObject, tmpPath))
					continue;

				if (!m_planner->plan(tmpPath->back().jointPose, tmpPath->back().pose, pose, cd.cdWithoutObject, pathToObject, true))
					continue;

				if (!m_planner->plan(pathToObject->back().jointPose, pathToObject->back().pose, postPose, cd.cdWithoutObject, pathToPost, true))
					continue;

				double length = tmpPath->getPathLength();

				tmpPath->append(*pathToObject);

				//add waypoint for opening the gripper
				Path::Waypoint wpOpen = Path::Waypoint::getOpenGripperWaypoint();
				tmpPath->append(wpOpen);

				tmpPath->append(*pathToPost);

				//add waypoint for closing the gripper
				Path::Waypoint wpClose = Path::Waypoint::getCloseGripperWaypoint();
				tmpPath->append(wpClose);

				tmpPath->cleanPath();

#pragma omp critical
				{
					if (length < bestLength)
					{
						bestLength = length;
						bestPath = tmpPath;
						++found;
					}
				}
			}
		}
	}
	progress.finish();

	if (bestPath.get() != NULL)
	{
		LOG_INFO("Shortest path has length " << bestLength);
		path = bestPath;
		return true;
	}

	return false;
}

bool ProblemDefinition::findStartTaskPose(Eigen::Affine3d& startPose,
		bool& hasToApproach)
{
	return false;
}

bool ProblemDefinition::findDropPosition(const boost::shared_ptr<GraspableObject>& object,
		Eigen::Affine3d& pose)
{
	using namespace ais_point_cloud;

	{
		//current no data available
		PD_READ_LOCK()
		if (!m_planningScene || !m_planningScene->rgbd)
		{
			return false;
		}
	}

	MyPointCloudP cloudIn = m_planningScene->rgbd->getCloud();

	//find points in grippers range
	pcl::KdTreeFLANN<ais_point_cloud::MyPointType> kdtree;
	kdtree.setInputCloud(cloudIn);
	boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
	std::vector<float> dists;

	//gripper point. Used to find closest point on
	//table or shelf
	ais_point_cloud::MyPointType pointGripper;
	Eigen::Affine3d gripperPose;
	m_robot->getCurrentFK(gripperPose);
	Eigen::Vector3d gripperPos = gripperPose.translation();
	pointGripper.x = gripperPos.x();
	pointGripper.y = gripperPos.y();
	pointGripper.z = gripperPos.z();

	//get nearest point on plane
	if (kdtree.nearestKSearch(pointGripper, 1, *indices, dists) == 0)
	{
		LOG_ERROR("Point cloud seems to be empty");
		return false;
	}

	if (kdtree.radiusSearch(cloudIn->points[(*indices)[0]], c_config.droppingConfig.searchRadius, *indices, dists) == 0)
	{
		LOG_ERROR("Point cloud seems to be empty");
		return false;
	}

	MyPointCloudP cloud(new MyPointCloud);
	point_cloud_helpers::extract(cloudIn, indices, cloud);
	MyPointCloudP cloudOutliers(new MyPointCloud);

	//Extracts planes as long as they are big enough. If the
	//resulting point clouds get to small, the algorithm stops
	//and returns false. Otherwise the algorithm searches for
	//the largest horizontal plane it can find. If one is found
	//the algorithm computes a random position on the plane.
	while (true)
	{
		boost::recursive_mutex::scoped_lock lock2(m_droppingRegionMutex);

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		if (m_droppingRegion.get() == NULL)
		{
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			bool bigEnough;

			if (!point_cloud_helpers::getPlaneWithOrientation(cloud, inliers, coefficients, bigEnough,
					c_config.droppingConfig.ransacTreshold, c_config.droppingConfig.ransacMinPoints,
					c_config.upDir, c_config.droppingConfig.maxPlaneAngleToUp))
			{
				if (!bigEnough)
					return false;

				if (!point_cloud_helpers::extract(cloud, inliers, cloudOutliers, true))
					return false;

				cloud = cloudOutliers;
				continue;
			}

			//we found a horizontal plane with an adequate size
			Eigen::Affine3d dropPose;
			MyPointCloudP cloudPlaneProjected(new MyPointCloud);
			boost::recursive_mutex::scoped_lock lock(m_droppingRegionMutex);

			pcl::ProjectInliers<ais_point_cloud::MyPointType> proj;
			proj.setModelType(pcl::SACMODEL_PLANE);
			proj.setIndices(inliers);
			proj.setInputCloud(cloud);
			proj.setModelCoefficients(coefficients);
			proj.filter(*cloudPlaneProjected);

			LOG_INFO("Creating new dropping region");

			coefficients->values[3] -= 0.025;
			m_droppingRegion.reset(new DroppingRegion(cloudPlaneProjected, coefficients, c_config.droppingConfig));
		}

		if (!m_droppingRegion->sample(pose))
		{
			point_cloud_helpers::extract(cloud, inliers, cloudOutliers, true);
			m_droppingRegion.reset();
			cloud = cloudOutliers;
			continue;
		}
		else
		{
			return true;
		}
	}

	return false;
}

Eigen::Affine3d ProblemDefinition::samplePose()
{
	KDL::JntArray q;
	m_robot->sampleValidChainJointState(q);

	Eigen::Affine3d pose;
	m_robot->getFK(q, pose);

	Eigen::Affine3d t;
	if (!ais_ros::RosBaseInterface::getRosTransformationWithResult(m_robot->getRootFrame(), c_config.planningFrame, t))
	{
		t.setIdentity();
	}

	pose = t * pose;

	m_constraint->findNearestValidPose(pose);

	return pose;
}

Eigen::Affine3d ProblemDefinition::getCurrentTaskPose()
{
	Eigen::Affine3d pose;
	m_robot->getCurrentFK(pose);
	if (!ais_ros::RosBaseInterface::getRosTransformationWithResult(m_robot->getRootFrame(), c_config.planningFrame, m_transformationPlanningToArm))
	{
		m_transformationPlanningToArm.setIdentity();
	}
	return m_transformationPlanningToArm * pose;
}

std::vector<boost::shared_ptr<InteractiveMarker> > ProblemDefinition::getInteractiveMarkers()
{
	if (m_interactiveMarker.empty())
	{
		m_interactiveMarker.push_back(
				boost::shared_ptr<InteractiveMarker>(new RobotArmInteractiveMarker(m_robot, m_plannerInterface, m_constraint, "goal")));
		std::unordered_map<std::string, boost::shared_ptr<GraspableObject>>& objects = ObjectManager::getInstance()->getObjects();
		if (!objects.empty())
		{
			m_interactiveMarker.push_back(
					boost::shared_ptr<InteractiveMarker>(new ObjectInteractiveMarker(m_plannerInterface, objects, c_config.planningFrame)));
		}
	}
	return m_interactiveMarker;
}

} /* namespace prm_planner */

