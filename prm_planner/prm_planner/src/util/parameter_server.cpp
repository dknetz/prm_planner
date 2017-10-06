/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: parameter_server.cpp
 */

#include <ros/ros.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <boost/algorithm/string/replace.hpp>
#include <boost/regex.hpp>
#include <prm_planner/util/parameter_server.h>

#include <ros/package.h>

#define GET_PARAMETER_ERROR(parameter, var)\
	if (!n.getParam(parameter, var))\
	{\
		LOG_FATAL("There is no parameter " << n.getNamespace() << "/" << parameter);\
		return false;\
	}

#define GET_PARAMETER_DEFAULT(parameter, var, defaultValue)\
	if (!n.getParam(parameter, var))\
	{\
		var = defaultValue;\
	}

#define GET_PARAMETER_DEFAULT_ENUM(parameter, var, Enum, defaultValue)\
		{\
	int _var;\
	if (n.getParam(parameter, _var))\
	{\
		var = (Enum)_var;\
	}\
	else \
		var = defaultValue;\
	}

#define IF_GET_PARAMETER(parameter, var) if (n.getParam(parameter, var))
#define IF_NOT_GET_PARAMETER(parameter, var) if (!n.getParam(parameter, var))

namespace prm_planner
{

using namespace parameters;

//internal
bool ParameterServer::m_receiveImage = true;
bool ParameterServer::m_imageReceived = false;

//general
bool ParameterServer::useShortcuts = true;
bool ParameterServer::visualize = true;
bool ParameterServer::verbose = true;
bool ParameterServer::startPublishers = true;
bool ParameterServer::startSubscribers = true;
bool ParameterServer::debugInternal = false;
std::string ParameterServer::problem = "";
bool ParameterServer::connectedToVRep = false;
bool ParameterServer::useCollisionDetection = true;
bool ParameterServer::useTrajectoryOptimization = true;
bool ParameterServer::usePathDatabase = true;
parameters::ImageMode ParameterServer::oneImageMode = parameters::Frequently;

//parameters
double ParameterServer::maxPlanningTime = -1; //inf

//execution
parameters::ExecutionMode ParameterServer::executionMode = parameters::NoExecution;

//prm
double ParameterServer::octomapResolution = 0;
double ParameterServer::octomapProbHit = 0.7;
double ParameterServer::octomapProbMiss = 0.4;
double ParameterServer::octomapClampingThresholdMin = 0.12;
double ParameterServer::octomapClampingThresholdMax = 0.97;

//topics
std::string ParameterServer::topicCameraDepth;
std::string ParameterServer::topicCameraDepthInfo;
std::string ParameterServer::topicCameraPrefix;

//problem configs
std::unordered_map<std::string, RobotConfig> ParameterServer::robotConfigs;
std::unordered_map<std::string, ProblemConfig> ParameterServer::problemConfigs;
std::unordered_map<std::string, ControllerConfig> ParameterServer::controllerConfigs;
std::unordered_map<std::string, std::unordered_map<std::string, RobotJointSampleLimits>> ParameterServer::sampleLimits;
std::vector<std::string> ParameterServer::armNames;

//general local parameters
bool ParameterServer::startRosSpinner = false;

//mutex
boost::mutex ParameterServer::s_parameterMutex;
bool ParameterServer::s_parametersLoaded = false;

ParameterServer::ParameterServer()
{
}

ParameterServer::~ParameterServer()
{
}

bool ParameterServer::loadParameters()
{
	boost::mutex::scoped_lock lock2(s_parameterMutex);

	if (s_parametersLoaded)
	{
		return true;
	}

	ros::NodeHandle n;

	//general
	GET_PARAMETER_ERROR("general/verbose", verbose);
	GET_PARAMETER_ERROR("general/use_shortcuts", useShortcuts);
	GET_PARAMETER_ERROR("general/visualize", visualize);
	GET_PARAMETER_ERROR("general/start_ros_spinner", startRosSpinner);
	GET_PARAMETER_ERROR("general/start_ros_publishers", startPublishers);
	GET_PARAMETER_ERROR("general/start_ros_subscribers", startSubscribers);
	GET_PARAMETER_ERROR("general/problem_definition", problem);
	GET_PARAMETER_ERROR("general/connected_to_vrep", connectedToVRep);
	GET_PARAMETER_DEFAULT("general/use_collision_detection", useCollisionDetection, true);
	GET_PARAMETER_DEFAULT("general/use_trajectory_optimization", useTrajectoryOptimization, true);
	GET_PARAMETER_DEFAULT("general/use_path_database", usePathDatabase, false);
	GET_PARAMETER_DEFAULT_ENUM("general/one_image_mode", oneImageMode, parameters::ImageMode, parameters::Frequently);

	m_receiveImage = true;
	m_imageReceived = false;

	if (oneImageMode == parameters::OneImage)
	{
		LOG_INFO("Using one image mode (thus assuming a static environment)!");
	}
	else if (oneImageMode == parameters::OneImagePerPlan)
	{
		LOG_INFO("Using one image mode per plan (thus assuming a static environment)!");
	}

	//planning
	GET_PARAMETER_ERROR("planning/max_planning_time", maxPlanningTime);

	//execution
	GET_PARAMETER_DEFAULT_ENUM("execution/mode", executionMode, parameters::ExecutionMode, parameters::NoExecution);

	if (executionMode == parameters::NoExecution)
	{
		LOG_INFO("No execution will happen!");
	}

	//octomap
	GET_PARAMETER_ERROR("octomap/resolution", octomapResolution);
	GET_PARAMETER_ERROR("octomap/prob_hit", octomapProbHit);
	GET_PARAMETER_ERROR("octomap/prob_miss", octomapProbMiss);
	GET_PARAMETER_ERROR("octomap/clamping_threshold_min", octomapClampingThresholdMin);
	GET_PARAMETER_ERROR("octomap/clamping_threshold_max", octomapClampingThresholdMax);

	//topics
	GET_PARAMETER_ERROR("topics/camera_depth", topicCameraDepth);
	GET_PARAMETER_ERROR("topics/camera_depth_info", topicCameraDepthInfo);
	GET_PARAMETER_ERROR("topics/camera_prefix", topicCameraPrefix);

	if (!loadRobotParameters() || !loadProblemDefinitions() || !loadSampleLimits())
	{
		LOG_FATAL("Error while loading robot or problem definitions");
		return false;
	}

	s_parametersLoaded = true;

	return true;
}

bool ParameterServer::loadRobotParameters()
{
	ros::NodeHandle n("robots");

	std::vector<std::string> robots, arms, handJointNames;
	std::string description, startLink, endLink, tfPrefix, collisionMatrix, controllerConfig;
	std::string handName, handJointStateTopic, handTopic, handInterface, handClass;
	double handGraspPreDistance, handGraspRadius, handDropPreDistance, handGraspPostHeight;
	std::string followJointTrajectoryTopic, jointStateTopic, interfacePackage, interfaceClass;
	std::string kinematicsPackage, kinematicsClass;
	double graspingMinHeight;
	bool useToolFrame, useHWInterface;

	//prm parameters
	std::string prmFile, singleArmPlanningFrame, singleArmConstraint, plannerType;
	double prmVisDistance;
	int prmSize;
	bool prmSave;

	//get names
	if (!n.getParam("names", robots))
		LOG_FATAL("Please provide the robot names on names parameter");

	for (auto& it : robots)
	{
		GET_PARAMETER_ERROR("robots/" + it + "/arm_names", arms);

		RobotConfig& rc = robotConfigs[it];
		rc.name = it;

		for (auto& it2 : arms)
		{
			ArmConfig& arm = rc.arms[it2];

			GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/description", description);
			GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/start_link", startLink);
			GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/end_link", endLink);
			GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/controller_config", controllerConfig);

			GET_PARAMETER_DEFAULT("/" + it2 + "/collision_matrix", collisionMatrix, "");
			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/tool_frame", useToolFrame, false);
			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/tf_prefix", tfPrefix, "");

			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/kinematics_package", kinematicsPackage, "prm_kinematics_kdl");
			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/kinematics_class", kinematicsClass, "prm_kinematics_kdl::KDLKinematics");

			//follow joint trajectory action client
			if (executionMode == parameters::FollowJointTrajectoryPublisher)
			{
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/follow_joint_trajectory_topic", followJointTrajectoryTopic);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/joint_state_topic", jointStateTopic);
			}
			//use hw interface class which needs to be provided as a ros plugin
			else if (executionMode == parameters::HardwareInterface)
			{
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/interface_package", interfacePackage);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/interface_class", interfaceClass);
			}

			//hand
			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/hand/name", handName, "");
			if (!handName.empty())
			{
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/joint_state_topic", handJointStateTopic);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/joint_names", handJointNames);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/min_grasping_height", graspingMinHeight);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/topic", handTopic);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/topic", handTopic);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/interface_package", handInterface);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/interface_class", handClass);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/grasp_pre_distance", handGraspPreDistance);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/grasp_radius", handGraspRadius);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/hand/drop_pre_distance", handDropPreDistance);
				GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/hand/grasp_post_height", handGraspPostHeight, 0.08);
			}

			//single arm planner, if planner type is defined we load the other config
			GET_PARAMETER_DEFAULT("robots/" + it + "/arms/" + it2 + "/single_arm/planner_type", plannerType, "");
			if (!plannerType.empty())
			{
				if (!loadPlannerConfigs(arm.singleArmPRMConfig,
						arm.singleArmRRTConfig,
						arm.singleArmDatabaseConfig,
						n,
						"robots/" + it + "/arms/" + it2 + "/single_arm",
						plannerType))
				{
					return false;
				}

				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/single_arm/planning_frame", singleArmPlanningFrame);
				GET_PARAMETER_ERROR("robots/" + it + "/arms/" + it2 + "/single_arm/constraint", singleArmConstraint);
			}

			RobotArmConfig& armBase = arm.arm;
			armBase.name = it2;
			armBase.rootLink = startLink;
			armBase.tipLink = endLink;
			armBase.useToolFrame = useToolFrame;
			armBase.robotDescriptionParam = description;
			armBase.tfPrefix = tfPrefix;
			armBase.executionInterface = (ArmExecutionMode) executionMode;
			armBase.collisionMatrixFile = collisionMatrix;
			armBase.controllerConfig = controllerConfig;
			armBase.followJointTrajectoryTopic = followJointTrajectoryTopic;
			armBase.jointStateTopic = jointStateTopic;
			armBase.interfacePackage = interfacePackage;
			armBase.interfaceClass = interfaceClass;
			armBase.kinematicsPackage = kinematicsPackage;
			armBase.kinematicsClass = kinematicsClass;

			arm.singleArmPlanningFrame = singleArmPlanningFrame;
			arm.singleArmConstraint = singleArmConstraint;
			arm.hasSingleArmPlanner = !plannerType.empty();
			arm.singleArmPlannerType = plannerType;

			//write gripper config
			if (!handName.empty() && !handJointStateTopic.empty())
			{
				arm.hand.name = handName;
				arm.hand.jointStateTopic = handJointStateTopic;
				arm.hand.jointNames = handJointNames;
				arm.hand.minHeight = graspingMinHeight;
				arm.hand.topic = handTopic;
				arm.hand.interfacePackage = handInterface;
				arm.hand.interfaceClass = handClass;
				arm.hand.graspPreDistance = handGraspPreDistance;
				arm.hand.graspRadius = handGraspRadius;
				arm.hand.dropPreDistance = handDropPreDistance;
				arm.hand.graspPostHeight = handGraspPostHeight;
				arm.hasHand = true;
			}
			else
			{
				arm.hasHand = false;
			}

			//write prm config
			if (!prmFile.empty())
			{
				arm.singleArmPRMConfig.filename = replaceFindPackage(prmFile);
				arm.singleArmPRMConfig.save = prmSave;
				arm.singleArmPRMConfig.size = prmSize;
				arm.singleArmPRMConfig.visibilityDistance = prmVisDistance;
			}

			if (std::find(armNames.begin(), armNames.end(), armBase.name) == armNames.end())
			{
				armNames.push_back(armBase.name);
			}

			//load other robot specific configs
			if (!CHECK_MAP(controllerConfigs, armBase.name))
			{
				if (!loadControllerConfig(armBase.controllerConfig))
				{
					return false;
				}
			}
		}
	}

	return true;
}

bool ParameterServer::loadProblemDefinitions()
{
	ros::NodeHandle n("problem_definitions");

	std::vector<std::string> problems;
	std::vector<std::string> objects;
	std::string robot, constraint, frame, rootFrame, objectFileName, objectFrameName;
	std::string pluginPackage, pluginClass;
	std::string plannerType;
	Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
	bool upFound = false;

	//get names
	GET_PARAMETER_ERROR("names", problems);

	//load configs for all known problem definitions
	for (auto& it : problems)
	{
		ros::NodeHandle n("problem_definitions/problems/" + it); //has the same name as in outer scope!

		ProblemConfig& pc = problemConfigs[it];

		//ROBOT//////////////////////////////////////////////////////////////////
		GET_PARAMETER_ERROR("robot", robot);
		GET_PARAMETER_ERROR("root_frame", rootFrame);

		//PLUGIN/////////////////////////////////////////////////////////////////
		GET_PARAMETER_ERROR("plugin_package", pluginPackage);
		GET_PARAMETER_ERROR("plugin_class", pluginClass);

		//PLANNER////////////////////////////////////////////////////////////////
		GET_PARAMETER_ERROR("planning/constraint", constraint);
		GET_PARAMETER_ERROR("planning/planning_frame", frame);
		GET_PARAMETER_ERROR("planning/planner_type", plannerType);

		//UP DIR/////////////////////////////////////////////////////////////////
		IF_GET_PARAMETER("up/x", up.x())
				{
			GET_PARAMETER_ERROR("up/y", up.y());
			GET_PARAMETER_ERROR("up/z", up.z());
			upFound = true;
		}

		//load planner config
		if (!loadPlannerConfigs(pc.prmConfig, pc.rrtConfig, pc.databaseCacheConfig, n, "planning", plannerType))
			return false;

		//load dropping config
		loadDroppingConfig(it, pc.droppingConfig);

		//load object config
		IF_GET_PARAMETER("objects/names", objects)
				{
			for (auto& object : objects)
			{
				GET_PARAMETER_ERROR("objects/objects/" + object + "/model", objectFileName);
				GET_PARAMETER_ERROR("objects/objects/" + object + "/frame_name", objectFrameName);

				ObjectConfig oc;
				oc.file = replaceFindPackage(objectFileName);
				oc.frameName = objectFrameName;
				oc.name = object;
				pc.objects[object] = oc;
			}
		}

		pc.name = it;
		pc.robotConfig = robot;
		pc.constraint = constraint;
		pc.planningFrame = frame;
		pc.rootFrame = rootFrame;
		pc.pluginPackage = pluginPackage;
		pc.pluginClass = pluginClass;
		pc.upDir = up;
		pc.plannerType = plannerType;
		pc.isSingleArmProblemDefinition = false;
	}

	return true;
}

bool ParameterServer::loadSampleLimits()
{
	ros::NodeHandle n("robots/joint_limits");
	std::string min, max;
	double dmin, dmax;
	std::vector<std::string> jointNames;
	static const std::string pi = "3.14159265359";

	for (auto& it : armNames)
	{
		IF_NOT_GET_PARAMETER(it + "/joints", jointNames)
			continue;

		for (auto& joint : jointNames)
		{
			//first try string because of pi replacement
			IF_NOT_GET_PARAMETER(it + "/" + joint + "/sample_limit_min", min)
			{
				//test double

			}
			else
			{
				boost::algorithm::replace_first(min, "pi", pi);
				dmin = std::stod(min);
			}

			//first try string because of pi replacement
			IF_NOT_GET_PARAMETER(it + "/" + joint + "/sample_limit_max", max)
			{
				//test double
				GET_PARAMETER_ERROR(it + "/" + joint + "/sample_limit_max", dmax);
			}
			else
			{
				boost::algorithm::replace_first(max, "pi", pi);
				dmax = std::stod(max);
			}

			sampleLimits[it][joint].min = dmin;
			sampleLimits[it][joint].max = dmax;
		}
	}

	for (auto& it3 : robotConfigs) //robot
	{
		for (auto& it : sampleLimits) //arms
		{
			if (CHECK_MAP(it3.second.arms, it.first)) //if arm contains to robot
			{
				it3.second.arms[it.first].arm.sampleLimits = it.second;
			}
		}
	}

	return true;
}

bool ParameterServer::loadPlannerConfigs(PRMConfig& prm,
		RRTConfig& rrt,
		DatabaseCacheConfig& dbCache,
		ros::NodeHandle& n,
		const std::string& configNamespace,
		const std::string& planner)
{
	//PRM////////////////////////////////////////////////////////////////
	if (planner == "prm_a_star")
	{
		std::string file;

		GET_PARAMETER_ERROR(configNamespace + "/prm/size", prm.size);
		GET_PARAMETER_ERROR(configNamespace + "/prm/visibility_distance", prm.visibilityDistance);
		GET_PARAMETER_ERROR(configNamespace + "/prm/filename", file);
		GET_PARAMETER_DEFAULT(configNamespace + "/prm/save", prm.save, true);
		GET_PARAMETER_DEFAULT(configNamespace + "/prm/min_height", prm.minHeight, -100.0);
		prm.filename = replaceFindPackage(file);
	}

	//RRT////////////////////////////////////////////////////////////////
	else if (planner == "rrt")
	{
		GET_PARAMETER_ERROR(configNamespace + "/rrt/sample_goal", rrt.sampleGoal);
		GET_PARAMETER_ERROR(configNamespace + "/rrt/max_expansion_distance", rrt.maxExpansionDistance);
	}

	//Database Cache/////////////////////////////////////////////////////
	else if (planner == "database_cache")
	{
		GET_PARAMETER_ERROR(configNamespace + "/database_cache/filename", dbCache.filename);
	}

	return true;
}

bool ParameterServer::loadControllerConfig(const std::string& configName)
{
	ros::NodeHandle n("robots/controller_configs/" + configName);

	ControllerConfig& executionControllerParams = controllerConfigs[configName];

	//read execution controller
	GET_PARAMETER_ERROR("name", executionControllerParams.controllerType);
	GET_PARAMETER_ERROR("frequency", executionControllerParams.frequency);
	GET_PARAMETER_ERROR("max_velocity", executionControllerParams.maxVelocity);
	GET_PARAMETER_ERROR("k", executionControllerParams.k);
	GET_PARAMETER_ERROR("lambda", executionControllerParams.lambda);
	GET_PARAMETER_ERROR("least_square_lambda_max", executionControllerParams.leastSquareLambdaMax);
	GET_PARAMETER_ERROR("least_square_epsilon", executionControllerParams.leastSquareEpsilon);
	GET_PARAMETER_ERROR("debug", executionControllerParams.debug);
	GET_PARAMETER_ERROR("collision_avoidance/use", executionControllerParams.collisionAvoidanceUse);
	GET_PARAMETER_ERROR("collision_avoidance/vel_0", executionControllerParams.collisionAvoidanceVel0);
	GET_PARAMETER_ERROR("collision_avoidance/distance_1", executionControllerParams.collisionAvoidanceDistance1);
	GET_PARAMETER_ERROR("collision_avoidance/distance_2", executionControllerParams.collisionAvoidanceDistance2);
	GET_PARAMETER_ERROR("collision_avoidance/max_vel", executionControllerParams.collisionAvoidanceMaxJointVel);
	GET_PARAMETER_ERROR("collision_avoidance/ratio_do_collision_checks",
			executionControllerParams.collisionAvoidanceRatioDoCollisionChecks);
	GET_PARAMETER_ERROR("goal_dist_pos", executionControllerParams.goalReachedDistPos);
	GET_PARAMETER_ERROR("goal_dist_ang", executionControllerParams.goalReachedDistAng);
	GET_PARAMETER_ERROR("max_task_vel/linear", executionControllerParams.maxTaskVelPos);
	GET_PARAMETER_ERROR("max_task_vel/angular", executionControllerParams.maxTaskVelAng);
	GET_PARAMETER_ERROR("joint_range_weight/default", executionControllerParams.jointRangeWeightDefault);
	GET_PARAMETER_ERROR("joint_range_weight/std", executionControllerParams.jointRangeWeightStd);

	return true;
}

void ParameterServer::loadDroppingConfig(const std::string& problem,
		parameters::DroppingConfig& config)
{
	ros::NodeHandle n("problem_definitions/problems/" + problem + "/dropping");

	GET_PARAMETER_DEFAULT("ransac_treshold", config.ransacTreshold, 0.01);
	GET_PARAMETER_DEFAULT("max_plane_angle_to_up", config.maxPlaneAngleToUp, 0.2);
	GET_PARAMETER_DEFAULT("ransac_min_points", config.ransacMinPoints, 100);
	GET_PARAMETER_DEFAULT("search_radius", config.searchRadius, 0.6);
	GET_PARAMETER_DEFAULT("region/resolution", config.regionResolution, 0.05);
	GET_PARAMETER_DEFAULT("region/margin", config.regionMargin, 0.05);
	GET_PARAMETER_DEFAULT("region/potential_treated_as_safe", config.regionPotentialTreatedAsSafe, 0.5);
	GET_PARAMETER_DEFAULT("region/smoothing_window", config.regionSmoothingWindow, 2);
	GET_PARAMETER_DEFAULT("region/max_number_of_points", config.regionMaxNumberOfPoints, 10);
	GET_PARAMETER_DEFAULT("region/object_growing_factor", config.regionObjectGrowingFactor, 3);
	GET_PARAMETER_DEFAULT("region/tries", config.tries, 100);
	GET_PARAMETER_DEFAULT("region/number_of_cores", config.numberOfCores, 8);
}

std::string ParameterServer::replaceFindPackage(const std::string& path)
{
	static const boost::regex findRegex("(\\$\\{find\\s*?(.*?)\\}).*");
	std::string pathOut = path;

	boost::smatch what;
	if (boost::regex_match(path, what, findRegex))
	{
		std::string absolutPath = ros::package::getPath(what[2].str());
		if (absolutPath.empty())
		{
			LOG_FATAL("Unknown ros package: " << what[2].str());
			exit(-1);
		}

		boost::algorithm::replace_first(pathOut, what[1].str(), absolutPath);
	}

	return pathOut;
}

bool ParameterServer::receiveImage()
{
	boost::mutex::scoped_lock lock2(s_parameterMutex);
	return m_receiveImage;
}

void ParameterServer::setReceiveImage()
{
	boost::mutex::scoped_lock lock2(s_parameterMutex);
	m_receiveImage = true;
	m_imageReceived = false;
}

bool ParameterServer::hasImageReceived()
{
	boost::mutex::scoped_lock lock2(s_parameterMutex);
	return m_imageReceived;
}

void ParameterServer::setImageReceived()
{
	boost::mutex::scoped_lock lock2(s_parameterMutex);

	m_imageReceived = true;

	if (oneImageMode != parameters::Frequently)
		m_receiveImage = false;
}

} /* namespace prm_planner */

