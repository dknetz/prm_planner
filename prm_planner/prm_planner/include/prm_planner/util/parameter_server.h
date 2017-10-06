/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: May 19, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: parameter_server.h
 */

#ifndef PARAMETER_SERVER_H_
#define PARAMETER_SERVER_H_
#include <boost/thread/mutex.hpp>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameters.h>
#include <prm_planner_robot/defines.h>

#define VERB ParameterServer::verbose

namespace prm_planner
{

class ParameterServer
{
public:
	ParameterServer();
	virtual ~ParameterServer();

	static bool loadParameters();
	static std::string replaceFindPackage(const std::string& path);

	/**
	 * If your image mode is not 'frequently' you can call this method to
	 * get a new image
	 */
	static bool receiveImage();
	static void setReceiveImage();

	/**
	 * Use this method to check, if a new image has been received
	 * since last receiveImage() call
	 */
	static bool hasImageReceived();
	static void setImageReceived();

	//=======================================
	//ROS parameters
	//=======================================
	static bool verbose;				//verbose output if true
	static bool useShortcuts;			//use shortcuts (no planning, directly controlling)
	static bool visualize;				//use visualization
	static bool startPublishers;		//needs to be true (default) to start publishers
	static bool startSubscribers;		//needs to be true (default) to start subscribers
	static bool startRosSpinner;		//do we need a spinner or not
	static std::string problem;			//defines the problem, must be defined in e.g., problems.yaml
	static bool connectedToVRep;		//used to mirror the depth images of V-Rep (don't know why this is necessary)
	static bool useCollisionDetection;	//activate/deactivate collision checks
	static bool useTrajectoryOptimization;	//use/don't use trajectory optimization
	static bool usePathDatabase;		//use path database (if true, trajectory optimization is used either way)
	static bool debugInternal;			//only for internal use!
	static parameters::ImageMode oneImageMode;		//if you only need one image you can activate it by setting this variable to true (static world)

	//planning
	static double maxPlanningTime;		//max time to wait for plan

	//execution
	static parameters::ExecutionMode executionMode;

	//octomap
	static double octomapResolution;
	static double octomapProbHit;
	static double octomapProbMiss;
	static double octomapClampingThresholdMin;
	static double octomapClampingThresholdMax;

	//topics
	static std::string topicCameraDepth;
	static std::string topicCameraDepthInfo;
	static std::string topicCameraPrefix;

	//problem configs
	static std::unordered_map<std::string, parameters::RobotConfig> robotConfigs;
	static std::unordered_map<std::string, parameters::ProblemConfig> problemConfigs;
	static std::unordered_map<std::string, parameters::ControllerConfig> controllerConfigs;

private:
	static std::vector<std::string> armNames;
	static std::unordered_map<std::string, std::unordered_map<std::string, RobotJointSampleLimits>> sampleLimits;
	static bool m_receiveImage;
	static bool m_imageReceived;

private:
	static bool loadRobotParameters();
	static bool loadProblemDefinitions();
	static bool loadSampleLimits();
	static bool loadControllerConfig(const std::string& configName);
	static bool loadPlannerConfigs(parameters::PRMConfig& prm,
			parameters::RRTConfig& rrt,
			parameters::DatabaseCacheConfig& dbCache,
			ros::NodeHandle& n,
			const std::string& configNamespace,
			const std::string& planner);
	static void loadDroppingConfig(const std::string& problem,
			parameters::DroppingConfig& config);

private:
	//parameter mutex
	static boost::mutex s_parameterMutex;
	static bool s_parametersLoaded;
};

} /* namespace prm_planner */

#endif /* PARAMETER_SERVER_H_ */
