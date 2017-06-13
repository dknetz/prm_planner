/*
 * Copyright (c) 2016 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Dec 20, 2016
 *      Author: kuhnerd
 * 	  Filename: parameters.h
 */

#ifndef HDCC9FA69_530A_422B_B238_C60595E0BB3E
#define HDCC9FA69_530A_422B_B238_C60595E0BB3E
#include <prm_planner_robot/defines.h>
#include <Eigen/Core>
#include <string>
#include <unordered_map>

#define DEPRECATED

namespace prm_planner
{
namespace parameters
{

enum ImageMode {
	OneImage,
	OneImagePerPlan,
	Frequently
};

enum ExecutionMode {
	//no execution is available
	NoExecution,

	//uses a provided hardware interface defined
	//in a library (using the corresponding parameters)
	HardwareInterface,

	//uses a FollowJointTrajectory action client
	//to publish the trajectory
	FollowJointTrajectoryPublisher
};

//Config for PRMA* planner
struct PRMConfig
{
	std::string filename;
	bool save;
	int size;
	double visibilityDistance;

	//minimal height of the lowest waypoint
	double minHeight;
};

//Config for RRT planner
struct RRTConfig
{
	//probability of sampling the goal state as next
	//node. Needs to be between 0 and 1.
	//@default: 0.2
	double sampleGoal;

	//the maximum distance a newly sampled node may
	//be away from the current graph.
	//default: 0.1
	double maxExpansionDistance;
};

//Config for Database Cache planner
struct DatabaseCacheConfig
{
	//the file, which contains the database
	//@default: no default, must be provided
	std::string filename;
};

struct HandConfig
{
	enum GraspingAxis
	{
		X,
		Y,
		Z
	};

	std::string name;
	std::string jointStateTopic;
	std::vector<std::string> jointNames;
	GraspingAxis graspingAxis;
	double minHeight;
	std::string topic;
	std::string interfacePackage;
	std::string interfaceClass;

	//the distance to the object of the pre-position
	double graspPreDistance;

	//the distance between the object center and the eef
	//point of the gripper
	double graspRadius;

	//the distance between pre-position and desired object
	//position when dropping an object
	double dropPreDistance;

	//the height after grasping
	double graspPostHeight;
};

struct ArmConfig
{
	//the config which is related to the arm itself
	RobotArmConfig arm;

	//if the robot has a hand, this attribute is true
	bool hasHand;

	//gripper related config
	HandConfig hand;

	//if the arm has a seperate planner, this must be true.
	//can be used in a multi robot scenario, where only one
	//robot should be controlled, or in a scenario where
	//the main problem have special constraints an you just
	//want to move the arm with some default constraint.
	bool hasSingleArmPlanner;

	//the planner type
	std::string singleArmPlannerType;

	//if you use a planner which requires a PRM you have to
	//specify the parameters here.
	PRMConfig singleArmPRMConfig;
	RRTConfig singleArmRRTConfig;
	DatabaseCacheConfig singleArmDatabaseConfig;

	//single arm planning frame
	std::string singleArmPlanningFrame;

	//single arm constraint
	std::string singleArmConstraint;

	//kinematics plugin
	std::string kinematicsPluginPackage;
	std::string kinematicsPluginClass;
};

struct RobotConfig
{
	std::string name;
	std::unordered_map<std::string, ArmConfig> arms;
};

struct ObjectConfig
{
	std::string name;
	std::string file;
	std::string frameName;
};

/**
 * Dropping Parameters
 */
struct DroppingConfig
{
	//the distance in which a point is considered to be in a plane
	double ransacTreshold;

	//the minimum number of points which must be part of the plane
	double ransacMinPoints;

	//the max angle between the up vector and the normal of the
	//(hoizontal) plane [rad].
	double maxPlaneAngleToUp;

	//the radius around the tip frame in which a drop
	//position should be searched for
	double searchRadius;

	//the resolution of the cells in a dropping region in meter
	double regionResolution;

	//the margin at borders, which should considered as
	//unsafe
	double regionMargin; //0.05

	//the potential of a cell needs to be higher than
	//the given value to be seen as a safe dropping position
	double regionPotentialTreatedAsSafe; //0.5

	//the number of cells in each direction of the
	//cell, for which the smoothed value is computed
	int regionSmoothingWindow; //2

	//the maximum number of points which will be added to
	//a cell. This is used to reduce the effect of lower
	//density in regions that are further away from the
	//camera.
	int regionMaxNumberOfPoints; //10

	//the number of cells around a given cell, which should
	//get a 0 potential too (if there is an object, there should
	//be a hole in the point cloud which leads to 0 potential cells).
	//A region growing algorithm increases these areas by the given
	//number of cells in all directions.
	int regionObjectGrowingFactor; //3

	//the number of tries used to find the optimal dropping pose
	int tries; //100

	//the number of CPU cores used to find a dropping configuration
	int numberOfCores; //8
};

/**
 * Parameters which are needed for the problem
 * definition
 */
struct ProblemConfig
{
	//the name of the problem, used for identification and visualization
	std::string name;

	//plugin package and class, used for loading the
	//problem definition of the problem via pluginlib.
	//use prm_planner and prm_planner::SingleArmProblemDefinition
	//for a default problem definition in a single arm
	//setup without special constraints
	std::string pluginPackage;
	std::string pluginClass;

	//the name of the robot config
	//@see RobotConfig
	std::string robotConfig;

	//the name of the constraint
	std::string constraint;

	//the planning frame (e.g., base_link of the robot)
	std::string planningFrame;

	//the root frame (e.g., map)
	std::string rootFrame;

	//planner type
	std::string plannerType;

	//the up direction, default: [0,0,1]
	Eigen::Vector3d upDir;

	//the propabilistic roadmap configuration
	//@see PRMConfig
	PRMConfig prmConfig;

	//rrt config
	RRTConfig rrtConfig;

	//database cache config
	DatabaseCacheConfig databaseCacheConfig;

	//dropping config
	DroppingConfig droppingConfig;

	//a list of objects involved in the problem
	//@see ObjectConfig
	std::unordered_map<std::string, ObjectConfig> objects;

	//Must be true for single arm planners. Otherwise
	//there will be a recursion while constructing the
	//single arm planners! This variable is used internally,
	//just initialize it with false. The planner will set
	//the value to true if necessary.
	bool isSingleArmProblemDefinition;
};

struct ControllerConfig
{
	std::string controllerType;
	int frequency;
	double maxVelocity;
	std::vector<double> k;
	double lambda;
	double leastSquareLambdaMax;
	double leastSquareEpsilon;
	bool debug;
	bool collisionAvoidanceUse;
	double collisionAvoidanceVel0;
	double collisionAvoidanceDistance1;
	double collisionAvoidanceDistance2;
	double collisionAvoidanceMaxJointVel;
	double collisionAvoidanceRatioDoCollisionChecks;
	double goalReachedDistPos;
	double goalReachedDistAng;
	double maxTaskVelPos;
	double maxTaskVelAng;

	//is used to compute joint range nullspace weight.
	//default is used as a weight, if there is no approaching joint range problem.
	//a normal distribution is used to get higher values if the joints
	//move into direction of joint limits. std is thus the standard deviation used
	//in the normal distribution (@see velocity_controller_7_dof)
	double jointRangeWeightDefault;
	double jointRangeWeightStd;
};
}
}

#endif /* HDCC9FA69_530A_422B_B238_C60595E0BB3E */
