/*
 * defines.h
 *
 *  Created on: Aug 18, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_DEFINES_H_
#define PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_DEFINES_H_

#include <boost/shared_ptr.hpp>
#include <kdl/jntarray.hpp>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <unordered_map>
#include <vector>

namespace prm_planner
{

struct RobotJointSampleLimits
{
	double min;
	double max;
};

enum ControlMode
{
	VelocityControl,
	TorqueControl
};

enum ArmExecutionMode {
	//no execution is available
	NoExecution,

	//uses a provided hardware interface defined
	//in a library (using the corresponding parameters)
	HardwareInterface,

	//uses a FollowJointTrajectory action client
	//to publish the trajectory
	FollowJointTrajectoryPublisher
};

/*
 * tfPrefix can be empty, if there is no tf prefix
 * sampleLimits are used to specify a range for joints without joint limits
 * toolFrameTransformation
 */
struct RobotArmConfig
{
	std::string name;
	std::string rootLink;
	std::string tipLink;
	bool useToolFrame;
	ArmExecutionMode executionInterface;
	std::string robotDescriptionParam;
	std::string tfPrefix;
	std::string collisionMatrixFile;
	std::string followJointTrajectoryTopic;
	std::string jointStateTopic;
	std::string interfacePackage;
	std::string interfaceClass;
	std::string controllerConfig;
	std::unordered_map<std::string, RobotJointSampleLimits> sampleLimits;
	ControlMode controlMode;
	std::string kinematicsPackage;
	std::string kinematicsClass;
};

struct TrajectoryWaypoint
{
	KDL::JntArray positions;
	Eigen::Vector3d taskPosition;
	Eigen::Quaterniond taskOrientation;
	ros::Time timeFromStart;
	double velocityLin, velocityAng;			//velocities between 0.0 and 1.0
};

class RobotArm;

typedef std::vector<TrajectoryWaypoint> ArmJointPath;

}

#endif /* PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_DEFINES_H_ */
