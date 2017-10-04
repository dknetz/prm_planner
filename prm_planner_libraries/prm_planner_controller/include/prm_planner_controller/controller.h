/*
 * velocity_controller.h
 *
 *  Created on: Aug 22, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_VELOCITY_CONTROLLER_H_
#define PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_VELOCITY_CONTROLLER_H_
#include <ais_log/log.h>
#include <ais_point_cloud/easy_kd_tree.h>
#include <boost/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <kdl/jntarray.hpp>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/robot_arm.h>
#include <ros/ros.h>
#include <Eigen/Geometry>

#define CONTROLLER_USE_TYPEDEFS_PARAM(Dim, Type) public:\
	using typename ControllerEigenDefines<Dim, Type>::VectorNd;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNx6;\
	using typename ControllerEigenDefines<Dim, Type>::Matrix6xN;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNxN;\
	using typename ControllerEigenDefines<Dim, Type>::Matrix3xN;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNx3;

#define CONTROLLER_USE_TYPEDEFS public:\
	using typename ControllerEigenDefines<Dim, Type>::VectorNd;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNx6;\
	using typename ControllerEigenDefines<Dim, Type>::Matrix6xN;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNxN;\
	using typename ControllerEigenDefines<Dim, Type>::Matrix3xN;\
	using typename ControllerEigenDefines<Dim, Type>::MatrixNx3;

namespace prm_planner
{

template<int Dim, class Type>
struct ControllerEigenDefines
{
	typedef Eigen::Matrix<Type, Dim, 1> VectorNd;
	typedef Eigen::Matrix<Type, Dim, 6> MatrixNx6;
	typedef Eigen::Matrix<Type, 6, Dim> Matrix6xN;
	typedef Eigen::Matrix<Type, Dim, Dim> MatrixNxN;
	typedef Eigen::Matrix<Type, 3, Dim> Matrix3xN;
	typedef Eigen::Matrix<Type, Dim, 3> MatrixNx3;
};

/**
 * The base class for all controllers. You need to
 * implement the missing methods which are defined
 * in this class.
 */
class Controller
{
public:
	/**
	 * @arm the robot arm
	 */
	Controller(boost::shared_ptr<RobotArm> arm);
	virtual ~Controller();

	/**
	 * Sets the path thats needs to be executed
	 * by the controller.
	 */
	virtual bool updateFromPath(const boost::shared_ptr<Path> path) = 0;

	/**
	 * You can implement this method in the child
	 * class, if you need to initialize the controller
	 * or anything you don't want to initialize in the
	 * constructor
	 */
	virtual void init();

//	/**
//	 * Can be implemented in case you need to get access
//	 * to the environmental data from the cameras. You get
//	 * a KD tree which you can for example you to do
//	 * online collision avoidance
//	 */
//	virtual void setKdTree(boost::shared_ptr<ais_point_cloud::EasyKDTree> kdtree);

	/**
	 * This method will be called in the body of
	 * the visualization loop. You can use it to
	 * publish visualization messages via ROS, e.g.,
	 * a marker
	 */
	virtual void publish();

	/**
	 * Implement this method to return the state
	 * of the current path execution. If the current
	 * goal is reached, this method should return
	 * true, otherwise false.
	 */
	virtual bool isGoalReached() const = 0;

	/**
	 * This method returns true, if the controller
	 * isn't in a error state. Otherwise it returns
	 * false.
	 */
	virtual bool isSuccess() const = 0;

	/**
	 * Returns true, if a path was set with updateFromPath
	 * and the execution is still ongoing. After reaching
	 * the goal, this method needs to return false
	 */
	virtual bool isTrajectoryAvailable() const = 0;

	/**
	 * Locks/Unlocks the internal mutex. Don't
	 * forget to unlock the mutex, if you lock it.
	 */
	virtual void lock();
	virtual void unlock();

	/**
	 * Resets the complete controller to some
	 * initial state.
	 */
	virtual void reset() = 0;

	/**
	 * Returns the path length of the computed
	 * end effector trajectory and the path length
	 * after execution.
	 */
	virtual double getPathLength() = 0;
	virtual double getExecutionPathLength() = 0;

	/**
	 * Set/Get the predicted execution time.
	 */
	virtual double getPredictedExecutionTime() const = 0;
	virtual void setPredictedExecutionTime(double predictedExecutionTime) = 0;

	/**
	 * You need to implement this method, that
	 * actually does the work of the controller.
	 * Here you need to send the commands to
	 * and receive the data from the robot. You
	 * also need to compute the motion commands
	 * in this method.
	 * @now: 	the current timestep
	 * @dt:  	time difference between current and last call
	 */
	virtual bool update(const ros::Time& now,
			const ros::Duration& dt) = 0;

	/**
	 * Returns the current (world) time
	 */
	static void getTime(ros::Time& now);

	/**
	 * Returns the robot state
	 */
	virtual KDL::JntArray getCurrentJointPosition();

	/**
	 * Returns the robot state which can be
	 * used for FCL.
	 */
	virtual fcl_robot_model::RobotState getCurrentRobotState();

	/**
	 * Sets the start and goal pose for the controller. Use
	 * this method only in the simulation controllers. We had
	 * to define it here because it was not possible to define it
	 * in one of the sub-classes because of the templates (initialization
	 * of the controller template with unknown dimension is not possible)
	 */
	virtual bool updateFromValues(const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goalPose,
			const KDL::JntArray& startJoints);
	virtual const ArmJointPath& getJointPositions() const;

	/**
	 * Can be used to write data to file. baseFileName is the
	 * filename and path without extension (writing multiple
	 * files is easier, if one just needs to add an extension).
	 */
	virtual bool writeData(const std::string& baseFileName);

protected:
	KDL::JntArray m_q;
	std::vector<std::string> m_jointNames;
	fcl_robot_model::RobotState m_robotState;
	boost::shared_ptr<RobotArm> m_robotArm;
};

} /* namespace prm_planner */

#endif /* PRM_PLANNER_LIBRARIES_PRM_PLANNER_CONTROLLER_INCLUDE_PRM_PLANNER_CONTROLLER_VELOCITY_CONTROLLER_H_ */
