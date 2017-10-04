/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 13, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: velocity_controller_n.h
 */

#ifndef HC1EF15B9_0C2F_403B_B619_9369E4A9240D
#define HC1EF15B9_0C2F_403B_B619_9369E4A9240D

#include <ais_definitions/class.h>
#include <ais_definitions/math.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <prm_planner_controller/controller.h>
#include <prm_planner_controller/controller_defines.h>
#include <prm_planner_robot/trajectory.h>
#include <Eigen/Core>
#include <vector>
#include <unordered_map>

FORWARD_DECLARE_N(ais_point_cloud, EasyKDTree);
FORWARD_DECLARE_N(urdf, JointLimits);

namespace prm_planner
{

FORWARD_DECLARE(Path);
FORWARD_DECLARE(Trajectory);
FORWARD_DECLARE(RobotArm);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(JacobianMultiSolver);
FORWARD_DECLARE(Kinematics);

template<int Dim, class Type = double>
class VelocityController: public Controller, public ControllerEigenDefines<Dim, Type>
{
CONTROLLER_USE_TYPEDEFS

public:
	VelocityController(const ControllerParameters& parameters,
			const boost::shared_ptr<Constraint> constraint,
			boost::shared_ptr<RobotArm> robotArm,
			const std::string& planningFrame,
			const Type octomapResolution = 0);
	virtual ~VelocityController();

	virtual bool updateFromPath(const boost::shared_ptr<Path> path);
	virtual void init();
	//	virtual void setKdTree(boost::shared_ptr<ais_point_cloud::EasyKDTree> kdtree);

	virtual void publish();

	virtual bool isGoalReached() const;
	virtual bool isSuccess() const;
	virtual bool isTrajectoryAvailable() const;

	virtual void lock();
	virtual void unlock();

	virtual void reset();

	virtual Type getPathLength();
	virtual Type getExecutionPathLength();

	virtual Type getPredictedExecutionTime() const;
	virtual void setPredictedExecutionTime(Type predictedExecutionTime);

	virtual bool update(const ros::Time& now,
			const ros::Duration& dt);

	virtual KDL::JntArray getJointStateFromRobot();

	virtual Trajectory::Pose getCurrentPose();
	virtual Trajectory::Pose getGoalPose();
	virtual Vector6d getDistToGoal();

	virtual bool writeData(const std::string& baseFileName);

protected:
	void initController();
	void initTime();
	void computeError();
	virtual void computePrediction();
	bool isValidCommand(const KDL::JntArray& cmd,
			const Type& dt,
			const Type& errorBound);
	void stopMotion();
	void limitFinalJointVelocities(KDL::JntArray& cmds);
	void limitJointVelocities(VectorNd& cmd,
			Type max);
//	void resetCollisionStrip(const KDL::JntArray& joints);

	void addNullSpaceMiddleJointRange();
	Type computeJointRangeWeight();

//	void addNullSpaceCollisionAvoidance(); //Kinematic Control Algorithms for On-Line Obstacle Avoidance for Redundant Manipulators
//	void updateObjectDistances();
//	bool checkCollision();
	virtual bool isGoalReached(Trajectory::Pose& current);

protected:
	Eigen::Vector3d m_posOld;
	//	std::vector<Matrix6xN> m_jacobians;
	Matrix6xN m_jacobian;
	KDL::Jacobian m_kdlJacobian;
	mutable boost::recursive_mutex m_mutex;
	bool m_printJointRangeWarning;
	bool m_success;
	//	JacobianMultiSolver* m_jacobianSolver;
//	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fkSolver;
	boost::shared_ptr<Kinematics> m_kinematics;
	Type m_executionLength;
	//	std::vector<KDL::JntArray> m_executedTrajectory;

	VectorNd m_qNullSpace; //Nx1
	VectorNd m_qNullSpaceJointRange; //Nx1
	VectorNd m_qNullSpaceCollisionAvoidance; //Nx1
	MatrixNxN m_N; //NxN
	MatrixNxN m_I; //NxN
	MatrixNx6 m_pseudoInverse; //Nx6

	Matrix6x6 m_k; //6x6
	VectorNd m_lambda; //Nx1

	//always the same, compute it only once
	VectorNd s_limitRangeCenters;
	VectorNd s_limitSquaredRange;
	VectorNd s_limitVelocities;
	VectorNd s_limitUpper;
	VectorNd s_limitLower;

	Eigen::Affine3d m_transformationPlanningFrameToArm;
	const std::string c_planningFrame;
	const std::string c_armFrame;
	const std::string c_armName;

	//execution of complete path
	std::vector<VectorNd> m_pathPoints;
	Trajectory::Pose m_pathOldPose;
	VectorNd m_pathOldPosition;
	Type m_pathOldTime;
	std::vector<VectorNd> m_pathVelocities;
	std::vector<Type> m_pathTimesFromStart;
	bool m_gotGoal;
	bool m_getFirstPathData;
	bool m_goalSend;
	boost::atomic_int m_currentWaypoint;

	//collision bubbles
//	std::vector<ControllerBubble> m_collisionStrip;

private:
//	static std::unordered_map<std::string, ros::Publisher> s_pubBubbles;
	static std::unordered_map<std::string, ros::Publisher> s_pubTrajectory;

public:
	const ControllerParameters c_parameters;
	const boost::shared_ptr<Constraint> c_constraint;
	const Type c_octomapResolution;

	Trajectory::Pose m_x, m_xOld, m_x0, m_xGoal, m_xPredicted, m_xPredictedOld;
	Vector6d m_error, m_xDesiredDot;
	ros::Time m_now, m_start;
	ros::Duration m_dt;
	int64_t m_counter;
	Type s_maxVelocityLimit;
	boost::shared_ptr<Path> m_path;
	boost::shared_ptr<Trajectory> m_trajectory;
};

} /* namespace prm_planner */

#endif /* HC1EF15B9_0C2F_403B_B619_9369E4A9240D */
