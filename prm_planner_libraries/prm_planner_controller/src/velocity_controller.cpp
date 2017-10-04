/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 13, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: velocity_controller_n.cpp
 */

#include <ais_definitions/exception.h>
#include <ais_log/log.h>
#include <ais_util/color.h>
#include <ais_util/math.h>
#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner_constraints/constraint.h>
#include <prm_planner_controller/jacobian_multi_solver.h>

#include <ais_ros/ros_base_interface.h>
#include <ais_point_cloud/easy_kd_tree.h>
#include <prm_planner_controller/velocity_controller.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner_robot/kinematics.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <limits>
#include <urdf_model/joint.h>

//#include <ais_log/data_logger.h>

//DATA_LOGGER_INIT();

namespace prm_planner
{

//template<int Dim, class Type>
//std::unordered_map<std::string, ros::Publisher> VelocityController<Dim, Type>::s_pubBubbles;

template<int Dim, class Type>
std::unordered_map<std::string, ros::Publisher> VelocityController<Dim, Type>::s_pubTrajectory;

template<int Dim, class Type>
VelocityController<Dim, Type>::VelocityController(const ControllerParameters& parameters,
		const boost::shared_ptr<Constraint> constraint,
		boost::shared_ptr<RobotArm> robotArm,
		const std::string& planningFrame,
		const Type octomapResolution) :
				Controller(robotArm),
				c_parameters(parameters),
				c_constraint(constraint),
				m_counter(0),
				m_success(true),
				m_printJointRangeWarning(true),
//				m_jacobianSolver(NULL),
				m_executionLength(0),
				m_xDesiredDot(Vector6d::Zero()),
				c_planningFrame(planningFrame),
				c_armFrame(m_robotArm->getRootFrame()),
				c_octomapResolution(octomapResolution),
				c_armName(robotArm->getName()),
				m_gotGoal(false),
				m_goalSend(false),
				m_getFirstPathData(false),
				m_currentWaypoint(0),
				m_kdlJacobian(Dim)
//				m_fkSolver(m_robotArm->getNewFkSolverInstance())
{
	initTime();
//	if (parameters.debug)
//		DATA_LOGGER_ADD("velocity", "/tmp/velocity.data");
}

template<int Dim, class Type>
VelocityController<Dim, Type>::~VelocityController()
{
//	DELETE_VAR(m_jacobianSolver);
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::updateFromPath(const boost::shared_ptr<Path> path)
{
	m_mutex.lock();
	Vector6d xDesiredDot = m_xDesiredDot;
	m_printJointRangeWarning = true;
	ros::Time now = m_now;
	m_executionLength = 0;
	m_posOld = m_x.x.head(3);
	m_pathVelocities.clear();
	m_pathPoints.clear();
	m_start = m_now;
	m_mutex.unlock();

	m_success = true;

	boost::shared_ptr<Trajectory> trajectory(
			new Trajectory(path, xDesiredDot, c_parameters.maxTaskVelPos, c_parameters.maxTaskVelAng, now, m_robotArm->getRootFrame()));
	if (trajectory->isValid())
	{
		//		trajectory->writeGnuplotFile("/tmp/trajectory.txt");
		boost::recursive_mutex::scoped_lock lock(m_mutex);
		m_path = path;
		m_xPredictedOld = m_xPredicted;
		m_xPredicted = m_x;
		m_trajectory = trajectory;
		m_xGoal = Trajectory::Pose(path->back().pose);
		m_gotGoal = true;
		m_goalSend = false;
		m_getFirstPathData = true;
		return true;
	}
	else
	{
		boost::recursive_mutex::scoped_lock lock(m_mutex);
		m_success = false;
		LOG_ERROR("trajectory is invalid (i.e. maxT of spline is nan)");
		return false;
	}
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::init()
{
	initController();

	//visualization
	ros::NodeHandle n;

//	if (!CHECK_MAP(s_pubBubbles, c_armName))
//	{
//		s_pubBubbles[c_armName] = n.advertise<visualization_msgs::MarkerArray>("collision_strip", 5);
//	}

	if (!CHECK_MAP(s_pubTrajectory, c_armName))
	{
		s_pubTrajectory[c_armName] = n.advertise<nav_msgs::Path>("trajectory/" + c_armName, 5);
	}
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::publish()
{
	static std_msgs::ColorRGBA red = ais_util::Color::red().toROSMsg();
	static std_msgs::ColorRGBA green = ais_util::Color::green().toROSMsg();

//	if (s_pubBubbles[c_armName].getNumSubscribers() > 0)
//	{
//		boost::recursive_mutex::scoped_try_lock lock(m_mutex);
//		if (lock.owns_lock())
//		{
//			visualization_msgs::MarkerArray markers;
//
//			int id = 0;
//
//			Type diameter;
//			for (auto& it : m_collisionStrip)
//			{
//				diameter = it.radius * 2.0;
//				{
//					visualization_msgs::Marker marker;
//					marker.header.frame_id = m_robotArm->getRootFrame();
//					marker.header.stamp = ros::Time();
//					marker.ns = "bubble";
//					marker.id = id++;
//					marker.type = visualization_msgs::Marker::SPHERE;
//					marker.action = visualization_msgs::Marker::ADD;
//					marker.scale.x = diameter;
//					marker.scale.y = diameter;
//					marker.scale.z = diameter;
//					marker.color = it.radius < c_parameters.collisionDetectionStopDistance ? red : green;
//					marker.color.a = 0.5;
//					marker.pose.position.x = it.position.x();
//					marker.pose.position.y = it.position.y();
//					marker.pose.position.z = it.position.z();
//					markers.markers.push_back(marker);
//				}
//				diameter = c_parameters.collisionDetectionStopDistance * 2;
//				{
//					visualization_msgs::Marker marker;
//					marker.header.frame_id = m_robotArm->getRootFrame();
//					marker.header.stamp = ros::Time();
//					marker.ns = "bubble_stop";
//					marker.id = id++;
//					marker.type = visualization_msgs::Marker::SPHERE;
//					marker.action = visualization_msgs::Marker::ADD;
//					marker.scale.x = diameter;
//					marker.scale.y = diameter;
//					marker.scale.z = diameter;
//					marker.color.a = 0.5;
//					marker.color.r = 1.0;
//					marker.color.g = 0.0;
//					marker.color.b = 0.0;
//					marker.pose.position.x = it.position.x();
//					marker.pose.position.y = it.position.y();
//					marker.pose.position.z = it.position.z();
//					markers.markers.push_back(marker);
//				}
//				{
//					visualization_msgs::Marker marker;
//					marker.header.frame_id = m_robotArm->getRootFrame();
//					marker.header.stamp = ros::Time();
//					marker.ns = "contact_point";
//					marker.id = id++;
//					marker.type = visualization_msgs::Marker::SPHERE;
//					marker.action = visualization_msgs::Marker::ADD;
//					marker.scale.x = 0.03;
//					marker.scale.y = 0.03;
//					marker.scale.z = 0.03;
//					marker.color = ais_util::Color::red().toROSMsg();
//					marker.pose.position.x = it.position.x() - it.obstacleToRobot.x();
//					marker.pose.position.y = it.position.y() - it.obstacleToRobot.y();
//					marker.pose.position.z = it.position.z() - it.obstacleToRobot.z();
//					markers.markers.push_back(marker);
//				}
//				{
//					visualization_msgs::Marker marker;
//					marker.header.frame_id = m_robotArm->getRootFrame();
//					marker.header.stamp = ros::Time();
//					marker.ns = "contact_point";
//					marker.id = id++;
//					marker.type = visualization_msgs::Marker::ARROW;
//					marker.action = visualization_msgs::Marker::ADD;
//					marker.scale.x = 0.005; //shaft diameter
//					marker.scale.y = 0.01; //head diameter
//					marker.scale.z = 0.02; //head length
//					marker.color = ais_util::Color::blue().toROSMsg();
//					marker.points.resize(2);
//					marker.points[0].x = it.position.x() - it.obstacleToRobot.x();
//					marker.points[0].y = it.position.y() - it.obstacleToRobot.y();
//					marker.points[0].z = it.position.z() - it.obstacleToRobot.z();
//					marker.points[1].x = it.position.x();
//					marker.points[1].y = it.position.y();
//					marker.points[1].z = it.position.z();
//					markers.markers.push_back(marker);
//				}
//			}
//
//			s_pubBubbles[c_armName].publish(markers);
//		}
//	}

	if (s_pubTrajectory[c_armName].getNumSubscribers() > 0)
	{
		boost::recursive_mutex::scoped_try_lock lock(m_mutex);
		if (lock.owns_lock() && m_trajectory.get() != NULL)
		{
			nav_msgs::PathConstPtr trajectory = m_trajectory->getRosTrajectory();
			if (trajectory.get() != NULL)
			{
				s_pubTrajectory[c_armName].publish(trajectory);
			}
		}
	}
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::isGoalReached() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return c_constraint->isGoalReached(m_xGoal, m_x,
			c_parameters.thresholdGoalReachedPos, c_parameters.thresholdGoalReachedAng);
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::isSuccess() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_success;
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::isTrajectoryAvailable() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_trajectory.get() != NULL;
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::lock()
{
	m_mutex.lock();
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::unlock()
{
	m_mutex.unlock();
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::reset()
{
	initTime();
	init();
}

template<int Dim, class Type>
Type VelocityController<Dim, Type>::getPathLength()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	Type length = 0;
	if (m_path.get() == NULL)
	{
		return length;
	}
	for (size_t i = 1; i < m_path->size(); ++i)
	{
		length += (m_path->operator [](i).pose.translation() - m_path->operator [](i - 1).pose.translation()).norm();
	}
	return length;
}

template<int Dim, class Type>
Type VelocityController<Dim, Type>::getExecutionPathLength()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_executionLength;
}

template<int Dim, class Type>
Type VelocityController<Dim, Type>::getPredictedExecutionTime() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_trajectory->getPredictedExecutionTime();
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::setPredictedExecutionTime(Type predictedExecutionTime)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_trajectory->setPredictedExecutionTime(predictedExecutionTime);
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::update(const ros::Time& now,
		const ros::Duration& dt)
{
	//	static std::ofstream file("/tmp/times.txt");
	const int nrOfJoints = m_robotArm->getChain().getNrOfJoints();
	KDL::JntArray cmd(nrOfJoints);
	cmd.data.fill(0);
	Vector6d goalError;
	Eigen::Vector3d pos;
	bool error = false;

	//update time
	m_now = now;
	m_dt = dt;

	//Get the data from the robot. If sendSimulatedTrajectoryPoses
	//is set, we don't need to receive the trajectory, because m_q
	//updated by time-integration over velocity at the end of the
	//loop
	//Hardware Interface: Get data as long as there is no trajectory
	//creation is active
	m_robotArm->receiveData(m_now, m_dt);
	m_q = m_robotArm->getKDLChainJointState();

	m_xOld = m_x;
	m_x.reset(m_kinematics, m_q);
	pos = m_x.x.head(3);
	m_mutex.lock();
	m_executionLength += (pos - m_posOld).norm();
	m_posOld = pos;
	m_mutex.unlock();

	//compute Jacobian
	m_kinematics->getJacobian(m_q, m_kdlJacobian);
	m_jacobian = m_kdlJacobian.data;

	//and inverse
	computeDampedLeastSquarePseudoInverse(m_jacobian, m_pseudoInverse, c_parameters);

	//compute where the robot should be
	computePrediction();

	//	file << (m_now - m_start).toSec() << std::endl;

	//compute error
	computeError();

	//compute desired velocity
	c_constraint->computeDifference(m_xPredicted, m_xPredictedOld, m_xDesiredDot);
	m_xDesiredDot /= m_dt.toSec();

//	if (c_parameters.debug)
//	{
////		LOG_INFO("DEBUG TRUE");
//		Vector6d xVel;
//		c_constraint->computeDifference(m_x, m_xOld, xVel);
//		xVel /= m_dt.toSec();
//		DATA_LOGGER_WRITE("velocity") << m_now.toSec() << xVel.head(3).norm() << xVel.tail(3).norm()
//				<< m_xDesiredDot.head(3).norm() << m_xDesiredDot.tail(3).norm();
//	}

	//compute null space projector
	//	m_N = m_I - m_pseudoInverse * m_jacobian;

	//add null space motion...
	//	m_qNullSpace.fill(0);

	//...joint range
	//	addNullSpaceMiddleJointRange();

	//...collision avoidance
	//	if (c_parameters.collisionAvoidanceUse)
	//	{
	//		boost::recursive_mutex::scoped_lock lock(m_mutex);
	//		if (m_kdtree.get() != NULL)
	//		{
	//			updateObjectDistances();
	//			addNullSpaceCollisionAvoidance();
	//		}
	//	}

	//compute command
//	Type weight = computeJointRangeWeight();
	cmd.data = m_pseudoInverse * (m_xDesiredDot + m_k * m_error);
	//			+ weight * m_qNullSpaceJointRange
	//			+ (1.0 - weight) * m_qNullSpaceCollisionAvoidance;

	if (!isValidCommand(cmd, m_dt.toSec(), 0.02))
	{
		stopMotion();
		boost::recursive_mutex::scoped_lock lock(m_mutex);
		if (m_printJointRangeWarning)
		{
			LOG_WARNING(c_armName << ": Command was invalid (nan or joint limits) or a collision check stopped the execution! Current pose:");
			LOG_INFO(c_armName << ": " << m_q.data.transpose());
			LOG_INFO(c_armName << " (cmd): " << (cmd.data * m_dt.toSec()).transpose());
			LOG_INFO(c_armName << " (lower limit): " << s_limitLower.transpose());
			LOG_INFO(c_armName << " (upper limit): " << s_limitUpper.transpose());
			LOG_INFO(m_dt.toSec());
			LOG_INFO(m_xDesiredDot.transpose());
			LOG_INFO(m_error.transpose());
			m_printJointRangeWarning = false;
		}
		cmd.data.fill(0);

		error = true;
	}

	m_robotArm->sendVelocity(m_now, m_dt, cmd);

	return error;
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::initController()
{
	m_trajectory.reset();
	m_path.reset();
	m_xDesiredDot = Vector6d::Zero();
	m_kinematics = m_robotArm->getKinematics()->getCopy(m_robotArm.get());

//	m_jacobianSolver = new JacobianMultiSolver(m_robotArm->getChain());

	//get transformation betweem planning and arm frame
	m_transformationPlanningFrameToArm = ais_ros::RosBaseInterface::getRosTransformation(c_planningFrame, c_armFrame);

	//pre-compute some required values for the redundant part of the controller
	//get joint limits
	std::vector<urdf::Joint> joints;
	m_robotArm->getChainJoints(joints);

	s_maxVelocityLimit = std::numeric_limits<double>::max();
	for (size_t i = 0; i < joints.size(); ++i)
	{
		boost::shared_ptr<urdf::JointLimits> limits;
		if (joints[i].type == urdf::Joint::CONTINUOUS)
		{
			limits.reset(new urdf::JointLimits);
			limits->upper = std::numeric_limits<double>::max();
			limits->lower = std::numeric_limits<double>::lowest();
			limits->velocity = joints[i].limits->velocity;
			limits->effort = joints[i].limits->effort;
		}
		else
		{
			limits = joints[i].limits;
		}

		s_limitUpper(i, 0) = limits->upper;
		s_limitLower(i, 0) = limits->lower;
		s_limitRangeCenters(i, 0) = (limits->lower + limits->upper) / 2.0;
		s_limitSquaredRange(i, 0) = (limits->upper - limits->lower) * (limits->upper - limits->lower);
		s_limitVelocities(i, 0) = std::min<double>(limits->velocity, c_parameters.maxVelocity);

		//get min velocity for normalization
		if (s_limitVelocities(i, 0) < s_maxVelocityLimit)
		{
			s_maxVelocityLimit = s_limitVelocities(i, 0);
		}
	}

	//resize some variables
	m_I.setIdentity();

	if (c_parameters.k.size() < 6)
	{
		throw ais_definitions::Exception("k vector needs to have 6 elements");
	}

	//set parameter vectors
	m_k.setZero();
	for (int i = 0; i < 6; ++i)
	{
		m_k(i, i) = c_parameters.k[i];
	}

	m_lambda.fill(c_parameters.lambda);

	//get joint state
	m_q = getJointStateFromRobot();
	m_x = m_x0 = m_xGoal = m_xPredicted = m_xPredictedOld = Trajectory::Pose(m_kinematics, m_q);

	//collision bubbles
//	resetCollisionStrip(m_q);

	m_counter = 0;
}

template<int Dim, class Type>
KDL::JntArray VelocityController<Dim, Type>::getJointStateFromRobot()
{
	return m_robotArm->getKDLChainJointState();
}

template<int Dim, class Type>
Trajectory::Pose VelocityController<Dim, Type>::getCurrentPose()
{
	return m_x;
}

template<int Dim, class Type>
Trajectory::Pose VelocityController<Dim, Type>::getGoalPose()
{
	return m_xGoal;
}

template<int Dim, class Type>
Vector6d VelocityController<Dim, Type>::getDistToGoal()
{
	Vector6d e;
	c_constraint->computeError(m_x, m_xGoal, e);
	return e;
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::initTime()
{	//	if (m_robotArm->c_type == INTERFACE_HARDWARE)
//	{
//		m_now = m_start = ros::Time(0);
//	}
//	else
//	{
	getTime(m_now);
	m_start = m_now;
	//	}

	m_dt = ros::Duration(0);
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::computeError()
{
	c_constraint->computeError(m_xPredicted, m_x, m_error);
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::computePrediction()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_xPredictedOld = m_xPredicted;
	if (m_trajectory.get() != NULL)
	{
		int currentWaypoint = 0;
		m_trajectory->getPose(m_now, m_xPredicted, currentWaypoint);
		m_currentWaypoint = currentWaypoint;
	}
	else
	{
		m_currentWaypoint = 0;
	}
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::isValidCommand(const KDL::JntArray& cmd,
		const Type& dt,
		const Type& errorBound)
{
	VectorNd xIntegrated = m_q.data + cmd.data * dt;

	for (size_t i = 0; i < s_limitLower.size(); ++i)
	{
		if (xIntegrated(i, 0) <= s_limitLower(i, 0) + errorBound || xIntegrated(i, 0) >= s_limitUpper(i, 0) - errorBound)
		{
			m_success = false;
			return false;
		}
		else if (std::isnan(xIntegrated(i, 0)))
		{
			m_success = false;
			return false;
		}
	}

	return true;
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::stopMotion()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_path.reset();
	m_trajectory.reset();
	m_xGoal = m_x;
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::limitFinalJointVelocities(KDL::JntArray& cmds)
{
	Type maxVelocity = 0;
	Type absVel;
	for (int i = 0; i < cmds.rows(); ++i)
	{
		absVel = fabs(cmds(i, 0));
		if (absVel > maxVelocity)
		{
			maxVelocity = absVel;
		}
	}

	if (maxVelocity > s_maxVelocityLimit)
	{
		const Type factor = maxVelocity / s_maxVelocityLimit;
		cmds.data /= factor;
	}
}

template<int Dim, class Type>
void VelocityController<Dim, Type>::limitJointVelocities(VectorNd& cmd,
		Type max)
{
	Type maxVelocity = 0;
	Type absVel;
	for (int i = 0; i < cmd.rows(); ++i)
	{
		absVel = fabs(cmd(i, 0));
		if (absVel > maxVelocity)
		{
			maxVelocity = absVel;
		}
	}

	if (maxVelocity > max)
	{
		const Type factor = maxVelocity / max;
		cmd /= factor;
	}
}

//template<int Dim, class Type>
//void VelocityController<Dim, Type>::resetCollisionStrip(const KDL::JntArray& joints)
//{
//	KDL::Frame x;
//	Matrix6xN j;
//	Matrix3xN j2;
//	MatrixNx3 pinv;
//
//	const size_t size = m_robotArm->getChain().getNrOfSegments();
//
//	m_collisionStrip.clear();
//
//	for (size_t i = 0; i < size; ++i)
//	{
//		if (m_robotArm->getChain().segments[i].getJoint().getType() == KDL::Joint::None)
//		{
//			continue;
//		}
//
//		m_kinematics->getFK(joints, x, i);
//		m_kinematics->getJacobian(m_q, m_kdlJacobian, i);
//		j = m_kdlJacobian.data;
////		m_fkSolver->JntToCart(joints, x, i);
////		m_jacobianSolver->JntToJac(m_q, j, i);
//		if (j.isZero())
//		{
//			continue;
//		}
//
//		Eigen::Vector3d pos(x.p.x(), x.p.y(), x.p.z());
//
//		ControllerBubble b;
//		b.radius = 0.15;
//		b.position = pos;
//		b.segment = i; //required to get the right J's
//		m_collisionStrip.push_back(b);
//	}
//}

template<int Dim, class Type>
void VelocityController<Dim, Type>::addNullSpaceMiddleJointRange()
{
	VectorNd jointRange = -m_lambda.cwiseProduct((2.0 * (m_q.data - s_limitRangeCenters)).cwiseQuotient(s_limitSquaredRange));

	m_qNullSpaceJointRange = m_N * jointRange;
}

template<int Dim, class Type>
Type VelocityController<Dim, Type>::computeJointRangeWeight()
{
	const Type std = c_parameters.jointRangeNullspaceWeightStd;
	const Type normalWeightJointRange = c_parameters.jointRangeNullspaceWeightDefault;
	Type x = std::numeric_limits<Type>::max();
	Type min;
	for (int i = 0; i < m_q.rows(); ++i)
	{
		min = std::min(fabs(m_q(i, 0) - s_limitLower(i, 0)), fabs(m_q(i, 0) - s_limitUpper(i, 0)));
		x = std::min(x, min);
	}

	return ((1.0 - normalWeightJointRange) * exp(-(x * x) / (2 * std * std))) + normalWeightJointRange;
}

//template<int Dim, class Type>
//void VelocityController<Dim, Type>::addNullSpaceCollisionAvoidance()
//{
//	boost::recursive_mutex::scoped_lock lock(m_mutex);
//
//	if (m_kdtree.get() == NULL)
//	{
//		m_qNullSpaceCollisionAvoidance.setZero();
//		return;
//	}
//
////algorithm from Kinematic Control Algorithms for On-Line Obstacle Avoidance for Redundant Manipulators
//
//	KDL::Frame x;
//	std::vector<int> indices(1);
//	std::vector<double> dists(1);
//	Type gain = 1.0;
//	Eigen::Vector3d n;
//	MatrixNx3 pinvJ;
//	Matrix3xN jacobian;
//	Type alphaV;
//	Type alphaH;
//	Type weight;
//	Type weightDivider = 0;
//	VectorNd cmd;
//	cmd.fill(0);
//	Eigen::Vector3d nearestPoint;
//
//	for (auto& it : m_collisionStrip)
//	{
//		weightDivider += (c_parameters.collisionAvoidanceDistance2 - it.obstacleDistance);
//	}
//
//	for (size_t i = 0; i < m_collisionStrip.size(); ++i)
//	{
//		ControllerBubble& bubble = m_collisionStrip[i];
//		n = bubble.obstacleToRobot / bubble.obstacleDistance;
//
//		jacobian = m_jacobians[bubble.segment].template block<3, Dim>(0, 0);
//
//		//get jacobian
//		if (jacobian.isZero())
//		{
//			continue;
//		}
//
//		computeDampedLeastSquarePseudoInverse(jacobian, pinvJ, c_parameters);
//
//		//alpha_v, alpha_h, weight
//		alphaV =
//				bubble.obstacleDistance < c_parameters.collisionAvoidanceDistance1 ?
//						pow(c_parameters.collisionAvoidanceDistance1 / bubble.obstacleDistance, 2) - 1.0 :
//						0.0;
//		alphaH = bubble.obstacleDistance <= c_parameters.collisionAvoidanceDistance1
//				? 1.0 : (bubble.obstacleDistance < c_parameters.collisionAvoidanceDistance2
//				? (0.5 + 0.5 * cos(M_PI * ((bubble.obstacleDistance - c_parameters.collisionAvoidanceDistance1)
//				/ (c_parameters.collisionAvoidanceDistance2 - c_parameters.collisionAvoidanceDistance1)))) :
//					0.0);
//		weight = (c_parameters.collisionAvoidanceDistance2 - bubble.obstacleDistance) / weightDivider;
//
//		cmd += weight * pinvJ * (n * c_parameters.collisionAvoidanceVel0);
//	}
//
//	m_qNullSpaceCollisionAvoidance = m_N * cmd;
//	limitJointVelocities(m_qNullSpaceCollisionAvoidance, c_parameters.collisionAvoidanceMaxJointVel);
//}

//template<int Dim, class Type>
//void VelocityController<Dim, Type>::updateObjectDistances()
//{
//	boost::recursive_mutex::scoped_lock lock(m_mutex);
//
//	//Eigen::Affine3d robotToPlanning = ais_ros::RosBaseInterface::getRosTransformation(m_robotArm->getRootFrame(), c_planningFrame);
//	//Eigen::Affine3d planningToRobot = ais_ros::RosBaseInterface::getRosTransformation(c_planningFrame, m_robotArm->getRootFrame());
//
//	if (m_kdtree.get() == NULL)
//	{
//		for (size_t i = 0; i < m_collisionStrip.size(); ++i)
//		{
//			ControllerBubble& bubble = m_collisionStrip[i];
//			bubble.radius = 1.0;
//			bubble.obstacleToRobot = Eigen::Vector3d::UnitX();
//			bubble.obstacleDistance = 1.0;
//		}
//
//		return;
//	}
//
//	KDL::Frame x;
//	Eigen::Vector3d nearestPoint;
//
//	for (size_t i = 0; i < m_collisionStrip.size(); ++i)
//	{
//		ControllerBubble& bubble = m_collisionStrip[i];
//		m_fkSolver->JntToCart(m_q, x, bubble.segment);
//		Eigen::Vector3d pos(x.p.x(), x.p.y(), x.p.z());
//		bubble.position = pos;
//
//		m_kdtree->getNearestNeighbor(bubble.position, nearestPoint);
//
//		/*//transform it to planning frame
//		 pos = robotToPlanning * pos;
//
//		 //get nearest neighbor
//		 m_kdtree->getNearestNeighbor(pos, nearestPoint);
//
//		 //transform it back
//		 nearestPoint = planningToRobot * nearestPoint;*/
//
//		//update bubble
//		bubble.radius = (bubble.position - nearestPoint).norm() - c_octomapResolution / 2.0; //since we use the center of each octomap voxel, we substract its half resolution to the radius
//		bubble.obstacleToRobot = bubble.position - nearestPoint;
//		bubble.obstacleDistance = bubble.radius;
//	}
//}

//template<int Dim, class Type>
//bool VelocityController<Dim, Type>::checkCollision()
//{
//	boost::recursive_mutex::scoped_lock lock(m_mutex);
//
//	if (m_kdtree.get() == NULL)
//	{
//		return false;
//	}
//
//	for (auto& it : m_collisionStrip)
//	{
//		if (it.obstacleDistance < c_parameters.collisionDetectionStopDistance)
//		{
//			return true;
//		}
//	}
//
//	return false;
//}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::isGoalReached(Trajectory::Pose& current)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return c_constraint->isGoalReached(m_xGoal, current,
			c_parameters.thresholdGoalReachedPos, c_parameters.thresholdGoalReachedAng);
}

template<int Dim, class Type>
bool VelocityController<Dim, Type>::writeData(const std::string& baseFileName)
{
	if (m_trajectory != NULL)
	{
		m_trajectory->writeGnuplotFile(baseFileName + "_spline.traj");
	}
	return true;
}

template class VelocityController<7, double> ;
template class VelocityController<10, double> ;

} /* namespace prm_planner */
