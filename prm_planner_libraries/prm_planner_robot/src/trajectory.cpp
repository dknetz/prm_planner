/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jan 28, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: trajectory.cpp
 */

#include <ais_log/log.h>
#include <ais_definitions/math.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <prm_planner_robot/kinematics.h>
#include <prm_planner_robot/trajectory.h>
#include <iostream>
#include <fstream>
#include <spline_library/splines/cubic_hermite_spline.h>
#include <spline_library/splines/quintic_hermite_spline.h>

//#include <ais_log/data_logger.h>

using KDL::ChainFkSolverPos_recursive;
using KDL::Frame;

//DATA_LOGGER_INIT();

namespace prm_planner
{

Trajectory::Trajectory(const boost::shared_ptr<Path>& path,
		const Vector6d& startVelocity,
		const double maxVelocityLinear,
		const double maxVelocityAngular,
		const ros::Time& now,
		const std::string frame) :
				m_waypoints(path->getWaypoints()),
				m_startVelocity(startVelocity),
				c_maxVelocityLinear(maxVelocityLinear),
				c_maxVelocityAngular(maxVelocityAngular),
				m_start(now),
				c_frame(frame)
{
	if (m_waypoints.size() < 2)
	{
		LOG_ERROR("path has less than 2 waypoints!");
		return;
	}

	initializeSpline();

	//compute time
	const Eigen::Affine3d& pose0 = m_waypoints[0].pose;
	Eigen::Quaterniond qOld(pose0.rotation());
	Eigen::Vector3d pOld(pose0.translation());
	double angularDiff, transDiff;
	double angularVel, transVel;
	const size_t waypointsSize = m_waypoints.size();
	m_predictedExecutionTime = 0;
	m_waypointInfos.resize(waypointsSize);
	m_waypointInfos[0].startTime = 0;
	m_waypointInfos[0].progress = 0;
	m_waypointInfos[0].distToStart = 0;
	double sumPathLength = 0;
	for (size_t i = 1; i < waypointsSize; ++i)
	{
		const Path::Waypoint& wpOld = m_waypoints[i - 1];
		const Eigen::Affine3d& poseI = m_waypoints[i].pose;
		Eigen::Quaterniond qCurrent(poseI.rotation());
		Eigen::Vector3d pCurrent(poseI.translation());
		angularDiff = qOld.angularDistance(qCurrent);
		transDiff = (pOld - pCurrent).norm();

		angularVel = wpOld.maxAngularVel <= 0 ? c_maxVelocityAngular : wpOld.maxAngularVel;
		transVel = wpOld.maxTranslationalVel <= 0 ? c_maxVelocityLinear : wpOld.maxTranslationalVel;

		sumPathLength += transDiff;

		m_predictedExecutionTime += std::max<double>(angularDiff / angularVel, transDiff / transVel);
		m_waypointInfos[i].startTime = m_predictedExecutionTime;
		m_waypointInfos[i].distToStart = sumPathLength;

		qOld = qCurrent;
		pOld = pCurrent;
	}

	//compute progress
	for (size_t i = 1; i < waypointsSize; ++i)
	{
		m_waypointInfos[i].progress = m_waypointInfos[i].distToStart / sumPathLength;
	}

//	double posDiff = m_spline.spline->totalLength();
//	m_predictedExecutionTime = std::max<double>(angleDiff / c_maxVelocityAngular, posDiff / c_maxVelocityLinear);
}

Trajectory::Trajectory(const std::vector<Path::Waypoint>& waypoints) :
				m_waypoints(waypoints),
				m_startVelocity(Vector6d::Zero()),
				c_maxVelocityLinear(1),
				c_maxVelocityAngular(1),
				m_start(ros::Time(0)),
				c_frame("no_frame"),
				m_predictedExecutionTime(0)
{
	if (m_waypoints.size() < 2)
	{
		LOG_ERROR("path has less than 2 waypoints!");
		return;
	}

	initializeSpline();
}

Trajectory::~Trajectory()
{
}

Trajectory::Pose::Pose(const Eigen::Affine3d& pose)
{
	reset(pose);
}

Trajectory::Pose::Pose(const KDL::Frame& pose)
{
	x(0, 0) = pose.p.data[0];
	x(1, 0) = pose.p.data[1];
	x(2, 0) = pose.p.data[2];
	pose.M.GetRPY(x(3, 0), x(4, 0), x(5, 0));

	Eigen::Affine3d eigenPose;
	tf::transformKDLToEigen(pose, eigenPose);
	quaternion = Eigen::Quaterniond(eigenPose.rotation());
}

Trajectory::Pose::Pose(const boost::shared_ptr<Kinematics>& fk,
		const KDL::JntArray& joints)
{
	reset(fk, joints);
}

void Trajectory::Pose::reset(const Eigen::Affine3d& pose)
{
	quaternion = Eigen::Quaterniond(pose.rotation());

	KDL::Frame kdlFrame;
	tf::transformEigenToKDL(pose, kdlFrame);

	x(0, 0) = kdlFrame.p.x();
	x(1, 0) = kdlFrame.p.y();
	x(2, 0) = kdlFrame.p.z();

	kdlFrame.M.GetRPY(x(3, 0), x(4, 0), x(5, 0));
}

void Trajectory::Pose::reset(const boost::shared_ptr<Kinematics>& fk,
		const KDL::JntArray& joints)
{
	KDL::Frame pose;
	fk->getFK(joints, pose);

	x(0, 0) = pose.p.data[0];
	x(1, 0) = pose.p.data[1];
	x(2, 0) = pose.p.data[2];
	pose.M.GetRPY(x(3, 0), x(4, 0), x(5, 0));

	Eigen::Affine3d eigenPose;
	tf::transformKDLToEigen(pose, eigenPose);
	quaternion = Eigen::Quaterniond(eigenPose.rotation());
}

void Trajectory::Pose::updateRotation()
{
	Eigen::Matrix<double, 3, 3> rot = quaternion.toRotationMatrix();
	KDL::Frame p;
	for (size_t i = 0; i < 9; ++i)
		p.M.data[i] = rot(i / 3, i % 3);
	p.M.GetRPY(x(3, 0), x(4, 0), x(5, 0));
}

void Trajectory::Pose::get(KDL::Frame& frame)
{
	frame.p(0) = x(0, 0);
	frame.p(1) = x(1, 0);
	frame.p(2) = x(2, 0);
	frame.M = KDL::Rotation::RPY((double) x(3, 0), (double) x(4, 0), (double) x(5, 0));
}

void Trajectory::Pose::get(Eigen::Affine3d& pose)
{
	KDL::Frame frame;
	get(frame);
	tf::transformKDLToEigen(frame, pose);
}

void Trajectory::Pose::transform(Eigen::Affine3d& t)
{
	Eigen::Affine3d pose;
	get(pose);

	pose = t * pose;

	reset(pose);
}

bool Trajectory::getPose(const ros::Time& now,
		Pose& pose,
		int& currentWaypoint)
{
	double duration = (now - m_start).toSec();
	double maxT = m_spline.spline->getMaxT();

//	LOG_INFO(duration);
//	LOG_INFO(maxT);
//	LOG_INFO(m_predictedExecutionTime);

	double t = maxT;
	size_t i;
	for (i = 0; i < m_waypointInfos.size() - 1; ++i)
	{
		WaypointInfos& wi = m_waypointInfos[i];
		WaypointInfos& wiNext = m_waypointInfos[i + 1];
		const double& startTime = wi.startTime;
		const double& nextTime = wiNext.startTime;
		if (duration >= startTime && duration < nextTime)
		{
			t = wi.progress + ((duration - startTime) / (nextTime - startTime)) * (wiNext.progress - wi.progress);
			t *= maxT;
			currentWaypoint = i;
			break;
		}
	}

	//	LOG_INFO(t);

//	double t = (duration / m_predictedExecutionTime) * maxT;
//	LOG_INFO(m_waypointInfos.size() << " " << i << " " << t);
	if (t > maxT)
	{
		t = maxT;
	}

	bool result = getPoseFromT(t, pose);
//	return result;

	currentWaypoint = 0;

	//estimate waypoint id
	if (result)
	{
		Eigen::Vector3d posTraj = pose.x.head(3);
		double min = std::numeric_limits<double>::max();
		for (size_t i = 1; i < m_waypoints.size() - 1; ++i)
		{
			const Path::Waypoint& current = m_waypoints[i];
			Eigen::Vector3d posCurrent = current.pose.translation();
			double dist = (posCurrent - posTraj).squaredNorm();

			if (dist < min)
			{
				min = dist;
				currentWaypoint = i;
			}
			else
			{
//				Path::Waypoint& next = (*m_path)[i];
//				Path::Waypoint& current = (*m_path)[i - 1];
//				Path::Waypoint& last = (*m_path)[i - 2];
//				Eigen::Vector3d posNext = next.pose.translation();
//				Eigen::Vector3d posLast = next.pose.translation();
//				double distLast = (posNext - posTraj).squaredNorm();
//				double distNext = (posLast - posTraj).squaredNorm();
//				double d1 = (posNext - posCurrent).squaredNorm();
//				double d2 = (posLast - posCurrent).squaredNorm();
//
//				if (distLast < d2)
//					currentWaypoint = i - 2;
//				else if (distNext < d1)
//					currentWaypoint = i - 1;
//				else
//					currentWaypoint = i;
//
//				LOG_INFO(i << " " << currentWaypoint << " " << m_path->size());
				break;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

bool Trajectory::getPoseFromT(const double& t,
		Pose& pose)
{
	Vector3 pos;
	if (t < m_spline.spline->getMaxT())
	{
		int index = (int) t;
		pos = m_spline.spline->getPosition(t);
		Eigen::Quaterniond q1(m_waypoints[index].pose.rotation());
		Eigen::Quaterniond q2(m_waypoints[index + 1].pose.rotation());
		pose.quaternion = q1.slerp(t - index, q2);

//		LOG_INFO(pose.quaternion.x() << " " << pose.quaternion.y() << " " << pose.quaternion.z() << " " << pose.quaternion.w());
	}
	else if (t == m_spline.spline->getMaxT())
	{
		//it's not possible to use tMax? (same knots in spline lib at the end? had no time to debug)
		pos = m_spline.spline->getPosition(t - 0.000001);
		pose.quaternion = Eigen::Quaterniond(m_waypoints.back().pose.rotation());
	}
	else
	{
		for (auto& it : m_waypoints)
		{
			LOG_INFO(it.pose.translation());
		}
		LOG_INFO(m_startVelocity);
		LOG_INFO(t << " " << m_spline.spline->getMaxT());
		LOG_ERROR("t > tmax");
		return false;
	}

	pose.x.x() = pos[0];
	pose.x.y() = pos[1];
	pose.x.z() = pos[2];
	pose.updateRotation();

	return true;
}

nav_msgs::PathConstPtr Trajectory::getRosTrajectory()
{
	if (m_rosTrajectory.get() == NULL)
	{
		if (c_frame == "no_frame")
		{
			LOG_WARNING("You might not be able to visualize the trajectory because frame name is 'no_frame'");
		}

		Vector3 pos;
		ros::Time time = ros::Time::now();
		int i = 0;
		m_rosTrajectory.reset(new nav_msgs::Path);
		m_rosTrajectory->header.frame_id = c_frame;
		for (float t = 0.0; t < m_spline.spline->getMaxT(); t += 0.01)
		{
			pos = m_spline.spline->getPosition(t);
//			Vector3 tang;
//			computeSmoothFirstDerivative(t, tang);

			geometry_msgs::PoseStamped pose;
			pose.pose.position.x = pos[0];
			pose.pose.position.y = pos[1];
			pose.pose.position.z = pos[2];
			pose.header.frame_id = c_frame;
			pose.header.seq = i++;
			pose.header.stamp = time;

			m_rosTrajectory->poses.push_back(pose);
		}
	}

	return m_rosTrajectory;
}

void Trajectory::initializeSpline()
{
	Vector3 tangent;
	Vector3 p1s, p2s;
	Eigen::Vector3d p1, p2;
	Eigen::Vector3d dir1;

	const size_t waypointsSize = m_waypoints.size();

	std::vector<Vector3> ps(waypointsSize), ts(waypointsSize);

	p1 = m_waypoints[0].getTaskPosition();
	p2 = m_waypoints[1].getTaskPosition();

	dir1 = (p2 - p1) / 2.0; // /2 wiki
	p1s[0] = p1.x();
	p1s[1] = p1.y();
	p1s[2] = p1.z();
	p2s[0] = p2.x();
	p2s[1] = p2.y();
	p2s[2] = p2.z();

	ps[0] = p1s;
//	ts[0][0] = m_startVelocity.x();
//	ts[0][1] = m_startVelocity.y();
//	ts[0][2] = m_startVelocity.z();
	ts[0][0] = dir1.x();
	ts[0][1] = dir1.y();
	ts[0][2] = dir1.z();

	if (waypointsSize == 2)
	{
		ps[1] = p2s;
		ts[1][0] = dir1.x();
		ts[1][1] = dir1.y();
		ts[1][2] = dir1.z();

		m_spline.spline.reset(new CubicHermiteSpline<Vector3>(ps, ts, 0.5));
		m_spline.length = m_spline.spline->totalLength();

		if (!isValid())
		{
			LOG_INFO("ps:");
			for (auto&it : ps)
			{
				LOG_INFO(it[0] << " " << it[1] << " " << it[2]);
			}

			LOG_INFO("ts:");
			for (auto&it : ts)
			{
				LOG_INFO(it[0] << " " << it[1] << " " << it[2]);
			}
		}
	}
	else
	{
		Vector3 p3s;

		Eigen::Vector3d p3;
		Eigen::Vector3d tangent;
		double maxLinVel;

		std::vector<Vector3> points(2);
		size_t i = 1;
		for (; i < waypointsSize - 1; ++i)
		{
			double wpVel = m_waypoints[i].maxTranslationalVel;
			maxLinVel = wpVel <= 0 ? c_maxVelocityLinear : wpVel;
			p3 = m_waypoints[i + 1].getTaskPosition();
			p3s[0] = p3.x();
			p3s[1] = p3.y();
			p3s[2] = p3.z();

			tangent = ((p3 - p1) / 2);

			ps[i] = p2s;
			ts[i][0] = tangent.x();
			ts[i][1] = tangent.y();
			ts[i][2] = tangent.z();

			//update positions
			p1 = p2;
			p1s = p2s;
			p2 = p3;
			p2s = p3s;
		}

		//create final spline, tangent of goal is in the direction of second last
		//point to last point
		ps[i] = p3s;
		ts[i] = (p3s - p2s) * 0.5f;

		m_spline.spline.reset(new CubicHermiteSpline<Vector3>(ps, ts, 0.5));
		m_spline.length = m_spline.spline->totalLength();

		if (!isValid())
		{
			LOG_INFO("ps:");
			for (auto&it : ps)
			{
				LOG_INFO(it[0] << " " << it[1] << " " << it[2]);
			}

			LOG_INFO("ts:");
			for (auto&it : ts)
			{
				LOG_INFO(it[0] << " " << it[1] << " " << it[2]);
			}
		}
	}
}

bool Trajectory::isValid()
{
	//don't know why maxT can get nan...
	return m_waypoints.size() == 1 || (m_spline.spline.get() != NULL && !std::isnan(m_spline.spline->getMaxT()));
}

void Trajectory::computeSmoothFirstDerivative(const double t,
		Vector3& d)
{
	if (t < 0.001)
	{
		Vector3 p1 = m_spline.spline->getPosition(t);
		Vector3 p2 = m_spline.spline->getPosition(t + 0.001);
		d = (p2 - p1) / 0.002f;
	}
	else if (t > m_spline.spline->getMaxT() - 0.001)
	{
		Vector3 p1 = m_spline.spline->getPosition(t - 0.001);
		Vector3 p2 = m_spline.spline->getPosition(t);
		d = (p2 - p1) / 0.002f;
	}
	else
	{
		Vector3 p1 = m_spline.spline->getPosition(t + 0.001);
		Vector3 p2 = m_spline.spline->getPosition(t - 0.001);
		d = (p2 - p1) / 0.002f;
	}
}

double Trajectory::getPredictedExecutionTime() const
{
	return m_predictedExecutionTime;
}

void Trajectory::setPredictedExecutionTime(const double predictionTime)
{
	m_predictedExecutionTime = predictionTime;
}

void Trajectory::setNow(const ros::Time& now)
{
	m_start = now;
}

void Trajectory::writeGnuplotFile(const std::string& file)
{
	std::ofstream f(file);
	for (double t = 0.000; t <= m_spline.spline->getMaxT(); t += 0.001)
	{
		Spline<Vector3>::InterpolatedPT v = m_spline.spline->getTangent(t);
		f << t << ", " << v.position[0] << ", " << v.position[1] << ", " << v.position[2] << ", " << v.tangent[0] << ", " << v.tangent[1] << ", " << v.tangent[2] << "\n";
	}
	f.close();
}

} /* namespace prm_planner */
