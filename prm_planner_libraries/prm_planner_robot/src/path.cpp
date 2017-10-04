/*
 * trajectory.cpp
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#include <ais_log/log.h>
#include <ais_util/color.h>
#include <eigen_conversions/eigen_msg.h>
#include <prm_planner_robot/path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <fstream>

namespace prm_planner
{

bool Path::m_pubInit = false;
ros::Publisher Path::m_pub;

Path::Waypoint::Waypoint() :
				type(Waypoint::RobotWaypoint),
				id(-2),
				pose(Eigen::Affine3d::Identity()),
				maxAngularVel(-1),
				maxTranslationalVel(-1)
{
}

Path::Waypoint::Waypoint(const Waypoint& other) :
				type(other.type),
				id(other.id),
				pose(other.pose),
				jointPose(other.jointPose),
				trajectory(other.trajectory),
				maxAngularVel(other.maxAngularVel),
				maxTranslationalVel(other.maxTranslationalVel),
				helper(other.helper)
{
}

Path::Waypoint::Waypoint(const int& id,
		const Eigen::Affine3d& pose) :
				type(Waypoint::RobotWaypoint),
				id(id),
				pose(pose),
				maxAngularVel(-1),
				maxTranslationalVel(-1)
{
}

Path::Waypoint::Waypoint(const int& id,
		const Eigen::Affine3d& pose,
		const KDL::JntArray& jointstate) :
				type(Waypoint::RobotWaypoint),
				id(id),
				pose(pose),
				jointPose(jointstate),
				maxAngularVel(-1),
				maxTranslationalVel(-1)
{
}

Path::Waypoint Path::Waypoint::getOpenGripperWaypoint()
{
	Waypoint wp;
	wp.type = Waypoint::OpenGripper;
	return wp;
}

Path::Waypoint Path::Waypoint::getCloseGripperWaypoint()
{
	Waypoint wp;
	wp.type = Waypoint::CloseGripper;
	return wp;
}

const Eigen::Vector3d Path::Waypoint::getTaskPosition() const
{
	return pose.translation();
}

const Eigen::Affine3d Path::Waypoint::getTaskPose() const
{
	return pose;
}

Path::Path(const std::string& frame) :
				m_length(0),
				m_lengthUpdated(false),
				c_frame(frame),
				m_debug(false),
				m_maxId(0),
				m_cachedPath(false)
{
	if (!m_pubInit)
	{
		ros::NodeHandle n;
		m_pub = n.advertise<visualization_msgs::MarkerArray>("complete_path", 1);
		m_pubInit = true;
	}
}

Path::Path(const Path& other) :
				m_waypoints(other.m_waypoints),
				m_length(other.m_length),
				m_lengthUpdated(other.m_lengthUpdated),
				c_frame(other.c_frame),
				m_debug(false),
				m_maxId(other.m_maxId),
				m_cachedPath(other.m_cachedPath)
{
	if (!m_pubInit)
	{
		ros::NodeHandle n;
		m_pub = n.advertise<visualization_msgs::MarkerArray>("complete_path", 1);
		m_pubInit = true;
	}
}

Path::Path() :
				m_length(0),
				m_lengthUpdated(false),
				m_debug(false),
				m_maxId(0),
				m_cachedPath(false)
{
}

Path::~Path()
{
}

void Path::append(const Waypoint& waypoint,
		const bool clean)
{
	m_lengthUpdated = false;
	m_waypoints.push_back(waypoint);

	if (waypoint.id > m_maxId)
		m_maxId = waypoint.id;

	//clean the path if requested. Otherwise
	//you should do it by yourself
	if (clean)
		cleanPath();
}

void Path::prepend(const Waypoint& waypoint,
		const bool clean)
{
	m_lengthUpdated = false;
	m_waypoints.insert(m_waypoints.begin(), waypoint);

	if (waypoint.id > m_maxId)
		m_maxId = waypoint.id;

	//clean the path if requested. Otherwise
	//you should do it by yourself
	if (clean)
		cleanPath();
}

void Path::append(const Path& path,
		const bool clean)
{
	m_lengthUpdated = false;

	m_waypoints.reserve(m_waypoints.size() + path.size());
	for (auto& it : path)
	{
		m_waypoints.push_back(it);

		if (it.id > m_maxId)
			m_maxId = it.id;
	}

	//clean the path if requested. Otherwise
	//you should do it by yourself
	if (clean)
		cleanPath();
}

Path& Path::operator +=(const Path& path)
{
	append(path, false);
	return *this;
}

Path& Path::operator +=(const Waypoint& waypoint)
{
	append(waypoint, false);
	return *this;
}

void Path::clear()
{
	m_waypoints.clear();
	m_lengthUpdated = false;
	m_length = 0;
	m_maxId = 0;
	m_rosPath.reset();
}

size_t Path::size() const
{
	return m_waypoints.size();
}

void Path::resize(const size_t size)
{
	m_lengthUpdated = false;
	m_waypoints.resize(size);
	m_rosPath.reset();
}

bool Path::empty() const
{
	return m_waypoints.empty();
}

void Path::setWaypoints(const std::vector<Waypoint>& waypoints)
{
	m_waypoints = waypoints;
	m_lengthUpdated = false;
	m_rosPath.reset();
	cleanPath();
}

Path::Waypoint& Path::operator [](const size_t index)
{
	m_lengthUpdated = false;
	return m_waypoints[index];
}

const Path::Waypoint& Path::operator [](const size_t index) const
		{
	return m_waypoints[index];
}

double Path::length()
{
	if (!m_lengthUpdated)
	{
		for (size_t i = 1; i < m_waypoints.size(); ++i)
		{
			m_length += (m_waypoints[i].getTaskPosition() - m_waypoints[i - 1].getTaskPosition()).norm();
		}
		m_lengthUpdated = true;
	}

	return m_length;
}

double Path::lengthToGoal(int startWaypoint)
{
	double length = 0;

	for (size_t i = startWaypoint + 1; i < m_waypoints.size(); ++i)
	{
		length += (m_waypoints[i].getTaskPosition() - m_waypoints[i - 1].getTaskPosition()).norm();
	}

	return length;
}

Path& Path::operator =(const Path& other)
{
	m_waypoints = other.m_waypoints;
	m_length = other.m_length;
	m_lengthUpdated = other.m_lengthUpdated;
	m_maxId = other.m_maxId;
	return *this;
}

std::vector<Path::Waypoint>::iterator Path::begin()
{
	return m_waypoints.begin();
}

std::vector<Path::Waypoint>::const_iterator Path::begin() const
{
	return m_waypoints.begin();
}

std::vector<Path::Waypoint>::iterator Path::end()
{
	return m_waypoints.end();
}

std::vector<Path::Waypoint>::const_iterator Path::end() const
{
	return m_waypoints.end();
}

const Path::Waypoint& Path::back() const
{
	return m_waypoints.back();
}

const Path::Waypoint& Path::front() const
{
	return m_waypoints.front();
}

Path::Waypoint& Path::back()
{
	return m_waypoints.back();
}

Path::Waypoint& Path::front()
{
	return m_waypoints.front();
}

nav_msgs::PathConstPtr Path::getRosPath()
{
	if (m_rosPath.get() == NULL && !m_waypoints.empty())
	{
		ros::Time time = ros::Time::now();
		m_rosPath.reset(new nav_msgs::Path);
		int seq = 0;
		for (auto& it : m_waypoints)
		{
			if (it.type == Waypoint::RobotWaypoint)
			{
				geometry_msgs::PoseStamped pose;
				tf::poseEigenToMsg(it.getTaskPose(), pose.pose);
				pose.header.frame_id = c_frame;
				pose.header.stamp = time;
				pose.header.seq = seq++;
				m_rosPath->poses.push_back(pose);
			}
		}
		m_rosPath->header.frame_id = c_frame;
		m_rosPath->header.stamp = time;
	}

	return m_rosPath;
}

void Path::resetRosMarkers()
{
	m_rosPath.reset();
}

bool Path::isSpecialCommand() const
{
	return m_waypoints.size() == 1 && m_waypoints[0].type != Waypoint::RobotWaypoint;
}

Path::Waypoint::Type Path::getFirstWaypointType() const
{
	return m_waypoints.front().type;
}

std::string Path::getFirstWaypointHelper() const
{
	return m_waypoints.front().helper;
}

void Path::cleanPath()
{
	bool lastWayPointSpecial = false;
	m_maxId = 0;
	for (auto it = m_waypoints.begin() + 1; it != m_waypoints.end();)
	{
		//we only consider 'RobotWaypoint' waypoints and
		//no gripper waypoints
		if (it->type != Waypoint::RobotWaypoint)
		{
			++it;
			lastWayPointSpecial = true;

			if (it->id > m_maxId)
				m_maxId = it->id;

			continue;
		}

		//add the first waypoint after a special command
		//in all cases to avoid situations were only one
		//waypoint is in a path (because the first one
		//was removed)
		if (!lastWayPointSpecial)
		{
			auto itOld = it;
			--itOld;
			if (it->pose.isApprox(itOld->pose))
			{
				it = m_waypoints.erase(it);
			}
			else
			{
				if (it->id > m_maxId)
					m_maxId = it->id;

				++it;
			}
		}
		else
		{
			if (it->id > m_maxId)
				m_maxId = it->id;

			++it;
		}
	}
}

void Path::reverse()
{
	std::reverse(m_waypoints.begin(), m_waypoints.end());
	for (auto& it : m_waypoints)
	{
		std::reverse(it.trajectory.begin(), it.trajectory.end());
	}
}

double Path::getPathLength() const
{
	double length = 0;
	for (size_t i = 1; i < m_waypoints.size(); ++i)
	{
		length += (m_waypoints[i].pose.translation() - m_waypoints[i - 1].pose.translation()).norm();
	}
	return length;
}

void Path::writeTrajectoryData(const std::string& filename)
{
	std::ofstream file(filename);

	if (!file.is_open())
	{
		LOG_ERROR("cannot write to " << filename);
		return;
	}

	if (isSpecialCommand())
	{
		return;
	}

	for (size_t w = 0; w < m_waypoints.size(); ++w)
	{
		Path::Waypoint& it = m_waypoints[w];

		KDL::JntArray wp = it.jointPose;

		//draw end point
		if (it.trajectory.empty())
		{
			for (size_t j = 0; j < 2; ++j)
				for (size_t i = 0; i < wp.data.size(); ++i)
				{
					file << wp.data(i);
					if (j == 0 || i < wp.data.size() - 1)
						file << ", ";
				}
			wp.data.setZero();
			file << "\n";
		}
		else
		{
			for (auto& it2 : it.trajectory)
			{
				for (size_t i = 0; i < it2.positions.data.size(); ++i)
				{
					file << it2.positions.data(i);
					file << ", ";
				}
				for (size_t i = 0; i < wp.data.size(); ++i)
				{
					file << wp.data(i);
					if (i < wp.data.size() - 1)
						file << ", ";
				}
				wp.data.setZero();
				file << "\n";
			}
		}

		file << "\n";
		file.flush();
	}
}

void Path::save(const std::string& filename,
		const std::vector<KDL::JntArray>& path)
{
	std::ofstream file(filename);

	if (!file.is_open())
	{
		LOG_ERROR("cannot write to " << filename);
		return;
	}

	for (auto& it : path)
	{
		for (size_t i = 0; i < it.data.size(); ++i)
		{
			file << it.data(i);
			if (i < it.data.size() - 1)
			{
				file << ", ";
			}
		}
		file << "\n";
		file.flush();
	}
}

void Path::makeDense(const int numberOfJointStates)
{
	//do nothing if path is empty
	if (m_waypoints.empty())
		return;

	std::vector<Waypoint> waypoints = m_waypoints;
	m_waypoints.clear();

	Eigen::Affine3d p1, p2;
	Eigen::Affine3d newPose;
	Eigen::Vector3d pos1, pos2, pos3, diff;
	Eigen::Quaterniond quat;

	double dist = 0;
	double distAng = 0;

	Eigen::Quaterniond rot1, rot2;
	size_t id = 0;
	double tStart = 0;

	//start state (to get initial joint state)
	Waypoint wpStart = waypoints.front();
	wpStart.id = id++;
	m_waypoints.push_back(wpStart);
	pos1 = wpStart.pose.translation();
	quat = Eigen::Quaterniond(wpStart.pose.linear());

	ArmJointPath tNew;

	for (size_t i = 0; i < waypoints.size(); ++i)
	{
		Waypoint& wpCurrent = waypoints[i];

		//skip special waypoints, i.e., make them
		//not dense and just copy the waypoint
		if (wpCurrent.type != Waypoint::RobotWaypoint)
		{
			Waypoint wp(wpCurrent);
			wp.id = id++;

			//add this waypoint to be able to reach the goal in a precise way
			if (i > 0)
			{
				m_waypoints.push_back(waypoints[i - 1]);
			}

			m_waypoints.push_back(wp);
			tNew.clear();
		}
		else
		{
//			Waypoint& wpNext = waypoints[i + 1];
//
//			//if the next waypoint is a special waypoint
//			//add the current waypoint as a end point
//			//to be able to exactly reach the sub goal
//			if (wpNext.type != Waypoint::RobotWaypoint)
//			{
//				Waypoint wp(wpCurrent);
//				wp.id = id++;
//
//				m_waypoints.push_back(wp);
//			}
//			else
//			{
//				p1 = wpCurrent.pose;
//				p2 = wpNext.pose;
//				pos1 = p1.translation();
//				pos2 = p2.translation();
//				rot1 = p1.rotation();
//				rot2 = p2.rotation();
//
//				double t = tStart;
//				diff = pos2 - pos1;
//				double dist = diff.norm();
//				diff /= dist;
//
//				while (t < dist)
//				{
//					newPose.setIdentity();
//					newPose.translation() = pos1 + diff * t;
//					newPose.linear() = rot1.slerp(t / dist, rot2).matrix();
//					t += distBetweenWaypoints;
//
//					Waypoint wp;
//					wp.id = id++;
//					wp.pose = newPose;
//					wp.maxTranslationalVel = waypoints[i].maxTranslationalVel;
//					wp.maxAngularVel = waypoints[i].maxAngularVel;
//
//					m_waypoints.push_back(wp);
//
//					//find tStart, which is distBetweenWaypoints away
//					if (t >= dist)
//					{
//						tStart = t - dist;
//					}
//				}

			//add start waypoint
//			if (i == 0)
//			{
//				m_waypoints.push_back(wpCurrent);
//			}

//			for (auto& it : wpCurrent.trajectory)
//			{
//				dist += (pos1 - it.taskPosition).norm();
//				distAng += (quat.angularDistance(it.taskOrientation));
//				pos1 = it.taskPosition;
//				quat = it.taskOrientation;
//
//				if (dist > distBetweenWaypoints || distAng > angDistBetweenWaypoints)
//				{
//					newPose.setIdentity();
//					newPose.translation() = it.taskPosition;
//					newPose.linear() = it.taskOrientation.matrix();
//
//					Waypoint wp;
//					wp.id = id++;
//					wp.pose = newPose;
//					wp.maxTranslationalVel = wpCurrent.maxTranslationalVel;
//					wp.maxAngularVel = wpCurrent.maxAngularVel;
//					wp.jointPose = it.positions;
//
//					//add it to the last waypoint
//					m_waypoints.back().trajectory = tNew;
//
//					m_waypoints.push_back(wp);
//
//					tNew.clear();
//					dist = 0;
//					distAng = 0;
//				}
//
//				tNew.push_back(it);
//			}

			for (auto& it : wpCurrent.trajectory)
			{
//				dist += (pos1 - it.taskPosition).norm();
//				distAng += (quat.angularDistance(it.taskOrientation));
//				pos1 = it.taskPosition;
//				quat = it.taskOrientation;

				if (tNew.size() > numberOfJointStates)
				{
					newPose.setIdentity();
					newPose.translation() = it.taskPosition;
					newPose.linear() = it.taskOrientation.matrix();

					Waypoint wp;
					wp.id = id++;
					wp.pose = newPose;
					wp.maxTranslationalVel = wpCurrent.maxTranslationalVel;
					wp.maxAngularVel = wpCurrent.maxAngularVel;
					wp.jointPose = it.positions;

					//add it to the last waypoint
					m_waypoints.back().trajectory = tNew;

					m_waypoints.push_back(wp);

					tNew.clear();
					dist = 0;
					distAng = 0;
				}

				tNew.push_back(it);
			}

//			}
		}
	}

	//add goal waypoint if necessary
	if (waypoints.size() > 1 && !m_waypoints.back().pose.isApprox(waypoints.back().pose))
	{
//		//create new waypoint
//		if (tNew.size() > 25)
//		{
//			m_waypoints.back().trajectory = tNew;
//
//			//		TrajectoryWaypoint& twp = tNew.back();
//			//
//			//		newPose.setIdentity();
//			//		newPose.translation() = twp.taskPosition;
//			//		newPose.linear() = twp.taskOrientation.matrix();
//
//			//both trajectory and joint position might be not available
//			//due to path database, which adds a start and goal waypoint
//			//to a path found in the database. These nodes can/might not
//			//have such information yet.
//			Waypoint wpGoal = waypoints.back();
//			wpGoal.id = id;
//
//			//		wpGoal.pose = newPose;
//			//		wpGoal.jointPose = tNew.back().positions;
//			m_waypoints.push_back(wpGoal);
//		}
//		//move last waypoint
//		else
//		{
			Waypoint& wp = m_waypoints.back();
			wp.trajectory.insert(wp.trajectory.end(), tNew.begin(), tNew.end());
			wp.pose = waypoints.back().pose;
			wp.jointPose = KDL::JntArray();
//		}

	}

	m_maxId = id;
}

void Path::transform(const Eigen::Affine3d& t)
{
	for (auto& it : m_waypoints)
	{
		it.pose = t * it.pose;
	}
}

boost::shared_ptr<Path> Path::getSubPath(size_t startWaypoint,
		size_t goalWaypoint)
{
	boost::shared_ptr<Path> path(new Path(c_frame));

	for (size_t i = startWaypoint; i < std::min(goalWaypoint + 1, m_waypoints.size()); ++i)
	{
		path->append(m_waypoints[i], false);
	}

	return path;
}

bool Path::computeSubPathWhichStartsAtCurrentPose(const boost::shared_ptr<Path>& path,
		const KDL::JntArray& currentPose,
		const double maxDistance,
		const int removeWaypoints,
		boost::shared_ptr<Path>& subPath)
{
	int i = 0;
	int closest = -1;
	double closestDist = std::numeric_limits<double>::max();
	double dist;

	for (auto& it : *path)
	{
		KDL::JntArray& j = it.jointPose;
		if ((dist = (currentPose.data - j.data).norm()) < closestDist)
		{
			closest = i;
			closestDist = dist;
		}

		++i;
	}

	if (closest == -1 || closestDist > maxDistance)
		return false;

	subPath = path->getSubPath(closest + removeWaypoints, path->size());
	return true;
}

void Path::publish()
{
	//prm
	if (m_pub.getNumSubscribers() > 0)
	{
		visualization_msgs::MarkerArray markers;
		int id = 0;

		getROSVisualizationMessage(markers, id, true, true);

		m_pub.publish(markers);
	}
}

void Path::getROSVisualizationMessage(visualization_msgs::MarkerArray& array,
		int& id,
		const bool plotAxis,
		const bool plotPath)
{
	static const std_msgs::ColorRGBA green = ais_util::Color::green().toROSMsg();
	static const std_msgs::ColorRGBA red = ais_util::Color::red().toROSMsg();
	static const std_msgs::ColorRGBA yellow = ais_util::Color::yellow().toROSMsg();
	static const std_msgs::ColorRGBA orange = ais_util::Color::orange().toROSMsg();
	static const std_msgs::ColorRGBA blue = ais_util::Color::blue().toROSMsg();

	//markers
	visualization_msgs::Marker nodeXMarker;
	visualization_msgs::Marker nodeYMarker;
	visualization_msgs::Marker nodeZMarker;
	visualization_msgs::Marker path;

	//path
	path.header.frame_id = c_frame;
	path.header.stamp = ros::Time();
	path.ns = "path";
	path.id = id++;
	path.type = visualization_msgs::Marker::LINE_STRIP;
	path.action = visualization_msgs::Marker::ADD;
	path.scale.x = m_cachedPath ? 0.005 : 0.003;
	path.color = m_cachedPath ? yellow : orange;

	//node or axis
	if (plotAxis)
	{
		nodeXMarker.header.frame_id = c_frame;
		nodeXMarker.header.stamp = ros::Time();
		nodeXMarker.ns = "nodes_x";
		nodeXMarker.id = id++;
		nodeXMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeXMarker.action = visualization_msgs::Marker::ADD;
		nodeXMarker.scale.x = 0.001;
		nodeXMarker.color = red;

		nodeYMarker.header.frame_id = c_frame;
		nodeYMarker.header.stamp = ros::Time();
		nodeYMarker.ns = "nodes_y";
		nodeYMarker.id = id++;
		nodeYMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeYMarker.action = visualization_msgs::Marker::ADD;
		nodeYMarker.scale.x = 0.001;
		nodeYMarker.color = green;

		nodeZMarker.header.frame_id = c_frame;
		nodeZMarker.header.stamp = ros::Time();
		nodeZMarker.ns = "nodes_z";
		nodeZMarker.id = id++;
		nodeZMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeZMarker.action = visualization_msgs::Marker::ADD;
		nodeZMarker.scale.x = 0.001;
		nodeZMarker.color = blue;
	}
	else
	{
		nodeXMarker.header.frame_id = c_frame;
		nodeXMarker.header.stamp = ros::Time();
		nodeXMarker.ns = "nodes";
		nodeXMarker.id = id++;
		nodeXMarker.type = visualization_msgs::Marker::POINTS;
		nodeXMarker.action = visualization_msgs::Marker::ADD;
		nodeXMarker.scale.x = 0.002;
		nodeXMarker.color = green;
	}

	for (auto& node : m_waypoints)
	{
		if (node.type == Waypoint::RobotWaypoint)
		{
			const Eigen::Vector3d t = node.getTaskPosition();
			const Eigen::Matrix3d rotation = node.getTaskPose().rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();

			if (plotAxis)
			{
				//x axis
				geometry_msgs::Point px;
				px.x = t.x() + x.x() * 0.03;
				px.y = t.y() + x.y() * 0.03;
				px.z = t.z() + x.z() * 0.03;
				nodeXMarker.points.push_back(p);
				nodeXMarker.points.push_back(px);

				//y axis
				geometry_msgs::Point py;
				py.x = t.x() + y.x() * 0.03;
				py.y = t.y() + y.y() * 0.03;
				py.z = t.z() + y.z() * 0.03;
				nodeYMarker.points.push_back(p);
				nodeYMarker.points.push_back(py);

				//z axis
				geometry_msgs::Point pz;
				pz.x = t.x() + z.x() * 0.03;
				pz.y = t.y() + z.y() * 0.03;
				pz.z = t.z() + z.z() * 0.03;
				nodeZMarker.points.push_back(p);
				nodeZMarker.points.push_back(pz);
			}
			else
			{
				nodeXMarker.points.push_back(p);
			}

			if (plotPath)
			{
				path.points.push_back(p);
			}
		}
	}

	array.markers.push_back(nodeXMarker);

	if (plotAxis)
	{
		array.markers.push_back(nodeYMarker);
		array.markers.push_back(nodeZMarker);
	}

	if (plotPath)
		array.markers.push_back(path);
}

int Path::getMaxId() const
{
	return m_maxId;
}

bool Path::isCachedPath() const
{
	return m_cachedPath;
}

void Path::setCachedPath(bool cachedPath)
{
	m_cachedPath = cachedPath;
}

const std::vector<Path::Waypoint>& Path::getWaypoints() const
{
	return m_waypoints;
}

} /* namespace prm_planner */

std::ostream& operator <<(std::ostream& stream,
		const prm_planner::Path& traj)
{
	stream << "Trajectory:\n";
	if (traj.size() == 0)
	{
		stream << "\tempty";
	}
	else
	{
		int i = 0;
		double length = 0;
		Eigen::Vector3d pOld = traj.front().getTaskPosition();
		for (auto& it : traj)
		{
			Eigen::Vector3d pos = it.getTaskPosition();
			length += (pOld - pos).norm();
			pOld = pos;
			stream << "\t" << i++ << ": pos=[" << pos.x() << ", " << pos.y() << ", " << pos.z() << "], l = " << length << " " << "\n";
		}
	}
	return stream;
}

