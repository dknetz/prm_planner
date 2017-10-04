/*
 * trajectory.h
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#ifndef PATH_H_
#define PATH_H_
#include <ais_util/serialization_std.h>
#include <ais_util/serialization_kdl.h>
#include <ais_util/serialization_eigen.h>
#include <kdl/jntarray.hpp>
#include <Eigen/Geometry>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <prm_planner_robot/defines.h>
#include <ros/ros.h>
#include <unordered_map>

namespace prm_planner
{

class PRMEdge;
class PRMNode;

class Path
{
public:
	struct Waypoint
	{
		enum Type
		{
			OpenGripper,
			CloseGripper,
			RobotWaypoint
		};

		Waypoint();
		Waypoint(const Waypoint& other);
		Waypoint(const int& id,
				const Eigen::Affine3d& pose);
		Waypoint(const int& id,
				const Eigen::Affine3d& pose,
				const KDL::JntArray& jointstate);

		const Eigen::Vector3d getTaskPosition() const;
		const Eigen::Affine3d getTaskPose() const;

		static Waypoint getOpenGripperWaypoint();
		static Waypoint getCloseGripperWaypoint();

		/**
		 * Serialization
		 */
		template<class Archive>
		void serialize(Archive& ar,
				const unsigned int version)
		{
			ar & type & id & pose & maxAngularVel & maxTranslationalVel & jointPose & helper;
		}

		//the type of the waypoint @see Type
		Type type;

		//can be used to store some id
		int id;

		//the pose of the waypoint. In case of an special
		//waypoint the pose can be used differently:
		// - grasping: pose of the object relative to the
		//	 planning frame
		// - droping: not used right now
		// - default: the end-effector pose
		Eigen::Affine3d pose;

		//the joint position
		KDL::JntArray jointPose;

		//used for visualization
		ArmJointPath trajectory;

		//if you set it to -1 the default velocities are assumed
		double maxAngularVel, maxTranslationalVel;

		//text can be used for special types to encode for example an object name (grasping)
		std::string helper;
	};

	/**
	 * Empty constructor, which is used for serialization
	 */
	Path();

	/**
	 * Default constructor, which generates an empty path
	 *
	 * @frame: frame name
	 */
	Path(const std::string& frame);

	/**
	 * Copy constructor
	 */
	Path(const Path& other);

	virtual ~Path();

	/**
	 * Adds a waypoint by copy.
	 *
	 * @waypoint [in]: the waypoint to add
	 * @clean [in]: call clean directly, otherwise you should
	 * 		call it by yourself
	 */
	void append(const Waypoint& waypoint,
			const bool clean = false);

	/**
	 * Adds a waypoint by copy to the front.
	 *
	 * @waypoint [in]: the waypoint to add
	 * @clean [in]: call clean directly, otherwise you should
	 * 		call it by yourself
	 */
	void prepend(const Waypoint& waypoint,
			const bool clean = false);

	/**
	 * Adds a path by copy.
	 *
	 * @path [in]: the path to add
	 * @clean [in]: call clean directly, otherwise you should
	 * 		call it by yourself
	 */
	void append(const Path& path,
			const bool clean = false);

	/**
	 * Operator to append a path
	 */
	Path& operator +=(const Path& path);

	/**
	 * Operator to append a waypoint
	 */
	Path& operator +=(const Waypoint& waypoint);

	/**
	 * Computes additional way points by linear approximation
	 */
	void makeDense(const int numberOfJointStates);

	/**
	 * Checks whether the path only contains a single
	 * waypoint, which has a type different from RobotWaypoint.
	 *
	 * @return: false, if the type is RobotWaypoint and the
	 * 		path only contains a single waypoint
	 */
	bool isSpecialCommand() const;

	/**
	 * Returns the type of the first waypoint.
	 */
	Waypoint::Type getFirstWaypointType() const;

	/**
	 * Returns the helper attribute of the first
	 * waypoint.
	 */
	std::string getFirstWaypointHelper() const;

	void clear();

	/**
	 * Return the number of waypoints
	 */
	size_t size() const;
	void resize(const size_t size);
	bool empty() const;
	void setWaypoints(const std::vector<Waypoint>& waypoints);
	const std::vector<Waypoint>& getWaypoints() const;

	/**
	 * Computes the length of the path in cartesian space.
	 * The length is only computed once and cached as long the path
	 * is not changing.
	 */
	double length();

	/**
	 * Computes the length from startWaypoint to the goal
	 * in cartesian space. This method doesn't cache the
	 * results!
	 *
	 * @startWaypoint [in]: the starts waypoint id
	 */
	double lengthToGoal(int startWaypoint);

	/**
	 * Returns a sub path which starts at startWaypoint
	 * and stops at goalWaypoint. The subpath contains
	 * all waypoints in the range, i.e., also the start
	 * and goal node. Boundary checks are performed.
	 *
	 * @startWaypoint [in]: the starts waypoint id
	 * @goalWaypoint [in]: the goal waypoint id
	 */
	boost::shared_ptr<Path> getSubPath(size_t startWaypoint,
			size_t goalWaypoint);

	/**
	 * Computes a sub path which starts at 'currentPose' and
	 * ends with the goal of 'path'. Therefore, it tries to find
	 * the nearest neighbor in 'path'. If this neighbor is closer than
	 * 'maxDistance' the algoritm removes 'removeWaypoints' from
	 * the remaining path to the goal and returns the 'subPath'.
	 * The path needs to provide joint poses!
	 *
	 * @return: true, if a nearest neighbor was found, otherwise
	 * 		the algorithm will return false
	 */
	static bool computeSubPathWhichStartsAtCurrentPose(const boost::shared_ptr<Path>& path,
			const KDL::JntArray& currentPose,
			const double maxDistance,
			const int removeWaypoints,
			boost::shared_ptr<Path>& subPath);

	std::vector<Waypoint>::iterator begin();
	std::vector<Waypoint>::const_iterator begin() const;
	std::vector<Waypoint>::iterator end();
	std::vector<Waypoint>::const_iterator end() const;

	Waypoint& operator[](const size_t index);
	const Waypoint& operator[](const size_t index) const;
	Path& operator=(const Path& other);

	const Waypoint& back() const;
	const Waypoint& front() const;
	Waypoint& back();
	Waypoint& front();

	nav_msgs::PathConstPtr getRosPath();
	void resetRosMarkers();
	const Eigen::Affine3d& getTcp() const;
	void setTcp(const Eigen::Affine3d& tcp);
	bool isUseTcp() const;
	void setUseTcp(bool useTcp);
	int getMaxId() const;
	bool isCachedPath() const;
	void setCachedPath(bool cachedPath);

	/**
	 * Removes duplicates from the waypoint list.
	 */
	void cleanPath();
	void reverse();

	double getPathLength() const;

	void transform(const Eigen::Affine3d& t);

	/**
	 * Writes the trajectory fields of the path into the
	 * file "filename"
	 */
	void writeTrajectoryData(const std::string& filename);

	static void save(const std::string& filename,
			const std::vector<KDL::JntArray>& path);

	/**
	 * Adds the path to the MarkerArray 'array' and
	 * increments the id accordingly
	 *
	 * @array [in/out]: the array to which the path messages
	 * 		will be added
	 * @id [in/out]: the counter will be incremented, such
	 * 		that it contains always the latest id
	 * @plotAxis [in]: true, if the orientation should
	 * 		be plotted
	 * @plotPath [in]: true, if path between waypoints should
	 * 		be plotted
	 */
	void getROSVisualizationMessage(visualization_msgs::MarkerArray& array,
			int& id,
			const bool plotAxis,
			const bool plotPath);

	/**
	 * Publishes the Path and its orientations as a
	 * ROS message
	 */
	void publish();

	/**
	 * Serialization
	 */
	template<class Archive>
	void serialize(Archive& ar,
			const unsigned int version)
	{
		ar & m_waypoints & const_cast<std::string &>(c_frame);
	}

private:
	std::vector<Waypoint> m_waypoints;
	double m_length;
	bool m_lengthUpdated;
	nav_msgs::PathPtr m_rosPath;
	int m_maxId;
	bool m_cachedPath;
	static ros::Publisher m_pub;
	static bool m_pubInit;

public:
	const std::string c_frame;

	//only needed for debugging. Otherwise you can safely
	//ignore it!
	bool m_debug;
};

} /* namespace prm_planner */

#endif /* PATH_H_ */
