/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: executer.h
 */

#ifndef HF616C863_2302_4695_99A9_E19F643FB98B
#define HF616C863_2302_4695_99A9_E19F643FB98B
#include <boost/atomic.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Geometry>
#include <unordered_map>

#define EXECUTER_LOCK() boost::recursive_mutex::scoped_lock lock(m_mutex);

FORWARD_DECLARE_N(ais_point_cloud, EasyKDTree);

namespace prm_planner
{

FORWARD_DECLARE (Path);
FORWARD_DECLARE (RobotArm);
FORWARD_DECLARE (Controller);
FORWARD_DECLARE (Robot);

class Executer
{
public:
	typedef std::vector<boost::shared_ptr<Path>> SubPath;

public:
	Executer();
	virtual ~Executer();

	/**
	 * This method takes a path and densifies it by adding additional
	 * waypoints. Then the path is transformed into the robots frame.
	 * Finally it splits the path into pieces such that special commands
	 * as opening or closing the gripper are handled correspondingly.
	 */
	virtual bool executePath(const boost::shared_ptr<Path> path);

	/**
	 * Executes a pre processed path map. This method
	 * should be used, if you want to execute a previously
	 * stored path map.
	 */
	virtual bool executePreprocessedPathMap(const boost::shared_ptr<Path>& paths);

	/**
	 * Stops the robots' motion
	 */
	virtual void stopMotion() = 0;

	virtual bool isGoalReached() const = 0;
	virtual bool hasErrors() const = 0;

	virtual double getPathLength() = 0;
	virtual double getExecutedPathLength() = 0;

	virtual void publish() = 0;

	virtual void lock() = 0;
	virtual void unlock() = 0;
	virtual void interrupt() = 0;

	virtual void init() = 0;
	virtual void reset() = 0;

	virtual bool writeTrajectory(const std::string& filename);

	/**
	 * Set/Get
	 */
	virtual std::string getPathFileName() const;
	virtual void setPathFileName(const std::string& pathFileName);
	virtual bool isSavePath() const;
	virtual void setSavePath(bool savePath);

	/**
	 * Searializes the path map
	 */
	bool savePath(const std::string& filename,
			const boost::shared_ptr<Path>& path);

	/**
	 * Reads a path map from file and stores it
	 * in 'paths'. You can execute it by calling
	 * executePreprocessedPathMap()
	 */
	static bool readPath(const std::string& filename,
			boost::shared_ptr<Path>& path,
			Eigen::Affine3d& tcp);

protected:
	/**
	 * Splits the arm paths into pieces of RobotWaypoints.
	 * The path is split at waypoints, which have additional
	 * information (e.g., Grasping or Droping). The thread
	 * needs to keep track of the current path segments
	 * and needs to switch to the next segment, if the
	 * additional action was finished.
	 *
	 * The executer assumes, that the pathes segments are
	 * the same for all robots, i.e., if there is a special
	 * waypoint for one robot, there are the same special
	 * waypoint for all other robots. To just open or close
	 * the gripper of a single robot, you have to send a path
	 * via executeRobotPaths() which only contains commands
	 * for a single robot. Otherwise the behavior is
	 * unpredictable.
	 *
	 * @paths [in]: the arm path
	 * @splittedPaths [out]: A vector of arm poses
	 */
	virtual void splitPath(const boost::shared_ptr<Path>& path,
			SubPath& splittedPath);

	/**
	 * Actually sends the paths to the robot. It estimates
	 * the runtime for each arm and sets the runtime of
	 * all controllers to the maximum runtime of the slowest
	 * controller, such that all arms have approximately
	 * the same task pose.
	 *
	 * @paths [in]: the sub paths
	 * @currentIndex [in]: current path segment size
	 * @controllers [in/out]: the controllers.
	 */
	virtual void sendPath(SubPath& paths,
			const int currentIndex,
			Controller* controller);

	/**
	 * Handles special commands which are encoded as
	 * a path with a single waypoint and special attributes.
	 *
	 * @pathSegments [in]: the map of sub pathes
	 * @currentIndex [in]: the index in the path vectors
	 */
	virtual void handleSpecialCommand(SubPath& pathSegments,
			int currentIndex);

protected:
	mutable boost::recursive_mutex m_mutex;

	//which of the current path segments to use
	boost::atomic_int m_currentPathSegment;

	//the path segments (@see splitPaths)
	SubPath m_pathSegments;

	//the robot and its arms
	boost::shared_ptr<Robot> m_robot;

	bool m_savePath;
	std::string m_pathFileName;

	mutable boost::recursive_mutex m_executedTrajectoryMutex;
	std::vector<KDL::JntArray> m_executedTrajectory;
};

}
/* namespace prm_planner */

#endif /* HF616C863_2302_4695_99A9_E19F643FB98B */
