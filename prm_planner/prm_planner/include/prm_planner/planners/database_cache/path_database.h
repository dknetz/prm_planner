/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 19, 2017
 *      Author: kuhnerd
 * 	  Filename: path_database.h
 */

#ifndef H16DE2496_611A_4CE2_A843_7BCF5F51114B
#define H16DE2496_611A_4CE2_A843_7BCF5F51114B
#include <ais_definitions/class.h>
#include <ais_point_cloud/kd_tree.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <ros/publisher.h>
#include <unordered_map>

namespace prm_planner
{

FORWARD_DECLARE(Path);

class PathDatabase
{
public:
	/**
	 * Constructor. Use read() to open a PathDatabase
	 * from file
	 */
	PathDatabase();

	virtual ~PathDatabase();

	/**
	 * Adds a path to the database
	 */
	bool add(boost::shared_ptr<Path>& path);

	/**
	 * Writes the database to disk give the fi
	 */
	bool write(const std::string& filename);

	/**
	 * Publishes the database as a ROS message
	 */
	void publish();

	/**
	 * Returns a data vector which contains all
	 * start and goal poses as a vector. This vector
	 * can be used to build a KD tree.
	 *
	 * @param [out]: points: the points for the KD tree
	 */
	void getKdTreeData(std::vector<ais_point_cloud::SearchWrapperEigenVectorXWithId>& points);

	/**
	 * Returns the path for the given id or a NULL pointer
	 * if the id doesn't exist.
	 */
	boost::shared_ptr<Path> getPath(const int id);

	/**
	 * Returns the size of the database
	 */
	unsigned int getSize() const;

	/**
	 * Reads the database from file
	 */
	static boost::shared_ptr<PathDatabase> read(const std::string& filename);

	/**
	 * Takes two affine poses (start and goal), converts them
	 * into two 6d vectors using Trajectry::Pose and builds
	 * a 12d vector by appending the goal to the start pose
	 *
	 * @param [in] start: the start pose
	 * @param [in] goal: the goal pose
	 * @param [out] point: the 12d vector
	 */
	static void convertToSearchPoint(const Eigen::Affine3d& start,
			const Eigen::Affine3d& goal,
			Eigen::VectorXd& point);

private:
	//all paths, which are stored in the database
	std::unordered_map<int, boost::shared_ptr<Path>> m_paths;

	int m_counter;

	//ROS
	ros::Publisher m_pub;

	//mutex
	mutable boost::shared_mutex m_mutex;
};

} /* namespace prm_planner */

#endif /* H16DE2496_611A_4CE2_A843_7BCF5F51114B */
