/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Aug 12, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_trajectory_visualizer.h
 */

#ifndef HDEA78156_CD61_4E98_BFE8_818213406B35
#define HDEA78156_CD61_4E98_BFE8_818213406B35
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <prm_planner/robot/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Path);

class RobotTrajectoryVisualizer
{
public:
	RobotTrajectoryVisualizer(boost::shared_ptr<Robot> robot,
			const std::string& type);
	virtual ~RobotTrajectoryVisualizer();

	void start();
	void setTrajectory(ArmJointPath& trajectory);
	void setTrajectory(boost::shared_ptr<Path>& path);

	void update();

private:
	void run();
	void initRobotModel(const std::string& robotDescriptionParam,
			const std::string& robotDescriptionParamNew,
			const std::string& prependString,
			boost::shared_ptr<RobotArm> robot,
			RobotStatePublisher*& statePublisher,
			ros::NodeHandle& myNodeHandle);

private:
	mutable boost::mutex m_mutex;
	boost::thread m_thread;
	boost::shared_ptr<Robot> m_robot;

	ArmJointPath m_trajectory;
	ros::NodeHandle m_nodeHandle;
	prm_planner::RobotStatePublisher* m_statePublisher;
	tf::TransformBroadcaster m_br;
	int m_counter;
	std::string m_type;
	size_t m_uniqueName;
};

} /* namespace prm_planner */

#endif /* HDEA78156_CD61_4E98_BFE8_818213406B35 */
