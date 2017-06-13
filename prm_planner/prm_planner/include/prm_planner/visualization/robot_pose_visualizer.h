/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jun 6, 2017
 *      Author: kuhnerd
 * 	  Filename: robot_pose_visualizer.h
 */

#ifndef H76F420FA_61F7_4450_AFE0_8E99BCB675A6
#define H76F420FA_61F7_4450_AFE0_8E99BCB675A6

#include <ais_definitions/class.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <prm_planner/robot/robot_state_publisher.h>
#include <tf/transform_broadcaster.h>

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Path);

class RobotPoseVisualizer
{
public:
	RobotPoseVisualizer(boost::shared_ptr<Robot> robot,
			const std::string& type);
	virtual ~RobotPoseVisualizer();

	void start();
	void setPose(KDL::JntArray& jointPose);

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

	KDL::JntArray m_pose;
	ros::NodeHandle m_nodeHandle;
	prm_planner::RobotStatePublisher* m_statePublisher;
	tf::TransformBroadcaster m_br;
	std::string m_type;
	size_t m_uniqueName;
};

} /* namespace prm_planner */

#endif /* H76F420FA_61F7_4450_AFE0_8E99BCB675A6 */
