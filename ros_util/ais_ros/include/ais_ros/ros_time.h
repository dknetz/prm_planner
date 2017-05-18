/*
 * ros_time.h
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#ifndef ROS_UTIL_INCLUDE_ROS_UTIL_ROS_TIME_H_
#define ROS_UTIL_INCLUDE_ROS_UTIL_ROS_TIME_H_

#include <ais_util/time.h>
#include <ros/time.h>

namespace ais_ros
{

class RosTime: public ais_util::Time
{
public:
	RosTime(const ros::Time& time);
	virtual ~RosTime();
};

} /* namespace ais_ros */

#endif /* ROS_UTIL_INCLUDE_ROS_UTIL_ROS_TIME_H_ */
