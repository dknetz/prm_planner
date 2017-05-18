/*
 * ros_time.cpp
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#include <ais_ros/ros_time.h>

namespace ais_ros
{

RosTime::RosTime(const ros::Time& time) :
				ais_util::Time(time.sec, time.nsec)
{
}

RosTime::~RosTime()
{
}

} /* namespace ais_ros */
