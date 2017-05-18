/*
 * tf_tree.h
 *
 *  Created on: Jul 9, 2015
 *      Author: kuhnerd
 */

#ifndef ROS_UTIL_INCLUDE_ROS_UTIL_TF_TREE_H_
#define ROS_UTIL_INCLUDE_ROS_UTIL_TF_TREE_H_
#include <ais_definitions/class.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>
#include <string>

namespace ais_ros
{

class TFTree
{
SINGLETON_HEADER(TFTree)

PUBLIC_METHODS:
	virtual ~TFTree();

	virtual Eigen::Affine3d getTransformation(const std::string& from,
			const std::string& to,
			ros::Time = ros::Time(0));

	virtual Eigen::Affine3d getTransformation(const std::string& from,
			const std::string& to,
			bool& found,
			ros::Time = ros::Time(0));

	Eigen::Affine3d transform(const Eigen::Affine3d& tInA,
			const std::string& aFrame,
			const std::string& bFrame);

PRIVATE_METHODS:
	tf::TransformListener m_listener;
};

} /* namespace ais_ros */

#endif /* ROS_UTIL_INCLUDE_ROS_UTIL_TF_TREE_H_ */
