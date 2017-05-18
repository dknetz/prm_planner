/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 9, 2017
 *      Author: kuhnerd
 * 	  Filename: point_cloud_helpers.cpp
 */

#include <ais_log/log.h>
#include <ais_point_cloud/point_cloud_helpers.h>

namespace ais_point_cloud
{
namespace point_cloud_helpers
{

bool extract(const MyPointCloudP& cloudIn,
		const pcl::PointIndicesPtr& inliers,
		MyPointCloudP& cloudOut,
		const bool inverse)
{
	assert(cloudOut);

	if (cloudIn->empty() || inliers->indices.size() > cloudIn->size())
		return false;

	MyPointCloudP cloudOutliers(new MyPointCloud);
	pcl::ExtractIndices<MyPointType> extract;
	extract.setInputCloud(cloudIn);
	extract.setIndices(inliers);
	extract.setNegative(inverse);
	extract.filter(*cloudOut);

	return true;
}

bool extract(const MyPointCloudP& cloudIn,
		const pcl::IndicesPtr& inliers,
		MyPointCloudP& cloudOut,
		const bool inverse)
{
	assert(cloudOut);

	if (cloudIn->empty() || inliers->size() > cloudIn->size())
		return false;

	MyPointCloudP cloudOutliers(new MyPointCloud);
	pcl::ExtractIndices<MyPointType> extract;
	extract.setInputCloud(cloudIn);
	extract.setIndices(inliers);
	extract.setNegative(inverse);
	extract.filter(*cloudOut);

	return true;
}

bool ransacPlane(const MyPointCloudP& cloudIn,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients,
		const double ransacThreshold)
{
	//to have a few points to work with
	if (cloudIn->size() < 50)
	{
		return false;
	}
	pcl::SACSegmentation<MyPointType> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(ransacThreshold);
	seg.setInputCloud(cloudIn);
	seg.segment(*inliers, *coefficients);
	return true;
}

bool getPlaneWithOrientation(const MyPointCloudP& cloudIn,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients,
		bool& bigEnough,
		const double& ransacThreshold,
		const int& ransacMinPoints,
		const Eigen::Vector3d& desiredNormalDirection,
		const double& maxAngleDistToNormal)
{
	//get plane and check if big enough
	if (!ransacPlane(cloudIn, inliers, coefficients, ransacThreshold)
			|| inliers->indices.size() < ransacMinPoints)
	{
		bigEnough = false;
		return false;
	}

	//get normal
	Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
	double angleToUp = acos(desiredNormalDirection.dot(normal) / (desiredNormalDirection.norm() * normal.norm()));

	//check if horizontal plane
	bigEnough = true;
	return angleToUp <= maxAngleDistToNormal || (M_PI - angleToUp) <= maxAngleDistToNormal;
}

} /* namespace point_cloud_helpers */
} /* namespace ais_point_cloud */

