/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jan 9, 2017
 *      Author: kuhnerd
 * 	  Filename: point_cloud_helpers.h
 */

#ifndef HA355CD84_B085_4C56_81A6_88DCD0C13D37
#define HA355CD84_B085_4C56_81A6_88DCD0C13D37
#include <ais_point_cloud/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <Eigen/Core>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace ais_point_cloud
{
namespace point_cloud_helpers
{

/**
 * Extracts the outliers of a point cloud based on the given
 * inliers. The output cloud is a copy of the outlier points.
 */
bool extract(const MyPointCloudP& cloudIn,
		const pcl::PointIndices::Ptr& inliers,
		MyPointCloudP& cloudOut,
		const bool inverse = false);
bool extract(const MyPointCloudP& cloudIn,
		const pcl::IndicesPtr& inliers,
		MyPointCloudP& cloudOut,
		const bool inverse = false);

bool ransacPlane(const MyPointCloudP& cloudIn,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients,
		const double ransacThreshold);

bool getPlaneWithOrientation(const MyPointCloudP& cloudIn,
		pcl::PointIndices::Ptr& inliers,
		pcl::ModelCoefficients::Ptr& coefficients,
		bool& bigEnough,
		const double& ransacThreshold,
		const int& ransacMinPoints,
		const Eigen::Vector3d& desiredNormalDirection,
		const double& maxAngleDistToNormal);

} /* namespace point_cloud_helpers */
} /* namespace ais_point_cloud */

#endif /* HA355CD84_B085_4C56_81A6_88DCD0C13D37 */
