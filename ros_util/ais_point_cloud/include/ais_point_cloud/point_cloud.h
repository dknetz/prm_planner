/*
 * Copyright (c) 2013 Daniel Kuhner.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Daniel Kuhner - kuhnerd@informatik.uni-freiburg.de
 */

#ifndef ROBOTIC_LIBS_POINT_CLOUD_POINT_CLOUD_H_
#define ROBOTIC_LIBS_POINT_CLOUD_POINT_CLOUD_H_

#include <ais_util/color.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>

namespace ais_point_cloud
{

typedef pcl::PointXYZRGB MyPointType;
typedef pcl::PointNormal MyPointTypeNormal;
typedef pcl::PointXYZRGBNormal MyPointTypeNormalRGB;
typedef pcl::PointXYZ MyPointTypeXYZ;
typedef pcl::RGB MyPointTypeRGBOnly;

typedef pcl::PointCloud<MyPointType> MyPointCloud;
typedef pcl::PointCloud<MyPointTypeNormal> MyPointCloudNormal;
typedef pcl::PointCloud<MyPointTypeNormalRGB> MyPointCloudNormalRGB;
typedef pcl::PointCloud<MyPointTypeXYZ> MyPointCloudXYZ;
typedef pcl::PointCloud<MyPointTypeRGBOnly> MyPointCloudRGBOnly;

typedef MyPointCloud::Ptr MyPointCloudP;
typedef MyPointCloudNormal::Ptr MyPointCloudNormalP;
typedef MyPointCloudNormalRGB::Ptr MyPointCloudNormalRGBP;
typedef MyPointCloudXYZ::Ptr MyPointCloudXYZP;
typedef MyPointCloudRGBOnly::Ptr MyPointCloudRGBOnlyP;

template<typename PointType>
void convert(const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud,
		std::vector<Eigen::Vector3d>& out)
{
	out.clear();

	for (const auto& it : *cloud)
	{
		out.push_back(Eigen::Vector3d(it.x, it.y, it.z));
	}
}

inline MyPointTypeNormalRGB convert(const Eigen::Vector3d& point,
		const Eigen::Vector3d& normal,
		const ais_util::Color& color)
{
	MyPointTypeNormalRGB p;
	p.x = point.x();
	p.y = point.y();
	p.z = point.z();
	p.normal_x = normal.x();
	p.normal_y = normal.y();
	p.normal_z = normal.z();
	p.r = color.rInt();
	p.g = color.gInt();
	p.b = color.bInt();
	return p;
}

inline Eigen::Vector3d transformPoint(const Eigen::Affine3d& affine,
		const Eigen::Vector3d& point)
{
	Eigen::Vector4d pH(point.x(), point.y(), point.z(), 1);
	Eigen::Vector4d resH = affine * pH;
	return resH.head(3);
}

inline Eigen::Vector3d transformVector(const Eigen::Affine3d& affine,
		const Eigen::Vector3d& vector)
{
	Eigen::Vector4d pH(vector.x(), vector.y(), vector.z(), 0);
	Eigen::Vector4d resH = affine * pH;
	return resH.head(3);
}

} /* namespace ais_point_cloud */

#endif /* ROBOTIC_LIBS_POINT_CLOUD_POINT_CLOUD_H_ */
