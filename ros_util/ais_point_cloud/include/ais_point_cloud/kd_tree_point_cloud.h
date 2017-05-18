/*
 * kd_tree_point_cloud.h
 *
 *  Created on: May 13, 2015
 *      Author: kuhnerd
 */

#ifndef ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_POINT_CLOUD_H_
#define ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_POINT_CLOUD_H_

#include <ais_point_cloud/kd_tree.h>
#include <ais_point_cloud/point_cloud.h>

namespace ais_point_cloud
{

class KdTreePointCloud: public KdTree<SearchWrapperEigenVector, double, flann::L2_Simple<double>>
{
public:
	typedef double DataType;
	public:
	KdTreePointCloud();
	virtual ~KdTreePointCloud();
	void setPointCloud(const MyPointCloudP& cloud);
	MyPointType& getCloudPoint(const int index);

protected:
	MyPointCloudP m_cloud;
};

inline KdTreePointCloud::KdTreePointCloud() :
				KdTree()
{
}

inline KdTreePointCloud::~KdTreePointCloud()
{
}

inline void KdTreePointCloud::setPointCloud(const MyPointCloudP& cloud)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_cloud = cloud;

	if (flannDataset != NULL)
	{
		DELETE_ARRAY2(flannDataset->ptr());
		delete flannDataset;
	}

	if (cloud->empty())
	{
		LOG_ERROR("The dataset is empty! Cannot create a KdTree");
		return;
	}

	int dim = 3;

	flannDataset = new flann::Matrix<DataType>(new DataType[cloud->size() * dim], cloud->size(), dim);
	inputDataset = new std::vector<SearchWrapperEigenVector>(cloud->size());

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		DataType* data = flannDataset->operator [](i);
		data[0] = cloud->points[i].x;
		data[1] = cloud->points[i].y;
		data[2] = cloud->points[i].z;
		inputDataset->operator [](i).vector = Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
	}

	if (flannIndex == NULL)
	{
		flannIndex = new flann::Index<flann::L2_Simple<double>>(*flannDataset, flann::KDTreeIndexParams(4));
		flannIndex->buildIndex();
	}
	else
	{
		flannIndex->buildIndex(*flannDataset);
	}
}

inline MyPointType& KdTreePointCloud::getCloudPoint(const int index)
{
	return m_cloud->points[index];
}

} /* namespace ais_point_cloud */

#endif /* ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_POINT_CLOUD_H_ */
