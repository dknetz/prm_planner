/*
 * This file (kd_tree.h) is part of the "ais_definitions" packages of Daniel Kuhner.
 *
 * It is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
 *
 * created:		Jun 17, 2014
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_H_
#define ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_H_

#include <unordered_map>
#include <thread>
#include <flann/general.h>
#include <flann/util/serialization.h>

/**
 * This is needed because of a bug? in flann. There seems
 * to be no serializer for unordered maps. Without serializer
 * a compilation is not possible (at least in my case).
 */
namespace flann
{
namespace serialization
{
// serializer for std::unordered_map
template<typename K, typename V>
struct Serializer<std::unordered_map<K, V> >
{
	template<typename InputArchive>
	static inline void load(InputArchive& ar,
			std::unordered_map<K, V>& map_val)
	{
		size_t size;
		ar & size;
		for (size_t i = 0; i < size; ++i)
		{
			K key;
			ar & key;
			V value;
			ar & value;
			map_val[key] = value;
		}
	}

	template<typename OutputArchive>
	static inline void save(OutputArchive& ar,
			const std::unordered_map<K, V>& map_val)
	{
		ar & map_val.size();
		for (typename std::unordered_map<K, V>::const_iterator i = map_val.begin(); i != map_val.end(); ++i)
		{
			ar & i->first;
			ar & i->second;
		}
	}
};
}
}

#include <ais_log/log.h>
#include <flann/flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/thread/recursive_mutex.hpp>

namespace ais_point_cloud
{

template<typename PointType, typename DataType = double, typename Metric = flann::L2<DataType>>
class KdTree
{
public:
	KdTree(const std::vector<PointType>& dataset);
	KdTree();
	virtual ~KdTree();

	void setDataset(const std::vector<PointType>& dataset);
	void addPoint(const PointType& point);
	PointType& getDataPoint(const int index);
	int knnSearch(const PointType& query,
			std::vector<int>& indices,
			std::vector<DataType>& dists,
			const int nn);
	int radiusSearch(const PointType& query,
			std::vector<int>& indices,
			std::vector<DataType>& dists,
			const double radius);
	bool hasPointsInRadius(const PointType& query,
			const DataType distance);
	void clear();
	size_t size() const;
	void saveIndex(const std::string& file);
	const std::vector<PointType>* getInputDataset() const;

protected:
	template<typename T>
	void convert(const flann::Matrix<T>& dataIn,
			std::vector<T>& dataOut);
	void setParameters();

protected:
	flann::Matrix<DataType>* flannDataset;
	flann::Index<Metric>* flannIndex;
	std::vector<PointType>* inputDataset;
	flann::SearchParams params;

	mutable boost::recursive_mutex m_mutex;
};

struct SearchWrapperEigenVector
{
	SearchWrapperEigenVector()
	{
	}

	SearchWrapperEigenVector(const Eigen::Vector3d& vector) :
					vector(vector)
	{
	}

	void setData(double* data) const
			{
		data[0] = vector.x();
		data[1] = vector.y();
		data[2] = vector.z();
	}

	int getDim() const
	{
		return 3;
	}

	Eigen::Vector3d vector;
};

struct SearchWrapperEigenVectorAndNormal
{
	SearchWrapperEigenVectorAndNormal()
	{
	}

	SearchWrapperEigenVectorAndNormal(const Eigen::Vector3d& vector,
			const Eigen::Vector3d& normal) :
					vector(vector),
					normal(normal)
	{
	}

	void setData(double* data) const
			{
		data[0] = vector.x();
		data[1] = vector.y();
		data[2] = vector.z();
	}

	int getDim() const
	{
		return 3;
	}

	Eigen::Vector3d vector;
	Eigen::Vector3d normal;
};

template<typename PointType>
void convert(const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud,
		std::vector<SearchWrapperEigenVector>& out)
{
	out.clear();

	out.resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		out[i] = SearchWrapperEigenVector(Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
	}
}

template<typename PointType>
void convert(const boost::shared_ptr<pcl::PointCloud<PointType>>& cloud,
		std::vector<SearchWrapperEigenVectorAndNormal>& out)
{
	out.clear();

	out.resize(cloud->size());
	for (size_t i = 0; i < cloud->size(); ++i)
	{
		out[i] = SearchWrapperEigenVectorAndNormal(Eigen::Vector3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z),
				Eigen::Vector3d(cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z));
	}
}

struct SearchWrapperEigenVectorWithId: SearchWrapperEigenVector
{
	SearchWrapperEigenVectorWithId() :
					SearchWrapperEigenVector(),
					id(-1)
	{
	}

	SearchWrapperEigenVectorWithId(const Eigen::Vector3d& vector,
			const int id) :
					SearchWrapperEigenVector(vector),
					id(id)
	{
	}

	int getId() const
	{
		return id;
	}

	int id;
};

struct SearchWrapperEigenVectorXWithId
{
	SearchWrapperEigenVectorXWithId() :
					id(-1)
	{
	}

	SearchWrapperEigenVectorXWithId(const Eigen::VectorXd& vector,
			const int id) :
					id(id),
					vector(vector)
	{
	}

	int getId() const
	{
		return id;
	}

	void setData(double* data) const
			{
		for (int i = 0; i < vector.size(); ++i)
		{
			data[i] = vector(i);
		}
	}

	int getDim() const
	{
		return vector.size();
	}

	int id;
	Eigen::VectorXd vector;
};

} /* namespace ais_point_cloud */

template<typename PointType, typename DataType, typename Metric>
inline ais_point_cloud::KdTree<PointType, DataType, Metric>::KdTree(const std::vector<PointType>& dataset) :
				flannDataset(NULL),
				flannIndex(NULL),
				inputDataset(NULL)
{
	setParameters();
	setDataset(dataset);
}

template<typename PointType, typename DataType, typename Metric>
inline ais_point_cloud::KdTree<PointType, DataType, Metric>::KdTree() :
				flannDataset(NULL),
				flannIndex(NULL),
				inputDataset(NULL)
{
	setParameters();
}

template<typename PointType, typename DataType, typename Metric>
inline ais_point_cloud::KdTree<PointType, DataType, Metric>::~KdTree()
{
	clear();
}

template<typename PointType, typename DataType, typename Metric>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::setDataset(const std::vector<PointType>& dataset)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	clear();

	if (dataset.empty())
	{
		LOG_ERROR("The dataset is empty! Cannot create a KdTree");
		return;
	}

	int dim = dataset[0].getDim();

	inputDataset = new std::vector<PointType>(dataset.size());
	flannDataset = new flann::Matrix<DataType>(new DataType[dataset.size() * dim], dataset.size(), dim);

	for (size_t i = 0; i < dataset.size(); ++i)
	{
		dataset[i].setData(flannDataset->operator [](i));
		(*inputDataset)[i] = dataset[i];
	}

	flannIndex = new flann::Index<Metric>(*flannDataset, flann::KDTreeIndexParams(4));
	flannIndex->buildIndex();
}

template<typename PointType, typename DataType, typename Metric>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::addPoint(const PointType& point)
{
	if (flannIndex == NULL)
	{
		std::vector<PointType> points;
		points.push_back(point);
		setDataset(points);
	}
	else
	{
		int dim = point.getDim();
		flann::Matrix<DataType> data(new DataType[dim], 1, dim);
		point.setData(data[0]);

		flannIndex->addPoints(data);
		inputDataset->push_back(point);
	}
}

template<typename PointType, typename DataType, typename Metric>
inline PointType& ais_point_cloud::KdTree<PointType, DataType, Metric>::getDataPoint(const int index)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

//	assert(CHECK_VECTOR((*inputDataset), index));
	return (*inputDataset)[index];
}

template<typename PointType, typename DataType, typename Metric>
inline int ais_point_cloud::KdTree<PointType, DataType, Metric>::knnSearch(const PointType& query,
		std::vector<int>& indices,
		std::vector<DataType>& dists,
		const int nn)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (flannIndex == NULL)
	{
		return 0;
	}

	if (flannIndex->size() < nn)
	{
		return 0;
	}

	indices.resize(nn);
	dists.resize(nn);

	flann::Matrix<int> flannIndices(&indices[0], 1, nn);
	flann::Matrix<DataType> flannDists(&dists[0], 1, nn);

	std::vector<DataType> queryVector(query.getDim());
	query.setData(&queryVector[0]);

	flann::Matrix<DataType> flannQuery(&queryVector[0], 1, query.getDim());

	flannIndex->knnSearch(flannQuery, flannIndices, flannDists, nn, params);

	return indices.size();
}

template<typename PointType, typename DataType, typename Metric>
inline int ais_point_cloud::KdTree<PointType, DataType, Metric>::radiusSearch(const PointType& query,
		std::vector<int>& indices,
		std::vector<DataType>& dists,
		const double radius)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	if (flannIndex->size() == 0)
	{
		return 0;
	}

	indices.resize(flannIndex->size());
	dists.resize(flannIndex->size());

	flann::Matrix<int> flannIndices(&indices[0], 1, flannIndex->size());
	flann::Matrix<DataType> flannDists(&dists[0], 1, flannIndex->size());

	std::vector<DataType> queryVector(query.getDim());
	query.setData(&queryVector[0]);

	flann::Matrix<DataType> flannQuery(&queryVector[0], 1, query.getDim());

	int found = flannIndex->radiusSearch(flannQuery, flannIndices, flannDists, radius, params);

	indices.resize(found);
	dists.resize(found);

	return indices.size();
}

template<typename PointType, typename DataType, typename Metric>
inline bool ais_point_cloud::KdTree<PointType, DataType, Metric>::hasPointsInRadius(const PointType& query,
		const DataType distance)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	std::vector<int> indices;
	std::vector<DataType> distances;
	knnSearch(query, indices, distances, 1);
	return (indices.empty()) ? (false) : (distances[0] <= distance);
}

template<typename PointType, typename DataType, typename Metric>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::clear()
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	DELETE_VAR(flannIndex);
	DELETE_VAR(inputDataset);

	if (flannDataset != NULL)
	{
		DELETE_ARRAY2(flannDataset->ptr());
		delete flannDataset;
	}
}

template<typename PointType, typename DataType, typename Metric>
template<typename T>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::convert(const flann::Matrix<T>& dataIn,
		std::vector<T>& dataOut)
{
	dataOut.resize(dataIn.rows);
	for (size_t i = 0; i < dataIn.rows; ++i)
	{
		dataOut[i] = dataIn.ptr()[i];
	}
}

template<typename PointType, typename DataType, typename Metric>
inline size_t ais_point_cloud::KdTree<PointType, DataType, Metric>::size() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);

	return flannIndex->size();
}

template<typename PointType, typename DataType, typename Metric>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::saveIndex(const std::string& file)
{
	if (flannIndex != NULL)
	{
		flannIndex->save(file);
	}
}

template<typename PointType, typename DataType, typename Metric>
inline void ais_point_cloud::KdTree<PointType, DataType, Metric>::setParameters()
{
	params.checks = 32;
	params.cores = std::thread::hardware_concurrency();
	params.sorted = false;
}

template<typename PointType, typename DataType, typename Metric>
inline const std::vector<PointType>* ais_point_cloud::KdTree<PointType, DataType, Metric>::getInputDataset() const
{
	return inputDataset;
}

#endif /* ROBOTIC_LIBS_POINT_CLOUD_KD_TREE_H_ */
