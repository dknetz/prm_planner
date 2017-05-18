/*
 * dropping_region.h
 *
 *  Created on: Feb 11, 2017
 *      Author: kuhnerd
 */

#ifndef H81A6D04A_C45F_4166_BE8B_9037E819F4EB
#define H81A6D04A_C45F_4166_BE8B_9037E819F4EB

#include <ais_definitions/class.h>
#include <ais_point_cloud/point_cloud.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <prm_planner/util/parameters.h>
#include <ros/publisher.h>

FORWARD_DECLARE_N(pcl, ModelCoefficients);

namespace prm_planner
{

class DroppingRegion
{
public:
	/**
	 * The point cloud 'cloud' is used to define the
	 * dropping region. 'model' are the coefficients of
	 * the plane, which was fitted into cloud.
	 */
	DroppingRegion(const ais_point_cloud::MyPointCloudP cloud,
			const boost::shared_ptr<pcl::ModelCoefficients>& model,
			const parameters::DroppingConfig& config);

	virtual ~DroppingRegion();

	void publish();

	bool sample(Eigen::Affine3d& sample);

private:
	void createGrid();
	void addPointCloudToGrid();
	void smoothGrid();
	void thresholdGrid();
	void normalizeGrid();

private:
	struct Cell
	{
		Cell();

		int counter;
		double potential;
	};

	static ros::Publisher m_publisherRegion, m_publisherConvexHall;
	static boost::atomic_bool m_publisherCreated;
	ais_point_cloud::MyPointCloudP m_cloud, m_cloudConvexHull;
	boost::shared_ptr<pcl::ModelCoefficients> m_model;
	std::vector<std::vector<Cell>> m_grid;
	std::vector<Eigen::Vector3d> m_samples;
	double m_width;
	double m_height;
	int m_binsX;
	int m_binsY;
	double m_xMin, m_yMin;
	bool m_published;

public:
	const parameters::DroppingConfig& c_config;
};

} /* namespace prm_planner */

#endif /* H81A6D04A_C45F_4166_BE8B_9037E819F4EB */
