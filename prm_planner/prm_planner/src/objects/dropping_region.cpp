/*
 * dropping_region.cpp
 *
 *  Created on: Feb 11, 2017
 *      Author: kuhnerd
 */

#include <ais_definitions/exception.h>
#include <ais_point_cloud/point_cloud.h>
#include <ais_util/color.h>
#include <pcl/ModelCoefficients.h>
#include <prm_planner/objects/dropping_region.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/surface/convex_hull.h>

namespace prm_planner
{

boost::atomic_bool DroppingRegion::m_publisherCreated(false);
ros::Publisher DroppingRegion::m_publisherRegion;
ros::Publisher DroppingRegion::m_publisherConvexHall;

DroppingRegion::DroppingRegion(const ais_point_cloud::MyPointCloudP cloud,
		const boost::shared_ptr<pcl::ModelCoefficients>& model,
		const parameters::DroppingConfig& config) :
				m_cloud(cloud),
				m_model(model),
				c_config(config),
				m_published(false)
{
	if (!m_publisherCreated)
	{
		ros::NodeHandle n("");
		m_publisherRegion = n.advertise<visualization_msgs::MarkerArray>("dropping/region", 1);
		m_publisherConvexHall = n.advertise<visualization_msgs::Marker>("dropping/convex_hull", 1);
		m_publisherCreated = true;
	}

	createGrid();
	addPointCloudToGrid();
	smoothGrid();
	normalizeGrid();
	thresholdGrid();
	publish();
}

DroppingRegion::Cell::Cell() :
				counter(1),
				potential(1)
{
}

DroppingRegion::~DroppingRegion()
{
}

void DroppingRegion::publish()
{
	const std::string& frame = ProblemDefinitionManager::getInstance()->getProblemDefinition()->getConfig().planningFrame;

	try
	{
		if (m_publisherRegion.getNumSubscribers() > 0 && !m_published)
		{
			visualization_msgs::MarkerArray array;

			int id = 0;
			for (size_t x = 0; x < m_binsX; ++x)
			{
				double xMin = m_xMin + x * c_config.regionResolution;
				double xMax = m_xMin + (x + 1) * c_config.regionResolution;
				for (size_t y = 0; y < m_binsY; ++y)
				{
					double yMin = m_yMin + y * c_config.regionResolution;
					double yMax = m_yMin + (y + 1) * c_config.regionResolution;

					const Cell& cell = m_grid[x][y];

					Eigen::Vector3d p1(xMin, yMin,
							-(m_model->values[0] * xMin + m_model->values[1] * yMin + m_model->values[3]) / m_model->values[2]);
					Eigen::Vector3d p2(xMin, yMax,
							-(m_model->values[0] * xMin + m_model->values[1] * yMax + m_model->values[3]) / m_model->values[2]);
					Eigen::Vector3d p3(xMax, yMin,
							-(m_model->values[0] * xMax + m_model->values[1] * yMin + m_model->values[3]) / m_model->values[2]);
					Eigen::Vector3d p4(xMax, yMax,
							-(m_model->values[0] * xMax + m_model->values[1] * yMax + m_model->values[3]) / m_model->values[2]);

					//create marker
					visualization_msgs::Marker fieldMarker;
					fieldMarker.header.frame_id = frame; //we use the name, which is equal to the frame
					fieldMarker.header.stamp = ros::Time::now();
					fieldMarker.id = id;
					fieldMarker.ns = "fields";
					fieldMarker.type = visualization_msgs::Marker::TRIANGLE_LIST;

					fieldMarker.color = ais_util::Color::scale(cell.potential, ais_util::Color::ColorScaleRedGreen).toROSMsg();
					fieldMarker.scale.x = fieldMarker.scale.y = fieldMarker.scale.z = 1.0;

					geometry_msgs::Point triangleP1;
					triangleP1.x = p1.x();
					triangleP1.y = p1.y();
					triangleP1.z = p1.z();

					geometry_msgs::Point triangleP2;
					triangleP2.x = p2.x();
					triangleP2.y = p2.y();
					triangleP2.z = p2.z();

					geometry_msgs::Point triangleP3;
					triangleP3.x = p3.x();
					triangleP3.y = p3.y();
					triangleP3.z = p3.z();

					geometry_msgs::Point triangleP4;
					triangleP4.x = p4.x();
					triangleP4.y = p4.y();
					triangleP4.z = p4.z();

					fieldMarker.points.push_back(triangleP3);
					fieldMarker.points.push_back(triangleP2);
					fieldMarker.points.push_back(triangleP1);

					fieldMarker.points.push_back(triangleP4);
					fieldMarker.points.push_back(triangleP2);
					fieldMarker.points.push_back(triangleP3);

					array.markers.push_back(fieldMarker);

					++id;
				}
			}
			m_publisherRegion.publish(array);
			m_published = true;
		}

		if (m_publisherConvexHall.getNumSubscribers() > 0)
		{
			//convex hull
			if (m_cloudConvexHull.get() != NULL && !m_cloudConvexHull->empty())
			{
				visualization_msgs::Marker hull;
				hull.header.frame_id = frame; //we use the name, which is equal to the frame
				hull.header.stamp = ros::Time::now();
				hull.id = 0;
				hull.ns = "hull";
				hull.type = visualization_msgs::Marker::LINE_STRIP;
				hull.color = ais_util::Color::blue().toROSMsg();
				hull.scale.x = 0.02;

				for (auto& p : m_cloudConvexHull->points)
				{
					geometry_msgs::Point _p;
					_p.x = p.x;
					_p.y = p.y;
					_p.z = -(m_model->values[0] * _p.x + m_model->values[1] * _p.y + m_model->values[3]) / m_model->values[2];

					hull.points.push_back(_p);
				}

				geometry_msgs::Point _p;
				_p.x = m_cloudConvexHull->points[0].x;
				_p.y = m_cloudConvexHull->points[0].y;
				_p.z = -(m_model->values[0] * _p.x + m_model->values[1] * _p.y + m_model->values[3]) / m_model->values[2];

				hull.points.push_back(_p);

				m_publisherConvexHall.publish(hull);
			}
		}
	}
	catch (ais_definitions::Exception& ex)
	{
//		LOG_ERROR("Exception: " << ex.what());
	}
}

void DroppingRegion::createGrid()
{
	double xMax;
	double yMax;

	m_xMin = m_yMin = std::numeric_limits<double>::max();
	xMax = yMax = std::numeric_limits<double>::lowest();

	for (auto& p : m_cloud->points)
	{
		if (p.x < m_xMin)
			m_xMin = p.x;
		else if (p.x > xMax)
			xMax = p.x;

		if (p.y < m_yMin)
			m_yMin = p.y;
		else if (p.y > yMax)
			yMax = p.y;
	}

	m_width = xMax - m_xMin;
	m_height = yMax - m_yMin;

	m_binsX = ceil(m_width / c_config.regionResolution);
	m_binsY = ceil(m_height / c_config.regionResolution);

	//create and initialize grid
	m_grid.resize(m_binsX);
	for (size_t x = 0; x < m_binsX; ++x)
	{
		m_grid[x].resize(m_binsY);
	}
}

void DroppingRegion::addPointCloudToGrid()
{
	//compute convex hull
	m_cloudConvexHull.reset(new pcl::PointCloud<ais_point_cloud::MyPointType>);
	pcl::ConvexHull<ais_point_cloud::MyPointType> chull;
	chull.setDimension(2);
	chull.setInputCloud(m_cloud);
	chull.reconstruct(*m_cloudConvexHull);

	//used to 'normalize' the fact the points which are
	//further away are have lower density
	for (auto& p : m_cloud->points)
	{
		int x = (p.x - m_xMin) / c_config.regionResolution;
		int y = (p.y - m_yMin) / c_config.regionResolution;

		Cell& cell = m_grid[x][y];

		if (cell.counter < c_config.regionMaxNumberOfPoints)
			++cell.counter;
	}

	//grow regions around objects
	std::vector<std::vector<Cell>> grid(m_binsX, std::vector<Cell>(m_binsY));
	for (int x = 0; x < m_binsX; ++x)
	{
		for (int y = 0; y < m_binsY; ++y)
		{
			Cell& cell = m_grid[x][y];

			//already in a region which was visited
			if (grid[x][y].counter == 0)
				continue;

			if (cell.counter < 5)
			{
				for (int xWindow = std::max(0, x - c_config.regionObjectGrowingFactor);
						xWindow < std::min(m_binsX, x + c_config.regionObjectGrowingFactor); ++xWindow)
					for (int yWindow = std::max(0, y - c_config.regionObjectGrowingFactor);
							yWindow < std::min(m_binsY, y + c_config.regionObjectGrowingFactor); ++yWindow)
						if (sqrt((xWindow - x) * (xWindow - x) + (yWindow - y) * (yWindow - y)) < c_config.regionObjectGrowingFactor)
							grid[xWindow][yWindow].counter = 0;
			}
			else
			{
				grid[x][y].counter = cell.counter;
				grid[x][y].potential = cell.potential;
			}
		}
	}
	m_grid = grid;

	for (int x = 0; x < m_binsX; ++x)
	{
		for (int y = 0; y < m_binsY; ++y)
		{
			double _y = m_yMin + y * c_config.regionResolution + c_config.regionResolution * 0.5;
			double _x = m_xMin + x * c_config.regionResolution + c_config.regionResolution * 0.5;

			//check if point in or out
			bool inside = true;
			for (size_t i = 0; i < m_cloudConvexHull->points.size(); ++i)
			{
				const auto& p1 = m_cloudConvexHull->points[i];
				const auto& p2 = (i == m_cloudConvexHull->points.size() - 1) ? m_cloudConvexHull->points[0] : m_cloudConvexHull->points[i + 1];

				//signed distance line point
				const double d = ((p1.y - p2.y) * _x + (p2.x - p1.x) * _y + (p1.x * p2.y - p2.x * p1.y))
						/ sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));

				if (d > -c_config.regionMargin)
				{
					inside = false;
					break;
				}
			}

			if (!inside)
				m_grid[x][y].counter = 0;
		}
	}

	int maxCounter = 1;
	for (auto& itX : m_grid)
		for (auto& itY : itX)
			if (itY.counter > maxCounter)
				maxCounter = itY.counter;

	for (auto& itX : m_grid)
		for (auto& itY : itX)
			itY.potential = (double) itY.counter / (double) maxCounter;
}

void DroppingRegion::smoothGrid()
{
	const int filterSize = c_config.regionSmoothingWindow;
	const int normalizer = (filterSize * 2 + 1) * (filterSize * 2 + 1);
	for (int x = 0; x < m_binsX; ++x)
	{
		for (int y = 0; y < m_binsY; ++y)
		{
			double sum = 0;
			for (int xWindow = std::max(0, x - filterSize); xWindow < std::min(m_binsX, x + filterSize); ++xWindow)
				for (int yWindow = std::max(0, y - filterSize); yWindow < std::min(m_binsY, y + filterSize); ++yWindow)
					sum += m_grid[xWindow][yWindow].potential;

			//because we always devide by the same normalizer
			//the cell at edges should have lower weight
			m_grid[x][y].potential = sum / normalizer;
		}
	}
}

void DroppingRegion::thresholdGrid()
{
	for (int x = 0; x < m_binsX; ++x)
	{
		for (int y = 0; y < m_binsY; ++y)
		{
			Cell& cell = m_grid[x][y];

			if (cell.potential > c_config.regionPotentialTreatedAsSafe)
			{
				cell.potential = 1.0;

				double _y = m_yMin + y * c_config.regionResolution + c_config.regionResolution * 0.5;
				double _x = m_xMin + x * c_config.regionResolution + c_config.regionResolution * 0.5;

				Eigen::Vector3d p(_x, _y,
						-(m_model->values[0] * _x + m_model->values[1] * _y + m_model->values[3]) / m_model->values[2]);

				m_samples.push_back(p);
			}
		}
	}
}

bool DroppingRegion::sample(Eigen::Affine3d& sample)
{
	if (m_samples.empty())
	{
		return false;
	}

	static std::mt19937 gen(time(0));

	std::uniform_int_distribution<int> dist(0, m_samples.size());
	sample.setIdentity();
	sample.translation() = m_samples[dist(gen)];
	return true;
}

void DroppingRegion::normalizeGrid()
{
	double max = 0;
	double min = std::numeric_limits<double>::max();
	for (auto& itX : m_grid)
		for (auto& itY : itX)
		{
			if (itY.potential > max)
				max = itY.potential;

			if (itY.potential < min)
				min = itY.potential;
		}

	double range = max - min;
	for (auto& itX : m_grid)
		for (auto& itY : itX)
			itY.potential = (itY.potential - min) / range;
}

} /* namespace prm_planner */

