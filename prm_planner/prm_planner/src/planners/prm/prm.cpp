/*
 * Copyright (c) 2015 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 17, 2015
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: prm.cpp
 */

#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <ais_util/math.h>
#include <ais_util/progress_bar.h>
#include <ais_util/stop_watch.h>
#include <ais_util/thread.h>
#include <ais_util/color.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <eigen_conversions/eigen_kdl.h>
#include <prm_planner_constraints/constraints.h>
#include <prm_planner/planners/prm/prm.h>
#include <prm_planner/planners/prm/prm_edge.h>
#include <prm_planner/planners/prm/prm_node.h>
#include <prm_planner/planners/prm/prm_view.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner_robot/defines.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner_robot/trajectory.h>
#include <prm_planner/util/parameters.h>
#include <iostream>
#include <fstream>
#include <octomap/OcTree.h>
#include <prm_planner/util/parameter_server.h>

#define PRM_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex)
#define PRM_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex)
#define PRM_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex)
#define PRM_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex)

namespace prm_planner
{

PRM::PRM(const boost::shared_ptr<Robot>& robot,
		const parameters::PRMConfig& config,
		boost::shared_ptr<ProblemDefinition> pd,
		bool loadedFromFile) :
				c_config(config),
				m_robot(robot),
				m_initialized(false),
				m_pd(pd),
				c_frame(m_pd->getFrame()),
				c_loadedFromFile(loadedFromFile),
				m_nextIdEdge(0),
				m_nextIdNode(0),
				c_needToDetermineStartsAndGoals(false)
{
	init();
	update(m_octomap); //empty octomap and kdtree for sampling
}

PRM::PRM(const boost::shared_ptr<Robot>& robot,
		const double edgeVisibilityMaxRange,
		boost::shared_ptr<ProblemDefinition> pd,
		PRMNodeMap nodes,
		PRMEdgeMap edges) :
				m_robot(robot),
				m_initialized(true),
				m_pd(pd),
				c_frame(m_pd->getFrame()),
				c_loadedFromFile(false),
				c_needToDetermineStartsAndGoals(true)
{
	c_config.size = m_nodes.size();
	c_config.visibilityDistance = edgeVisibilityMaxRange;
	resetNodesAndEdges(nodes, edges);
}

PRM::~PRM()
{
	DELETE_MAP(m_nodes);
	DELETE_MAP(m_edges);
}

void PRM::update(const boost::shared_ptr<octomap::OcTree> octree)
{
	PRM_WRITE_LOCK();

	m_octomap = octree;

	//initialize if required
	if (!m_initialized && !c_loadedFromFile)
	{
		sampleNodes();
		connectNodes();
		cleanUpPRM();
		m_initialized = true;
	}

	updateVisibilities();
}

void PRM::sampleNodes()
{
	ais_util::ProgressBar progress("Sampling nodes", c_config.size);
	for (size_t i = 0; i < c_config.size; ++i)
	{
		Eigen::Affine3d pose = m_pd->samplePose();
		if (pose.translation().z() < c_config.minHeight)
		{
			--i;
			continue;
		}
		progress.increment();
		PRMNode* node = new PRMNode(pose, m_nextIdNode);

		m_nodes[node->getId()] = node;
	}
	progress.finish();
}

void PRM::connectNodes()
{
	const int n = m_nodes.size();
	const int numberOfEdges = (n * (n + 1)) / 2.0 - n;
	const int maxTests = 50;
	const int successfulTests = 15;

	ais_util::ProgressBar progress("prm update", numberOfEdges);

//#	pragma omp parallel
//#	pragma omp single nowait
	{
		for (auto it = m_nodes.begin(); it != m_nodes.end(); ++it)
		{
//#			pragma omp task
			{
				KDL::JntArray randomState, ikSolution;
				KDL::Frame kdlPose;

				PRMNode* n1 = (*it).second;
				Eigen::Vector3d position1 = n1->getPosition();

				auto it2 = it;
				++it2;

				for (; it2 != m_nodes.end(); ++it2)
				{
					PRMNode* n2 = (*it2).second;
					Eigen::Vector3d position2 = n2->getPosition();

//					if (isVisible(position1, position2))
					if ((position1 - position2).norm() < c_config.visibilityDistance)
					{
						PRMEdge* edge = new PRMEdge(n1, n2, m_nextIdEdge);

//#						pragma omp critical
//						{
						m_edges[edge->getId()] = edge;
						n1->addEdge(edge);
						n2->addEdge(edge);
//						}
					}

					progress.increment();
				}
			}
		}
	}

	progress.finish();

	LOG_INFO("Found " << m_edges.size() << " edges");
}

void PRM::publish()
{
	static const std_msgs::ColorRGBA green = ais_util::Color::green().toROSMsg();
	static const std_msgs::ColorRGBA red = ais_util::Color::red().toROSMsg();
	static const std_msgs::ColorRGBA orange = ais_util::Color::orange().toROSMsg();
	static const std_msgs::ColorRGBA blue = ais_util::Color::blue().toROSMsg();

	//prm
	if (m_pubMarker.getNumSubscribers() > 0)
	{
		PRM_READ_LOCK();

		visualization_msgs::MarkerArray markers;

		int id = 0;

		//nodes
		visualization_msgs::Marker nodeMarker;
		nodeMarker.header.frame_id = c_frame;
		nodeMarker.header.stamp = ros::Time();
		nodeMarker.ns = "nodes";
		nodeMarker.id = id++;
		nodeMarker.type = visualization_msgs::Marker::POINTS;
		nodeMarker.action = visualization_msgs::Marker::ADD;
		nodeMarker.scale.x = 0.02;
		nodeMarker.scale.y = 0.02;
		nodeMarker.scale.z = 0.02;
		nodeMarker.color = blue;

		visualization_msgs::Marker nodeXMarker;
		nodeXMarker.header.frame_id = c_frame;
		nodeXMarker.header.stamp = ros::Time();
		nodeXMarker.ns = "nodes_x";
		nodeXMarker.id = id++;
		nodeXMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeXMarker.action = visualization_msgs::Marker::ADD;
		nodeXMarker.scale.x = 0.005;
		nodeXMarker.color = red;

		visualization_msgs::Marker nodeYMarker;
		nodeYMarker.header.frame_id = c_frame;
		nodeYMarker.header.stamp = ros::Time();
		nodeYMarker.ns = "nodes_y";
		nodeYMarker.id = id++;
		nodeYMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeYMarker.action = visualization_msgs::Marker::ADD;
		nodeYMarker.scale.x = 0.005;
		nodeYMarker.color = green;

		visualization_msgs::Marker nodeZMarker;
		nodeZMarker.header.frame_id = c_frame;
		nodeZMarker.header.stamp = ros::Time();
		nodeZMarker.ns = "nodes_z";
		nodeZMarker.id = id++;
		nodeZMarker.type = visualization_msgs::Marker::LINE_LIST;
		nodeZMarker.action = visualization_msgs::Marker::ADD;
		nodeZMarker.scale.x = 0.005;
		nodeZMarker.color = blue;

		for (auto& node : m_nodes)
		{
			const Eigen::Vector3d t = node.second->getPosition();
			const Eigen::Matrix3d rotation = node.second->getPose().rotation();
			const Eigen::Vector3d x = rotation.col(0);
			const Eigen::Vector3d y = rotation.col(1);
			const Eigen::Vector3d z = rotation.col(2);

			//position
			geometry_msgs::Point p;
			p.x = t.x();
			p.y = t.y();
			p.z = t.z();
			nodeMarker.points.push_back(p);
			nodeMarker.colors.push_back(blue);

			//x axis
			geometry_msgs::Point px;
			px.x = t.x() + x.x() * 0.03;
			px.y = t.y() + x.y() * 0.03;
			px.z = t.z() + x.z() * 0.03;
			nodeXMarker.points.push_back(p);
			nodeXMarker.points.push_back(px);

			//y axis
			geometry_msgs::Point py;
			py.x = t.x() + y.x() * 0.03;
			py.y = t.y() + y.y() * 0.03;
			py.z = t.z() + y.z() * 0.03;
			nodeYMarker.points.push_back(p);
			nodeYMarker.points.push_back(py);

			//z axis
			geometry_msgs::Point pz;
			pz.x = t.x() + z.x() * 0.03;
			pz.y = t.y() + z.y() * 0.03;
			pz.z = t.z() + z.z() * 0.03;
			nodeZMarker.points.push_back(p);
			nodeZMarker.points.push_back(pz);
		}
		markers.markers.push_back(nodeMarker);
		markers.markers.push_back(nodeXMarker);
		markers.markers.push_back(nodeYMarker);
		markers.markers.push_back(nodeZMarker);

		//edges
		visualization_msgs::Marker edgeMarker;
		edgeMarker.header.frame_id = c_frame;
		edgeMarker.header.stamp = ros::Time();
		edgeMarker.ns = "edges_active";
		edgeMarker.id = id++;
		edgeMarker.type = visualization_msgs::Marker::LINE_LIST;
		edgeMarker.action = visualization_msgs::Marker::ADD;
		edgeMarker.scale.x = 0.002;
		edgeMarker.color = red;

		visualization_msgs::Marker edgeMarkerInactive;
		edgeMarkerInactive.header.frame_id = c_frame;
		edgeMarkerInactive.header.stamp = ros::Time();
		edgeMarkerInactive.ns = "edges_inactive";
		edgeMarkerInactive.id = id++;
		edgeMarkerInactive.type = visualization_msgs::Marker::LINE_LIST;
		edgeMarkerInactive.action = visualization_msgs::Marker::ADD;
		edgeMarkerInactive.scale.x = 0.002;
		edgeMarkerInactive.color = green;

		visualization_msgs::Marker edgeMarkerInvisible;
		edgeMarkerInvisible.header.frame_id = c_frame;
		edgeMarkerInvisible.header.stamp = ros::Time();
		edgeMarkerInvisible.ns = "edges_invisible";
		edgeMarkerInvisible.id = id++;
		edgeMarkerInvisible.type = visualization_msgs::Marker::LINE_LIST;
		edgeMarkerInvisible.action = visualization_msgs::Marker::ADD;
		edgeMarkerInvisible.scale.x = 0.002;
		edgeMarkerInvisible.color = orange;

		visualization_msgs::Marker* marker;
		std_msgs::ColorRGBA const* color;
		for (auto& edge : m_edges)
		{
//			if (edge.second->isBlocked())
//			{
//				marker = &edgeMarkerInvisible;
//				color = &orange;
//			}
//			else if (edge.second->wasControllerSuccessfull())
//			{
			marker = &edgeMarker;
			color = &green;
//			}
//			else
//			{
//				marker = &edgeMarkerInactive;
//				color = &red;
//			}

			const Eigen::Vector3d pos1 = edge.second->getNode1()->getPosition();
			const Eigen::Vector3d pos2 = edge.second->getNode2()->getPosition();
			geometry_msgs::Point p1, p2;
			p1.x = pos1.x();
			p1.y = pos1.y();
			p1.z = pos1.z();
			marker->points.push_back(p1);
			p2.x = pos2.x();
			p2.y = pos2.y();
			p2.z = pos2.z();
			marker->points.push_back(p2);
			marker->colors.push_back(*color);
		}
		markers.markers.push_back(edgeMarker);
		markers.markers.push_back(edgeMarkerInactive);
		markers.markers.push_back(edgeMarkerInvisible);

		m_pubMarker.publish(markers);
	}
}

PRMView* PRM::getShallowDataCopy()
{
	PRM_READ_LOCK();

	PRMView* data = new PRMView(m_nextIdNode, m_nextIdEdge);

	for (auto& it : m_nodes)
	{
		PRMView::NodeData& nodeData = data->m_nodeData[it.first];
		nodeData.m_edges = it.second->m_edges;
		nodeData.m_node = it.second;
	}

	for (auto& it : m_edges)
	{
		PRMView::EdgeData& edgeData = data->m_edgeData[it.first];
		edgeData.m_controllerSuccessful = true;
		edgeData.m_edge = it.second;
		edgeData.m_isBlocked = it.second->m_isBlocked;
	}

	data->m_prm = this;

	return data;
}

PRM* PRM::load(const parameters::PRMConfig& config,
		const boost::shared_ptr<Robot>& robot,
		const boost::shared_ptr<ProblemDefinition>& pd)
{
	struct Container
	{
		PRMNode* node;
		std::vector<int> edges;
	};

	std::ifstream file(config.filename);
	if (!file.is_open())
	{
		LOG_FATAL("Cannot open file " << config.filename);
	}

	size_t edges, nodes, nodeEdges, jointSize;
	PRMNode::Type nodeType;
	int nodeTypeI;
	Eigen::Affine3d pose;
	KDL::JntArray joints;
	int nodeId;
	int prmVersion;
	std::vector<int> nodeEdgeIds;
	std::unordered_map<int, Container> nodeMap;
	int nodeId1, nodeId2, edgeId;
	int sizeSampling;
	double edgeVisibilityMaxRange;
	std::string frame;
	pose.setIdentity();

	file >> prmVersion;
	if (prmVersion != PRM_VERSION)
	{
		LOG_ERROR("Cannot read the prm file because it was generated with an "
				"another version of the PRM class. Please delete it and/or generate "
				"a new PRM file using the newest version.");
		return NULL;
	}

	file >> sizeSampling >> edgeVisibilityMaxRange;
	std::getline(file, frame); //first just jumps to the next line
	std::getline(file, frame);
	file >> nodes;

	PRM* prm = new PRM(robot, config, pd, true);

	for (size_t i = 0; i < nodes; ++i)
	{
		file >> nodeId >> nodeTypeI >> nodeEdges;
		nodeType = (PRMNode::Type) nodeTypeI;
		nodeEdgeIds.resize(nodeEdges);
		for (size_t j = 0; j < nodeEdges; ++j)
		{
			file >> nodeEdgeIds[j];
		}
		file >> pose(0, 0) >> pose(0, 1) >> pose(0, 2) >> pose(0, 3)
				>> pose(1, 0) >> pose(1, 1) >> pose(1, 2) >> pose(1, 3)
				>> pose(2, 0) >> pose(2, 1) >> pose(2, 2) >> pose(2, 3);
		PRMNode* node = new PRMNode(nodeId, prm->m_nextIdNode, nodeType);
		node->m_pose = pose;
		Container c;
		c.edges = nodeEdgeIds;
		c.node = node;
		nodeMap[nodeId] = c;
	}
	file >> edges;

	for (size_t i = 0; i < edges; ++i)
	{
		file >> edgeId >> nodeId1 >> nodeId2;

		PRMEdge* edge = new PRMEdge(nodeMap[nodeId1].node, nodeMap[nodeId2].node, edgeId, prm->m_nextIdEdge);
		prm->m_edges[edge->m_id] = edge;
	}

	for (auto& it : nodeMap)
	{
		for (auto& it2 : it.second.edges)
		{
			it.second.node->m_edges.push_back(prm->m_edges[it2]);
		}
		prm->m_nodes[it.second.node->getId()] = it.second.node;
	}

	prm->m_initialized = true;

	return prm;
}

void PRM::save(const std::string& filename)
{
	PRM_READ_LOCK();

	LOG_INFO("Saving PRM");

	std::ofstream file(filename);
	if (!file.is_open())
	{
		LOG_FATAL("Cannot open file " << filename);
	}

	file << PRM_VERSION << "\n" << c_config.size << "\n" << c_config.visibilityDistance
			<< "\n" << c_frame << "\n" << m_nodes.size() << " ";
	for (auto& it : m_nodes)
	{
		file << it.second->getId() << " " << it.second->getType() << " " << it.second->m_edges.size() << " ";
		for (auto& it2 : it.second->m_edges)
		{
			file << it2->m_id << " ";
		}
		file << it.second->m_pose(0, 0) << " " << it.second->m_pose(0, 1) << " " << it.second->m_pose(0, 2) << " " << it.second->m_pose(0, 3) << " "
				<< it.second->m_pose(1, 0) << " " << it.second->m_pose(1, 1) << " " << it.second->m_pose(1, 2) << " " << it.second->m_pose(1, 3)
				<< " "
				<< it.second->m_pose(2, 0) << " " << it.second->m_pose(2, 1) << " " << it.second->m_pose(2, 2) << " " << it.second->m_pose(2, 3)
				<< " ";
	}
	file << "\n" << m_edges.size() << "\n";
	for (auto& it : m_edges)
	{
		if (!CHECK_MAP(m_nodes, it.second->getNode1()->getId()) || !CHECK_MAP(m_nodes, it.second->getNode2()->getId()))
		{
			LOG_FATAL("node doesn't exist");
		}
		file << it.second->getId() << " " << it.second->m_node1->getId() << " "
				<< it.second->m_node2->getId()
				<< " ";
	}
	file.close();
}

bool PRM::isVisible(const Eigen::Vector3d& pos1,
		const Eigen::Vector3d& pos2) const
		{
	return isVisible(pos1, pos2, c_config.visibilityDistance);
}

bool PRM::isVisible(const Eigen::Vector3d& pos1,
		const Eigen::Vector3d& pos2,
		const double maxRange) const
		{
	//same node
	if (pos1.isApprox(pos2))
	{
		return true;
	}

	Eigen::Vector3d direction = pos2 - pos1;
	double lengthDir = direction.norm();

	return lengthDir < maxRange;

//	if (lengthDir > maxRange)
//	{
//		return false;
//	}

//	if (m_octomap.get() == NULL)
//		return true;
//
//	octomap::point3d octoDirection(direction.x(), direction.y(), direction.z());
//	octomap::point3d octoPos1(pos1.x(), pos1.y(), pos1.z());
//	octomap::point3d octoEnd;
//	if (!m_octomap->castRay(octoPos1, octoDirection, octoEnd, true, maxRange))
//	{
//		return true; //max visibility or bounds reached
//	}
//
//	double distP1End = (octoPos1 - octoEnd).norm();
//
//	return distP1End > lengthDir;
}

void PRM::init()
{
	ros::NodeHandle n;
	m_pubMarker = n.advertise<visualization_msgs::MarkerArray>("prm/" + m_robot->c_robotName + "_" + m_pd->getConfig().name, 0);
}

void PRM::cleanUpPRM()
{
	for (auto it = m_nodes.begin(); it != m_nodes.end();)
	{
		PRMNode* node = (*it).second;
		if (node->m_edges.empty())
		{
			it = m_nodes.erase(it);
			continue;
		}
		++it;
	}
}

int PRM::nodeSize() const
{
	return m_nodes.size();
}

int PRM::edgeSize() const
{
	return m_edges.size();
}

void PRM::updateVisibilities()
{
//	boost::recursive_mutex::scoped_lock lock(m_mutex);
//	Eigen::Vector3d nn;
//
//	const double minDist = /*0.075 + */ParameterServer::octomapResolution / 2.0;
//
//	if (m_kdtree.get() == NULL || m_kdtree->getInputDataset().empty())
//	{
//		for (auto& it : m_nodes)
//		{
//			it.second->setCollisionFree(true);
//		}
//	}
//	else
//	{
//
//		for (auto& it : m_nodes)
//		{
//			const Eigen::Vector3d& pos = it.second->getPosition();
//			if (m_kdtree->getNearestNeighbor(pos, nn))
//				it.second->setCollisionFree((nn - pos).norm() > minDist);
//		}
//	}
}

void PRM::setConstraint(boost::shared_ptr<Constraint> constraint)
{
	PRM_WRITE_LOCK();
	for (auto& it : m_nodes)
	{
		Eigen::Affine3d pose = it.second->getPose();
//		it.second->setWasControllerSuccessful(constraint->findValidPose(pose));
		constraint->findValidPose(pose);
		it.second->setPose(pose);
	}
}

void PRM::computeCenterAndRadius(double& radius,
		Eigen::Vector3d& center) const
		{
	PRM_READ_LOCK();
	double norm;
	center.setZero();
	radius = 0;
	for (auto& it : m_nodes)
	{
		const Eigen::Vector3d& pos = it.second->getPosition();
		center += pos;
	}
	center /= m_nodes.size();

	for (auto& it : m_nodes)
	{
		const Eigen::Vector3d& pos = it.second->getPosition();
		norm = (pos - center).norm();
		if (norm > radius)
		{
			radius = norm;
		}
	}
}

boost::shared_ptr<Robot> PRM::getRobot() const
{
	return m_robot;
}

const PRMNodeMap& PRM::getStartNodes() const
{
	return m_startNodes;
}

const PRMNodeMap& PRM::getGoalNodes() const
{
	return m_goalNodes;
}

void PRM::resetNodesAndEdges(PRMNodeMap nodes,
		PRMEdgeMap edges)
{
	PRM_WRITE_LOCK();

	DELETE_MAP(m_nodes);
	DELETE_MAP(m_edges);

	m_nextIdNode = 0;
	m_nextIdEdge = 0;
	m_nodes = nodes;
	m_edges = edges;

	for (auto& it : m_nodes)
	{
		//update counter
		ais_util::atomicMax(m_nextIdNode, it.first);

		//extract start nodes
		if (it.second->getType() == PRMNode::StartNode)
			m_startNodes[it.first] = it.second;

		//extract goal nodes
		if (it.second->getType() == PRMNode::GoalNode)
			m_goalNodes[it.first] = it.second;
	}

	//update counters
	for (auto& it : m_edges)
	{
		ais_util::atomicMax(m_nextIdEdge, it.first);
	}

	//increase both counters to get the next id
	++m_nextIdNode;
	++m_nextIdEdge;

	init();
}

} /* namespace prm_planner */

