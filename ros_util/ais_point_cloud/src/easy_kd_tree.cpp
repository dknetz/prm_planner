/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: easy_kd_tree.cpp
 */

#include <ais_definitions/class.h>
#include <ais_log/log.h>
#include <ais_point_cloud/easy_kd_tree.h>
#include <iostream>

namespace ais_point_cloud
{

EasyKDTree::EasyKDTree(int maxLeafSize) :
				c_maxLeafSize(maxLeafSize),
				m_root(NULL)
{
}

EasyKDTree::~EasyKDTree()
{
	DELETE_VAR (m_root);
}

EasyKDTree::Node::Node() :
				n1(NULL),
				n2(NULL),
				dir(Node::X),
				center(0)
{
}

EasyKDTree::Node::~Node()
{
	DELETE_VAR (n1);
	DELETE_VAR (n2);
}

void EasyKDTree::setData(const std::vector<Eigen::Vector3d>& points)
{
	DELETE_VAR (m_root);
	m_root = new Node;
	m_root->points = points;
	setChilds(m_root, Node::X);
}

bool EasyKDTree::getNearestNeighbor(const Eigen::Vector3d& query,
		Eigen::Vector3d& nearestNeighbor)
{
	if (m_root == NULL)
	{
		return false;
	}

	search(query, m_root, nearestNeighbor);
	return true;
}

int EasyKDTree::getNearestOctantNeighbors(const Eigen::Vector3d& query,
		std::vector<Eigen::Vector3d>& nearestNeighbors)
{
	if (m_root == NULL)
	{
		return -1;
	}

	nearestNeighbors.reserve(8);
	Eigen::Vector3d point;
	double dist = std::numeric_limits<double>::max();
	double d;
	int nearestNeighbor;

	std::vector<bool> bools { true, false };

	for (size_t x = 0; x < bools.size(); ++x)
		for (size_t y = 0; y < bools.size(); ++y)
			for (size_t z = 0; z < bools.size(); ++z)
			{
				if ((d = searchOctant(m_root, { query, bools[x], bools[y], bools[z] }, point)) >= 0)
				{
					nearestNeighbors.push_back(point);

					if (d < dist)
					{
						dist = d;
						nearestNeighbor = nearestNeighbors.size() - 1;
					}

					nearestNeighbors.back() -= query;
				}
				else
				{
					nearestNeighbors.push_back( { bools[x] ? -10000.0 : 10000.0,
							bools[y] ? -10000.0 : 10000.0,
							bools[z] ? -10000.0 : 10000.0 });
				}
			}
	return nearestNeighbor;
}

double EasyKDTree::getCenter(const std::vector<Eigen::Vector3d>& points,
		const Node::Direction& dir)
{
	double center = 0;
	for (auto& it : points)
	{
		switch (dir)
		{
			case Node::X:
				center += it.x();
				break;
			case Node::Y:
				center += it.y();
				break;
			case Node::Z:
				center += it.z();
				break;
		}
	}
	return center /= points.size();
}

void EasyKDTree::setChilds(Node*& node,
		const Node::Direction& dir)
{
	if (node->points.size() < c_maxLeafSize)
	{
		return;
	}
	else
	{
		node->n1 = new Node;
		node->n2 = new Node;
		node->dir = dir;
		node->center = getCenter(node->points, dir);
		for (auto& it : node->points)
		{
			if (getValue(it, dir) < node->center)
			{
				node->n1->points.push_back(it);
			}
			else
			{
				node->n2->points.push_back(it);
			}
		}

		int intDir = dir;
		Node::Direction nextDir = (Node::Direction)(++intDir % 3);

		setChilds(node->n1, nextDir);
		setChilds(node->n2, nextDir);
	}
}

void EasyKDTree::search(const Eigen::Vector3d& query,
		Node* node,
		Eigen::Vector3d& nearestNeighbor)
{
	if (node->n1 != NULL && node->n2 != NULL)
	{
		switch (node->dir)
		{
			case Node::X:
				if (query.x() < node->center)
					search(query, node->n1, nearestNeighbor);
				else
					search(query, node->n2, nearestNeighbor);
				break;
			case Node::Y:
				if (query.y() < node->center)
					search(query, node->n1, nearestNeighbor);
				else
					search(query, node->n2, nearestNeighbor);
				break;
			case Node::Z:
				if (query.z() < node->center)
					search(query, node->n1, nearestNeighbor);
				else
					search(query, node->n2, nearestNeighbor);
				break;
		}
	}
	else
	{
		double dist = std::numeric_limits<double>::max();
		double norm;
		for (auto& it : node->points)
		{
			norm = (it - query).squaredNorm();
			if (norm < dist)
			{
				dist = norm;
				nearestNeighbor = it;
			}
		}
	}
}

double EasyKDTree::searchOctant(Node* node,
		const Condition& condition,
		Eigen::Vector3d& nearestNeighbor)
{
	if (node->n1 != NULL && node->n2 != NULL)
	{
		double dist = std::numeric_limits<double>::max();
		double norm;
		Eigen::Vector3d diff;
		bool found = false;
		for (auto& it : node->points)
		{
			diff = it - condition.center;
			if (((condition.negX && diff.x() < 0)
					|| (!condition.negX && diff.x() >= 0)) &&
					((condition.negY && diff.y() < 0)
							|| (!condition.negY && diff.y() >= 0)) &&
					((condition.negZ && diff.z() < 0)
							|| (!condition.negZ && diff.z() >= 0)))
			{
				norm = diff.squaredNorm();
				if (norm < dist)
				{
					dist = norm;
					nearestNeighbor = it;
					found = true;
				}
			}

		}

		return found ? dist : -1;
	}
	else
	{
		Eigen::Vector3d nearest1, nearest2;
		double d1, d2;
		switch (node->dir)
		{
			case Node::X:
				if (condition.negX)
				{
					if (condition.center.x() < node->center)
					{
						return searchOctant(node->n1, condition, nearestNeighbor);
					}
					else if (condition.center.x() >= node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
				}
				else
				{
					if (condition.center.x() < node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
					else if (condition.center.x() >= node->center)
					{
						return searchOctant(node->n2, condition, nearestNeighbor);
					}
				}
				break;
			case Node::Y:
				if (condition.negY)
				{
					if (condition.center.y() < node->center)
					{
						return searchOctant(node->n1, condition, nearestNeighbor);
					}
					else if (condition.center.y() >= node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
				}
				else
				{
					if (condition.center.y() < node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
					else if (condition.center.y() >= node->center)
					{
						return searchOctant(node->n2, condition, nearestNeighbor);
					}
				}
				break;
			case Node::Z:
				if (condition.negZ)
				{
					if (condition.center.z() < node->center)
					{
						return searchOctant(node->n1, condition, nearestNeighbor);
					}
					else if (condition.center.z() >= node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
				}
				else
				{
					if (condition.center.z() < node->center)
					{
						d1 = searchOctant(node->n1, condition, nearest1);
						d2 = searchOctant(node->n2, condition, nearest2);
						if (d1 < d2)
						{
							nearestNeighbor = nearest1;
							return d1;
						}
						else
						{
							nearestNeighbor = nearest2;
							return d2;
						}
					}
					else if (condition.center.z() >= node->center)
					{
						return searchOctant(node->n2, condition, nearestNeighbor);
					}
				}
				break;
		}
	}
}

const std::vector<Eigen::Vector3d>& EasyKDTree::getInputDataset()
{
	return m_root->points;
}

} /* namespace ais_point_cloud */

