/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 29, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: easy_kd_tree.h
 */

#ifndef EASY_KD_TREE_H_
#define EASY_KD_TREE_H_
#include <Eigen/Core>
#include <vector>

namespace ais_point_cloud
{

class EasyKDTree
{
	struct Node
	{
		enum Direction
		{
			X = 0,
			Y = 1,
			Z = 2
		};

		Node();
		virtual ~Node();

		Node* n1;
		Node* n2;
		std::vector<Eigen::Vector3d> points;
		Direction dir;
		double center;
	};

	struct Condition
	{
		Eigen::Vector3d center;
		bool negX;
		bool negY;
		bool negZ;
	};

public:
	EasyKDTree(int maxLeafSize);
	virtual ~EasyKDTree();

	void setData(const std::vector<Eigen::Vector3d>& points);
	bool getNearestNeighbor(const Eigen::Vector3d& query,
			Eigen::Vector3d& nearestNeighbor);
	/**
	 * Returns 8 nearest neighbors in octants around center point:
	 * 0: -x, -y, -z
	 * 1: -x, -y, z
	 * 2: -x, y, -z
	 * 3: -x, y, z
	 * 4: x, -y, -z
	 * 5: x, -y, z
	 * 6: x, y, -z
	 * 7: x, y, z
	 * return index of nearest neighbor
	 */
	int getNearestOctantNeighbors(const Eigen::Vector3d& query,
			std::vector<Eigen::Vector3d>& nearestNeighbors);
	const std::vector<Eigen::Vector3d>& getInputDataset();

private:
	double getCenter(const std::vector<Eigen::Vector3d>& points,
			const Node::Direction& dir);
	void setChilds(Node*& node,
			const Node::Direction& dir);
	void search(const Eigen::Vector3d& query,
			Node* node,
			Eigen::Vector3d& nearestNeighbor);
	double searchOctant(Node* node,
			const Condition& condition,
			Eigen::Vector3d& nearestNeighbor);
	inline double getValue(const Eigen::Vector3d& point,
			const Node::Direction& dir)
	{
		return (dir == Node::X ? point.x() : (dir == Node::Y ? point.y() : point.z()));
	}

private:
	Node* m_root;

public:
	const int c_maxLeafSize;
};

} /* namespace ais_point_cloud */

#endif /* EASY_KD_TREE_H_ */
