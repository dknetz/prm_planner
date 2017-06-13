/*
 * This file (mesh.h) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Feb 2, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_MESH_H_
#define KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_MESH_H_

#include <assimp/scene.h>
#include <fcl_wrapper/robot_model/geometry.h>
#include <Eigen/Core>

namespace fcl_robot_model
{

class Mesh
{
public:
	struct Face
	{
		Face(const int i1,
				const int i2,
				const int i3);
		Face(const std::vector<unsigned int>& vertices);
		std::vector<unsigned int> vertices;
	};

	struct Vertex
	{
		Vertex(const Eigen::Vector3d& position);
		Eigen::Vector3d position;
	};

	typedef std::vector<Face*> Faces;
	typedef std::vector<Vertex*> Vertices;

	Mesh(const aiMesh* mesh,
			const Eigen::Vector3d& scaling);
	Mesh(const double boxWidthX,
			const double boxWidthY,
			const double boxWidthZ);
	Mesh();
	virtual ~Mesh();

	Faces m_faces;
	Vertices m_vertices;
	const Eigen::Vector3d m_scaling;
};

class Meshes: public Geometry
{
public:
	Meshes(const std::string& name);
	virtual void getFCLModel(const fcl::Transform3f& transform,
			FCL_POINTER<fcl::CollisionObject>& fclCollisionModel);
	std::vector<FCL_POINTER<Mesh>> m_meshes;
};

} /* namespace urdf_model */

#endif /* KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_MESH_H_ */
