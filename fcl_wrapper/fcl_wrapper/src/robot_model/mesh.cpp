/*
 * This file (mesh.cpp) is part of the Scene Analyzer of Daniel Kuhner.
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

#include <ais_definitions/class.h>
#include <ais_definitions/macros.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl_wrapper/collision_detection/common.h>
#include <fcl_wrapper/robot_model/mesh.h>
#include <iostream>

using namespace Eigen;

namespace fcl_robot_model
{

Mesh::Mesh(const aiMesh* mesh,
		const Eigen::Vector3d& scaling) :
				m_scaling(scaling)
{
	m_faces.resize(mesh->mNumFaces);
	FOR_COUNTER(i, mesh->mNumFaces)
		{
		std::vector<unsigned int> vertices(mesh->mFaces[i].mNumIndices);
		FOR_COUNTER(j, mesh->mFaces[i].mNumIndices)
				{
			vertices[j] = mesh->mFaces[i].mIndices[j];
		}

		m_faces[i] = new Face(vertices);
	}

	m_vertices.resize(mesh->mNumVertices);
	FOR_COUNTER(i, mesh->mNumVertices)
		{
		Eigen::Vector3d pos(mesh->mVertices[i].x * m_scaling.x(),
				mesh->mVertices[i].y * m_scaling.y(),
				mesh->mVertices[i].z * m_scaling.z());
		m_vertices[i] = new Vertex(pos);
	}
}

Mesh::Mesh() :
				m_scaling(1.0, 1.0, 1.0)
{
}

Mesh::Mesh(const double boxWidthX,
		const double boxWidthY,
		const double boxWidthZ)
{
	double x2 = boxWidthX / 2.0;
	double y2 = boxWidthY / 2.0;
	double z2 = boxWidthZ / 2.0;
	m_vertices.push_back(new Vertex(Vector3d(-x2, -y2, -z2)));
	m_vertices.push_back(new Vertex(Vector3d(x2, -y2, -z2)));
	m_vertices.push_back(new Vertex(Vector3d(x2, -y2, z2)));
	m_vertices.push_back(new Vertex(Vector3d(-x2, -y2, z2)));

	m_vertices.push_back(new Vertex(Vector3d(-x2, y2, -z2)));
	m_vertices.push_back(new Vertex(Vector3d(x2, y2, -z2)));
	m_vertices.push_back(new Vertex(Vector3d(x2, y2, z2)));
	m_vertices.push_back(new Vertex(Vector3d(-x2, y2, z2)));

	m_faces.push_back(new Face(0, 1, 3));
	m_faces.push_back(new Face(1, 2, 3));

	m_faces.push_back(new Face(1, 5, 2));
	m_faces.push_back(new Face(5, 6, 2));

	m_faces.push_back(new Face(5, 4, 7));
	m_faces.push_back(new Face(5, 7, 6));

	m_faces.push_back(new Face(4, 0, 3));
	m_faces.push_back(new Face(4, 3, 7));

	m_faces.push_back(new Face(0, 1, 4));
	m_faces.push_back(new Face(1, 5, 4));

	m_faces.push_back(new Face(3, 2, 7));
	m_faces.push_back(new Face(2, 6, 7));
}

Mesh::~Mesh()
{
	DELETE_VECTOR(m_faces);
	DELETE_VECTOR(m_vertices);
}

Mesh::Face::Face(const int i1,
		const int i2,
		const int i3)
{
	vertices.push_back(i1);
	vertices.push_back(i2);
	vertices.push_back(i3);
}

Mesh::Face::Face(const std::vector<unsigned int>& vertices) :
				vertices(vertices)
{
}

Mesh::Vertex::Vertex(const Eigen::Vector3d& position) :
				position(position)
{
}

Meshes::Meshes(const std::string& name):Geometry(name)
{
}

void Meshes::getFCLModel(const fcl::Transform3f& transform,
		FCL_POINTER<fcl::CollisionObject>& fclCollisionModel)
		{
			using namespace fcl;

			std::vector<Vec3f> vertices;
			std::vector<Triangle> triangles;
			int offset = 0; //because we may have several meshes
			for (auto& mesh : m_meshes)
			{
				for (auto& point : mesh->m_vertices)
				{
					vertices.push_back(Vec3f(point->position.x(), point->position.y(), point->position.z()));
				}

				for (auto& face : mesh->m_faces)
				{
					assert(face->vertices.size() == 3);

					Triangle triangle(face->vertices[0] + offset,
					face->vertices[1] + offset,
					face->vertices[2] + offset);

					triangles.push_back(triangle);
				}

				offset += mesh->m_vertices.size();
			}

			if (!triangles.empty())
			{
				FCL_POINTER<BVHModel<OBBRSS>> model(new BVHModel<OBBRSS>());
				model->setUserData(&c_name);

				model->beginModel();
				model->addSubModel(vertices, triangles);
				model->endModel();

				fclCollisionModel.reset(new CollisionObject(model, transform));
			}
			else
			{
				fclCollisionModel.reset();
			}
		}

	}
	/* namespace urdf_model */

