/*
 * This file (robot_link.h) is part of the Scene Analyzer of Daniel Kuhner.
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
#ifndef iNCZRqQbKXHLr0rNlwGP
#define iNCZRqQbKXHLr0rNlwGP

#include <assimp/scene.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>
#include <fcl_wrapper/collision_detection/physical_object.h>
#include <fcl_wrapper/robot_model/geometry.h>
#include <Eigen/Geometry>
#include <unordered_map>

namespace fcl_robot_model
{

class Mesh;
class RobotJoint;
class RobotState;

class RobotLink: public fcl_collision_detection::PhysicalObject
{
public:
	struct RobotLinkModel
	{
		RobotLinkModel();

		virtual ~RobotLinkModel();
		std::string meshFileName;
		Eigen::Vector3d scale;
		Eigen::Affine3d origin;
		FCL_POINTER<Geometry> geometry;
		bool hasMesh;
	};

public:
	typedef std::unordered_map<std::string, FCL_WEAK_POINTER<RobotJoint>> JointWeakMap;
	typedef std::unordered_map<std::string, FCL_POINTER<RobotJoint>> JointMap;
	typedef FCL_POINTER<RobotLink> Ptr;

public:
	RobotLink(const std::string& name,
			const std::string& rootFrame,
			const std::string& worldFrame,
			const FCL_POINTER<RobotLinkModel>& collisionModel,
			const FCL_POINTER<RobotLinkModel>& visualModel);
	virtual ~RobotLink();

	void setParentJoint(FCL_POINTER<RobotJoint>& parent);
	void addChildJoint(FCL_POINTER<RobotJoint>& child);
//	int getCollisionMeshSize() const;
//	int getVisualMeshSize() const;
	FCL_POINTER<RobotJoint> getParentJoint();
	FCL_POINTER<RobotLink> getParentLink();
	JointMap getChildJoints();
	Eigen::Affine3d getTransformationToParentLink() const;
	Eigen::Affine3d getTransformationToParentLink(const double angle) const;
	Eigen::Affine3d getTransformationToParentJoint() const;
	Eigen::Affine3d getOrigin() const;
	Eigen::Vector3d getPositionInBaseFrame() const;
	Eigen::Affine3d getTransformationInBaseFrame() const;
	Eigen::Affine3d getTransformationFromBaseFrame() const;
	void setTransformationToParentLink(const Eigen::Affine3d& t);
	void updateTransformationToParentLink();
	void updateTransformations(const Eigen::Affine3d& transformationToBaseOfParent,
			const Eigen::Affine3d& transformationFromBaseOfParent);

	virtual void initFCLModel();
	virtual CollisionObjectPtr getFCLModel();
	virtual CollisionObjectPtr getFCLModel(const RobotState& state);
	Eigen::Affine3d getFCLTransformation() const;

	void print(int depth = 0) const;

protected:
	virtual void loadMesh(FCL_POINTER<RobotLinkModel>& model);

private:
	FCL_POINTER<RobotLinkModel> m_collisionModel;
	FCL_POINTER<RobotLinkModel> m_visualModel;

	FCL_WEAK_POINTER<RobotJoint> m_parentJoint;
	JointWeakMap m_childJoints;

	Eigen::Affine3d m_transformationToParentJoint;
	Eigen::Affine3d m_transformationToParentLink;
	Eigen::Affine3d m_transformationToBase;
	Eigen::Affine3d m_transformationFromBase;

public:
	friend class RobotModel;
};

} /* namespace robot_model */

#endif /* iNCZRqQbKXHLr0rNlwGP */
