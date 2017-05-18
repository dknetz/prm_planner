/*
 * This file (robot_model.h) is part of the Scene Analyzer of Daniel Kuhner.
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
 * created:		Jan 30, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_ROBOT_MODEL_H_
#define KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_ROBOT_MODEL_H_

#include <urdf/model.h>
#include <Eigen/Geometry>
#include <unordered_map>

#include <fcl/BVH/BVH_model.h>
#include <fcl_wrapper/collision_detection/collection.h>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <fcl_wrapper/collision_detection/common.h>

#include <ais_definitions/hashes.h>
#include <fcl_wrapper/collision_detection/fcl_pointer.h>

using fcl_collision_detection::ObjectMap;

namespace fcl_robot_model
{

class RobotLink;
class RobotJoint;
class RobotState;

class RobotModel: public fcl_collision_detection::Collection
{
public:
	typedef std::unordered_map<std::pair<std::string, std::string>, Eigen::Affine3d, ais_definitions::HasherStringPair> TransformationMap;

	RobotModel(const std::string& robotDescriptionParamName,
			const std::string& name = "",
			const std::string& tfPrefix = "",
			const std::string& worldFrame = "",
			const std::string& collisionMatrixFile = "");
	virtual ~RobotModel();

	//LINKS
	void getChildLinks(const std::string name,
			std::vector<FCL_POINTER<RobotLink>>& childs);
	void getParentLink(const std::string name,
			FCL_POINTER<RobotLink>& parent);
	FCL_POINTER<RobotLink> getLink(const std::string& name);
	double getDistance(const std::string& frame1,
			const std::string& frame2);

	Eigen::Vector3d getPositionOfFrame(const std::string& frame);
	Eigen::Affine3d getTransformationOfFrameToBase(const std::string& frame);
	Eigen::Affine3d getTransformationOfFrameFromBase(const std::string& frame);
	Eigen::Affine3d getTransformationToSomeParent(const std::string& child,
			const std::string& someParent);
	Eigen::Affine3d getTransformationToRoot(const std::string& frame);
	Eigen::Affine3d getFrameOfLinkInSomeParentFrame(const std::string& frame,
			const std::string& someParent);
	void sampleValidJoints(RobotState& state);

	//JOINTS
	FCL_POINTER<RobotJoint> getJoint(const std::string& name);
	void getJointNames(std::vector<std::string>& names);
	double getLowerJointLimit(const std::string& jointName);
	double getUpperJointLimit(const std::string& jointName);
	std::unordered_map<std::string, double> getAllVelocityLimits();

	//JOINT CONFIGURATION
	void setRobotState(const RobotState& state);
	void setRobotState(const std::unordered_map<std::string, double>& state);
	void getRobotState(RobotState& state);

	//PHYSICS
	virtual fcl_collision_detection::CollisionMatrix::Ptr getCollisionMatrix();
	virtual ObjectMap getObjects();

	//OTHER, please add tf_prefix_
	static void parseURI(const std::string& path,
			std::string& absPath);
	std::string getEndEffectorFrameName() const;
	void setEndEffectorFrameName(const std::string& name);
	std::string getManipulatorBaseFrameName() const;
	void setManipulatorBaseFrameName(const std::string& name);
	Eigen::Affine3d getManipulatorBaseFrame() const;
	void print() const;

	virtual void setTransform(const Eigen::Affine3d& tf);

protected:
	void load();
	void loadHierarchy(const boost::shared_ptr<const urdf::Link>& parent,
			FCL_POINTER<RobotLink>& myParent);

	static void toEigenTransform(const urdf::Pose& pose,
			Eigen::Affine3d& transform);

	bool getTransformationToSomeParentRec(const std::string& child,
			const std::string& someParent,
			Eigen::Affine3d& t);
	bool getFrameOfLinkInSomeParentFrameRec(const std::string& child,
			const std::string& someParent,
			Eigen::Affine3d& t);

private:
	urdf::Model* m_robotModel;
	std::unordered_map<std::string, FCL_POINTER<RobotLink>> m_links;
	ObjectMap m_linksWorldObjects;
	std::unordered_map<std::string, FCL_POINTER<RobotJoint>> m_joints;
	FCL_POINTER<RobotLink> m_root;
//	TransformationMap m_transformations;
	fcl_collision_detection::CollisionMatrix::Ptr m_collisionMatrix;
	std::string m_frameEndEffector;
	std::string m_frameManipulatorBase;
	std::string m_tfPrefix;
	Eigen::Affine3d m_transformationManipulatorBaseToRoot;
};

} /* namespace urdf_model */

#endif /* KUHNERD_ROADMAP_ROBOT_MODEL_INCLUDE_ROBOT_MODEL_ROBOT_MODEL_H_ */
