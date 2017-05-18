/*
 * graspable_object.h
 *
 *  Created on: Sep 10, 2016
 *      Author: kuhnerd
 */

#ifndef H01C93924_61D5_4FF0_830B_344C0E0BD72B
#define H01C93924_61D5_4FF0_830B_344C0E0BD72B

#include <ais_ros/ros_base_interface.h>
#include <boost/atomic.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <octomap/OcTree.h>
#include <prm_planner/util/parameters.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace prm_planner
{

class GripperInterface;

class GraspableObject
{
public:
	enum GraspType
	{
		Cylinder
	};

	struct ObjectParameters
	{
		std::string name;
		std::string meshFilename;
		double octomapResolution;
		std::string frame;
		std::string objectFrameName;
	};

	GraspableObject(const ObjectParameters& parameters,
			const parameters::DroppingConfig& droppingParameters);
	virtual ~GraspableObject();

	void updatePoseFromTF();
	void removeObjectFromOctomap(boost::shared_ptr<octomap::OcTree>& octomap) const;
	void addObjectToOctomap(boost::shared_ptr<octomap::OcTree>& octomap) const;
	void updateBoundingBox();
	void publish();
	bool sampleGraspPose(Eigen::Affine3d& prePose,
			Eigen::Affine3d& pose,
			Eigen::Affine3d& postPose,
			const parameters::HandConfig& handConfig,
			const boost::shared_ptr<GripperInterface> gripper);
	bool getTransformationFromFrame(const std::string& frame,
			Eigen::Affine3d& t);

	const Eigen::Affine3d getPose() const;
	GraspType getGraspType() const;
	double getHeight() const;
	double getRadius() const;
	double getHeightAboveCenter() const;
	double getHeightBelowCenter() const;
	bool isDoUpdate() const;
	void setDoUpdate(bool doUpdate);

	bool isActive() const;
	bool isInGripper(const boost::shared_ptr<GripperInterface>& gripper);

	void print();

	/**
	 * Actually computes a the drop poses (orientation)
	 */
	bool sampleDropPose(Eigen::Affine3d& prePose,
			Eigen::Affine3d& pose,
			Eigen::Affine3d& postPose,
			const Eigen::Vector3d goalPos,
			const boost::shared_ptr<GripperInterface> gripper);

	void lock();
	void unlock();

	const ObjectParameters c_params;
	const parameters::DroppingConfig c_droppingConfig;

private:
	void loadModel();
	void computeBoundingBox(const std::vector<Eigen::Vector3d>& points,
			const Eigen::Vector3d& center,
			Eigen::Vector3d& minBBPoint,
			Eigen::Vector3d& maxBBPoint);

private:
	mutable boost::recursive_mutex m_mutex;
	std::vector<Eigen::Vector3d> m_vertices;

	boost::atomic_bool m_doUpdate;

	//pose in world frame
	Eigen::Affine3d m_pose;

	Eigen::Vector3d m_upAxis;

	GraspType m_graspType;
	double m_radius;
	double m_height;
	double m_heightBelowCenter;
	double m_heightAboveCenter;
	bool m_activeObject;
	Eigen::Vector3d m_bbMin;
	Eigen::Vector3d m_bbMax;
	Eigen::Vector3d m_center;

	//visualization and debugging
	boost::atomic_bool m_graspingPosesComputed;
	boost::atomic_bool m_dropingPosesComputed;

	//grasping (visualization)
	Eigen::Affine3d m_graspingPrePose;
	Eigen::Affine3d m_graspingPose;
	Eigen::Affine3d m_graspingPostPose;

	//dropping (visualization)
	Eigen::Affine3d m_dropingPrePose;
	Eigen::Affine3d m_dropingPose;
	Eigen::Affine3d m_dropingPostPose;

	//publisher
	ros::Publisher m_pubGraspPoses;
	ros::Publisher m_pubDropPoses;
};

} /* namespace prm_planner */

#endif /* H01C93924_61D5_4FF0_830B_344C0E0BD72B */
