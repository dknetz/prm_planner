/*
 * gripper.h
 *
 *  Created on: Aug 29, 2016
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_GRIPPER_H_
#define PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_GRIPPER_H_
#include <boost/thread/pthread/shared_mutex.hpp>
#include <Eigen/Geometry>
#include <string>
#include <pluginlib/class_loader.h>
#include <unordered_map>

//Read and write lock which locks m_mutex
#define GRIPPER_INTERFACE_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex);
#define GRIPPER_INTERFACE_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex);
#define GRIPPER_INTERFACE_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex);
#define GRIPPER_INTERFACE_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex);

namespace prm_planner
{

class GripperInterface
{
public:
	struct GripperInterfaceParameters
	{
		std::string name;
		std::string jointStateTopic;
		std::string graspFrame;
		std::string topic;
		std::vector<std::string> jointNames;

		//the distance to the object of the pre-position
		double graspPreDistance;

		//the distance between the object center and the eef
		//point of the gripper
		double graspRadius;

		//the gripper moves back to graspPreDistance after
		//grasping. Finally it can move up bei graspPostHeight
		double graspPostHeight;

		//the distance between pre-position and desired object
		//position when dropping an object
		double dropPreDistance;

	};

public:
	GripperInterface();
	virtual ~GripperInterface();

	virtual void init(GripperInterfaceParameters& parameters);

	virtual bool open() = 0;
	virtual bool close() = 0;

	/**
	 * Returns the name of the object in the gripper.
	 * If there is no object, the method will return
	 * an empty string.
	 */
	const std::string& getCurrentObject() const;

	/**
	 * Set the current object. If you set it to
	 * the empty string "" we'll assume that there
	 * is no object.
	 */
	void setCurrentObject(const std::string& currentObject);

	const Eigen::Affine3d& getTGripperToObject() const;
	void setTGripperToObject(const Eigen::Affine3d& tGripperToObject);
	const std::unordered_map<std::string, double> getJoints() const;
	void setJoints(const std::unordered_map<std::string, double>& joints);

	GripperInterfaceParameters c_parameters;

	/**
	 * Loads an robot interface from a ROS plugin.
	 * @package: the ROS package name
	 * @library: namespace::class_name
	 */
	static boost::shared_ptr<GripperInterface> load(const std::string& package,
			const std::string& library);

private:
	/**
	 * The object needs to live the complete lifetime
	 * of the program. Otherwise we get an annoying warning
	 */
	static pluginlib::ClassLoader<GripperInterface>* s_loader;

protected:
	std::string m_currentObject;
	Eigen::Affine3d m_tGripperToObject;
	std::unordered_map<std::string, double> m_joints;
	mutable boost::shared_mutex m_mutex;
};

} /* namespace prm_planner */

#endif /* PRM_PLANNER_LIBRARIES_PRM_PLANNER_ROBOT_INCLUDE_PRM_PLANNER_ROBOT_GRIPPER_H_ */
