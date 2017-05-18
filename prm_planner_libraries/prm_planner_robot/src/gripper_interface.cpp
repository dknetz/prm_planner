/*
 * gripper.cpp
 *
 *  Created on: Aug 29, 2016
 *      Author: kuhnerd
 */

#include <ais_log/log.h>
#include <prm_planner_robot/gripper_interface.h>

namespace prm_planner
{

pluginlib::ClassLoader<GripperInterface>* GripperInterface::s_loader = NULL;

GripperInterface::GripperInterface()

{
}

GripperInterface::~GripperInterface()
{
}

const std::string& GripperInterface::getCurrentObject() const
{
	GRIPPER_INTERFACE_READ_LOCK();
	return m_currentObject;
}

void GripperInterface::setCurrentObject(const std::string& currentObject)
{
	GRIPPER_INTERFACE_WRITE_LOCK();
	m_currentObject = currentObject;
}

const Eigen::Affine3d& GripperInterface::getTGripperToObject() const
{
	GRIPPER_INTERFACE_READ_LOCK();
	return m_tGripperToObject;
}

void GripperInterface::setTGripperToObject(const Eigen::Affine3d& tGripperToObject)
{
	GRIPPER_INTERFACE_WRITE_LOCK();
	m_tGripperToObject = tGripperToObject;
}

void GripperInterface::init(GripperInterfaceParameters& parameters)
{
	GRIPPER_INTERFACE_WRITE_LOCK();
	c_parameters = parameters;

	//init joint state to 0
	for (auto& it: c_parameters.jointNames)
	{
		m_joints[it] = 0;
	}
}

boost::shared_ptr<GripperInterface> GripperInterface::load(const std::string& package,
		const std::string& library)
{
	//create instance if not already available
	if (s_loader == NULL)
		s_loader = new pluginlib::ClassLoader<GripperInterface>("prm_planner", "prm_planner::GripperInterface");

	boost::shared_ptr<GripperInterface> interface;

	try
	{
		interface = s_loader->createInstance(library);
	}
	catch (pluginlib::PluginlibException& ex)
	{
		LOG_FATAL("The plugin failed to load for some reason. Error: " << ex.what());
		exit(3);
	}

	return interface;
}

const std::unordered_map<std::string, double> GripperInterface::getJoints() const
{
	GRIPPER_INTERFACE_READ_LOCK();
	return m_joints;
}

void GripperInterface::setJoints(const std::unordered_map<std::string, double>& joints)
{
	GRIPPER_INTERFACE_WRITE_LOCK();

	m_joints = joints;
}

}
/* namespace prm_planner */

