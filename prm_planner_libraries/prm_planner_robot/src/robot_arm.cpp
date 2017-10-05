/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 21, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: robot_arm.cpp
 */

#include <ais_definitions/exception.h>
#include <ais_definitions/macros.h>
#include <ais_log/log.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <prm_planner_robot/robot_arm.h>
#include <prm_planner_robot/defines.h>
#include <ros/ros.h>
#include <chrono>
#include <random>

#define ROBOT_ARM_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex)
#define ROBOT_ARM_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex)
#define ROBOT_ARM_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex)
#define ROBOT_ARM_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex)

namespace prm_planner
{

int RobotArm::s_idCounter = 0;

RobotArm::RobotArm(const RobotArmConfig& armConfig,
		bool initRobotConnection) :
				c_armConfig(armConfig),
				m_gripper(NULL),
				m_rootTfLink(armConfig.tfPrefix + (!armConfig.tfPrefix.empty() ? "/" : "") + armConfig.rootLink),
				m_tipLink(armConfig.tipLink),
				m_originalTipLinkFrame(armConfig.tfPrefix + (!armConfig.tfPrefix.empty() ? "/" : "") + armConfig.tipLink),
				m_receivedState(false),
				m_passiveMode(false),
				c_id(s_idCounter++)
{
	ros::NodeHandle n;

	//load robot model
	//we also need to load the urdf model to get the joint limits
	m_urdf.initParam(c_armConfig.robotDescriptionParam);

	//kdl model for Jacobian, FK, and IK
	if (!kdl_parser::treeFromUrdfModel(m_urdf, m_robot))
	{
		throw ais_definitions::Exception("Cannot open robot model of robot " + c_armConfig.name);
	}

	if (!m_robot.getChain(c_armConfig.rootLink, m_tipLink, m_chain))
	{
		throw ais_definitions::Exception("Cannot extract kinematic chain from "
				+ c_armConfig.rootLink + " to " + m_tipLink + " of robot " + c_armConfig.name);
	}

	//mapping joint names to ids
	for (size_t i = 0; i < m_chain.getNrOfSegments(); i++)
	{
		if (m_chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
		{
			std::string name = m_chain.getSegment(i).getJoint().getName();
			m_chainJointNames.push_back(name);
		}
	}

	//kdl joint limits
	m_chainLimitMin.resize(m_chainJointNames.size());
	m_chainLimitMax.resize(m_chainJointNames.size());
	size_t i = 0;
	for (auto& it : m_chainJointNames)
	{
		//don't apply the offsets to continuous joints
		if (m_urdf.joints_[it]->type == urdf::Joint::CONTINUOUS)
		{
			m_chainLimitMin(i, 0) = -M_PI;
			m_chainLimitMax(i, 0) = M_PI;
		}
		else
		{
			m_chainLimitMin(i, 0) = m_urdf.joints_[it]->limits->lower + 0.05;
			m_chainLimitMax(i, 0) = m_urdf.joints_[it]->limits->upper - 0.05;
		}

//		LOG_INFO(m_chain.getSegment(i).getJoint().getName() << " "<<m_chainLimitMin(i, 0) << " " << m_chainLimitMax(i, 0));
		++i;
	}

	//setup kinematics
	LOG_DEBUG("Loading kinematics " << c_armConfig.kinematicsClass << " in " << c_armConfig.kinematicsPackage)
	m_kinematics = Kinematics::load(c_armConfig.kinematicsPackage, c_armConfig.kinematicsClass);
	m_kinematics->init(this);

	m_chainPositions.resize(m_chainJointNames.size());
	m_chainPositions.data.fill(0);

	m_chainVelocities.resize(m_chainJointNames.size());
	m_chainCommands.resize(m_chainJointNames.size());

	LOG_INFO("Init robot connection");
	if (initRobotConnection)
	{
		//connect to robot
		initHardwareInterface();

		//get first data
		receiveData(ros::Time::now(), ros::Duration(0.0));
		usleep(100000);
	}

	LOG_DEBUG("Loaded robot model with " << m_robot.getNrOfJoints() << " joints " << m_robot.getNrOfSegments() << " links");
	LOG_DEBUG("Loaded chain with " << m_chain.getNrOfJoints() << " joints " << m_chain.getNrOfSegments() << " links");
}

RobotArm::~RobotArm()
{
	if (m_interface)
	{
		m_interface->stop();
	}

	DELETE_VAR(m_actionClient);
}

void RobotArm::initHardwareInterface()
{
	//create an instance of the corresponding hardware interface
	//if the useHWInterface flag is true. It loads the interface
	//from a ros plugin. You need to provide the paramaters
	//interface_package and interface_class to use the loader
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface)
	{
		LOG_DEBUG("Initialize robot interface");
		m_interface = RobotInterface::load(c_armConfig.interfacePackage, c_armConfig.interfaceClass);
		if (!m_interface->start())
		{
			LOG_FATAL("Cannot connect to robot (start() returned false!)");
			exit(10);
		}
	}
	//otherwise we use the default ros interface, i.e.,
	//we subscribe to a joint_state publisher and use
	//the FollowJointTrajectory action client
	else if (c_armConfig.executionInterface == ArmExecutionMode::FollowJointTrajectoryPublisher)
	{
		LOG_INFO("Waiting for follow joint trajectory action server on topic " << c_armConfig.followJointTrajectoryTopic);
		m_actionClient = new FollowJointTrajectoryAC(c_armConfig.followJointTrajectoryTopic, true);
		m_actionClient->waitForServer();
		LOG_INFO("Found action server");

		ros::NodeHandle n;
		m_subJointState = n.subscribe(c_armConfig.jointStateTopic, 1, &RobotArm::receiveJointState, this);
	}
	//no execution, thus we set the joint state to 0 and
	//do nothing else
	else
	{
		int i = 0;
		for (auto& it : m_chainJointNames)
		{
			m_chainPositions.data[i] = 0;
			m_chainVelocities.data[i] = 0;
			++i;
		}
	}
}

boost::shared_ptr<urdf::JointLimits> RobotArm::getJointLimits(const std::string& name)
{
	return m_urdf.joints_[name]->limits;
}

const KDL::JntArray& RobotArm::getChainLimitMax() const
{
	return m_chainLimitMax;
}

const KDL::JntArray& RobotArm::getChainLimitMin() const
{
	return m_chainLimitMin;
}

void RobotArm::getChainJoints(std::vector<urdf::Joint>& out)
{
	std::string jointName;
	for (auto& seg : m_chainJointNames)
	{
		out.push_back(*m_urdf.joints_[seg]);
	}
}

KDL::JntArray RobotArm::getKDLChainJointState() const
{
	ROBOT_ARM_READ_LOCK();
	return m_chainPositions;
}

std::unordered_map<std::string, double> RobotArm::getAllJointStates() const
{
	ROBOT_ARM_READ_LOCK();
	return m_allPositions;
}

KDL::JntArray RobotArm::getKDLChainVelocities() const
{
	ROBOT_ARM_READ_LOCK();
	return m_chainVelocities;
}

void RobotArm::receiveData(const ros::Time& time,
		const ros::Duration& period)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->read(time, period);

		//set data
		RobotInterface::Data& data = m_interface->m_data;

		ROBOT_ARM_WRITE_LOCK();

		//get all joint states
		for (auto& it : data)
		{
			m_allPositions[it.first] = it.second.pos;
		}

		//fill kdl vectors
		int i = 0;
		for (auto& it : m_chainJointNames)
		{
			RobotInterface::JointData& j = data[it];
			m_chainPositions.data[i] = j.pos;
			m_chainVelocities.data[i] = j.vel;

			++i;
		}

		m_receivedState = true;
	}
}

void RobotArm::receiveJointState(const sensor_msgs::JointStateConstPtr& jointState)
{
	if (m_passiveMode)
		return;

	std::unordered_map<std::string, size_t> mapping;
	double pos = 0, vel = 0;
	std::string n;

	//no data => return
	if (jointState->name.empty())
		return;

	//wronge joint state => return
	if (std::find(jointState->name.begin(), jointState->name.end(), m_chainJointNames[0]) == jointState->name.end())
		return;

	//Create a mapping between name and id of the message
	ROBOT_ARM_WRITE_LOCK();
	for (size_t i = 0; i < jointState->name.size(); ++i)
	{
		n = jointState->name[i];
		m_allPositions[n] = jointState->position[i];
		mapping[n] = i;
	}

	for (size_t i = 0; i < m_chainJointNames.size(); ++i)
	{
		//get id
		const int id = mapping[m_chainJointNames[i]];

		//get data
		pos = jointState->position[id];

		//since velocity is not allways send
		if (jointState->velocity.size() >= m_chainJointNames.size())
			vel = jointState->velocity[id];

		m_chainPositions.data[i] = pos;
		m_chainVelocities.data[i] = vel;
	}

	m_receivedState = true;
}

void RobotArm::sendVelocity(const ros::Time& time,
		const ros::Duration& period,
		const KDL::JntArray& cmd)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->setJointVelocityCommand(cmd, m_chainJointNames);

		if (!m_interface->write(time, period))
			LOG_ERROR("Cannot send data!");
	}
}

void RobotArm::sendTorque(const ros::Time& time,
		const ros::Duration& period,
		const KDL::JntArray& cmd)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->setJointTorqueCommand(cmd, m_chainJointNames);

		if (!m_interface->write(time, period))
			LOG_ERROR("Cannot send data!");
	}
}

void RobotArm::sendChainJointPosition(const KDL::JntArray& joints)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->setJointPositionCommand(joints, m_chainJointNames);

		m_interface->write();
	}
}

void RobotArm::sendTrajectory(const control_msgs::FollowJointTrajectoryGoal& trajectory)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::FollowJointTrajectoryPublisher && !m_passiveMode)
		m_actionClient->sendGoal(trajectory);
}

void RobotArm::stopMotion()
{
	if (!m_passiveMode)
	{
		if (c_armConfig.executionInterface == ArmExecutionMode::FollowJointTrajectoryPublisher)
		{
			LOG_INFO("Cancel Goal");
			m_actionClient->cancelGoal();
		}
	}
}

void RobotArm::sampleValidChainEEFPose(Eigen::Affine3d& pose,
		KDL::JntArray& joints,
		const double borderAvoidance)
{
	sampleValidChainJointState(joints, borderAvoidance);
	getFK(joints, pose);
}

void RobotArm::sampleValidChainJointState(KDL::JntArray& jointState,
		const double borderAvoidance)
{
	std::default_random_engine re(std::chrono::system_clock::now().time_since_epoch().count());

	jointState.resize(m_chainJointNames.size());

	double lower, upper;

	int i = 0;
	for (auto& it : m_chainJointNames)
	{
		if (CHECK_MAP(c_armConfig.sampleLimits, it))
		{
			RobotJointSampleLimits& l = c_armConfig.sampleLimits[it];
			lower = l.min;
			upper = l.max;
		}
		else
		{
			lower = m_urdf.joints_[it]->limits->lower + borderAvoidance;
			upper = m_urdf.joints_[it]->limits->upper - borderAvoidance;
		}

		std::uniform_real_distribution<double> unif(lower, upper);
		jointState.data(i++, 0) = unif(re);
	}
}

bool RobotArm::getIK(const Eigen::Affine3d& pose,
		KDL::JntArray& ik)
{
	KDL::JntArray init(m_chainJointNames.size());
	return getIKWithInit(init, pose, ik);
}

bool RobotArm::getIKWithInitCurrent(const Eigen::Affine3d& pose,
		KDL::JntArray& ik)
{
	return getIKWithInit(getKDLChainJointState(), pose, ik);
}

bool RobotArm::getIKWithInit(const KDL::JntArray& init,
		const Eigen::Affine3d& pose,
		KDL::JntArray& ik)
{
	return m_kinematics->getIK(pose, init, ik);
}

bool RobotArm::getFK(const KDL::JntArray& joints,
		Eigen::Affine3d& pose)
{
	return m_kinematics->getFK(joints, pose);
}

bool RobotArm::getCurrentFK(Eigen::Affine3d& pose)
{
	KDL::JntArray joints = getKDLChainJointState();
	return getFK(joints, pose);
}

const KDL::Chain& RobotArm::getChain() const
{
	return m_chain;
}

const std::vector<std::string>& RobotArm::getChainJointNames() const
{
	return m_chainJointNames;
}

const KDL::Tree& RobotArm::getRobot() const
{
	return m_robot;
}

const std::string& RobotArm::getRobotDescription() const
{
	return c_armConfig.robotDescriptionParam;
}

const std::string& RobotArm::getRootLink() const
{
	return c_armConfig.rootLink;
}

const std::string& RobotArm::getRootFrame() const
{
	return m_rootTfLink;
}

const std::string& RobotArm::getTipLink() const
{
	ROBOT_ARM_READ_LOCK();
	return m_tipLink;
}

const std::string& RobotArm::getOriginalTipLink() const
{
	return c_armConfig.tipLink;
}

const std::string& RobotArm::getName() const
{
	return c_armConfig.name;
}

const std::string& RobotArm::getCollisionMatrixFile() const
{
	return c_armConfig.collisionMatrixFile;
}

const boost::shared_ptr<GripperInterface> RobotArm::getGripper() const
{
	return m_gripper;
}

void RobotArm::setGripper(const boost::shared_ptr<GripperInterface> gripper)
{
	m_gripper = gripper;
}

const RobotArmConfig& RobotArm::getParameters()
{
	return c_armConfig;
}

bool RobotArm::isTrajectoryExecutionFinished()
{
	m_actionClient->waitForResult(ros::Duration(0.1));
	return m_actionClient->getState().isDone();
}

bool RobotArm::waitForData()
{
	if (c_armConfig.executionInterface == ArmExecutionMode::NoExecution)
		return true;

	ros::Rate r(100);
	ros::Time start = ros::Time::now();
	while ((ros::Time::now() - start).toSec() < 0.1)
	{
		//call receiveData in the case of the hardware interface
		//is going to be used.
		if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface)
			receiveData(ros::Time::now(), ros::Duration(0.01)); //use some default value for dt

		if (m_receivedState)
			return true;
	}

	return false;
}

void RobotArm::setToolCenterPointTransformation(const Eigen::Affine3d& tcp)
{
	ROBOT_ARM_WRITE_LOCK();

	Eigen::Vector3d translation = tcp.translation();
	Eigen::Quaterniond rotation(tcp.linear());

	urdf::Pose pose;
	pose.position = urdf::Vector3(translation.x(), translation.y(), translation.z());
	pose.rotation = urdf::Rotation(rotation.x(), rotation.y(), rotation.z(), rotation.w());

	boost::shared_ptr<urdf::Joint> toolJoint;
	boost::shared_ptr<urdf::Link> toolLink;

	//update
	if (CHECK_MAP(m_urdf.joints_, "tool_tip_joint"))
	{
		toolJoint = m_urdf.joints_["tool_tip_joint"];
		toolLink = m_urdf.links_["tool_tip_link"];
	}
	//add
	else
	{
		toolJoint.reset(new urdf::Joint);
		toolLink.reset(new urdf::Link);

		toolLink->parent_joint = toolJoint;
		toolLink->name = "tool_tip_link";

		toolJoint->type = urdf::Joint::FIXED;
		toolJoint->name = "tool_tip_joint";
		toolJoint->parent_link_name = c_armConfig.tipLink;

		m_urdf.joints_[toolJoint->name] = toolJoint;
		m_urdf.links_[toolLink->name] = toolLink;

		//make connection between tip link and tool link
		m_urdf.links_[m_tipLink]->child_joints.push_back(toolJoint);
		m_urdf.links_[m_tipLink]->child_links.push_back(toolLink);
	}

	toolJoint->parent_to_joint_origin_transform = pose;

	m_originalTipLinkFrame = c_armConfig.tfPrefix + (!c_armConfig.tfPrefix.empty() ? "/" : "") + c_armConfig.tipLink;
	m_tipLink = toolLink->name;

	//kdl model for Jacobian, FK, and IK
	if (!kdl_parser::treeFromUrdfModel(m_urdf, m_robot))
	{
		throw ais_definitions::Exception("Cannot open robot model of robot " + c_armConfig.name);
	}

	if (!m_robot.getChain(c_armConfig.rootLink, m_tipLink, m_chain))
	{
		throw ais_definitions::Exception("Cannot extract kinematic chain from "
				+ c_armConfig.rootLink + " to " + m_tipLink + " of robot " + c_armConfig.name);
	}

	//setup solvers
	m_kinematics->init(this);

	m_tcp = tcp;
	m_useTCP = true;
}

const std::string& RobotArm::getOriginalTipLinkFrame() const
{
	return m_originalTipLinkFrame;
}

void RobotArm::setPassiveMode(const bool passive)
{
	m_passiveMode = passive;
}

void RobotArm::setChainJointState(const KDL::JntArray& joints)
{
	ROBOT_ARM_WRITE_LOCK();
	m_chainPositions = joints;
}

Eigen::Affine3d RobotArm::getTcp() const
{
	ROBOT_ARM_READ_LOCK();
	return m_tcp;
}

bool RobotArm::isUseTcp() const
{
	ROBOT_ARM_READ_LOCK();
	return m_useTCP;
}

const boost::shared_ptr<Kinematics>& RobotArm::getKinematics() const
{
	return m_kinematics;
}

const urdf::Model& RobotArm::getUrdf() const
{
	return m_urdf;
}

} /* namespace prm_planner */

