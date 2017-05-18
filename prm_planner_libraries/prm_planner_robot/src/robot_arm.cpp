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
			m_jointNames.push_back(name);
			m_rosJointState.name.push_back(name);
		}
	}

	//kdl joint limits
	m_limitMin.resize(m_jointNames.size());
	m_limitMax.resize(m_jointNames.size());
	size_t i = 0;
	for (auto& it : m_jointNames)
	{
		m_limitMin(i, 0) = m_urdf.joints_[it]->limits->lower + 0.05;
		m_limitMax(i, 0) = m_urdf.joints_[it]->limits->upper - 0.05;
		++i;
	}

	//setup solvers
	m_fkSolver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
	m_ikVelSolver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
	m_ikSolver.reset(new KDL::ChainIkSolverPos_NR_JL(m_chain, m_limitMin, m_limitMax, *m_fkSolver, *m_ikVelSolver));
	m_jacobianSolver.reset(new KDL::ChainJntToJacSolver(m_chain));

	m_rosJointState.position.resize(m_jointNames.size());
	m_rosJointState.velocity.resize(m_jointNames.size());
	m_positions.resize(m_jointNames.size());

	m_positions.data.fill(0);

	m_velocities.resize(m_jointNames.size());
	m_commands.resize(m_jointNames.size());

	if (initRobotConnection)
	{
		//connect to robot
		initHardwareInterface();

		//get first data
		receiveData(ros::Time::now(), ros::Duration(0.0));
		usleep(100000);
	}

//	LOG_DEBUG("Loaded robot model with " << m_robot.getNrOfJoints() << " joints " << m_robot.getNrOfSegments() << " links");
//	LOG_DEBUG("Loaded chain with " << m_chain.getNrOfJoints() << " joints " << m_chain.getNrOfSegments() << " links");
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
		m_actionClient = new FollowJointTrajectoryAC(c_armConfig.followJointTrajectoryTopic, true);
		m_actionClient->waitForServer();

		ros::NodeHandle n;
		m_subJointState = n.subscribe(c_armConfig.jointStateTopic, 1, &RobotArm::receiveJointState, this);
	}
	//no execution, thus we set the joint state to 0 and
	//do nothing else
	else
	{
		int i = 0;
		for (auto& it : m_jointNames)
		{
			m_positions.data[i] = 0;
			m_velocities.data[i] = 0;
			m_rosJointState.header.stamp = ros::Time::now();
			m_rosJointState.position[i] = 0;
			m_rosJointState.velocity[i] = 0;
			++i;
		}
	}
}

boost::shared_ptr<urdf::JointLimits> RobotArm::getJointLimits(const std::string& name)
{
	return m_urdf.joints_[name]->limits;
}

void RobotArm::getChainJointLimits(std::vector<urdf::JointLimits>& out)
{
	std::string jointName;
	for (auto& seg : m_jointNames)
	{
		out.push_back(*m_urdf.joints_[seg]->limits);
	}
}

sensor_msgs::JointState RobotArm::getROSJointPose() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_rosJointState;
}

KDL::JntArray RobotArm::getKDLJointState() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_positions;
}

KDL::JntArray RobotArm::getKDLVelocities() const
{
	return m_velocities;
}

void RobotArm::receiveData(const ros::Time& time,
		const ros::Duration& period)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->read(time, period);

		//set data
		RobotInterface::Data& data = m_interface->m_data;

		boost::recursive_mutex::scoped_lock lock(m_mutex);

		//fill kdl vectors
		int i = 0;
		for (auto& it : m_jointNames)
		{
			RobotInterface::JointData& j = data[it];
			m_positions.data[i] = j.pos;
			m_velocities.data[i] = j.vel;
			m_rosJointState.header.stamp = ros::Time::now();
			m_rosJointState.position[i] = j.pos;
			m_rosJointState.velocity[i] = j.vel;

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

	//no data => return
	if (jointState->name.empty())
		return;

	//wronge joint state => return
	if (std::find(jointState->name.begin(), jointState->name.end(), m_jointNames[0]) == jointState->name.end())
		return;

	//Create a mapping between name and id of the message
	for (size_t i = 0; i < jointState->name.size(); ++i)
	{
		mapping[jointState->name[i]] = i;
	}

	boost::recursive_mutex::scoped_lock lock(m_mutex);

	for (size_t i = 0; i < m_jointNames.size(); ++i)
	{
		//get id
		const int id = mapping[m_jointNames[i]];

		//get data
		pos = jointState->position[id];

		//since velocity is not allways send
		if (jointState->velocity.size() >= m_jointNames.size())
			vel = jointState->velocity[id];

		m_positions.data[i] = pos;
		m_velocities.data[i] = vel;
		m_rosJointState.header.stamp = ros::Time::now();
		m_rosJointState.position[i] = pos;
		m_rosJointState.velocity[i] = vel;
	}

	m_receivedState = true;
}

void RobotArm::sendVelocity(const ros::Time& time,
		const ros::Duration& period,
		const KDL::JntArray& cmd)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->setJointVelocityCommand(cmd, m_jointNames);

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
		m_interface->setJointTorqueCommand(cmd, m_jointNames);

		if (!m_interface->write(time, period))
			LOG_ERROR("Cannot send data!");
	}
}

void RobotArm::sendJointPosition(const KDL::JntArray& joints)
{
	if (c_armConfig.executionInterface == ArmExecutionMode::HardwareInterface && !m_passiveMode)
	{
		m_interface->setJointPositionCommand(joints, m_jointNames);

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

boost::shared_ptr<KDL::ChainFkSolverPos_recursive> RobotArm::getNewFkSolverInstance() const
{
	return boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(m_chain));
}

//const boost::shared_ptr<KDL::ChainIkSolverPos> RobotArm::getIkSolver() const
//{
//	return m_ikSolver;
//}

//const boost::shared_ptr<KDL::ChainJntToJacSolver> RobotArm::getJacobianSolver() const
//{
//	return m_jacobianSolver;
//}

void RobotArm::sampleValidEEFPose(Eigen::Affine3d& pose,
		KDL::JntArray& joints,
		const double borderAvoidance)
{
	sampleValidJointState(joints, borderAvoidance);
	getFK(joints, pose);
}

void RobotArm::sampleValidJointState(KDL::JntArray& jointState,
		const double borderAvoidance)
{
	std::default_random_engine re(std::chrono::system_clock::now().time_since_epoch().count());

	jointState.resize(m_jointNames.size());

	double lower, upper;

	int i = 0;
	for (auto& it : m_jointNames)
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
	KDL::JntArray init(m_jointNames.size());
	return getIKWithInit(init, pose, ik);
}

bool RobotArm::getIKWithInitCurrent(const Eigen::Affine3d& pose,
		KDL::JntArray& ik)
{
	return getIKWithInit(getKDLJointState(), pose, ik);
}

bool RobotArm::getIKWithInit(const KDL::JntArray& init,
		const Eigen::Affine3d& pose,
		KDL::JntArray& ik)
{
	boost::recursive_mutex::scoped_lock lock(m_kdlMutex);
	KDL::Frame x;
	tf::transformEigenToKDL(pose, x);
	return m_ikSolver->CartToJnt(init, x, ik) >= 0;
}

bool RobotArm::getFK(const KDL::JntArray& joints,
		Eigen::Affine3d& pose)
{
	boost::recursive_mutex::scoped_lock lock(m_kdlMutex);
	KDL::Frame x;
	m_fkSolver->JntToCart(joints, x);
	tf::transformKDLToEigen(x, pose);
	double r,p,y;
	x.M.GetRPY(r,p,y);

	return true;
}

bool RobotArm::getCurrentFK(Eigen::Affine3d& pose)
{
	KDL::JntArray joints = getKDLJointState();
	return getFK(joints, pose);
}

const KDL::Chain& RobotArm::getChain() const
{
	return m_chain;
}

const std::vector<std::string>& RobotArm::getJointNames() const
{
	return m_jointNames;
}

const KDL::Tree& RobotArm::getRobot() const
{
	return m_robot;
}

void RobotArm::getNewFKandIK(KDL::ChainIkSolverVel_pinv*& ikVel,
		KDL::ChainFkSolverPos_recursive*& fk,
		KDL::ChainIkSolverPos_NR_JL*& ik)
{
	std::vector<urdf::JointLimits> limits;
	getChainJointLimits(limits);

	KDL::JntArray upper(limits.size()), lower(limits.size());
	for (size_t i = 0; i < limits.size(); ++i)
	{
		upper(i, 0) = limits[i].upper - 0.05;
		lower(i, 0) = limits[i].lower + 0.05;
	}

	ikVel = new KDL::ChainIkSolverVel_pinv(m_chain);
	fk = new KDL::ChainFkSolverPos_recursive(m_chain);
	ik = new KDL::ChainIkSolverPos_NR_JL(m_chain, lower, upper, *fk, *ikVel, 100, 1e-2);
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
	boost::recursive_mutex::scoped_lock lock(m_mutex);

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
	m_fkSolver.reset(new KDL::ChainFkSolverPos_recursive(m_chain));
	m_ikVelSolver.reset(new KDL::ChainIkSolverVel_pinv(m_chain));
	m_ikSolver.reset(new KDL::ChainIkSolverPos_NR_JL(m_chain, m_limitMin, m_limitMax, *m_fkSolver, *m_ikVelSolver));
	m_jacobianSolver.reset(new KDL::ChainJntToJacSolver(m_chain));

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

void RobotArm::setJointState(const KDL::JntArray& joints)
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	m_positions = joints;

	for (size_t i = 0; i < joints.rows(); ++i)
		m_rosJointState.position[i] = joints(i);
}

Eigen::Affine3d RobotArm::getTcp() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_tcp;
}

bool RobotArm::isUseTcp() const
{
	boost::recursive_mutex::scoped_lock lock(m_mutex);
	return m_useTCP;
}

} /* namespace prm_planner */

