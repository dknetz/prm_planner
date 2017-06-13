/*
 * sample_allowed_collisions.cpp
 *
 *  Created on: Aug 27, 2016
 *      Author: kuhnerd
 */
#include <ais_log/log.h>
#include <ais_util/progress_bar.h>
#include <boost/shared_ptr.hpp>
#include <fcl_wrapper/collision_detection/collision_matrix.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/robot_model/robot_link.h>
#include <fcl_wrapper/robot_model/robot_joint.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <ros/ros.h>

using namespace fcl_robot_model;
using namespace fcl_collision_detection;

boost::shared_ptr<RobotModel> robot;

//void findSameChildLinks(RobotLink::JointMap& childs,
//		RobotLink* current,
//		std::vector<std::string>& links)
//{
//	for (auto& it : childs)
//	{
//		if (it.second->getChildLink().second->getTransformationToParentLink().isApprox(Eigen::Affine3d::Identity()))
//		{
//			links.push_back(it.second->getChildLink().second->getName());
//			findSameChildLinks(it.second->getChildLink().second->getChildJoints(), it.second->getChildLink().second.get(), links);
//		}
//	}
//}
//
//void getZeroTransformLinks(RobotLink* link,
//		std::vector<std::string>& links)
//{
//	RobotLink* current = link;
//	RobotLink* parent = link->getParentLink().get();
//	while (parent != NULL)
//		if (current->getTransformationToParentLink().isApprox(Eigen::Affine3d::Identity()))
//		{
//			links.push_back(parent->getName());
//			parent = parent->getParentLink().get();
//			current = parent;
//		}
//		else
//			break;
//
//	current = link;
//	RobotLink::JointMap childs = link->getChildJoints();
//	findSameChildLinks(childs, current, links);
//
//	for (auto& it : links)
//	{
//		LOG_INFO(link->getName() << "=" << it);
//	}
//}

bool isParentOrChild(RobotLink* l1,
		RobotLink* l2)
{
	std::string first = l1->getName();
	std::string second = l2->getName();

	RobotLink* parentOfL1 = l1->getParentLink().get();
	RobotLink* parentOfL2 = l2->getParentLink().get();

	//check if the objects are neighbors
	//l2 is parent of l1
	if (parentOfL1 != NULL && parentOfL1->getName() == second)
	{
		return true;
	}
	else if (parentOfL2 != NULL && parentOfL2->getName() == first)
	{
		return true;
	}

//	std::vector<std::string> sameLink;
//	getZeroTransformLinks(l1, sameLink);
//
//	return std::find(sameLink.begin(), sameLink.end(), second) != sameLink.end();

	return false;
}

int main(int argc,
		char** argv)
{
	if (argc != 4)
	{
		LOG_WARNING("please provide the filename where the collision matrix should be stored and how many samples should be tested:");
		LOG_WARNING("filename samples robot_description");
		exit(-1);
	}

	const std::string filename = std::string(argv[1]);
	const int samples = std::atoi(argv[2]);
	const std::string robotDescription = std::string(argv[3]);

	LOG_INFO("Testing " << samples << " samples...");

	ros::init(argc, argv, "sample_allowed_collisions");
	ros::NodeHandle n;

	robot.reset(new RobotModel(robotDescription));
	RobotState state;

	FCLWrapper fcl(""); //world frame doesn't matter here
	fcl.setUseCollisionMatrix(false);
	FCLWrapper::CollisionsVectorObjects collisions;

	CollisionMatrix cm;

	fcl.addObject(robot);

	ais_util::ProgressBar p("testing", samples);

	for (int i = 0; i < samples; ++i)
	{
		p.set(i);

		robot->sampleValidJoints(state);
		robot->setRobotState(state);

//		robot->print();

		if (fcl.checkPairwiseCollisions(true))
		{
			fcl.getPairwiseCollisions(collisions);

			for (auto& it : collisions)
			{
				RobotLink* o1 = static_cast<RobotLink*>(it.first);
				RobotLink* o2 = static_cast<RobotLink*>(it.second);

				if (isParentOrChild(o1, o2))
				{
//					LOG_INFO("parent/child collision found between " << it.first->getName() << " and " << it.second->getName());
				}
				else
				{
					cm.set(o1->getName(), o2->getName(), true);
				}
			}
		}

		if (!ros::ok())
		{
			return 0;
		}
	}

	p.finish();

	LOG_WARNING("Allow all collision checks with environment. Please remove them (partly) in the file if you don't need them!");
	for (auto& it : robot->getObjects())
	{
		cm.set(it.first, "environment", true);
	}

	LOG_INFO("Found following collisions:");

	cm.print();

	cm.write(filename);
}

