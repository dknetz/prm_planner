/*
 * collision_detector.cpp
 *
 *  Created on: Feb 4, 2017
 *      Author: kuhnerd
 */

#include <boost/smart_ptr/shared_ptr.hpp>
#include <prm_planner/collision_detection/collision_detector.h>
#include <fcl_wrapper/robot_model/robot_model.h>
#include <fcl_wrapper/collision_detection/fcl_wrapper.h>
#include <fcl_wrapper/collision_detection/octomap.h>
#include <fcl_wrapper/collision_detection/box.h>
#include <fcl_wrapper/collision_detection/sphere.h>
#include <fcl_wrapper/robot_model/robot_state.h>
#include <prm_planner/environment/planning_object.h>
#include <prm_planner/environment/planning_scene.h>
#include <prm_planner/objects/graspable_object.h>
#include <prm_planner/objects/object_manager.h>
#include <prm_planner/problem_definitions/problem_definition_manager.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/robot/robot.h>

namespace prm_planner
{

CollisionDetector::OpenMPCollisionDetectorStruct::OpenMPCollisionDetectorStruct(OpenMPCollisionDetectorStruct& other) :
				robot(other.robot),
				planningScene(other.planningScene),
				objects(other.objects)
{
	cdWithObject.reset(new CollisionDetector(other.robot, other.planningScene));
	if (!other.objects.empty())
	{
		cdWithoutObject.reset(new CollisionDetector(other.robot, other.planningScene, other.objects));
	}
}

CollisionDetector::OpenMPCollisionDetectorStruct::OpenMPCollisionDetectorStruct(boost::shared_ptr<Robot> robot,
		boost::shared_ptr<PlanningScene> planningScene,
		std::vector<boost::shared_ptr<GraspableObject>> objects) :
				robot(robot),
				planningScene(planningScene),
				objects(objects)
{
}

CollisionDetector::CollisionDetector(boost::shared_ptr<Robot> robotInterface,
		boost::shared_ptr<PlanningScene> planningScene,
		std::vector<boost::shared_ptr<GraspableObject>> ignoreObjects)
{
	fcl.reset(new fcl_collision_detection::FCLWrapper(ProblemDefinitionManager::getInstance()->getProblemDefinition()->getRootFrame()));

	boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

	std::string gripperObject;

	//add robot
	boost::shared_ptr<fcl_robot_model::RobotModel> fclArm(
			new fcl_robot_model::RobotModel(robotInterface->getRobotDescription(), robotInterface->getName(),
					robotInterface->getName(), pd->getRootFrame(), robotInterface->getCollisionMatrixFile()));

	//set gripper position to current (since it rarely is explicitly used in planning)
	boost::shared_ptr<GripperInterface> gripper = robotInterface->getGripper();
	if (gripper.get() != NULL)
	{
		fclArm->setRobotState(gripper->getJoints());

		//check if there's an object in the gripper
		gripperObject = gripper->getCurrentObject();
		if (!gripperObject.empty())
		{
			boost::shared_ptr<GraspableObject> object = ObjectManager::getInstance()->getObject(gripperObject);
			ignoreObjects.push_back(object);
		}
	}

	fclArm->setRobotState(robotInterface->getAllJointStates());
	fcl->addObject(fclArm);
	robot = fclArm;

	//add world
	if (planningScene.get() != NULL)
	{
		//add octomap
		if (planningScene->octomap.get() != NULL && planningScene->octomap->size() > 0)
		{
			boost::shared_ptr<ProblemDefinition> pd = ProblemDefinitionManager::getInstance()->getProblemDefinition();

			//create copy of octomap
			planningScene->lock();
			boost::shared_ptr<octomap::OcTree> octomapCopy(new octomap::OcTree(*planningScene->octomap));
			planningScene->unlock();

			//remove objects from octomap
			for (auto& it : ignoreObjects)
			{
				//if a transformation is available, we remove it from the octomap
				//remove object in gripper either way
				if (it->isActive() || (!gripperObject.empty() && it->c_params.name == gripperObject))
				{
					it->removeObjectFromOctomap(octomapCopy);
				}
			}

			boost::shared_ptr<fcl_collision_detection::Octomap> fclOctomap(
					new fcl_collision_detection::Octomap(octomapCopy, "environment", pd->getFrame(), pd->getRootFrame()));
			fcl->addObject(fclOctomap);
		}

		//add objects
		planningScene->lock();
		for (auto& o : planningScene->objects)
		{
			boost::shared_ptr<fcl_collision_detection::PhysicalObject> fclObject;

			if (o.second->m_type == PlanningObject::BOX)
			{
				fclObject.reset(new fcl_collision_detection::Box(o.second->m_size[0], o.second->m_size[1], o.second->m_size[2],
						o.second->m_name, "", o.second->m_frame));
			}
			else if (o.second->m_type == PlanningObject::SPHERE)
			{
				fclObject.reset(new fcl_collision_detection::Sphere(o.second->m_size[0],
						o.second->m_name, "", o.second->m_frame));
			}
			else
			{
				LOG_FATAL("Unknown object type");
				continue;
			}

			fclObject->setTransform(o.second->m_transformation);

			fcl->addObject(fclObject, o.second->m_collisionMatrix.checkAllCollisions());
		}
		planningScene->unlock();
	}
}

CollisionDetector::~CollisionDetector()
{
}

} /* namespace prm_planner */
