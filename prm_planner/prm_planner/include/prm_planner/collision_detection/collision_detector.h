/*
 * collision_detector.h
 *
 *  Created on: Feb 4, 2017
 *      Author: kuhnerd
 */

#ifndef PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_COLLISION_DETECTION_COLLISION_DETECTOR_H_
#define PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_COLLISION_DETECTION_COLLISION_DETECTOR_H_

#include <ais_definitions/class.h>
#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <vector>

FORWARD_DECLARE_N(fcl_robot_model, RobotModel);
FORWARD_DECLARE_N(fcl_collision_detection, FCLWrapper);
FORWARD_DECLARE_N(octomap, OcTree);

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(GraspableObject);
FORWARD_DECLARE(PlanningScene);

typedef boost::shared_ptr<fcl_robot_model::RobotModel> FCLRobotModel;

class CollisionDetector
{
public:
	/**
	 * Struct which is used to allow each thread to create its own
	 * collision detector instances. The instances are created only
	 * once at the beginning.
	 */
	struct OpenMPCollisionDetectorStruct
	{
		//cdWithObject gets only initialized if objects is not empty!
		boost::shared_ptr<CollisionDetector> cdWithObject, cdWithoutObject;

		boost::shared_ptr<Robot> robot;
		boost::shared_ptr<PlanningScene> planningScene;
		std::vector<boost::shared_ptr<GraspableObject>> objects;

		//used to initialize collision detectors
		OpenMPCollisionDetectorStruct(OpenMPCollisionDetectorStruct& other);

		//used for initialization
		OpenMPCollisionDetectorStruct(boost::shared_ptr<Robot> robot,
				boost::shared_ptr<PlanningScene> planningScene,
				std::vector<boost::shared_ptr<GraspableObject>> objects = std::vector<boost::shared_ptr<GraspableObject>>());
	};

public:
	/**
	 * Creates a collision detector. 'robotInterface' will only
	 * be used to get general information about robot description
	 * and gripper joint state. If an 'octomap' is povided (i.e., no NULL)
	 * it will be deep copied for collision detection. Finally,
	 * ignoreObjects can be used to specify objects, which will
	 * be removed from the octomap.
	 */
	CollisionDetector(boost::shared_ptr<Robot> robotInterface,
			boost::shared_ptr<PlanningScene> planningScene,
			std::vector<boost::shared_ptr<GraspableObject>> ignoreObjects = std::vector<boost::shared_ptr<GraspableObject>>());
	virtual ~CollisionDetector();

	boost::shared_ptr<fcl_collision_detection::FCLWrapper> fcl;
	FCLRobotModel robot;
};

} /* namespace prm_planner */

#endif /* PRM_PLANNER_PRM_PLANNER_INCLUDE_PRM_PLANNER_COLLISION_DETECTION_COLLISION_DETECTOR_H_ */
