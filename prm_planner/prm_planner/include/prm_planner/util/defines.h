/*
 * defines.h
 *
 *  Created on: Jan 28, 2016
 *      Author: kuhnerd
 */

#ifndef ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_DEFINES_H_
#define ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_DEFINES_H_

#include <string.h>

namespace prm_planner
{

/**
 * Planning Modes:
 * 	- Default: Calls the plan() method of the
 * 		problem definition. If no plan method
 * 		was implemented, the default plan method
 * 		of ProblemDefinition will be called
 * 	- GraspObject: Grasps an object, which must
 * 		be known to the object manager
 * 	- DropObject: Drops an object to a known
 * 		pose on a stable ground
 */
enum PlanningMode
{
	Default,
	GraspObject,
	DropObject
};

/**
 * The planning parameters are used to specify
 * planning related parameters such as:
 * - directConnectionRequired: if true the planner
 * 		only tries to directly connect the start and
 * 		goal planner by using a controller (no planning
 * 		at all)
 * - mode: The planning mode (default, grasp, drop)
 * 		see @PlanningMode
 * - objectName: the object name is used for grasping
 * 		and dropping
 * - input1/2: additional parameters which can be used
 * 		without restriction in user implementations
 * 		of the problem definition
 */
struct PlanningParameters
{
	PlanningParameters() :
					directConnectionRequired(false),
					useCollisionDetection(true),
					mode(Default)
	{
	}

	bool directConnectionRequired;
	bool useCollisionDetection;
	PlanningMode mode;
	std::string objectName;
	std::vector<std::string> input;
};

}

#endif /* ELASTIC_ROADMAP_TEST_ELASTIC_ROADMAP_PLANNER_INCLUDE_ELASTIC_ROADMAP_PLANNER_DEFINES_H_ */
