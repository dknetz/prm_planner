/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Jul 22, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: problem_definition.h
 */

#ifndef HF2F83269_14DF_4C06_87AC_BC4EF7A4E10F
#define HF2F83269_14DF_4C06_87AC_BC4EF7A4E10F

#include <ais_definitions/class.h>
#include <ais_ros/ros_base_interface.h>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <pcl/PointIndices.h>

#include <prm_planner/util/parameters.h>
#include <prm_planner/util/defines.h>

#include <Eigen/Geometry>

#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>

//Read and write lock which locks m_mutex
#define PD_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex);
#define PD_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex);
#define PD_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex);
#define PD_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex);

FORWARD_DECLARE_N(octomap, OcTree);
FORWARD_DECLARE_N(ais_point_cloud, RGBDImage);

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(Constraint);
FORWARD_DECLARE(PRMPlanner);
FORWARD_DECLARE(Path);
FORWARD_DECLARE(PathPlanner);
FORWARD_DECLARE(ObjectManager);
FORWARD_DECLARE(InteractiveMarker);
FORWARD_DECLARE(GraspableObject);
FORWARD_DECLARE(DroppingRegion);
FORWARD_DECLARE(PlanningScene);
FORWARD_DECLARE(PathDatabase);

/**
 * The Problem definition class specifies the complete
 * basic functionality:
 * 	- Loading & calling planners
 * 	- Handling collision checks
 * 	- Handling of objects
 * You can implement custom planning problems by deriving
 * a corresponding child class, in which you can adept
 * the functionality to your problem
 */
class ProblemDefinition: public boost::enable_shared_from_this<ProblemDefinition>
{
public:
	ProblemDefinition();
	virtual ~ProblemDefinition();

	/**
	 * Initializes the ProblemDefinition instance. You need
	 * to call this method after creating a new instance.
	 */
	virtual void init(const boost::shared_ptr<Robot> robot,
			const boost::shared_ptr<Constraint> constraint,
			const boost::shared_ptr<PRMPlanner> plannerInterface,
			const parameters::ProblemConfig& config);

	/**
	 * Resets the problem definition to its initial
	 * state:
	 *  - reset planer
	 */
	virtual void reset();

	/**
	 * This method needs to be implemented to if the
	 * planner needs to be able to react to environmental
	 * changes. The method is called frequently. One should
	 * make sure, that the call doesn't block the path planning
	 * algorithm. The default behavior is simply storing
	 * both values in the problem definition.
	 *
	 * @planningScene [in]: the planning scene
	 */
	virtual void update(const boost::shared_ptr<PlanningScene>& planningScene);

	/**
	 * Samples a task pose. It's used e.g. for sampling PRMs
	 */
	virtual Eigen::Affine3d samplePose();

//	/**
//	 * Given a task pose the method computes the poses
//	 * for all arms
//	 */
//	virtual void computePosesForArms(const Eigen::Affine3d& taskPose,
//			std::unordered_map<std::string, Eigen::Affine3d>& armPoses) = 0;

	/**
	 * Returns the current task pose
	 * TODO: Add flag if there is no current task pose
	 */
	virtual Eigen::Affine3d getCurrentTaskPose();

	/**
	 * Computes the pose where to go first before running
	 * the actual task (e.g., approaching a tray)
	 * @return: true if startPose could be computed, otherwise false
	 */
	virtual bool findStartTaskPose(Eigen::Affine3d& startPose,
			bool& hasToApproach);

	/**
	 * This method updates the tool frames of the
	 * robot arm(s). The default behaviour is doing
	 * nothing.
	 */
	virtual bool updateToolFrames();

	/**
	 * Planning method, which is called, if a planning
	 * is requested. This method assumes that the goal
	 * pose is in task space. It has to compute the
	 * goals in each robots planning space.
	 *
	 * @goal [in]:			the goal
	 * @path [out]: 		the path of the robots
	 * @parameters [in]:
	 *
	 * @return: true, if successful, otherwise false
	 */
	virtual bool plan(const Eigen::Affine3d& goal,
			boost::shared_ptr<Path>& path,
			const PlanningParameters& parameters);

	/**
	 * Planning method, which is called, if a planning
	 * is requested. This method assumes that the goal
	 * pose is in task space. It has to compute the
	 * goals in each robots planning space. Additionally,
	 * you can specify the start pose.
	 *
	 * @startJoint [in]:	the joint position of the start pose
	 * @startPose [in]:		the corresponding task pose
	 * @goal [in]:			the goal
	 * @path [out]: 		the path of the robots
	 * @parameters [in]:
	 *
	 * @return: true, if successful, otherwise false
	 */
	virtual bool plan(const KDL::JntArray& startJoint,
			const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goalPose,
			boost::shared_ptr<Path>& path,
			const PlanningParameters& parameters);

	//visualization
	virtual std::vector<boost::shared_ptr<InteractiveMarker>> getInteractiveMarkers();

	/**
	 * Visualization method, which is called periodically to
	 * publish e.g. ROS visualization messages
	 */
	virtual void publish();

	/**
	 * Returns the workspace radius, which
	 * is used to compute the relevant part
	 * of the point cloud to reduce computing
	 * power
	 * @return: 1.3 (ok for our robots)
	 */
	virtual double getWorkspaceRadius();

	/**
	 * Returns the workspace center, which
	 * is used to compute the relevant part
	 * of the point cloud to reduce computing
	 * power. The center point must be provided
	 * in the planning frame.
	 * @return: (0, 0, 0)
	 */
	virtual Eigen::Vector3d getWorkspaceCenter();

	//get/set
	virtual const std::string& getFrame() const;
	virtual const std::string& getRootFrame() const;
	virtual const parameters::ProblemConfig& getConfig() const;
	virtual parameters::ProblemConfig getConfig();
	virtual const boost::shared_ptr<Constraint>& getConstraint() const;
	virtual const boost::shared_ptr<Robot>& getRobot() const;
	virtual boost::shared_ptr<PathPlanner> getPathPlanner() const;

	/**
	 * Loads an robot interface from a ROS plugin.
	 * @package: the ROS package name
	 * @library: namespace::class_name
	 */
	static boost::shared_ptr<ProblemDefinition> load(const std::string& package,
			const std::string& library);

protected:
	/**
	 * Initializes the motion planner
	 */
	virtual void initPlanner();

	/**
	 * Default planning behavior: By setting
	 * a goal, the planner tries to find a path
	 * from start to goal and considers constraints
	 *
	 * @startJoint [in]: the start joints. If you
	 * 		want to use the current joint poses, just
	 * 		provide an empty KDL::JntArray map.
	 */
	virtual bool planDefault(const KDL::JntArray& startJoint,
			const Eigen::Affine3d& startPose,
			const Eigen::Affine3d& goal,
			boost::shared_ptr<Path>& path,
			const PlanningParameters& params);

	//#########################################################################
	// GRASPING ###############################################################
	//#########################################################################

	/**
	 * Grasps an object, which state is known to
	 * the object manager.
	 * @pre-condition: The gripper is empty
	 * @object [in]: the name of the object, which
	 * 		needs to be grasped
	 * @path [out]:
	 */
	virtual bool planGrasping(const std::string& object,
			boost::shared_ptr<Path>& path);

	//#########################################################################
	// DROPPING ###############################################################
	//#########################################################################
	/**
	 * Drops an object, which state is known to
	 * the object manager.
	 * @pre-condition: There is an corresponding object
	 * 		in the gripper.
	 * @object [in]: the name of the object, which
	 * 		needs to be droped
	 * @goal [in]: the goal pose of the object
	 * @path [out]:
	 */
	virtual bool planDropping(const std::string& object,
			const Eigen::Affine3d& goal,
			boost::shared_ptr<Path>& path);

	/**
	 * Runs a RanSaC on the current environment point cloud until
	 * a horizontal plane was found, which is large enough to be able
	 * to place the object on it.
	 *
	 * @object [in]: the object, which needs to be placed
	 * @pose [out]: the pose of the object
	 *
	 * @return: true, if a position was found
	 */
	virtual bool findDropPosition(const boost::shared_ptr<GraspableObject>& object,
			Eigen::Affine3d& pose);

protected:
	mutable boost::shared_mutex m_mutex;
	mutable boost::recursive_mutex m_droppingRegionMutex;

	boost::shared_ptr<Robot> m_robot;
	boost::shared_ptr<Constraint> m_constraint;
	parameters::ProblemConfig c_config;

	//the planner interface
	boost::shared_ptr<PRMPlanner> m_plannerInterface;
	boost::shared_ptr<PathDatabase> m_pathDatabase;

	//the current task motion planner (e.g., PRMAStar or RRT)
	//it is used for all task related planner queries. If you
	//just need to move a single arm, or without any constraints
	//you can use the arm planners.
	boost::shared_ptr<PathPlanner> m_planner;

	//the planning scene
	boost::shared_ptr<PlanningScene> m_planningScene;

	//object manager
	ObjectManager* m_objectManager;

	boost::shared_ptr<DroppingRegion> m_droppingRegion;

	Eigen::Affine3d m_transformationPlanningToArm;

	std::vector<boost::shared_ptr<InteractiveMarker>> m_interactiveMarker;

private:
	/**
	 * The object needs to live the complete lifetime
	 * of the program. Otherwise we get an annoying warning
	 */
	static pluginlib::ClassLoader<ProblemDefinition>* s_loader;
};

} /* namespace prm_planner */

#endif /* HF2F83269_14DF_4C06_87AC_BC4EF7A4E10F */
