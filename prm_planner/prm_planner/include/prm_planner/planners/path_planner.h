/*
 * planner.h
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#ifndef HF1A31456_1AEC_4DAD_B6BA_FB983CB30FDB
#define HF1A31456_1AEC_4DAD_B6BA_FB983CB30FDB

#include <ais_definitions/class.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread.hpp>
#include <kdl/jntarray.hpp>
#include <prm_planner_robot/defines.h>
#include <Eigen/Geometry>

//Read and write lock which locks m_mutex
#define PLANNER_READ_LOCK() boost::shared_lock<boost::shared_mutex> lock(m_mutex)
#define PLANNER_WRITE_LOCK() boost::unique_lock< boost::shared_mutex > lock(m_mutex)
#define PLANNER_UPGRADABLE_LOCK() boost::upgrade_lock<boost::shared_mutex> lock(m_mutex)
#define PLANNER_UPGRADE_LOCK() boost::upgrade_to_unique_lock<boost::shared_mutex> uniqueLock(m_mutex)

namespace prm_planner
{

FORWARD_DECLARE(Robot);
FORWARD_DECLARE(ProblemDefinition);
FORWARD_DECLARE(Path);
FORWARD_DECLARE(CollisionDetector);
FORWARD_DECLARE(PlanningScene);

class PathPlanner
{
public:
	PathPlanner(boost::shared_ptr<ProblemDefinition> pd);
	virtual ~PathPlanner();

	/**
	 * Default planning method with start and goal
	 *
	 * Important: call the this method in the first line
	 * of your implementation of planSingleStartMultipleGoal
	 * to be able to use the OneImagePerPlan mode!
	 */
	virtual bool plan(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path,
			bool directConnectionRequired = false);

	/**
	 * Use this method if you have specified start
	 * and goal in the PRM. It uses the next node as
	 * start pose and tries to find a goal node.
	 * startNodeId identifies the start node
	 *
	 * Important: call the this method in the first line
	 * of your implementation of planSingleStartMultipleGoal
	 * to be able to use the OneImagePerPlan mode!
	 */
	virtual bool planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const int startNodeId,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

	/**
	 * This method needs to be implemented to if the
	 * planner needs to be able to react to environmental
	 * changes. The method is called frequently. One should
	 * make sure, that the call doesn't block the path planning
	 * algorithm. Call this method from the derived method
	 * to store them in the planner.
	 * @planningScene [in]: the planning scene
	 */
	virtual void update(const boost::shared_ptr<prm_planner::PlanningScene>& planningScene);

	/**
	 * This method resets the planner to its original
	 * state. The default behavior is to do nothing.
	 */
	virtual void reset();

	/**
	 * This method is called frequently to publish visualization
	 * messages. The default behavior is to do nothing.
	 */
	virtual void publish();

	const boost::shared_ptr<ProblemDefinition> getProblemDefinition() const;

	void stopAllThreads();
	void resetStopAllThreads();

protected:
	mutable boost::shared_mutex m_mutex;
	boost::shared_ptr<PlanningScene> m_planningScene;

	boost::shared_ptr<Robot> m_robot;
	boost::shared_ptr<ProblemDefinition> m_pd;

	boost::atomic_bool m_stopAllThreads;
};

} /* namespace prm_planner */

#endif /* HF1A31456_1AEC_4DAD_B6BA_FB983CB30FDB */
