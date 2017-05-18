/*
 * a_star_planner.h
 *
 *  Created on: Jan 3, 2016
 *      Author: daniel
 */

#ifndef A_STAR_H_
#define A_STAR_H_

#include <ais_definitions/class.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/planners/path_planner.h>
#include <prm_planner_robot/defines.h>
#include <prm_planner/util/defines.h>
#include <prm_planner/util/parameters.h>
#include <unordered_map>

namespace prm_planner
{

FORWARD_DECLARE(PRM);
FORWARD_DECLARE(PRMView);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(PRMNode);
FORWARD_DECLARE(PRMEdge);
FORWARD_DECLARE(Path);

class PRMAStar: public PathPlanner
{
public:
	struct AStarNode;
	struct AStarNodeComparator
	{
		bool operator()(const AStarNode* n1,
				const AStarNode* n2) const
				{
			return n1->f > n2->f;
		}
	};

	struct AStarNode
	{
		double h;
		double g;
		double f;
		const PRMNode* node;
		const PRMEdge* edgeToThisNode;
		AStarNode* predecessor;
		uint64_t id;
		KDL::JntArray startJoints;
		boost::heap::fibonacci_heap<AStarNode*, boost::heap::compare<AStarNodeComparator>>::handle_type handle;

		AStarNode(const PRMNode* node,
				const PRMEdge* edgeToThisNode,
				const uint64_t id,
				AStarNode* pre,
				const double g);

		void updateF();
	};

	typedef std::unordered_map<uint64_t, AStarNode*> AStarNodeMap;
	typedef std::list<const PRMEdge*> EdgeList;
	typedef std::unordered_map<uint64_t, int> CounterMap;
	typedef boost::heap::fibonacci_heap<AStarNode*, boost::heap::compare<AStarNodeComparator>> Heap;

	/**
	 * Constructor, which loads or generates a roadmap.
	 * If the roadmap can be found in the given file
	 * (problem definition config), the roadmap is loaded
	 * from this file. Otherwise a new roadmap is generated
	 * based on the given parameters.
	 *
	 * @pd [in]: problem definition
	 * @timeout [in]: the maximum planning time
	 */
	PRMAStar(boost::shared_ptr<ProblemDefinition> pd,
			const double timeout);

	/**
	 * Constructor, which takes an externally generated roadmap.
	 * The PRMAStar object takes the ownership of the PRM
	 * and deletes it in the destruction phase.
	 *
	 * @pd [in]: problem definition
	 * @prm [in]: the externally generated roadmap
	 * @timeout [in]: the maximum planning time
	 */
	PRMAStar(boost::shared_ptr<ProblemDefinition> pd,
			PRM* prm,
			const double timeout);

	virtual ~PRMAStar();

	/**
	 * @see PathPlanner
	 */
	virtual bool plan(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path,
			bool directConnectionRequired = false);

	/**
	 * @see PathPlanner
	 */
	virtual bool planSingleStartMultipleGoal(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const int startNodeId,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

	/**
	 * @see PathPlanner
	 */
	virtual void update(const boost::shared_ptr<PlanningScene> planningScene);

	/**
	 * @see PathPlanner
	 */
	virtual void publish();

	double getTimeout() const;
	void setTimeout(double timeout);

	/**
	 * Sets a new roadmap. The planner takes ownership of the
	 * object! You may not delete it by yourself.
	 *
	 * @prm [in]: the new roadmap.
	 */
	void setPRM(PRM* prm);

	/**
	 * @see PathPlanner
	 */
	virtual void initCollisionAvoidance();

private:
	virtual bool plan(const KDL::JntArray& currentJointPose,
			const Eigen::Affine3d& currentTaskPose,
			const Eigen::Affine3d& goalTaskPose,
			const PRMNode* startNode,
			const PRMNode* goalNode,
			PRMView* view,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

	void expandNode(AStarNode* goal,
			AStarNode* node,
			KDL::JntArray& goalJoints,
			AStarNodeMap& closedList,
			AStarNodeMap& openListElements,
			Heap& openList,
			PRMView* view,
			CounterMap& nodeCounter);

	//Extracts the path after planning was finished
	bool extractPath(AStarNode* goalNode,
			AStarNode* startNode,
			PRMView* view,
			Path& path,
			EdgeList& blockedEdges);

	double heuristic(AStarNode* n1,
			AStarNode* n2);

	bool testDirectConnection(const PRMNode* startNode,
			const PRMNode* goalNode,
			KDL::JntArray& startJoints,
			PRMView* view,
			boost::shared_ptr<CollisionDetector>& cd,
			boost::shared_ptr<Path>& path);

	/**
	 * Loads a probabilistic roadmap from a file, if
	 * it exists. Otherwise it generates a new one and
	 * saves it to a file.
	 *
	 * @config [in]: config
	 *
	 * @return: a pointer to the loaded/generated PRM. You
	 * 		need to make sure that the pointer is deleted
	 * 		after usage.
	 */
	PRM* loadPRM(const parameters::PRMConfig& config);

private:
	PRM* m_prm;
	double m_timeout;
};

} /* namespace prm_planner */

#endif /* A_STAR_H_ */
