/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 7, 2017
 *      Author: kuhnerd
 * 	  Filename: trajectory_optimizer.h
 */

#ifndef H1B5D056A_B8AE_4E4D_A681_13ED674AE923
#define H1B5D056A_B8AE_4E4D_A681_13ED674AE923
#include <ais_definitions/class.h>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace prm_planner
{

FORWARD_DECLARE(Path);
FORWARD_DECLARE(CollisionDetector);
FORWARD_DECLARE(Robot);
FORWARD_DECLARE(ProblemDefinition);

class TrajectoryOptimizer
{
public:
	TrajectoryOptimizer(boost::shared_ptr<Path>& path,
			boost::shared_ptr<CollisionDetector> cd,
			boost::shared_ptr<Robot> robot,
			boost::shared_ptr<ProblemDefinition> pd);
	virtual ~TrajectoryOptimizer();

	bool optimize(boost::shared_ptr<Path>& newPath, bool initialFeasibilityCheck);

private:
	boost::shared_ptr<Path> m_path;
	boost::shared_ptr<CollisionDetector> m_cd;
	boost::shared_ptr<Robot> m_robot;
	boost::shared_ptr<ProblemDefinition> m_pd;
};

} /* namespace prm_planner */

#endif /* H1B5D056A_B8AE_4E4D_A681_13ED674AE923 */
