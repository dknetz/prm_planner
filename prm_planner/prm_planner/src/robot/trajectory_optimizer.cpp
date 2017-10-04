/*
 * Copyright (c) 2017 kuhnerd.
 * All rights reserved.
 * 
 *  Created on: Jul 7, 2017
 *      Author: kuhnerd
 * 	  Filename: trajectory_optimizer.cpp
 */

#include <ais_util/stop_watch.h>
#include <prm_planner/collision_detection/collision_detector.h>
#include <prm_planner/controllers/helpers.h>
#include <prm_planner/controllers/simulation_robot_controller.h>
#include <prm_planner/problem_definitions/problem_definition.h>
#include <prm_planner/robot/robot.h>
#include <prm_planner/robot/trajectory_optimizer.h>
#include <prm_planner_robot/path.h>
#include <prm_planner_robot/trajectory.h>
#include <fstream>

namespace prm_planner
{

TrajectoryOptimizer::TrajectoryOptimizer(boost::shared_ptr<Path>& path,
		boost::shared_ptr<CollisionDetector> cd,
		boost::shared_ptr<Robot> robot,
		boost::shared_ptr<ProblemDefinition> pd) :
				m_path(path),
				m_cd(cd),
				m_robot(robot),
				m_pd(pd)
{
}

TrajectoryOptimizer::~TrajectoryOptimizer()
{
}

bool TrajectoryOptimizer::optimize(boost::shared_ptr<Path>& newPath,
		bool initialFeasibilityCheck)
{
	static const double sigma = 1.5;
	static const int numberOfJointStatesBetweenDenseWaypoints = 50;
	static const int iterations = 10;
	static const double minDistImprovementPerIter = 0.001;
	static const double minAngImprovementPerIter = 0.0025;
	static const double weightChangeInEachIteration = 0.7;

	//copy the path
	boost::shared_ptr<Path> oldPath;
	newPath.reset(new Path(*m_path));
	newPath->makeDense(numberOfJointStatesBetweenDenseWaypoints);
	boost::shared_ptr<Path> originalPath(new Path(*newPath));

	const int intSigma = 3.0 * sigma;
	const int pathSize = newPath->size();

	//+ moves point into "direct connection" direction, - away
	double weight = 1.0;

	//waypoints which cannot be moved. Therefore, we optimize
	//the trajectory based on such points (elastic band will use
	//these points)
	std::vector<size_t> notMovableWaypoints;
	notMovableWaypoints.push_back(0);
	notMovableWaypoints.push_back(pathSize - 1);
	size_t currentNonMovableWaypoint = 0;

	ControllerParameters params = getControllerParametersFromParameterServer(m_robot->getParameters().controllerConfig);
	params.optimizeJointRangeInNullspace = false;

	bool replaceOld = true;
	static int drawCounter = 0;
	int c2 = 0;

//	ais_util::StopWatch::getInstance()->start("optimize");
	//plot for [i=1:7] "/tmp/traj_1.traj" u i w l title "".i, for [i=8:14] "" u ((column(i)==0)?(1/0):column(i)) pt 7 lc (i-7) notitle
	for (int i = 0; i < iterations; ++i)
	{
		if (replaceOld)
			oldPath.reset(new Path(*newPath));

		double distImprovement = 0;
		double angImprovement = 0;

		//check if trajectory is possible at all
		if (i == 0)
		{
			if (initialFeasibilityCheck)
			{
				for (size_t j = 1; j < pathSize; ++j)
				{
					Path::Waypoint& wCurrent = (*newPath)[j];
					Path::Waypoint& wPrev = (*newPath)[j - 1];

					SimulationRobotController controller(wPrev.pose, wCurrent.pose, wPrev.jointPose, params, m_robot, m_pd);
					if (!controller.canControl(10000, 0.01))
					{
						LOG_WARNING("Path is not feasible at index " << j);
						return false;
					}
				}
			}
		}
		else
		{
			int lastError = -2;
			currentNonMovableWaypoint = 0;

			std::vector<Path::Waypoint> waypoints;
			for (auto& it : notMovableWaypoints)
			{
				Path::Waypoint& wp = (*newPath)[it];
				waypoints.push_back(wp);
			}

			Trajectory spline(waypoints);
			spline.writeGnuplotFile(std::string("/tmp/opt_") + std::to_string(drawCounter) + "_" + std::to_string(i) + "_" + std::to_string(c2) + ".traj");

			double pathLength = newPath->getPathLength();
			double currentLength = 0;

			for (int j = 1; j < pathSize; ++j)
			{
				bool last = j == newPath->size() - 1;
				Path::Waypoint& wCurrent = (*newPath)[j];
				Path::Waypoint& wPrev = (*newPath)[j - 1];
				Path::Waypoint& wNext = (*newPath)[last ? j : j + 1];

				Path::Waypoint& wOldCurrent = (*oldPath)[j];
				Path::Waypoint& wOldPrev = (*oldPath)[j - 1];
				Path::Waypoint& wOldNext = (*oldPath)[last ? j : j + 1];

				Eigen::Affine3d newPose = wCurrent.pose;

				if (!last)
				{
					size_t currentNonMovableIndex = notMovableWaypoints[currentNonMovableWaypoint];
					size_t currentNonMovableIndexNext = notMovableWaypoints[currentNonMovableWaypoint + 1];

					const Path::Waypoint& wStart = (*newPath)[currentNonMovableIndex];
					const Path::Waypoint& wEnd = (*newPath)[currentNonMovableIndexNext];
					const Eigen::Vector3d& pStart = wStart.pose.translation();
					const Eigen::Vector3d& pEnd = wEnd.pose.translation();
					const Eigen::Vector3d& pCurrent = wCurrent.pose.translation();

					Trajectory::Pose pSline;

					spline.getPoseFromT((double)currentNonMovableWaypoint + (pCurrent - pStart).norm() / (pEnd - pStart).norm(), pSline);

					if (j >= currentNonMovableIndexNext)
					{
						++currentNonMovableWaypoint;
					}

					//https://en.wikibooks.org/wiki/Linear_Algebra/Orthogonal_Projection_Onto_a_Line
//					const Eigen::Vector3d v = pCurrent - pStart;
//					const Eigen::Vector3d s = pEnd - pStart;
//					Eigen::Vector3d proj = (v.dot(s) / s.dot(s)) * s;
//					Eigen::Vector3d orth = v - proj;
					Eigen::Vector3d orth = pCurrent - pSline.x.head(3);
					LOG_INFO(j << " " << currentNonMovableWaypoint << " " << orth.transpose());

					//check border conditions: if we get close to
					//borders we decrease the width for the following
					//loop to avoid moving the waypoints to the center
					//of the path
					int lowerBound = j - intSigma;
					int upperBound = j + intSigma;
					if (lowerBound < 0)
					{
						lowerBound = 0;
						upperBound = 2 * j;
					}
					if (upperBound >= pathSize)
					{
						lowerBound = j - (pathSize - 1 - j);
						upperBound = pathSize - 1;
					}

					//use normal distribution to compute smooth path
					double sum = 0;
					Eigen::Vector3d pos = Eigen::Vector3d::Zero();
					for (double k = lowerBound; k <= upperBound; ++k)
					{
						double ratio = exp(-((k - j) * (k - j)) / (2.0 * sigma * sigma)); /* 1.0 / sqrt(2.0 * M_PI * sigma) */
						sum += ratio;
						pos += ratio * (*oldPath)[k].pose.translation();
					}

					//compute new position
					newPose.translation() = pos / sum - weight * (0.1 * orth);

					//and orientation
					Eigen::Quaterniond qNew = Eigen::Quaterniond(wOldPrev.pose.linear()).slerp(0.5, Eigen::Quaterniond(wOldNext.pose.linear()));
					newPose.linear() = qNew.toRotationMatrix();

					//compute the change of the current iteration
					//to be able to stop as soon as there is no
					//change anymore
					distImprovement += (wCurrent.pose.translation() - newPose.translation()).norm();
					angImprovement += Eigen::Quaterniond(wCurrent.pose.linear()).angularDistance(qNew);

//					LOG_INFO(i << " " << j << " " << (weight * (0.1 * orth)));
//					newPose.translation() = 0.25 * wOldPrev.pose.translation() + 0.5 * wOldCurrent.pose.translation() + 0.25 * wOldNext.pose.translation() - weight * (0.1 * orth);
				}

				SimulationRobotController controller(wPrev.pose, newPose, wPrev.jointPose, params, m_robot, m_pd);
				controller.setCollisionDetection(m_cd);
				if (controller.canControl(10000, 0.01))
				{
					if (!last)
					{
						ArmJointPath path;
						KDL::JntArray jNew;
						controller.getFinalJointState(jNew);
						controller.getJointPath(path);
						controller.updateFromValues(newPose, wNext.pose, jNew);
						if (controller.canControl(10000, 0.01))
						{
							wCurrent.pose = newPose;
							wCurrent.jointPose = jNew;
							wPrev.trajectory = path;

							//update joint state of next node
							controller.getFinalJointState(wNext.jointPose);
							controller.getJointPath(wCurrent.trajectory);
						}
						else
						{
							//same error for two neighboring waypoints, i.e., we connect two
							//old waypoints. Since start joints at previous waypoint might have
							//changed we need to try to re-connect the waypoints
							if (j == lastError + 1)
							{
								controller.updateFromValues(wPrev.pose, wCurrent.pose, wPrev.jointPose);
								if (controller.canControl(10000, 0.01))
								{
									controller.getFinalJointState(wCurrent.jointPose);
									controller.getJointPath(wPrev.trajectory);
								}
								else
								{
									LOG_ERROR("no further optimization possible (case 2): " << j << ", it " << i);
									newPath = oldPath;
									replaceOld = false;
									weight *= weightChangeInEachIteration;
									break;
								}
							}

							lastError = j;
						}
					}
					else
					{
//						wCurrent.pose = newPose;
						controller.getFinalJointState(wCurrent.jointPose);
						controller.getJointPath(wPrev.trajectory);
					}
				}
				else
				{
					controller.updateFromValues(wPrev.pose, wCurrent.pose, wPrev.jointPose);
					if (controller.canControl(10000, 0.01))
					{
						controller.getFinalJointState(wCurrent.jointPose);

						controller.getJointPath(wPrev.trajectory);

						LOG_INFO("Not movable " << j);

						if (std::find(notMovableWaypoints.begin(), notMovableWaypoints.end(), j) == notMovableWaypoints.end())
						{
							notMovableWaypoints.push_back(j);
							std::sort(notMovableWaypoints.begin(), notMovableWaypoints.end());
							LOG_INFO(notMovableWaypoints);
							currentNonMovableWaypoint = 0;
							i = 0;
							weight = 1.0;
							newPath.reset(new Path(*originalPath));
							replaceOld = true;
							c2++;
							break;
						}

//						++allowChange[j];
					}
					else
					{
//						wPrev.pose = wOldPrev.pose;
//						allowChange[j - 1] += 1;
//						int oldJ = j;
//						j -= 1 + allowChange[j - 1];
//
//						if (j < 0)
//						{
						LOG_ERROR("no further optimization possible: " << j << ", it " << i);

						//because of smaller weights it might work in the next iteration.
						//therefore, we just continue with the old path and lower weights
						newPath = oldPath;
						replaceOld = false;
						weight *= weightChangeInEachIteration;
						break;
//						goto finish;
//						}
//
//						for (int l = oldJ - 2; l >= j; --l)
//						{
//							allowChange[l] += 1;
//						}
//						LOG_ERROR("backtracking: " << j << ", it " << i);
					}
				}

				//reduce weight to smooth path at the end and don't move it to the connecting line
				weight *= weightChangeInEachIteration;
				currentLength += (wCurrent.pose.translation() - wPrev.pose.translation()).norm();
			}

			//further improvement seems not possible
			distImprovement /= pathSize;
			angImprovement /= pathSize;
			if (distImprovement < minDistImprovementPerIter && angImprovement < minAngImprovementPerIter)
			{
				LOG_INFO("Stopped after iteration " << i << ": " << distImprovement << " " << angImprovement);
				break;
			}

			replaceOld = true;

//			newPath->writeTrajectoryData(std::string("/tmp/traj_iter_") + std::to_string(i) + ".traj");
//
//			std::ofstream file(std::string("/tmp/weights_") + std::to_string(i) + ".traj");
//
//			for (size_t w = 0; w < weight.size(); ++w)
//			{
//				file << weight[w] << "\n";
//			}
//
//			file.flush();

		}
	}

	finish:
	drawCounter++;

//	newPath = m_path;
	static int k = 0;
	LOG_INFO("Writing " << k);
	newPath->writeTrajectoryData(std::string("/tmp/traj_") + std::to_string(k++) + ".traj");

	return true;
//	ais_util::StopWatch::getInstance()->stopPrint("optimize");

}

} /* namespace prm_planner */
