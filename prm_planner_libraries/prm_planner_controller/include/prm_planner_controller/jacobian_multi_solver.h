/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Feb 26, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: jacobian_multi_solver.h
 */

#ifndef JACOBIAN_MULTI_SOLVER_H_
#define JACOBIAN_MULTI_SOLVER_H_
#include <ais_log/log.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace prm_planner
{

class JacobianMultiSolver
{
public:
	static const int E_JAC_FAILED = -100; //! Jac solver failed

	JacobianMultiSolver(const KDL::Chain& _chain);
	virtual ~JacobianMultiSolver();

	template<int DOF>
	int GetAllJacobians(const KDL::JntArray& q_in,
			std::vector<Eigen::Matrix<double, 6, DOF>>& jacobians)
	{
		KDL::Jacobian jac(DOF);
		int error = 0;
		unsigned int segmentNr = chain.getNrOfSegments();

		//Initialize Jacobian to zero since only segmentNr colunns are computed
		SetToZero(jac);
		jacobians.resize(segmentNr);

		T_tmp = KDL::Frame::Identity();
		SetToZero(t_tmp);
		int j = 0;
		int k = 0;
		KDL::Frame total;
		for (unsigned int i = 0; i < segmentNr; i++)
		{
			const KDL::Segment& segment = chain.getSegment(i);
			const bool& isLocked = locked_joints_[j];
			//Calculate new Frame_base_ee
			if (segment.getJoint().getType() != KDL::Joint::None)
			{
				//pose of the new end-point expressed in the base
				total = T_tmp * segment.pose(q_in(j));
				//changing base of new segment's twist to base frame if it is not locked
				//t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
//				LOG_INFO(segment.twist(q_in(j), 1.0)[0] << " " <<
//						segment.twist(q_in(j), 1.0)[1] << " " <<
//						segment.twist(q_in(j), 1.0)[2] << " " <<
//						segment.twist(q_in(j), 1.0)[3] << " " <<
//						segment.twist(q_in(j), 1.0)[4] << " " <<
//						segment.twist(q_in(j), 1.0)[5]);
				if (!isLocked)
					t_tmp = T_tmp.M * segment.twist(q_in(j), 1.0);
			}
			else
			{
				total = T_tmp * segment.pose(0.0);

			}

			//Changing Refpoint of all columns to new ee
			changeRefPoint(jac, total.p - T_tmp.p, jac);

			//Only increase jointnr if the segment has a joint
			if (segment.getJoint().getType() != KDL::Joint::None)
			{
				//Only put the twist inside if it is not locked
				if (!isLocked)
				{
					jac.setColumn(k++, t_tmp);
				}
				j++;
			}

			jacobians[i] = jac.data;

			T_tmp = total;
		}

		return (error != E_JAC_FAILED);
	}

	template<int DOF>
	int GetAllJacobians(const KDL::JntArray& q_in,
			std::vector<KDL::Jacobian>& jacobians)
	{
		KDL::Jacobian jac(DOF);
		int error = 0;
		unsigned int segmentNr = chain.getNrOfSegments();

		//Initialize Jacobian to zero since only segmentNr colunns are computed
		SetToZero(jac);
		jacobians.resize(segmentNr);

		T_tmp = KDL::Frame::Identity();
		SetToZero(t_tmp);
		int j = 0;
		int k = 0;
		KDL::Frame total;
		for (unsigned int i = 0; i < segmentNr; i++)
		{
			const KDL::Segment& segment = chain.getSegment(i);
			const bool& isLocked = locked_joints_[j];
			//Calculate new Frame_base_ee
			if (segment.getJoint().getType() != KDL::Joint::None)
			{
				//pose of the new end-point expressed in the base
				total = T_tmp * segment.pose(q_in(j));
				//changing base of new segment's twist to base frame if it is not locked
				//t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
				//				LOG_INFO(segment.twist(q_in(j), 1.0)[0] << " " <<
				//						segment.twist(q_in(j), 1.0)[1] << " " <<
				//						segment.twist(q_in(j), 1.0)[2] << " " <<
				//						segment.twist(q_in(j), 1.0)[3] << " " <<
				//						segment.twist(q_in(j), 1.0)[4] << " " <<
				//						segment.twist(q_in(j), 1.0)[5]);
				if (!isLocked)
					t_tmp = T_tmp.M * segment.twist(q_in(j), 1.0);
			}
			else
			{
				total = T_tmp * segment.pose(0.0);

			}

			//Changing Refpoint of all columns to new ee
			changeRefPoint(jac, total.p - T_tmp.p, jac);

			//Only increase jointnr if the segment has a joint
			if (segment.getJoint().getType() != KDL::Joint::None)
			{
				//Only put the twist inside if it is not locked
				if (!isLocked)
				{
					jac.setColumn(k++, t_tmp);
				}
				j++;
			}

			jacobians[i] = jac;

			T_tmp = total;
		}

		return (error != E_JAC_FAILED);
	}

	/**
	 * Calculate the jacobian expressed in the base frame of the
	 * chain, with reference point at the end effector of the
	 * *chain. The alghoritm is similar to the one used in
	 * KDL::ChainFkSolverVel_recursive
	 *
	 * @param q_in input joint positions
	 * @param jac output jacobian
	 *
	 * @return always returns 0
	 */
	template<int DOF>
	int JntToJac(const KDL::JntArray& q_in,
			Eigen::Matrix<double, 6, DOF>& jacobian,
			int segmentNR = -1)
	{
		KDL::Jacobian jac(DOF);
		int error = 0;
		unsigned int segmentNr;
		if (segmentNR < 0)
			segmentNr = chain.getNrOfSegments();
		else
			segmentNr = segmentNR;

		//Initialize Jacobian to zero since only segmentNr colunns are computed
		SetToZero(jac);

		if (q_in.rows() != chain.getNrOfJoints() || nr_of_unlocked_joints_ != jac.columns())
			return (error = E_JAC_FAILED);
		else if (segmentNr > chain.getNrOfSegments())
			return (error = E_JAC_FAILED);

		T_tmp = KDL::Frame::Identity();
		SetToZero(t_tmp);
		int j = 0;
		int k = 0;
		KDL::Frame total;
		for (unsigned int i = 0; i < segmentNr; i++)
		{
			//Calculate new Frame_base_ee
			if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
			{
				//pose of the new end-point expressed in the base
				total = T_tmp * chain.getSegment(i).pose(q_in(j));
				//changing base of new segment's twist to base frame if it is not locked
				//t_tmp = T_tmp.M*chain.getSegment(i).twist(1.0);
				if (!locked_joints_[j])
					t_tmp = T_tmp.M * chain.getSegment(i).twist(q_in(j), 1.0);
			}
			else
			{
				total = T_tmp * chain.getSegment(i).pose(0.0);

			}

			//Changing Refpoint of all columns to new ee
			changeRefPoint(jac, total.p - T_tmp.p, jac);

			//Only increase jointnr if the segment has a joint
			if (chain.getSegment(i).getJoint().getType() != KDL::Joint::None)
			{
				//Only put the twist inside if it is not locked
				if (!locked_joints_[j])
				{
					jac.setColumn(k++, t_tmp);
				}
				j++;
			}

			T_tmp = total;
		}

		jacobian = jac.data;

		return (error != E_JAC_FAILED);
	}

	int setLockedJoints(const std::vector<bool> locked_joints);
	void setChain(const KDL::Chain& _chain);

	/// @copydoc KDL::SolverI::strError()
	virtual const char* strError(const int error) const;

private:
	KDL::Chain chain;
	KDL::Twist t_tmp;
	KDL::Frame T_tmp;
	std::vector<bool> locked_joints_;
	unsigned int nr_of_unlocked_joints_;
};

} /* namespace prm_planner */

#endif /* JACOBIAN_MULTI_SOLVER_H_ */
