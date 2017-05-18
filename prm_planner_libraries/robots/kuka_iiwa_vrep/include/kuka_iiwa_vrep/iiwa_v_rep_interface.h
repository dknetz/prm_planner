/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_v_rep_interface.h
 */

#ifndef H20A551E2_AB04_46AA_8DD3_4590AD22F069
#define H20A551E2_AB04_46AA_8DD3_4590AD22F069

#include <prm_planner_robot/robot_interface.h>

#ifdef FOUND_VREP
#include <vrep_interface/v_rep_interface.h>
#include <vrep_interface/v_rep_iiwa.h>
#endif

namespace kuka_iiwa_vrep
{

class IiwaVRepInterface: public prm_planner::RobotInterface
{
public:
	IiwaVRepInterface();
	virtual ~IiwaVRepInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();

private:
#ifdef FOUND_VREP
	struct VrepData
	{
		VrepData() :
						vrepHandle(-1),
						vrepDirection(1.0)
		{
		}

		double vrepDirection; //used to compensate for other frames in vrep
		simxInt vrepHandle;
	};

	typedef std::unordered_map<std::string, VrepData> VrepDataType;

	/**
	 * vrep interface instance. Don't delete it, because
	 * it is managed by a singleton class
	 */
	vrep_interface::VRepInterface* m_vrep;

	VrepDataType m_vrepData;

	bool m_firstRead;
#endif
};

} /* namespace kuka_iiwa_vrep */

#endif /* H20A551E2_AB04_46AA_8DD3_4590AD22F069 */
