/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Oct 10, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: iiwa_robot_interface.h
 */

#ifndef HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215
#define HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215

#ifdef FOUND_IIWA_FRI

#include <prm_planner_robot/robot_interface.h>
#include <iiwa_fri/friUdpConnection.h>
#include <iiwa_fri/friClientApplication.h>
#include <iiwa_fri/friLBRClient.h>
#include <boost/thread.hpp>

namespace kuka_iiwa_robot
{

class IiwaRobotInterface: public prm_planner::RobotInterface
{
public:
	IiwaRobotInterface();
	virtual ~IiwaRobotInterface();

	virtual bool start();
	virtual bool read(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));

	/**
	 * This method sends the commands to the robot.
	 * You need to call this method to update the
	 * state of the system, because it calls the step()
	 * method of the ClientApplication.
	 */
	virtual bool write(const ros::Time time = ros::Time(0),
			const ros::Duration period = ros::Duration(0));
	virtual bool stop();

	RobotInterface::JointData::Mode setCmd(double& cmd,
			JointData& jd);

	void run();

public:
	struct Client: public KUKA::FRI::LBRClient
	{
		Client(Data& data);

		/**
		 * \brief Callback for the FRI state 'Commanding Active'.
		 */
		virtual void command();

		virtual void onStateChange(KUKA::FRI::ESessionState oldState,
				KUKA::FRI::ESessionState newState);

		void setCMD(double* cmd,
				RobotInterface::JointData::Mode& mode);

	private:
		double m_cmd[7];
		RobotInterface::JointData::Mode m_mode;
		boost::mutex m_mutex;
	};

private:
	KUKA::FRI::UdpConnection* m_udpConnection;
	Client* m_client;
	KUKA::FRI::ClientApplication* m_clientApplication;
	bool m_firstRead;
	boost::thread m_thread;

};

} /* namespace kuka_iiwa_robot */

#endif

#endif /* HF0FDEC6B_B2B2_45DA_A8B0_494EFAF39215 */
