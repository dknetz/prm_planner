/*
 * progress_bar.h
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#ifndef SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_PROGRESS_BAR_H_
#define SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_PROGRESS_BAR_H_

#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread.hpp>
#include <ros/publisher.h>
#include <string>

namespace ais_util
{

class ProgressBar
{
public:
	/**
	 * Create ProgressBar with total = 0 to get a moving marker
	 * instead of a increasing bar
	 * @param label
	 * @param total
	 */
	ProgressBar(const std::string& label,
			int total,
			bool shortBar = true);
	ProgressBar(boost::thread* waitFor,
			const std::string& label,
			int total,
			bool shortBar = true);
	virtual ~ProgressBar();

	void set(int current,
			const std::string& additionalInfos = "");
	void increment(const std::string& additionalInfos = "");

	void finish(const std::string& additionalInfos = "");

	bool isPrint() const;
	void setPrint(bool print);

	static void activateRosPublisher(const std::string& rosPub);

private:
	void update();
	void updateText();

private:
	std::string label;
	std::string additionalInfos;

	int total;
	int current;
	timeval tStart;
	timeval t2;
	bool runTimer;
	bool stopTimer;
	bool movingDirectionForward;
	int lastDirectionChange;
	bool print;

	mutable boost::recursive_mutex mutex;
	boost::thread* thread;

	boost::thread* threadWaitFor;

	static int nameCounter;
	std::string threadName;

	bool m_shortBar;
	static std::string s_rosPub;
	static bool s_pubInit;
	static ros::Publisher s_pub;
};

} /* namespace ais_util */

#endif /* SUBPROJECTS__ROBOTIC_LIBS_UTIL_INCLUDE_ROBOTIC_LIBS_UTIL_PROGRESS_BAR_H_ */
