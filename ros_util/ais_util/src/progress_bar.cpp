/*
 * progress_bar.cpp
 *
 *  Created on: May 20, 2015
 *      Author: kuhnerd
 */

#include <ais_util/progress_bar.h>

#include <ais_log/ansi_macros.h>
#include <ais_util/date.h>

#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

namespace ais_util
{

int ProgressBar::nameCounter = 0;
bool ProgressBar::s_pubInit = false;
std::string ProgressBar::s_rosPub;
ros::Publisher ProgressBar::s_pub;

ProgressBar::ProgressBar(const std::string& label,
		int total,
		bool shortBar) :
				label(label),
				total(total),
				current(0),
				runTimer(false),
				stopTimer(false),
				movingDirectionForward(true),
				lastDirectionChange(0),
				print(true),
				thread(NULL),
				threadWaitFor(NULL),
				m_shortBar(shortBar)
{
	gettimeofday(&tStart, NULL);
	threadName = "progress_bar_" + std::to_string(nameCounter++);
}

ProgressBar::ProgressBar(boost::thread* waitFor,
		const std::string& label,
		int total,
		bool shortBar) :
				label(label),
				total(total),
				current(0),
				runTimer(false),
				stopTimer(false),
				movingDirectionForward(true),
				lastDirectionChange(0),
				print(true),
				thread(NULL),
				threadWaitFor(waitFor),
				m_shortBar(shortBar)
{
	gettimeofday(&tStart, NULL);
	threadName = "progress_bar_" + std::to_string(nameCounter++);
}

ProgressBar::~ProgressBar()
{
	thread->interrupt();
	thread->join();
	DELETE_VAR(thread);
}

void ProgressBar::set(int current,
		const std::string& additionalInfos)
{
	boost::recursive_mutex::scoped_lock lock(mutex);

	this->current = current;
	this->additionalInfos = additionalInfos;

	if (!runTimer)
	{
		runTimer = true;
		thread = new boost::thread(boost::bind(&ProgressBar::update, this));
	}
}

void ProgressBar::increment(const std::string& additionalInfos)
{
	boost::recursive_mutex::scoped_lock lock(mutex);

	++current;
	this->additionalInfos = additionalInfos;

	if (!runTimer)
	{
		runTimer = true;
		thread = new boost::thread(boost::bind(&ProgressBar::update, this));
	}
}

void ProgressBar::finish(const std::string& additionalInfos)
{
	set(total, additionalInfos);
	updateText();
	printf("\n");
	fflush(stdout);
	{
		boost::recursive_mutex::scoped_lock lock(mutex);
		stopTimer = true;
	}
	thread->join();
}

void ProgressBar::update()
{
	while (!stopTimer && !boost::this_thread::interruption_requested())
	{
		if (print)
		{
			updateText();
			if (s_pubInit)
			{
				std_msgs::Float32 f;
				f.data = ((double) current * 100.0) / (double) total;
				s_pub.publish(f);
			}
		}
		boost::this_thread::sleep(boost::posix_time::millisec(50));
	}
}

void ProgressBar::updateText()
{
	boost::recursive_mutex::scoped_lock lock(mutex);

	Date date;

	gettimeofday(&t2, NULL);
	double duration = (t2.tv_sec + (t2.tv_usec / 1000000.0)) - (tStart.tv_sec + (tStart.tv_usec / 1000000.0));

	//progress width
	const int pwidth = m_shortBar ? 36 : 72;
	const double speed = 15;

	//minus label len
	const int width = pwidth - label.size();

	printf("%s[%s%s", label.c_str(), ANSI_GREEN, ANSI_BOLD);

	//normal progress bar
	if (total > 0)
	{
		int pos = (current * width) / total;

		double percent = ((double) current * 100.0) / (double) total;

		//fill progress bar with =
		for (int i = 0; i < pos; i++)
			printf("%c", '=');

		//fill progress bar with spaces
		printf("%*s", width - pos, "");
		printf("%s] %3.2f%%", ANSI_RESET, percent);

		double complete = (100 / percent) * duration;
		double remaining = complete - duration;
		bool remainingUse = true;
		static const double thresRemaining = 0.01 * total;
		if (current < thresRemaining)
		{
			remainingUse = false;
		}

		if (m_shortBar)
		{
			printf(" (%d/%d) %s\r",
					current,
					total, additionalInfos.c_str());
		}
		else
		{
			printf(" (%d/%d) - Runtime: %22s, Complete: %22s, Remaining: %22s  %s\r",
					current,
					total,
					date.getFormatedDuration(duration).c_str(),
					date.getFormatedDuration(complete).c_str(),
					remainingUse ? date.getFormatedDuration(remaining).c_str() : "---",
					additionalInfos.c_str());
		}
	}
	//moving bar
	else
	{
		int pos = ((int) (duration * speed)) % (width - 2) + 1;

		if (pos == 1 && !movingDirectionForward && lastDirectionChange > 2)
		{
			movingDirectionForward = true;
			lastDirectionChange = 0;
		}
		else if (pos >= width - 2 && movingDirectionForward && lastDirectionChange > 2)
		{
			movingDirectionForward = false;
			pos = 1;
			lastDirectionChange = 0;
		}

		if (!movingDirectionForward && !(lastDirectionChange <= 3 && pos == width - 2))
		{
			pos = width - pos - 1;
		}

		printf("%*s", pos - 1, "");
		printf("%s", "===");
		printf("%*s%s]", width - pos - 2, "", ANSI_RESET);

		++lastDirectionChange;

		if (!m_shortBar)
			printf("\r");
		else
			printf(" - Runtime: %22s  %s\r", date.getFormatedDuration(duration).c_str(), additionalInfos.c_str());
	}

	fflush(stdout);
}

bool ProgressBar::isPrint() const
{
	boost::recursive_mutex::scoped_lock lock(mutex);
	return print;
}

void ProgressBar::setPrint(bool print)
{
	boost::recursive_mutex::scoped_lock lock(mutex);
	this->print = print;
}

void ProgressBar::activateRosPublisher(const std::string& rosPub)
{
	s_rosPub = rosPub;
	if (!s_rosPub.empty() && !s_pubInit)
	{
		ros::NodeHandle n;
		s_pub = n.advertise<std_msgs::Float32>(s_rosPub, 1);
		std::cout << "Sending progress on " << s_rosPub << std::endl;
		s_pubInit = true;
	}
}

} /* namespace ais_util */

