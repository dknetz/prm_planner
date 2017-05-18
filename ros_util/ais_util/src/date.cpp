/*
 * This file (date.cpp) is part of the "ais_definitions" packages of Daniel Kuhner.
 *
 * It is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this code files.  If not, see <http://www.gnu.org/licenses/>.
 *
 * created:		Mar 13, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#include <ais_util/date.h>

namespace ais_util
{

Date::Date()
{
}

Date::~Date()
{
}

std::string Date::getFormatedDate(const std::string& format)
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[100];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 100, format.c_str(), timeinfo);

	return std::string(buffer);
}

std::string Date::getFormatedDuration(double duration)
{
	int hours = 0;
	int minutes = 0;
	int seconds = 0;
	double microseconds = (duration - (int) duration);
	int totalSeconds = (int) duration;

	char buff[100];

	if (duration < 60)
	{
		seconds = totalSeconds;
		sprintf(buff, "%02d:%.2f seconds", seconds, microseconds);
	}
	else if (duration < 3600)
	{
		minutes = totalSeconds / 60;
		seconds = totalSeconds % 60;
		sprintf(buff, "%02d:%02d:%.2f minutes", minutes, seconds, microseconds);
	}
	else
	{
		hours = duration / 3600;
		minutes = (totalSeconds % 3600) / 60;
		seconds = (totalSeconds % 3600) % 60;
		sprintf(buff, "%02d:%02d:%02d:%.2f hours", hours, minutes, seconds, microseconds);
	}

	return std::string(buff);
}

} /* namespace ais_util */
