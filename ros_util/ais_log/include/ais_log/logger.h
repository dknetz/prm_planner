/*
 * This file (logger.h) is part of the "ais_definitions" packages of Daniel Kuhner.
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
 * created:		Feb 6, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef ROBOTIC_LIBS_LOG_LOGGER_H_
#define ROBOTIC_LIBS_LOG_LOGGER_H_

#include <ais_definitions/class.h>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>

class CommanderLogger
{
SINGLETON_HEADER(CommanderLogger)

PUBLIC_METHODS:
	virtual ~CommanderLogger();

	std::ostream& printFatal(const std::string& file,
			const int& line);
	std::ostream& printError(const std::string& file,
			const int& line);
	std::ostream& printWarning(const std::string& file,
			const int& line);
	std::ostream& printInfo(const std::string& file,
			const int& line);
	std::ostream& printDebug(const std::string& file,
			const int& line);
};

std::ostream& operator<<(std::ostream& stream,
		const std::unordered_map<std::string, double>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::unordered_map<std::string, int>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::unordered_map<std::string, long unsigned int>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::map<std::string, double>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::map<std::string, int>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::vector<std::string>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::vector<int>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::vector<long unsigned int>& data);
std::ostream& operator<<(std::ostream& stream,
		const std::vector<double>& data);

#endif /* ROBOTIC_LIBS_LOG_LOGGER_H_ */
