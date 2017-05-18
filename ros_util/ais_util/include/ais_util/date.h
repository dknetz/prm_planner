/*
 * This file (date.h) is part of the "ais_definitions" packages of Daniel Kuhner.
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
#ifndef ROBOTIC_LIBS_UTIL_DATE_H_
#define ROBOTIC_LIBS_UTIL_DATE_H_

#include <ais_definitions/class.h>
#include <string>

namespace ais_util
{

class Date
{
PUBLIC_METHODS:
	Date();
	virtual ~Date();

	std::string getFormatedDate(const std::string& format);

	static std::string getFormatedDuration(double duration);
};

} /* namespace ais_util */

#endif /* ROBOTIC_LIBS_UTIL_DATE_H_ */
