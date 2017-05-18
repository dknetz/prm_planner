/*
 * This file (exception.h) is part of the "ais_definitions" packages of Daniel Kuhner.
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
 * created:		Feb 2, 2015
 * authors:     Daniel Kuhner  <kuhnerd@informatik.uni-freiburg.de>
 *
 */
#ifndef ROBOTIC_LIBS_DEFITIONS_EXCEPTION_H_
#define ROBOTIC_LIBS_DEFITIONS_EXCEPTION_H_

#include <ais_definitions/class.h>
#include <iostream>

namespace ais_definitions
{
class Exception
{
PUBLIC_METHODS:
	Exception(const std::string& what) :
					m_what(what)
	{
	}

	std::string what() const
	{
		return m_what;
	}

	void print() const
	{
		std::cerr << "Exception: " << m_what << std::endl;
	}

PRIVATE_VARIABLES:
	const std::string m_what;
};

class NotImplementedException: public Exception
{
PUBLIC_METHODS:
	NotImplementedException() :
					Exception("This function/method is not implemented!")
	{
	}
};
}

#endif /* ROBOTIC_LIBS_DEFITIONS_EXCEPTION_H_ */
