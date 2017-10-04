/*
 * This file (logger.cpp) is part of the "ais_definitions" packages of Daniel Kuhner.
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

#include <ais_log/logger.h>
#include <ais_log/ansi_macros.h>

SINGLETON_SOURCE(CommanderLogger)

CommanderLogger::~CommanderLogger()
{
}

CommanderLogger::CommanderLogger()
{
}

std::ostream& CommanderLogger::printFatal(const std::string& file,
		const int& line)
{
	std::cout << ANSI_BOLD << ANSI_BACK_RED << "[" << ANSI_WHITE << "F"
			<< ANSI_WHITE << ", " << file << ", " << line << "]:" << ANSI_RESET << " ";
	std::flush(std::cout);
	return std::cout;
}

std::ostream& CommanderLogger::printError(const std::string& file,
		const int& line)
{
	std::cout << ANSI_BOLD << ANSI_BACK_RED << "[" << ANSI_RED << "E"
			<< ANSI_WHITE << ", " << file << ", " << line << "]:" << ANSI_RESET << " ";
	return std::cout;
}

std::ostream& CommanderLogger::printWarning(const std::string& file,
		const int& line)
{
	std::cout << ANSI_BOLD << ANSI_BACK_YELLOW << "[" << ANSI_BLACK << "W"
			<< ANSI_WHITE << ", " << file << ", " << line << "]:" << ANSI_RESET << " ";
	return std::cout;
}

std::ostream& CommanderLogger::printInfo(const std::string& file,
		const int& line)
{
	std::cout << ANSI_BOLD << ANSI_BACK_LIGHT_BLUE << "[" << ANSI_GREEN << "I"
			<< ANSI_WHITE << ", " << file << ", " << line << "]:" << ANSI_RESET << " ";
	return std::cout;
}

std::ostream& CommanderLogger::printDebug(const std::string& file,
		const int& line)
{
	std::cout << ANSI_BOLD << ANSI_BACK_LIGHT_BLUE << "[" << ANSI_MAGENTA << "D"
			<< ANSI_WHITE << ", " << file << ", " << line << "]:" << ANSI_RESET << " ";
	return std::cout;
}

std::ostream& operator <<(std::ostream& stream,
		const std::unordered_map<std::string, double>& data)
{
	stream << "STD: UnorderedMap [string -> double]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it.first.c_str() << "->" << it.second << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::unordered_map<std::string, int>& data)
{
	stream << "STD: UnorderedMap [string -> int]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it.first.c_str() << "->" << it.second << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::unordered_map<std::string, long unsigned int>& data)
{
	stream << "STD: UnorderedMap [string -> long unsigned int]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it.first.c_str() << "->" << it.second << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::map<std::string, double>& data)
{
	stream << "STD: Map [string -> double]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it.first.c_str() << "->" << it.second << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::map<std::string, int>& data)
{
	stream << "STD: Map [string -> int]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it.first.c_str() << "->" << it.second << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::vector<std::string>& data)
{
	stream << "STD: Vector [string]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::vector<int>& data)
{
	stream << "STD: Vector [int]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::vector<long unsigned int>& data)
{
	stream << "STD: Vector [long unsigned int]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it << "\n";
	}
	stream << "\n";
	return stream;
}

std::ostream& operator <<(std::ostream& stream,
		const std::vector<double>& data)
{
	stream << "STD: Vector [double]: \n";
	for (auto& it : data)
	{
		stream << "\t- " << it << "\n";
	}
	stream << "\n";
	return stream;
}
