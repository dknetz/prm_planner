/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 9, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: data_logger.cpp
 */

#include <ais_log/data_logger.h>
#include <iomanip>

namespace ais_log
{

DataLogger::DataLogger(const std::string& filename) :
				m_stream(filename),
				m_firstLine(true),
				m_counter(0)
{
	m_stream << std::setprecision(10);
}

DataLogger::~DataLogger()
{
}

DataLogger& DataLogger::operator<<(bool val)
{
	m_stream << (val ? "true" : "false") << " ";
	m_firstLine = false;
	increment();
	return *this;
}

DataLogger& DataLogger::operator<<(int val)
{
	m_stream << std::to_string(val) << " ";
	m_firstLine = false;
	increment();
	return *this;
}

DataLogger& DataLogger::operator<<(double val)
{
	m_stream << std::to_string(val) << " ";
	m_firstLine = false;
	increment();
	return *this;
}

DataLogger& DataLogger::operator<<(float val)
{
	m_stream << std::to_string(val) << " ";
	m_firstLine = false;
	increment();
	return *this;
}

DataLogger& DataLogger::operator<<(std::string val)
{
	m_stream << val << " ";
	m_firstLine = false;
	increment();
	return *this;
}

void DataLogger::newLine()
{
	if (!m_firstLine)
		m_stream << "\n";
}

void DataLogger::close()
{
	m_stream.flush();
	m_stream.close();
}

void DataLogger::increment()
{
	if (++m_counter > 200)
	{
		m_counter = 0;
		m_stream.flush();
	}
}

} /* namespace ais_log */

