/*
 * Copyright (c) 2016 Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>.
 * All rights reserved.
 * 
 *  Created on: Dec 9, 2016
 *      Author: Daniel Kuhner <kuhnerd@informatik.uni-freiburg.de>
 * 	  Filename: data_logger.h
 */

#ifndef HA6462550_3CB5_4A0F_8907_C635DA5A7C53
#define HA6462550_3CB5_4A0F_8907_C635DA5A7C53

#include <unordered_map>
#include <string.h>
#include <fstream>

#define DATA_LOGGER_INIT() std::unordered_map<std::string, boost::shared_ptr<ais_log::DataLogger>> __data_ofstreams
#define DATA_LOGGER_ADD(name, filename) __data_ofstreams[name].reset(new ais_log::DataLogger(filename));
#define DATA_LOGGER_WRITE(name) __data_ofstreams[name]->newLine(); *(__data_ofstreams[name])
#define DATA_LOGGER_CLOSE(name) __data_ofstreams[name]->close()

namespace ais_log
{

class DataLogger
{
public:
	DataLogger(const std::string& filename);
	virtual ~DataLogger();

	DataLogger& operator<<(bool val);
	DataLogger& operator<<(int val);
	DataLogger& operator<<(double val);
	DataLogger& operator<<(float val);
	DataLogger& operator<<(std::string val);

	void newLine();

	void close();

private:
	void increment();

private:
	std::ofstream m_stream;
	bool m_firstLine;
	int m_counter;
};

} /* namespace ais_log */

#endif /* HA6462550_3CB5_4A0F_8907_C635DA5A7C53 */
