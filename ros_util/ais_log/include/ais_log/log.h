/*
 * This file (log.h) is part of the "ais_definitions" packages of Daniel Kuhner.
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
#ifndef ROBOTIC_LIBS_LOG_LOG_H_
#define ROBOTIC_LIBS_LOG_LOG_H_

/*
 * Copyright (c) 2013 Daniel Kuhner.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * Contributors:
 *     Daniel Kuhner - kuhnerd@informatik.uni-freiburg.de
 */

#ifndef LOG_H_
#define LOG_H_

#include <string.h>
#include <ais_log/logger.h>

#define __FILENAME_ONLY__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define LOG_FATAL(TEXT) CommanderLogger::getInstance()->printFatal(__FILENAME_ONLY__, __LINE__) << TEXT << std::endl;
#define LOG_ERROR(TEXT) CommanderLogger::getInstance()->printError(__FILENAME_ONLY__, __LINE__) << TEXT << std::endl;
#define LOG_WARNING(TEXT) CommanderLogger::getInstance()->printWarning(__FILENAME_ONLY__, __LINE__) << TEXT << std::endl;
#define LOG_INFO(TEXT) CommanderLogger::getInstance()->printInfo(__FILENAME_ONLY__, __LINE__) << TEXT << std::endl;
#define LOG_DEBUG(TEXT) CommanderLogger::getInstance()->printDebug(__FILENAME_ONLY__, __LINE__) << TEXT << std::endl;

#define LOG_FATAL_COND(CONDITION, TEXT) if (CONDITION) {LOG_FATAL(TEXT)}
#define LOG_ERROR_COND(CONDITION, TEXT) if (CONDITION) {LOG_ERROR(TEXT)}
#define LOG_WARNING_COND(CONDITION, TEXT) if (CONDITION) {LOG_WARNING(TEXT)}
#define LOG_INFO_COND(CONDITION, TEXT) if (CONDITION) {LOG_INFO(TEXT)}
#define LOG_DEBUG_COND(CONDITION, TEXT) if (CONDITION) {LOG_DEBUG(TEXT)}

#endif /* LOG_H_ */


#endif /* ROBOTIC_LIBS_LOG_LOG_H_ */
