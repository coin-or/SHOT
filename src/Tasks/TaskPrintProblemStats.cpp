/*
 * TaskPrintProblemStats.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include <TaskPrintProblemStats.h>

TaskPrintProblemStats::TaskPrintProblemStats()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskPrintProblemStats::~TaskPrintProblemStats()
{
	// TODO Auto-generated destructor stub
}

void TaskPrintProblemStats::run()
{
	//TODO: Refactor print stats from problem class
	processInfo->originalProblem->printProblemStatistics();
}

std::string TaskPrintProblemStats::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
