/*
 * TaskTerminate.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include <TaskTerminate.h>

TaskTerminate::TaskTerminate()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskTerminate::~TaskTerminate()
{
	// TODO Auto-generated destructor stub
}

void TaskTerminate::run()
{
	//processInfo->tasks->clearTasks();
}
