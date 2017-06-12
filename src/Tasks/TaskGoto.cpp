/*
 * TaskGoto.cpp
 *
 *  Created on: Apr 1, 2015
 *      Author: alundell
 */

#include "TaskGoto.h"

TaskGoto::TaskGoto(std::string taskID)
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	gotoTaskID = taskID;
}

TaskGoto::~TaskGoto()
{
	// TODO Auto-generated destructor stub
}

void TaskGoto::run()
{
	processInfo->tasks->setNextTask(gotoTaskID);
}

std::string TaskGoto::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
