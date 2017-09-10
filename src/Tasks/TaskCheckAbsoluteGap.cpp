/*
 * TaskCheckConstraintTolerance.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: alundell
 */

#include "TaskCheckAbsoluteGap.h"

TaskCheckAbsoluteGap::TaskCheckAbsoluteGap(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}

TaskCheckAbsoluteGap::~TaskCheckAbsoluteGap()
{
// TODO Auto-generated destructor stub
}

void TaskCheckAbsoluteGap::run()
{
	if (ProcessInfo::getInstance().isAbsoluteObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::AbsoluteGap;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckAbsoluteGap::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
