/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckRelativeGap.h"

TaskCheckRelativeGap::TaskCheckRelativeGap(std::string taskIDTrue)
{
	taskIDIfTrue = taskIDTrue;
}

TaskCheckRelativeGap::~TaskCheckRelativeGap()
{
}

void TaskCheckRelativeGap::run()
{
	if (ProcessInfo::getInstance().isRelativeObjectiveGapToleranceMet())
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::RelativeGap;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckRelativeGap::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
