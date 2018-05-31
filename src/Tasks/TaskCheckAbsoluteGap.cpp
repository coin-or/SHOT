/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckAbsoluteGap.h"

TaskCheckAbsoluteGap::TaskCheckAbsoluteGap(std::string taskIDTrue)
{
	taskIDIfTrue = taskIDTrue;
}

TaskCheckAbsoluteGap::~TaskCheckAbsoluteGap()
{
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
