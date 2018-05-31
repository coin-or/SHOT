/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskCheckIterationLimit.h"

TaskCheckIterationLimit::TaskCheckIterationLimit(std::string taskIDTrue)
{

	taskIDIfTrue = taskIDTrue;
}

TaskCheckIterationLimit::~TaskCheckIterationLimit()
{
}

void TaskCheckIterationLimit::run()
{
	auto currIter = ProcessInfo::getInstance().getCurrentIteration();

	if (currIter->iterationNumber >= Settings::getInstance().getIntSetting("Relaxation.IterationLimit", "Dual") + Settings::getInstance().getIntSetting("IterationLimit", "Termination"))
	{
		ProcessInfo::getInstance().terminationReason = E_TerminationReason::IterationLimit;
		ProcessInfo::getInstance().tasks->setNextTask(taskIDIfTrue);
	}
}

std::string TaskCheckIterationLimit::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
