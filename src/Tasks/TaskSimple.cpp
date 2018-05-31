/**
   The Supporting Hyperplane Optimization Toolkit (SHOT).

   @author Andreas Lundell, Åbo Akademi University

   @section LICENSE 
   This software is licensed under the Eclipse Public License 2.0. 
   Please see the README and LICENSE files for more information.
*/

#include "TaskSimple.h"

TaskSimple::TaskSimple()
{
}

TaskSimple::TaskSimple(std::function<bool()> taskFunction)
{
	task = taskFunction;
}

TaskSimple::~TaskSimple()
{
}

void TaskSimple::setFunction(std::function<bool()> taskFunction)
{
	task = taskFunction;
}

void TaskSimple::run()
{
	if (task != nullptr)
	{
		task();
	}
	else
	{
		TaskExceptionFunctionNotDefined e("TaskSimple");

		throw(e);
	}
}

std::string TaskSimple::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
