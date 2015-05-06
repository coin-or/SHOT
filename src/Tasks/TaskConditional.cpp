#include "TaskConditional.h"

TaskConditional::TaskConditional()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
	taskFalseIsSet = false;
}

TaskConditional::TaskConditional(std::function<bool()> conditionFunct, TaskBase *taskTrue, TaskBase *taskFalse)
{
	condition = conditionFunct;
	taskIfTrue = taskTrue;
	taskIfFalse = taskFalse;
	taskFalseIsSet = true;
}

TaskConditional::~TaskConditional()
{
}

void TaskConditional::setCondition(std::function<bool()> conditionFunct)
{
	condition = conditionFunct;
}

void TaskConditional::setTaskIfTrue(TaskBase * task)
{
	taskIfTrue = task;
}

void TaskConditional::setTaskIfFalse(TaskBase * task)
{
	taskIfFalse = task;
	taskFalseIsSet = true;
}

void TaskConditional::run()
{
	bool tmpCondition;

	if (condition != nullptr)
	{
		tmpCondition = condition();
	}

	if (tmpCondition)
	{
		taskIfTrue->run();
	}
	else
	{
		if (taskFalseIsSet == true) taskIfFalse->run();
	}
}
std::string TaskConditional::getType()
{
	std::string type = typeid(this).name();
	return (type);

}

