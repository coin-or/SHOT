#include "TaskSimple.h"

TaskSimple::TaskSimple()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
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
