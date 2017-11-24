#include "TaskInitializeIteration.h"

TaskInitializeIteration::TaskInitializeIteration()
{

}

TaskInitializeIteration::~TaskInitializeIteration()
{
}

void TaskInitializeIteration::run()
{
	ProcessInfo::getInstance().startTimer("Subproblems");
	ProcessInfo::getInstance().createIteration();
}

std::string TaskInitializeIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);
}
