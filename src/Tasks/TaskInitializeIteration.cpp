#include "TaskInitializeIteration.h"

TaskInitializeIteration::TaskInitializeIteration()
{
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();
}

TaskInitializeIteration::~TaskInitializeIteration()
{
}

void TaskInitializeIteration::run()
{
	processInfo->startTimer("Subproblems");
	processInfo->createIteration();

	//TODO: fix line below...
	//processInfo->getCurrentIteration()->type = processInfo->relaxationStrategy->getProblemType();

}

std::string TaskInitializeIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
