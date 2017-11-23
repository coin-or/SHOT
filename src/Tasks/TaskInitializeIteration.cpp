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

	//TODO: fix line below...
	//ProcessInfo::getInstance().getCurrentIteration()->type = ProcessInfo::getInstance().relaxationStrategy->getProblemType();

}

std::string TaskInitializeIteration::getType()
{
	std::string type = typeid(this).name();
	return (type);

}
