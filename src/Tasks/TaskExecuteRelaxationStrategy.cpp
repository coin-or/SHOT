#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;

	relaxationStrategy = new RelaxationStrategyStandard(this->MILPSolver);

	ProcessInfo::getInstance().relaxationStrategy = relaxationStrategy;

	isInitialized = false;
}

TaskExecuteRelaxationStrategy::~TaskExecuteRelaxationStrategy()
{
	delete relaxationStrategy;
}

void TaskExecuteRelaxationStrategy::run()
{
	if (!isInitialized)
	{
		relaxationStrategy->setInitial();
		isInitialized = true;
	}
	else
	{
		relaxationStrategy->executeStrategy();
	}
}
std::string TaskExecuteRelaxationStrategy::getType()
{
	std::string type = typeid(this).name();
	return (type);
}

