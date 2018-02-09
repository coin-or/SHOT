#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(IMIPSolver *MIPSolver)
{
	this->MIPSolver = MIPSolver;

	relaxationStrategy = new RelaxationStrategyStandard(this->MIPSolver);

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

