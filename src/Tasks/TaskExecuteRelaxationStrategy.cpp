#include "TaskExecuteRelaxationStrategy.h"

TaskExecuteRelaxationStrategy::TaskExecuteRelaxationStrategy(IMILPSolver *MILPSolver)
{
	this->MILPSolver = MILPSolver;
	processInfo = ProcessInfo::getInstance();
	settings = SHOTSettings::Settings::getInstance();

	if (settings->getIntSetting("RelaxationStrategy", "Algorithm") == static_cast<int>(ES_RelaxationStrategy::Adaptive))
	{
		relaxationStrategy = new RelaxationStrategyAdaptive(this->MILPSolver);
	}
	else if (settings->getIntSetting("RelaxationStrategy", "Algorithm")
			== static_cast<int>(ES_RelaxationStrategy::Standard))
	{
		relaxationStrategy = new RelaxationStrategyStandard(this->MILPSolver);
	}
	else if (settings->getIntSetting("RelaxationStrategy", "Algorithm")
			== static_cast<int>(ES_RelaxationStrategy::None))
	{
		relaxationStrategy = new RelaxationStrategyNone(this->MILPSolver);
	}

	processInfo->relaxationStrategy = relaxationStrategy;

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

